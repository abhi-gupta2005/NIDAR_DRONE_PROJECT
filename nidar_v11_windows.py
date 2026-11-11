"""
NIDAR V10 - PERCEPTION + ADAPTIVE SPATIAL PREDICTION + SENSOR FUSION READY

Upgrade from V4:
- Keeps YOLOE open-vocabulary detection.
- Uses canonical object labels instead of exposing aliases as separate objects.
- Groups aliases such as:
    cell phone / smartphone -> phone
    headset / earphones -> headphones
    computer mouse -> mouse
    water bottle -> bottle
    remote control -> remote
- Trackerless detections (tracker_id=None) NEVER create or recover stable IDs.
- New stable IDs require consecutive real tracker hits.
- Recovery is one-to-one and conservative.
- Old tracker IDs are retired when they switch.
- Duplicate detections of the same canonical object are merged before tracking.
- Small/noisy boxes are filtered.
- CSV contains stable-object statistics.
- Designed for Windows + Ultralytics YOLOE.

Run:
    /usr/local/bin/python3 multi_object_tracking_v4_5.py
"""

import csv
import copy
import json
import math
import time
import os
import socket
import threading
from queue import Queue, Empty, Full
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
from ultralytics import YOLOE



# ============================================================
# RASPBERRY PI PERCEPTION LINK
# ============================================================

PI_HOST = os.environ.get("NIDAR_PI_HOST", "").strip()
PI_PORT = int(os.environ.get("NIDAR_PI_PORT", "5000"))
PI_LINK_QUEUE_SIZE = 1
PI_CONNECT_TIMEOUT = 0.5
PI_SEND_TIMEOUT = 0.5

# 2026-11-02: camera source.
#   webcam (default, unchanged) -> the laptop webcam
#   gazebo                      -> frames from the Gazebo simulation, sent by
#        gazebo/ros2/nidar_camera_relay.py in WSL (TCP, 4-byte big-endian
#        length + JPEG -- the same format gazebo_camera_receiver.py reads).
#   PowerShell:  $env:NIDAR_CAMERA_SOURCE = 'gazebo'
CAMERA_SOURCE = os.environ.get("NIDAR_CAMERA_SOURCE", "webcam").strip().lower()
GAZEBO_CAMERA_ADDR = os.environ.get("NIDAR_GAZEBO_CAMERA", "127.0.0.1:5012").strip()


class GazeboCameraCapture:
    """
    Minimal cv2.VideoCapture stand-in for the Gazebo camera relay.

    A background thread keeps only the NEWEST decoded frame. read() returns
    each frame once and waits up to `read_timeout` s for a new one, so a
    stopped simulation shows up as read failures instead of a frozen image
    being re-processed as fresh detections.
    """

    def __init__(self, address, connect_timeout=10.0, read_timeout=1.0):
        import struct
        import numpy as np
        self._struct, self._np = struct, np
        host, _, port = address.rpartition(":")
        self.host, self.port = host or "127.0.0.1", int(port)
        self.read_timeout = float(read_timeout)
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._read_seq = 0
        self._running = True
        self._sock = None
        self.frames_received = 0
        self.error = None
        try:
            self._sock = socket.create_connection((self.host, self.port), timeout=connect_timeout)
            self._sock.settimeout(5.0)
        except OSError as exc:
            self.error = exc
            self._running = False
            return
        self._thread = threading.Thread(target=self._loop, name="gazebo-camera", daemon=True)
        self._thread.start()
        # Wait for the first frame so get(WIDTH/HEIGHT) reports the real size.
        with self._cond:
            self._cond.wait_for(lambda: self._frame is not None or not self._running, timeout=connect_timeout)

    def _recv_exact(self, n):
        data = bytearray()
        while len(data) < n:
            chunk = self._sock.recv(n - len(data))
            if not chunk:
                raise ConnectionError("Gazebo camera relay closed the connection")
            data.extend(chunk)
        return bytes(data)

    def _loop(self):
        try:
            while self._running:
                size = self._struct.unpack("!I", self._recv_exact(4))[0]
                if size <= 0 or size > 10_000_000:
                    raise ValueError(f"invalid frame size {size}")
                payload = self._recv_exact(size)
                frame = cv2.imdecode(self._np.frombuffer(payload, dtype=self._np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue
                with self._cond:
                    self._frame = frame
                    self._seq += 1
                    self.frames_received += 1
                    self._cond.notify_all()
        except Exception as exc:                 # disconnect, timeout, bad data
            self.error = exc
        finally:
            with self._cond:
                self._running = False
                self._cond.notify_all()

    def isOpened(self):
        return self._running                     # False once the relay connection is gone

    def read(self):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != self._read_seq or not self._running,
                                timeout=self.read_timeout)
            if self._seq == self._read_seq:
                return False, None
            self._read_seq = self._seq
            return True, self._frame

    def get(self, prop):
        if self._frame is None:
            return 0.0
        if prop == cv2.CAP_PROP_FRAME_WIDTH:
            return float(self._frame.shape[1])
        if prop == cv2.CAP_PROP_FRAME_HEIGHT:
            return float(self._frame.shape[0])
        return 0.0

    def set(self, prop, value):
        return False                             # size is fixed by the Gazebo camera model

    def release(self):
        self._running = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass


class PiPerceptionPublisher:
    """
    Sends the latest perception packet from the Windows laptop to a Raspberry Pi
    over a TCP newline-delimited JSON stream.

    The queue has size 1 intentionally: if the Pi is slower than the
    camera, old packets are discarded and only the newest perception
    state is sent. This keeps navigation data fresh.
    """

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.queue = Queue(maxsize=PI_LINK_QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.thread = None
        self.sock = None
        self.connected = False
        self.sent_packets = 0
        self.dropped_packets = 0
        self.last_error = ""
        self._lock = threading.Lock()

    def start(self):
        if not self.host:
            print("[PI LINK] Disabled. Set NIDAR_PI_HOST to enable Raspberry Pi streaming.")
            return

        print(f"[PI LINK] Target: {self.host}:{self.port}")
        self.thread = threading.Thread(
            target=self._worker,
            name="nidar-pi-publisher",
            daemon=True,
        )
        self.thread.start()

    def submit(self, packet):
        if not self.host:
            return

        try:
            self.queue.put_nowait(packet)
        except Full:
            # Keep only the newest packet.
            try:
                self.queue.get_nowait()
            except Empty:
                pass
            try:
                self.queue.put_nowait(packet)
            except Full:
                self.dropped_packets += 1

    def _close_socket(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        with self._lock:
            self.connected = False

    def _connect(self):
        self._close_socket()
        try:
            sock = socket.create_connection(
                (self.host, self.port),
                timeout=PI_CONNECT_TIMEOUT,
            )
            sock.settimeout(PI_SEND_TIMEOUT)
            self.sock = sock
            with self._lock:
                self.connected = True
            print(f"[PI LINK] Connected to {self.host}:{self.port}")
            return True
        except Exception as exc:
            self.last_error = str(exc)
            return False

    def _worker(self):
        while not self.stop_event.is_set():
            try:
                packet = self.queue.get(timeout=0.2)
            except Empty:
                continue

            payload = (json.dumps(packet, separators=(",", ":")) + "\n").encode("utf-8")

            if self.sock is None and not self._connect():
                continue

            try:
                self.sock.sendall(payload)
                self.sent_packets += 1
            except Exception as exc:
                self.last_error = str(exc)
                self._close_socket()

        self._close_socket()

    def stop(self):
        if not self.host:
            return
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self._close_socket()

    def status(self):
        with self._lock:
            connected = self.connected
        return {
            "enabled": bool(self.host),
            "connected": connected,
            "host": self.host,
            "port": self.port,
            "sent_packets": self.sent_packets,
            "dropped_packets": self.dropped_packets,
            "last_error": self.last_error,
        }


# ============================================================
# CONFIGURATION
# ============================================================

MODEL_NAME = "yoloe-26n-seg.pt"
TRACKER = "botsort.yaml"

IMG_SIZE = 448
CONF = 0.22
IOU = 0.50

CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Stable-ID rules
CONFIRM_HITS = 3
CONFIRM_WINDOW = 4

# Temporal class stabilization. A tracker can briefly change its
# open-vocabulary label (e.g. toy <-> teddy bear). Keep the stable
# identity label until the new class persists.
LABEL_SWITCH_HITS = 6
LABEL_SWITCH_MARGIN = 0.10
LABEL_MATCH_CENTER = 0.16
TENTATIVE_MAX_MISSES = 2

# V10 adaptive prediction: the prediction window is selected when an
# object first becomes visually lost, based on motion and identity quality.
LOST_BUFFER_FRAMES = 45
MAX_PREDICTION_GAP = 15
PREDICTION_MAX_DX = 120.0
PREDICTION_MAX_DY = 90.0
VELOCITY_DECAY_WHEN_LOST = 0.88

PREDICTION_BUDGET_HIGH = 15
PREDICTION_BUDGET_MEDIUM = 10
PREDICTION_BUDGET_LOW = 6
PREDICTION_BUDGET_VERY_LOW = 3

# Conservative tracker-ID recovery.
MAX_RECOVERY_GAP = 50
RECOVERY_MIN_IOU = 0.08
RECOVERY_MAX_CENTER_DISTANCE = 0.24
RECOVERY_MIN_SCORE = 0.62
RECOVERY_MIN_SIZE_RATIO = 0.30

# Detection quality filters.
MIN_BOX_WIDTH = 14
MIN_BOX_HEIGHT = 14
MIN_BOX_AREA_RATIO = 0.00018
MAX_BOX_AREA_RATIO = 0.92

# Duplicate/alias merge.
DUPLICATE_IOU = 0.65
DUPLICATE_CENTER_DISTANCE = 0.08

# Draw only confirmed objects.
SHOW_TENTATIVE = False

# ============================================================
# V5 DETECTION-QUALITY / CLASS-VOTING SETTINGS
# ============================================================

# A class must repeatedly win before it can replace an established
# object's label. This is deliberately stricter than raw YOLOE output.
CLASS_VOTE_WINDOW = 12
CLASS_LOCK_MIN_HITS = 6
CLASS_LOCK_MIN_CONF = 0.45
CLASS_CHANGE_MARGIN = 0.18
CLASS_CHANGE_RATIO = 1.35

# Detection confidence is class-dependent. Small/ambiguous objects
# need slightly more evidence than large, obvious objects.
MIN_CONF_DEFAULT = 0.24
MIN_CONF_SMALL_OBJECT = 0.28

# Require a detection to be spatially consistent with the current
# stable object before adding it to that object's class vote history.
VOTE_MAX_CENTER_DISTANCE = 0.20
VOTE_MIN_IOU = 0.02

# Duplicate suppression after canonicalization.
V5_DUPLICATE_IOU = 0.55
V5_DUPLICATE_CENTER_DISTANCE = 0.07

# A stable object's label becomes locked after enough strong evidence.
# Locked labels can still change, but only with substantially stronger,
# persistent evidence.
LOCK_LABELS = True

# V5 identity protections.
# A tracker ID that has been used by one stable object is not allowed
# to bounce between stable objects. Tracker-ID switches must also be
# spatially consistent before being accepted.
MAX_TRACKER_SWITCH_DISTANCE = 0.18
MAX_SOFT_RECOVERY_DISTANCE = 0.20
SOFT_RECOVERY_SCORE = 0.72
REQUIRE_CLASS_MATCH_FOR_RECOVERY = True
MIN_OBJECT_AGE_FOR_RECOVERY = 3


# ============================================================
# OPEN-VOCABULARY PROMPTS
# ============================================================
#
# The model gets aliases to improve recall, but the application
# converts them to one canonical label before tracking.
#
# This avoids:
#   smartphone + cell phone = two conceptual objects
#   headset + earphones + headphones = three IDs
#   bottle + water bottle = two IDs
#
# Earbuds remain separate from headphones intentionally.
# ============================================================

PROMPT_TO_CANONICAL = {
    "person": "person",
    "child": "person",

    "dog": "dog",
    "cat": "cat",
    "bird": "bird",

    "headphones": "headphones",
    "over-ear headphones": "headphones",
    "headset": "headphones",
    "earphones": "headphones",
    "wireless headphones": "headphones",
    "bluetooth headphones": "headphones",
    "over ear headphones": "headphones",

    "earbuds": "earbuds",
    "wireless earbuds": "earbuds",

    "cell phone": "phone",
    "smartphone": "phone",
    "mobile phone": "phone",

    "tablet": "tablet",
    "laptop": "laptop",
    "computer": "computer",
    "keyboard": "keyboard",
    "computer mouse": "mouse",
    "mouse": "mouse",

    "monitor": "monitor",
    "television": "television",
    "camera": "camera",
    "remote control": "remote",
    "remote": "remote",

    "bottle": "bottle",
    "water bottle": "bottle",
    "cup": "cup",
    "mug": "cup",
    "glass": "glass",

    "backpack": "backpack",
    "handbag": "handbag",
    "suitcase": "suitcase",
    "wallet": "wallet",

    "chair": "chair",
    "table": "table",
    "desk": "desk",
    "bed": "bed",
    "sofa": "sofa",
    "couch": "sofa",

    "book": "book",
    "notebook": "book",
    "pen": "pen",
    "pencil": "pencil",
    "scissors": "scissors",

    "watch": "watch",
    "glasses": "glasses",
    "sunglasses": "glasses",
    "shoe": "shoe",
    "hat": "hat",
    "umbrella": "umbrella",

    "teddy bear": "teddy bear",
    "toy": "toy",
}

PROMPTS = list(PROMPT_TO_CANONICAL.keys())


# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass
class Detection:
    class_id: int
    prompt: str
    label: str
    confidence: float
    box: tuple
    tracker_id: Optional[int]


@dataclass
class StableObject:
    stable_id: int
    label: str
    box: tuple
    confidence: float
    tracker_id: int

    hits: int = 1
    consecutive_hits: int = 1
    missed: int = 0
    confirmed: bool = False

    # V7 object lifecycle state.
    state: str = "TENTATIVE"
    state_since_frame: int = 0
    max_missed: int = 0
    recovery_count: int = 0

    # V9 spatial prediction / quality state.
    last_observed_box: tuple = None
    motion_confidence: float = 0.0
    track_stability_score: float = 0.0
    prediction_horizon: int = 0
    prediction_budget: int = 0

    first_frame: int = 0
    last_seen_frame: int = 0

    recoveries: int = 0
    tracker_switches: int = 0
    total_visible: int = 1
    best_confidence: float = 0.0

    # Simple center velocity (pixels/frame), used to predict where the
    # object should be after a tracker-ID switch.
    vx: float = 0.0
    vy: float = 0.0

    # Temporal class stabilization state.
    candidate_label: Optional[str] = None
    candidate_hits: int = 0
    candidate_best_confidence: float = 0.0

    # V5 class evidence. We keep a short temporal vote history instead
    # of trusting the detector's label on a single frame.
    class_votes: dict = None
    class_conf_sum: dict = None
    class_last_frame: dict = None
    label_locked: bool = False

    # Tracker IDs recently associated with this object. This prevents
    # the same tracker ID from rapidly bouncing between objects.
    tracker_history: list = None
    last_tracker_change_frame: int = -999999

    def __post_init__(self):
        if self.tracker_history is None:
            self.tracker_history = []
        if self.tracker_id is not None and self.tracker_id not in self.tracker_history:
            self.tracker_history.append(self.tracker_id)
        if self.class_votes is None:
            self.class_votes = {}
        if self.class_conf_sum is None:
            self.class_conf_sum = {}
        if self.class_last_frame is None:
            self.class_last_frame = {}


# ============================================================
# GEOMETRY
# ============================================================

def center(box):
    x1, y1, x2, y2 = box
    return ((x1 + x2) * 0.5, (y1 + y2) * 0.5)


def box_area(box):
    x1, y1, x2, y2 = box
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def box_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih

    union = box_area(a) + box_area(b) - inter
    return inter / union if union > 0 else 0.0


def normalized_center_distance(a, b, width, height):
    ax, ay = center(a)
    bx, by = center(b)

    dx = (ax - bx) / max(1.0, width)
    dy = (ay - by) / max(1.0, height)

    return math.sqrt(dx * dx + dy * dy)


def size_similarity(a, b):
    aa = box_area(a)
    ab = box_area(b)

    if aa <= 0 or ab <= 0:
        return 0.0

    return min(aa, ab) / max(aa, ab)


def color_for_id(stable_id):
    return (
        (37 * stable_id + 80) % 220 + 20,
        (97 * stable_id + 60) % 220 + 20,
        (173 * stable_id + 30) % 220 + 20,
    )


# ============================================================
# DETECTION FILTERING
# ============================================================

def safe_tracker_id(value):
    """Return a valid integer tracker ID or None.

    Ultralytics can expose tracker IDs as tensors/NumPy values and,
    during tracker initialization/re-association, an individual entry
    can still be None. Never call int(None).
    """
    if value is None:
        return None

    try:
        # Unwrap scalar tensor/NumPy values.
        if hasattr(value, "item"):
            value = value.item()
    except Exception:
        pass

    if value is None:
        return None

    try:
        if isinstance(value, float) and not math.isfinite(value):
            return None
    except Exception:
        pass

    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def canonical_label(prompt):
    return PROMPT_TO_CANONICAL.get(prompt.strip().lower(), prompt.strip().lower())


def filter_detection(det, frame_width, frame_height):
    x1, y1, x2, y2 = det.box

    w = x2 - x1
    h = y2 - y1
    area_ratio = box_area(det.box) / max(1.0, frame_width * frame_height)

    if w < MIN_BOX_WIDTH or h < MIN_BOX_HEIGHT:
        return False

    if area_ratio < MIN_BOX_AREA_RATIO:
        return False

    if area_ratio > MAX_BOX_AREA_RATIO:
        return False

    # A tracker ID is mandatory for application-level identity.
    if det.tracker_id is None:
        return False

    return True


def merge_duplicate_detections(detections, width, height):
    """
    Merge duplicate boxes belonging to the same canonical class.

    The highest-confidence detection is retained.
    A second prompt/alias for the same physical object does not
    create a second stable identity.
    """

    ordered = sorted(
        detections,
        key=lambda d: d.confidence,
        reverse=True,
    )

    kept = []

    for det in ordered:
        duplicate = False

        for existing in kept:
            if existing.label != det.label:
                continue

            iou = box_iou(existing.box, det.box)
            dist = normalized_center_distance(
                existing.box,
                det.box,
                width,
                height,
            )

            if iou >= DUPLICATE_IOU:
                duplicate = True
                break

            # For people, center-distance alone is too aggressive: two
            # nearby people can legitimately be separate objects.
            if existing.label != "person" and dist <= DUPLICATE_CENTER_DISTANCE:
                duplicate = True
                break

        if not duplicate:
            kept.append(det)

    return kept


# ============================================================
# STABLE ID MANAGER
# ============================================================

class StableIdentityManager:

    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.next_id = 1
        self.objects = {}

        # V9: permanent lifecycle archive. Objects remain here after they
        # leave the active manager, so the final summary contains the
        # complete history of every stable ID created during the test.
        self.history = {}

        # V9 lifecycle event log. Each transition is retained for audit,
        # debugging, and later sensor-fusion integration.
        self.lifecycle_events = []

        # (canonical_label, tracker_id) -> stable ID
        self.track_to_stable = {}

        self.total_new = 0
        self.total_recoveries = 0
        self.total_switches = 0
        self.total_lost = 0

    def _event(self, obj, frame_no, event, old_state=None, new_state=None):
        """Record a compact lifecycle event for one stable object."""
        self.lifecycle_events.append({
            "frame": int(frame_no),
            "id": int(obj.stable_id),
            "class": str(obj.label),
            "event": str(event),
            "old_state": old_state,
            "new_state": new_state,
            "tracker_id": obj.tracker_id,
        })

    def _set_state(self, obj, new_state, frame_no, event=None):
        old_state = obj.state
        if old_state != new_state:
            obj.state = new_state
            obj.state_since_frame = frame_no
            self._event(
                obj,
                frame_no,
                event or f"{old_state}->{new_state}",
                old_state,
                new_state,
            )
        elif event:
            self._event(obj, frame_no, event, old_state, new_state)

    def _tracker_owned_by_other(self, tracker_id, sid):
        if tracker_id is None:
            return False
        for other_sid, other in self.objects.items():
            if other_sid != sid and other.tracker_id == tracker_id:
                return True
        return False

    def _accept_tracker_switch(self, obj, det, frame_no):
        if det.tracker_id is None:
            return False
        if obj.tracker_id == det.tracker_id:
            return True
        if self._tracker_owned_by_other(det.tracker_id, obj.stable_id):
            return False
        dist = normalized_center_distance(obj.box, det.box, self.width, self.height)
        if dist > MAX_TRACKER_SWITCH_DISTANCE:
            return False
        return True

    def _bind_tracker(self, obj, new_tid, frame_no):
        old_tid = obj.tracker_id
        if old_tid == new_tid:
            return old_tid
        for key, mapped_sid in list(self.track_to_stable.items()):
            if mapped_sid == obj.stable_id:
                self.track_to_stable.pop(key, None)
        obj.tracker_id = new_tid
        obj.last_tracker_change_frame = frame_no
        if new_tid is not None:
            if new_tid not in obj.tracker_history:
                obj.tracker_history.append(new_tid)
                if len(obj.tracker_history) > 12:
                    obj.tracker_history.pop(0)
            self.track_to_stable[(obj.label, new_tid)] = obj.stable_id
        return old_tid

    def _update_motion(self, obj, det, frame_no):
        old_cx, old_cy = center(obj.box)
        new_cx, new_cy = center(det.box)
        dt = max(1, frame_no - obj.last_seen_frame)
        inst_vx = (new_cx - old_cx) / dt
        inst_vy = (new_cy - old_cy) / dt
        obj.vx = 0.70 * obj.vx + 0.30 * inst_vx
        obj.vy = 0.70 * obj.vy + 0.30 * inst_vy

    def _apply_detection(self, obj, det, frame_no, recovered=False):
        old_tid = obj.tracker_id
        if old_tid != det.tracker_id:
            old_tid = self._bind_tracker(obj, det.tracker_id, frame_no)
            obj.tracker_switches += 1
            self.total_switches += 1
        self._update_motion(obj, det, frame_no)
        self._update_label_stability(obj, det.label, det.confidence, frame_no)
        obj.box = det.box
        obj.last_observed_box = det.box
        obj.confidence = det.confidence
        obj.best_confidence = max(obj.best_confidence, det.confidence)
        obj.last_seen_frame = frame_no
        obj.prediction_horizon = 0
        obj.prediction_budget = 0
        obj.missed = 0
        obj.hits += 1
        obj.consecutive_hits += 1
        obj.total_visible += 1
        if obj.consecutive_hits >= CONFIRM_HITS:
            if not obj.confirmed:
                print(f"[CONFIRMED] {obj.label} ID:{obj.stable_id}")
            obj.confirmed = True

        if recovered:
            obj.recoveries += 1
            obj.recovery_count += 1
            self._set_state(obj, "RECOVERED", frame_no, "RECOVERED")
        elif obj.confirmed:
            self._set_state(obj, "CONFIRMED", frame_no)
        else:
            self._set_state(obj, "TENTATIVE", frame_no)
        return old_tid

    def _create(self, det, frame_no):
        sid = self.next_id
        self.next_id += 1

        obj = StableObject(
            stable_id=sid,
            label=det.label,
            box=det.box,
            confidence=det.confidence,
            tracker_id=det.tracker_id,
            hits=1,
            consecutive_hits=1,
            missed=0,
            confirmed=False,
            state="TENTATIVE",
            state_since_frame=frame_no,
            max_missed=0,
            recovery_count=0,
            first_frame=frame_no,
            last_seen_frame=frame_no,
            best_confidence=det.confidence,
            last_observed_box=det.box,
            motion_confidence=0.0,
            track_stability_score=0.0,
            prediction_horizon=0,
            prediction_budget=0,
        )

        self.objects[sid] = obj
        self.history[sid] = copy.deepcopy(obj)
        self.track_to_stable[(det.label, det.tracker_id)] = sid

        self.total_new += 1
        self._event(obj, frame_no, "CREATED", None, "TENTATIVE")

        print(
            f"[TENTATIVE] {obj.label} "
            f"ID:{sid} T:{det.tracker_id}"
        )

        return obj

    def _direct_match(self, det):
        if det.tracker_id is None:
            return None
        sid = self.track_to_stable.get((det.label, det.tracker_id))
        if sid is not None:
            obj = self.objects.get(sid)
            if obj is not None:
                return obj
        # Same tracker ID with a temporary label change. Only accept if
        # the tracker ID is not currently owned by another stable object.
        owners = [o for o in self.objects.values() if o.tracker_id == det.tracker_id]
        if len(owners) > 1:
            return None
        if len(owners) == 1:
            obj = owners[0]
            dist = normalized_center_distance(obj.box, det.box, self.width, self.height)
            if dist <= LABEL_MATCH_CENTER:
                return obj
        return None

    def _prune_class_votes(self, obj, frame_no):
        cutoff = frame_no - CLASS_VOTE_WINDOW
        for label in list(obj.class_last_frame.keys()):
            if obj.class_last_frame[label] < cutoff:
                obj.class_last_frame.pop(label, None)
                obj.class_votes.pop(label, None)
                obj.class_conf_sum.pop(label, None)

    def _update_label_stability(self, obj, observed_label, confidence, frame_no):
        """
        V5 class voting.

        Raw open-vocabulary labels can fluctuate:
            bottle -> book -> phone -> teddy bear

        We therefore accumulate recent evidence and only change an
        established label when the challenger has enough repeated,
        sufficiently strong evidence.
        """
        self._prune_class_votes(obj, frame_no)

        obj.class_votes[observed_label] = obj.class_votes.get(observed_label, 0) + 1
        obj.class_conf_sum[observed_label] = (
            obj.class_conf_sum.get(observed_label, 0.0) + confidence
        )
        obj.class_last_frame[observed_label] = frame_no

        # Existing label gets a strong prior. This prevents one noisy
        # open-vocabulary classification from flipping the object.
        current = obj.label
        current_votes = obj.class_votes.get(current, 0)
        current_conf = obj.class_conf_sum.get(current, 0.0)

        if observed_label == current:
            current_votes += 1
            current_conf += confidence

        challenger_scores = {}
        for label, votes in obj.class_votes.items():
            conf_sum = obj.class_conf_sum.get(label, 0.0)
            avg_conf = conf_sum / max(1, votes)
            # Confidence-weighted evidence.
            challenger_scores[label] = votes * (0.65 + 0.35 * avg_conf)

        if current not in challenger_scores:
            challenger_scores[current] = current_votes * (
                0.65 + 0.35 * (current_conf / max(1, current_votes))
            )

        best_label = max(challenger_scores, key=challenger_scores.get)
        best_score = challenger_scores[best_label]
        current_score = challenger_scores.get(current, 0.0)

        if best_label == current:
            if (
                LOCK_LABELS
                and obj.confirmed
                and current_votes >= CLASS_LOCK_MIN_HITS
                and (current_conf / max(1, current_votes)) >= CLASS_LOCK_MIN_CONF
            ):
                obj.label_locked = True
            return

        challenger_votes = obj.class_votes.get(best_label, 0)
        challenger_avg_conf = (
            obj.class_conf_sum.get(best_label, 0.0)
            / max(1, challenger_votes)
        )

        enough_votes = challenger_votes >= CLASS_LOCK_MIN_HITS
        enough_conf = challenger_avg_conf >= CLASS_LOCK_MIN_CONF
        enough_margin = best_score >= max(
            current_score * CLASS_CHANGE_RATIO,
            current_score + CLASS_CHANGE_MARGIN,
        )

        # For an unlocked object, allow a little faster stabilization.
        if not obj.label_locked:
            enough_votes = challenger_votes >= 5

        if enough_votes and enough_conf and enough_margin:
            old_label = obj.label
            obj.label = best_label
            obj.label_locked = (
                LOCK_LABELS
                and obj.confirmed
                and challenger_votes >= CLASS_LOCK_MIN_HITS
            )

            # Clear competing evidence after a deliberate class change.
            obj.class_votes = {best_label: challenger_votes}
            obj.class_conf_sum = {
                best_label: obj.class_conf_sum.get(best_label, 0.0)
            }
            obj.class_last_frame = {
                best_label: frame_no
            }

            print(
                f"[V5 CLASS LOCK] ID:{obj.stable_id} "
                f"{old_label} -> {obj.label} "
                f"votes={challenger_votes} conf={challenger_avg_conf:.2f}"
            )

    def _predicted_box(self, obj, frame_no):
        gap = max(0, frame_no - obj.last_seen_frame)
        dx = max(-160.0, min(160.0, obj.vx * gap))
        dy = max(-120.0, min(120.0, obj.vy * gap))
        x1, y1, x2, y2 = obj.box
        return (x1 + dx, y1 + dy, x2 + dx, y2 + dy)

    def _soft_score(self, obj, det, frame_no):
        if obj.last_seen_frame <= 0:
            return -1.0
        gap = frame_no - obj.last_seen_frame
        if gap < 0 or gap > LOST_BUFFER_FRAMES:
            return -1.0
        predicted = self._predicted_box(obj, frame_no)
        iou = box_iou(predicted, det.box)
        dist = normalized_center_distance(predicted, det.box, self.width, self.height)
        size = size_similarity(predicted, det.box)
        center_score = max(0.0, 1.0 - dist / 0.34)
        iou_score = min(1.0, iou / 0.10)
        class_bonus = 0.12 if obj.label == det.label else -0.08
        score = (
            0.46 * center_score
            + 0.28 * iou_score
            + 0.14 * size
            + class_bonus
        )
        if dist > 0.34:
            return -1.0
        return score

    def _recovery_score(self, obj, det, frame_no):
        if det.tracker_id is None:
            return -1.0
        gap = frame_no - obj.last_seen_frame
        if gap <= 0 or gap > MAX_RECOVERY_GAP:
            return -1.0
        if obj.hits < MIN_OBJECT_AGE_FOR_RECOVERY:
            return -1.0
        if self._tracker_owned_by_other(det.tracker_id, obj.stable_id):
            return -1.0
        predicted = self._predicted_box(obj, frame_no)
        iou = box_iou(predicted, det.box)
        dist = normalized_center_distance(predicted, det.box, self.width, self.height)
        size = size_similarity(predicted, det.box)
        if dist > RECOVERY_MAX_CENTER_DISTANCE or size < RECOVERY_MIN_SIZE_RATIO:
            return -1.0
        if REQUIRE_CLASS_MATCH_FOR_RECOVERY and obj.label != det.label:
            return -1.0
        iou_score = min(1.0, iou / max(0.001, RECOVERY_MIN_IOU))
        center_score = max(0.0, 1.0 - dist / RECOVERY_MAX_CENTER_DISTANCE)
        score = 0.50 * iou_score + 0.35 * center_score + 0.15 * size
        if iou < RECOVERY_MIN_IOU and dist > 0.10:
            return -1.0
        return score

    def _recover(self, detections, assigned, frame_no):
        candidates = []
        for det_index, det in enumerate(detections):
            if det.tracker_id is None or det_index in assigned:
                continue
            for sid, obj in self.objects.items():
                if sid in assigned or obj.missed <= 0 or obj.missed > LOST_BUFFER_FRAMES:
                    continue
                score = self._recovery_score(obj, det, frame_no)
                if score >= RECOVERY_MIN_SCORE:
                    candidates.append((score, det_index, sid))
        candidates.sort(reverse=True)
        used_detections, used_objects = set(), set()
        for score, det_index, sid in candidates:
            if det_index in used_detections or sid in used_objects:
                continue
            det = detections[det_index]
            obj = self.objects.get(sid)
            if obj is None or not self._accept_tracker_switch(obj, det, frame_no):
                continue
            old_tid = self._apply_detection(obj, det, frame_no, recovered=True)
            self.total_recoveries += 1
            print(f"[RECOVERED] {obj.label} ID:{sid} T:{old_tid}->{det.tracker_id} score={score:.2f}")
            assigned.add(det_index)
            used_detections.add(det_index)
            used_objects.add(sid)

    def update(self, detections, frame_no):
        visible = []
        assigned_detection_indices = set()
        assigned_sids = set()

        # 1) Exact tracker-ID matches.
        for index, det in enumerate(detections):
            if det.tracker_id is None:
                continue
            obj = self._direct_match(det)
            if obj is None or obj.stable_id in assigned_sids:
                continue
            self._apply_detection(obj, det, frame_no)
            assigned_detection_indices.add(index)
            assigned_sids.add(obj.stable_id)
            visible.append(obj)

        # 2) Recover objects whose tracker ID changed. One-to-one and class-aware.
        self._recover(detections, assigned_detection_indices, frame_no)

        for sid, obj in self.objects.items():
            if obj.last_seen_frame == frame_no and sid not in assigned_sids:
                visible.append(obj)
                assigned_sids.add(sid)

        # 3) Conservative soft recovery for fresh tracker IDs.
        for index, det in enumerate(detections):
            if index in assigned_detection_indices or det.tracker_id is None:
                continue
            best = None
            for sid, obj in self.objects.items():
                if sid in assigned_sids or obj.missed <= 0 or obj.missed > 20:
                    continue
                if obj.label != det.label:
                    continue
                if self._tracker_owned_by_other(det.tracker_id, sid):
                    continue
                predicted = self._predicted_box(obj, frame_no)
                dist = normalized_center_distance(predicted, det.box, self.width, self.height)
                if dist > MAX_SOFT_RECOVERY_DISTANCE:
                    continue
                iou = box_iou(predicted, det.box)
                size = size_similarity(predicted, det.box)
                score = 0.55 * max(0.0, 1.0 - dist / MAX_SOFT_RECOVERY_DISTANCE) + 0.30 * min(1.0, iou / 0.10) + 0.15 * size
                if score >= SOFT_RECOVERY_SCORE and (best is None or score > best[0]):
                    best = (score, sid)
            if best is not None:
                score, sid = best
                obj = self.objects[sid]
                if not self._accept_tracker_switch(obj, det, frame_no):
                    continue
                old_tid = self._apply_detection(obj, det, frame_no, recovered=True)
                self.total_recoveries += 1
                print(f"[SOFT RECOVERED] {obj.label} ID:{sid} T:{old_tid}->{det.tracker_id} score={score:.2f}")
                assigned_detection_indices.add(index)
                assigned_sids.add(sid)
                visible.append(obj)

        # 4) Create new IDs only from detections that survived all filters.
        for index, det in enumerate(detections):
            if index in assigned_detection_indices or det.tracker_id is None:
                continue
            existing = self._direct_match(det)
            if existing is not None and existing.stable_id not in assigned_sids:
                self._apply_detection(existing, det, frame_no)
                assigned_sids.add(existing.stable_id)
                visible.append(existing)
                assigned_detection_indices.add(index)
                continue
            obj = self._create(det, frame_no)
            visible.append(obj)
            assigned_sids.add(obj.stable_id)
            assigned_detection_indices.add(index)

        # 5) Age objects not observed in this frame.
        for sid, obj in list(self.objects.items()):
            if obj.last_seen_frame == frame_no:
                continue
            obj.missed += 1
            obj.max_missed = max(obj.max_missed, obj.missed)
            obj.consecutive_hits = 0
            if obj.confirmed:
                obj.motion_confidence = motion_confidence_for(obj, frame_no)

                # Freeze the prediction budget at the first missed frame.
                # It does not grow while the object remains unseen.
                if obj.missed == 1:
                    obj.track_stability_score = track_stability_score(obj, frame_no)
                    obj.prediction_budget = prediction_budget_for(obj)
                    obj.prediction_horizon = 1
                    self._set_state(obj, "TEMP_LOST", frame_no, "TEMP_LOST")
                    print(
                        f"[TEMP LOST] {obj.label} ID:{sid} "
                        f"budget:{obj.prediction_budget} "
                        f"motion:{obj.motion_confidence:.2f}"
                    )
                else:
                    obj.prediction_horizon = min(
                        obj.missed,
                        max(1, obj.prediction_budget or MAX_PREDICTION_GAP),
                    )

                # Once the adaptive prediction budget is exceeded, the
                # visual position is stale and the object is declared LOST.
                if obj.missed > max(1, obj.prediction_budget):
                    self._set_state(obj, "LOST", frame_no, "LOST")
                    print(
                        f"[LOST] {obj.label} ID:{sid} "
                        f"missed:{obj.missed} budget:{obj.prediction_budget}"
                    )
                    self._remove(sid)
            elif obj.missed > TENTATIVE_MAX_MISSES:
                self._set_state(obj, "LOST", frame_no, "TENTATIVE_LOST")
                print(f"[TENTATIVE REMOVED] {obj.label} ID:{sid}")
                self._remove(sid)

        # V9: keep the latest snapshot of every active object in the
        # permanent archive as well. Removed objects were already frozen
        # by _remove().
        for sid, obj in self.objects.items():
            self.history[sid] = copy.deepcopy(obj)

        valid_sids = set(self.objects.keys())
        self.track_to_stable = {k: sid for k, sid in self.track_to_stable.items() if sid in valid_sids}
        return visible

    def _remove(self, sid):
        obj = self.objects.pop(sid, None)
        if obj is not None:
            # Keep a frozen copy for the complete V9 lifecycle report.
            self.history[sid] = copy.deepcopy(obj)
            self.total_lost += 1

        for key, mapped_sid in list(
            self.track_to_stable.items()
        ):
            if mapped_sid == sid:
                self.track_to_stable.pop(key, None)


# ============================================================
# DRAWING
# ============================================================


def clamp_box(box, width, height):
    x1, y1, x2, y2 = box
    x1 = max(0.0, min(float(width - 1), x1))
    y1 = max(0.0, min(float(height - 1), y1))
    x2 = max(0.0, min(float(width - 1), x2))
    y2 = max(0.0, min(float(height - 1), y2))

    if x2 <= x1:
        x2 = min(float(width - 1), x1 + 1.0)
    if y2 <= y1:
        y2 = min(float(height - 1), y1 + 1.0)

    return (x1, y1, x2, y2)


def prediction_budget_for(obj):
    """Choose a conservative prediction budget from motion and identity quality."""
    mc = float(getattr(obj, "motion_confidence", 0.0))
    stability = float(getattr(obj, "track_stability_score", 0.0))

    if mc >= 0.85 and stability >= 75.0:
        return PREDICTION_BUDGET_HIGH
    if mc >= 0.65 and stability >= 60.0:
        return PREDICTION_BUDGET_MEDIUM
    if mc >= 0.45 and stability >= 45.0:
        return PREDICTION_BUDGET_LOW
    return PREDICTION_BUDGET_VERY_LOW


def prediction_reliability_for(obj, frame_no):
    """Return a sensor-fusion-friendly prediction reliability label."""
    gap = max(0, frame_no - obj.last_seen_frame)
    budget = int(getattr(obj, "prediction_budget", 0))

    if gap <= 0:
        return "MEASURED"
    if budget <= 0 or gap > budget:
        return "STALE"
    if gap <= max(1, int(0.33 * budget)):
        return "HIGH"
    if gap <= max(1, int(0.66 * budget)):
        return "MEDIUM"
    return "LOW"


def predicted_box_for(obj, frame_no, width, height):
    """Return the V9 predicted box for a temporarily lost object."""
    gap = max(0, frame_no - obj.last_seen_frame)

    if gap <= 0:
        return obj.box

    budget = int(getattr(obj, "prediction_budget", 0))
    gap = min(gap, budget if budget > 0 else MAX_PREDICTION_GAP)
    decay = VELOCITY_DECAY_WHEN_LOST ** max(0, gap - 1)

    # Sum the decaying velocity instead of multiplying the final velocity
    # by the whole gap. This prevents runaway prediction after occlusion.
    if abs(1.0 - VELOCITY_DECAY_WHEN_LOST) < 1e-6:
        travel_scale = float(gap)
    else:
        travel_scale = (
            (1.0 - (VELOCITY_DECAY_WHEN_LOST ** gap))
            / (1.0 - VELOCITY_DECAY_WHEN_LOST)
        )

    dx = max(
        -PREDICTION_MAX_DX,
        min(PREDICTION_MAX_DX, obj.vx * travel_scale),
    )
    dy = max(
        -PREDICTION_MAX_DY,
        min(PREDICTION_MAX_DY, obj.vy * travel_scale),
    )

    x1, y1, x2, y2 = obj.box
    return clamp_box(
        (x1 + dx, y1 + dy, x2 + dx, y2 + dy),
        width,
        height,
    )


def track_stability_score(obj, frame_no):
    """0-100 score describing identity continuity, not detector confidence."""
    age = max(1, frame_no - obj.first_frame + 1)
    visible_ratio = min(1.0, obj.total_visible / age)

    confidence_component = max(
        0.0,
        min(1.0, (obj.best_confidence - 0.20) / 0.70),
    )
    confirmation_component = 1.0 if obj.confirmed else 0.45
    recovery_penalty = max(0.0, 1.0 - 0.06 * obj.recoveries)
    switch_penalty = max(0.0, 1.0 - 0.05 * obj.tracker_switches)
    miss_penalty = max(
        0.0,
        1.0 - min(1.0, obj.missed / max(1, LOST_BUFFER_FRAMES)),
    )

    score = (
        0.34 * visible_ratio
        + 0.22 * confidence_component
        + 0.18 * confirmation_component
        + 0.12 * recovery_penalty
        + 0.08 * switch_penalty
        + 0.06 * miss_penalty
    )

    return round(max(0.0, min(100.0, score * 100.0)), 1)


def motion_confidence_for(obj, frame_no):
    """Confidence in the short-term visual motion estimate."""
    age = max(1, frame_no - obj.first_frame + 1)
    visible_ratio = min(1.0, obj.total_visible / age)
    miss_factor = max(0.0, 1.0 - obj.missed / max(1, LOST_BUFFER_FRAMES))
    switch_factor = max(0.0, 1.0 - 0.10 * obj.tracker_switches)

    # Motion confidence is intentionally independent of raw detection
    # confidence. It describes whether velocity-based prediction is useful.
    value = 0.55 * visible_ratio + 0.30 * miss_factor + 0.15 * switch_factor
    return round(max(0.0, min(1.0, value)), 3)


def object_record(
    obj,
    frame_no,
    timestamp,
    width,
    height,
    state_override=None,
    box_override=None,
):
    state = state_override or obj.state
    observed = obj.last_seen_frame == frame_no

    if box_override is not None:
        active_box = box_override
    elif observed:
        active_box = obj.box
    else:
        active_box = predicted_box_for(obj, frame_no, width, height)

    x1, y1, x2, y2 = active_box
    cx, cy = center(active_box)

    predicted = predicted_box_for(obj, frame_no, width, height)
    px1, py1, px2, py2 = predicted
    pcx, pcy = center(predicted)

    age_frames = max(0, frame_no - obj.first_frame + 1)
    observation_age = max(0, frame_no - obj.last_seen_frame)

    stability = track_stability_score(obj, frame_no)
    motion_conf = motion_confidence_for(obj, frame_no)
    prediction_reliability = prediction_reliability_for(obj, frame_no)

    return {
        "frame": frame_no,
        "timestamp": timestamp,
        "id": obj.stable_id,
        "class": obj.label,
        "state": state,
        "confidence": round(float(obj.confidence), 4),
        "tracker_id": obj.tracker_id,

        "bbox": [round(float(x1), 2), round(float(y1), 2),
                 round(float(x2), 2), round(float(y2), 2)],
        "center": [round(float(cx), 2), round(float(cy), 2)],

        "predicted_bbox": [
            round(float(px1), 2), round(float(py1), 2),
            round(float(px2), 2), round(float(py2), 2),
        ],
        "predicted_center": [
            round(float(pcx), 2), round(float(pcy), 2)
        ],

        "normalized_center": [
            round((float(cx) / max(1.0, width)) * 2.0 - 1.0, 4),
            round((float(cy) / max(1.0, height)) * 2.0 - 1.0, 4),
        ],
        "bbox_size": [round(float(x2 - x1), 2),
                      round(float(y2 - y1), 2)],

        "velocity_px_frame": [round(float(obj.vx), 2),
                              round(float(obj.vy), 2)],

        "motion_confidence": motion_conf,
        "track_stability_score": stability,
        "observation_age_frames": observation_age,
        "prediction_horizon_frames": observation_age,
        "prediction_budget_frames": int(getattr(obj, "prediction_budget", 0)),
        "prediction_reliability": prediction_reliability,
        "spatial_status": "OBSERVED" if observed else "PREDICTED",

        "confirmed": bool(obj.confirmed),
        "age_frames": age_frames,
        "missed_frames": int(obj.missed),
        "max_missed_frames": int(obj.max_missed),
        "recoveries": int(obj.recoveries),
        "tracker_switches": int(obj.tracker_switches),
        "first_frame": int(obj.first_frame),
        "last_seen_frame": int(obj.last_seen_frame),
        "state_since_frame": int(obj.state_since_frame),
        "current_observation": bool(observed),
    }


def draw_object(frame, obj, box_override=None, state_override=None):

    draw_box = box_override if box_override is not None else obj.box
    x1, y1, x2, y2 = map(int, draw_box)

    color = color_for_id(obj.stable_id)

    thickness = 3 if obj.confirmed else 2
    state = state_override or obj.state

    if state == "TEMP_LOST":
        # Dashed box = predicted position, not a current camera observation.
        for xa in range(x1, x2, 16):
            xb = min(xa + 8, x2)
            cv2.line(frame, (xa, y1), (xb, y1), color, thickness)
            cv2.line(frame, (xa, y2), (xb, y2), color, thickness)
        for ya in range(y1, y2, 16):
            yb = min(ya + 8, y2)
            cv2.line(frame, (x1, ya), (x1, yb), color, thickness)
            cv2.line(frame, (x2, ya), (x2, yb), color, thickness)
    else:
        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            color,
            thickness,
        )

    text = (
        f"{obj.label.upper()} "
        f"ID:{obj.stable_id} "
        f"C:{obj.confidence:.2f} "
        f"{state}"
    )

    (tw, th), _ = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        2,
    )

    label_y = max(28, y1)

    cv2.rectangle(
        frame,
        (x1, label_y - th - 10),
        (x1 + tw + 10, label_y),
        color,
        -1,
    )

    cv2.putText(
        frame,
        text,
        (x1 + 5, label_y - 5),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 0, 0),
        2,
        cv2.LINE_AA,
    )

    cx, cy = center(obj.box)

    cv2.circle(
        frame,
        (int(cx), int(cy)),
        4,
        color,
        -1,
    )


def draw_header(
    frame,
    fps,
    detections_count,
    stable_count,
    ignored_trackerless,
    state_counts,
):
    overlay = frame.copy()

    cv2.rectangle(
        overlay,
        (0, 0),
        (470, 108),
        (15, 15, 15),
        -1,
    )

    cv2.addWeighted(
        overlay,
        0.78,
        frame,
        0.22,
        0,
        frame,
    )

    lines = [
        f"NIDAR V9   FPS:{fps:.1f}",
        f"Det:{detections_count}  Active:{stable_count}",
        f"CONF:{state_counts.get('CONFIRMED', 0)}  TEMP:{state_counts.get('TEMP_LOST', 0)}  REC:{state_counts.get('RECOVERED', 0)}",
        f"Trackerless:{ignored_trackerless}  PRED:{state_counts.get('TEMP_LOST', 0)}",
    ]

    y = 28

    for i, text in enumerate(lines):

        scale = 0.62 if i < 2 else 0.48

        cv2.putText(
            frame,
            text,
            (14, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (255, 255, 255),
            2 if i < 2 else 1,
            cv2.LINE_AA,
        )

        y += 25


# ============================================================
# CSV
# ============================================================

def save_csv(
    path,
    manager,
    duration,
    frame_count,
    total_detections,
    ignored_trackerless,
):
    fps = frame_count / max(0.001, duration)

    with open(
        path,
        "w",
        newline="",
        encoding="utf-8",
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "stable_id",
            "label",
            "final_state",
            "hits",
            "visible_frames",
            "confirmed",
            "best_confidence",
            "recoveries",
            "tracker_switches",
            "track_stability_score",
            "motion_confidence",
            "max_missed_frames",
            "first_frame",
            "last_seen_frame",
            "age_frames",
            "test_age_frames",
            "final_state_since_frame",
            "last_tracker_id",
        ])

        # V9: report EVERY stable ID, including objects that reached LOST.
        for sid in sorted(manager.history):

            obj = manager.history[sid]

            writer.writerow([
                sid,
                obj.label,
                obj.state,
                obj.hits,
                obj.total_visible,
                obj.confirmed,
                f"{obj.best_confidence:.4f}",
                obj.recoveries,
                obj.tracker_switches,
                track_stability_score(obj, frame_count),
                motion_confidence_for(obj, frame_count),
                obj.max_missed,
                obj.first_frame,
                obj.last_seen_frame,
                max(0, obj.last_seen_frame - obj.first_frame + 1),
                max(0, frame_count - obj.first_frame + 1),
                obj.state_since_frame,
                obj.tracker_id,
            ])

    return fps


# ============================================================
# V5 CLASS / DETECTION QUALITY
# ============================================================

def v5_confidence_floor(label):
    small_objects = {
        "headphones", "earbuds", "phone", "remote", "watch",
        "glasses", "mouse", "pen", "pencil", "scissors"
    }
    return MIN_CONF_SMALL_OBJECT if label in small_objects else MIN_CONF_DEFAULT


def v5_merge_cross_alias_detections(detections, width, height):
    """
    YOLOE can return multiple prompts for one physical object.
    After canonicalization, merge only genuinely overlapping boxes.

    Unlike V4.5, we do not merge nearby people and we require stronger
    overlap/center evidence for small objects.
    """
    ordered = sorted(detections, key=lambda d: d.confidence, reverse=True)
    kept = []

    for det in ordered:
        if det.confidence < v5_confidence_floor(det.label):
            continue

        duplicate = False
        for existing in kept:
            if existing.label != det.label:
                continue

            iou = box_iou(existing.box, det.box)
            dist = normalized_center_distance(
                existing.box, det.box, width, height
            )

            if iou >= V5_DUPLICATE_IOU:
                duplicate = True
                break

            if existing.label != "person" and dist <= V5_DUPLICATE_CENTER_DISTANCE:
                duplicate = True
                break

        if not duplicate:
            kept.append(det)

    return kept



# ============================================================
# MAIN
# ============================================================

def main():

    print("=" * 82)
    print("NIDAR V10 - PERCEPTION + ADAPTIVE SPATIAL PREDICTION + SENSOR FUSION READY")
    print("=" * 82)

    print(f"Model                  : {MODEL_NAME}")
    print(f"Tracker                : {TRACKER}")
    print(f"YOLO size              : {IMG_SIZE}")
    print(f"Detector confidence   : {CONF}")
    print(f"Class switch hits     : {LABEL_SWITCH_HITS}")
    print(f"Class switch margin   : {LABEL_SWITCH_MARGIN}")
    print(f"Confirmation hits     : {CONFIRM_HITS}")
    print(f"Prediction budgets    : {PREDICTION_BUDGET_VERY_LOW}/{PREDICTION_BUDGET_LOW}/{PREDICTION_BUDGET_MEDIUM}/{PREDICTION_BUDGET_HIGH} frames")
    print(f"Recovery minimum      : {RECOVERY_MIN_SCORE}")
    print(f"Class vote window     : {CLASS_VOTE_WINDOW}")
    print(f"Class lock hits       : {CLASS_LOCK_MIN_HITS}")
    print(f"Vocabulary prompts    : {len(PROMPTS)}")
    print(f"Raspberry Pi link     : {PI_HOST}:{PI_PORT}" if PI_HOST else "Raspberry Pi link     : DISABLED (set NIDAR_PI_HOST)")
    print()

    print("CANONICAL OBJECTS")
    print("-" * 82)

    canonical = sorted(
        set(PROMPT_TO_CANONICAL.values())
    )

    print(", ".join(canonical))
    print()

    print("IMPORTANT V10 RULES")
    print("-" * 82)
    print("1. Tracker ID = None -> ignored for stable identity")
    print("2. New ID requires a real tracker ID")
    print("3. Recovery requires a real new tracker ID")
    print("4. Recovery is one-to-one")
    print("5. Aliases are merged into canonical labels")
    print("6. Tentative objects need consecutive hits")
    print("7. Labels are stabilized across consecutive frames")
    print("8. Tracker IDs cannot bounce between stable objects")
    print("9. Recovery is class-aware and one-to-one")
    print("10. Soft recovery requires stronger spatial evidence")
    print("11. Confirmed objects are exported frame-by-frame for LiDAR/SLAM integration")
    print("12. Object lifecycle state is exported explicitly")
    print("13. Temporary loss is preserved for downstream sensor fusion")
    print("14. Every stable ID is retained in a permanent lifecycle archive")
    print("15. LOST objects remain available in the final summary")
    print("16. Lifecycle state transitions are exported as an event log")
    print("17. Temporary-loss objects use adaptive velocity-based spatial prediction")
    print("18. Predicted boxes are visually shown with dashed outlines")
    print("19. Motion confidence and track stability score are exported")
    print("20. Observation age and prediction horizon are exported")
    print("21. Prediction budget is frozen at first visual loss")
    print("22. Prediction reliability is exported for future sensor fusion")
    print("23. Active and historical object records can be joined by stable ID")
    print()

    print("MULTI-OBJECT + TEMPORARY-OCCLUSION TEST")
    print("-" * 82)
    print("Keep one person/object visible and move it left/right.")
    print("Partially hide it for 1-3 seconds, then reveal it.")
    print("Watch the dashed TEMP_LOST box follow the filtered motion.")
    print("Observe the printed prediction budget and reliability.")
    print("Do not judge the dashed box as a current detection.")
    print()

    print("Q = Quit")
    print("=" * 82)

    # --------------------------------------------------------
    # LOAD MODEL
    # --------------------------------------------------------

    print("\nLoading YOLOE...")

    try:
        model = YOLOE(MODEL_NAME)
    except Exception as e:
        print("\n[ERROR] Could not load YOLOE.")
        print("Run:")
        print("  pip install -U ultralytics")
        print()
        print("Details:", e)
        return

    print("YOLOE loaded successfully.")

    # Open-vocabulary prompt setup.
    model.set_classes(PROMPTS)

    print("Text vocabulary loaded.")
    print(f"Prompts configured: {len(PROMPTS)}")
    print("Tracker ID parser     : SAFE (None/NaN handled)")

    # --------------------------------------------------------
    # CAMERA
    # --------------------------------------------------------

    if CAMERA_SOURCE == "gazebo":
        print(f"Camera source           : GAZEBO relay {GAZEBO_CAMERA_ADDR}")
        cap = GazeboCameraCapture(GAZEBO_CAMERA_ADDR)
        camera_label = "gazebo_sim"
        if not cap.isOpened() or cap.get(cv2.CAP_PROP_FRAME_WIDTH) <= 0:
            print(
                f"[ERROR] No frames from the Gazebo camera relay at {GAZEBO_CAMERA_ADDR} "
                f"({cap.error}). Start gazebo/ros2/nidar_camera_relay.py in WSL first."
            )
            cap.release()
            return
    else:
        # Windows webcam: prefer DirectShow, then fall back to OpenCV default backend.
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(CAMERA_INDEX)
        camera_label = "windows_laptop_webcam"

    if not cap.isOpened():
        print(
            "[ERROR] Camera could not be opened. "
            "Check Windows camera permission and verify that another application is not using the webcam."
        )
        return

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        CAMERA_WIDTH,
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        CAMERA_HEIGHT,
    )

    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1,
    )

    actual_width = int(
        cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    )

    actual_height = int(
        cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    print(
        f"Camera resolution       : "
        f"{actual_width} x {actual_height}"
    )

    manager = StableIdentityManager(
        actual_width,
        actual_height,
    )

    # --------------------------------------------------------
    # RASPBERRY PI LINK
    # --------------------------------------------------------
    pi_publisher = PiPerceptionPublisher(PI_HOST, PI_PORT)
    pi_publisher.start()

    # --------------------------------------------------------
    # RUNTIME
    # --------------------------------------------------------

    frame_no = 0
    total_detections = 0
    ignored_trackerless = 0

    start_time = time.time()

    fps_display = 0.0
    fps_counter = 0
    fps_start = time.time()

    base_dir = Path(__file__).resolve().parent
    csv_path = base_dir / "multi_object_tracking_v10_summary.csv"
    frame_csv_path = base_dir / "multi_object_tracking_v10_frames.csv"
    jsonl_path = base_dir / "multi_object_tracking_v10.jsonl"
    events_csv_path = base_dir / "multi_object_tracking_v10_lifecycle_events.csv"

    frame_csv = open(frame_csv_path, "w", newline="", encoding="utf-8")
    frame_writer = csv.writer(frame_csv)
    frame_writer.writerow([
        "frame", "timestamp", "stable_id", "class", "state", "confidence",
        "tracker_id", "x1", "y1", "x2", "y2", "center_x", "center_y",
        "norm_x", "norm_y", "box_width", "box_height", "vx_px_frame",
        "vy_px_frame", "motion_confidence", "track_stability_score",
        "observation_age_frames", "prediction_horizon_frames", "prediction_budget_frames", "prediction_reliability",
        "spatial_status", "pred_x1", "pred_y1", "pred_x2", "pred_y2",
        "pred_center_x", "pred_center_y",
        "confirmed", "age_frames", "missed_frames",
        "max_missed_frames", "recoveries", "tracker_switches",
        "first_frame", "last_seen_frame", "state_since_frame",
        "current_observation"
    ])
    jsonl_file = open(jsonl_path, "w", encoding="utf-8")

    try:

        while True:

            ret, frame = cap.read()

            if not ret:
                print("[WARNING] Camera frame read failed.")
                if CAMERA_SOURCE == "gazebo" and not cap.isOpened():
                    print(f"[ERROR] Gazebo camera relay disconnected: {cap.error}")
                    break
                continue

            frame_no += 1

            # ------------------------------------------------
            # YOLOE + BoT-SORT
            # ------------------------------------------------

            try:
                results = model.track(
                    frame,
                    persist=True,
                    tracker=TRACKER,
                    imgsz=IMG_SIZE,
                    conf=CONF,
                    iou=IOU,
                    verbose=False,
                )

            except Exception as e:

                print(
                    f"[ERROR] Inference failed: {e}"
                )

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), ord("Q")):
                    break

                continue

            detections = []

            if results:

                result = results[0]
                boxes = result.boxes

                if (
                    boxes is not None
                    and len(boxes) > 0
                ):

                    xyxy = (
                        boxes.xyxy
                        .cpu()
                        .numpy()
                    )

                    confs = (
                        boxes.conf
                        .cpu()
                        .numpy()
                    )

                    class_ids = (
                        boxes.cls
                        .cpu()
                        .numpy()
                        .astype(int)
                    )

                    if boxes.id is not None:

                        ids = (
                            boxes.id
                            .cpu()
                            .numpy()
                            .astype(int)
                        )

                    else:
                        ids = [None] * len(xyxy)

                    names = result.names

                    for i in range(len(xyxy)):

                        confidence = float(
                            confs[i]
                        )

                        if confidence < CONF:
                            continue

                        cid = int(
                            class_ids[i]
                        )

                        prompt = str(
                            names[cid]
                        ).strip().lower()

                        label = canonical_label(
                            prompt
                        )

                        tracker_id = None

                        if (
                            ids is not None
                            and i < len(ids)
                        ):
                            # IMPORTANT: boxes.id may contain None for an
                            # individual detection. Never do int(None).
                            tracker_id = safe_tracker_id(ids[i])

                        x1, y1, x2, y2 = (
                            map(float, xyxy[i])
                        )

                        det = Detection(
                            class_id=cid,
                            prompt=prompt,
                            label=label,
                            confidence=confidence,
                            box=(x1, y1, x2, y2),
                            tracker_id=tracker_id,
                        )

                        # Keep trackerless detections out of
                        # the stable identity pipeline.
                        if tracker_id is None:
                            ignored_trackerless += 1
                            continue

                        if not filter_detection(
                            det,
                            actual_width,
                            actual_height,
                        ):
                            continue

                        detections.append(det)

            # ------------------------------------------------
            # MERGE SAME-OBJECT ALIASES / DUPLICATES
            # ------------------------------------------------

            detections = merge_duplicate_detections(
                detections,
                actual_width,
                actual_height,
            )

            total_detections += len(detections)

            # ------------------------------------------------
            # STABLE TRACKING
            # ------------------------------------------------

            visible = manager.update(
                detections,
                frame_no,
            )

            # ------------------------------------------------
            # V7 PERCEPTION + OBJECT STATE OUTPUT
            # ------------------------------------------------
            timestamp = time.time()
            frame_objects = []

            # Confirmed objects remain in the output during short
            # occlusions, with state=TEMP_LOST.
            for sid, obj in list(manager.objects.items()):
                if not obj.confirmed:
                    continue

                export_state = obj.state if obj.last_seen_frame == frame_no else "TEMP_LOST"

                record = object_record(
                    obj, frame_no, timestamp, actual_width, actual_height,
                    state_override=export_state
                )
                frame_objects.append(record)

                x1, y1, x2, y2 = record["bbox"]
                cx, cy = record["center"]
                nx, ny = record["normalized_center"]
                bw, bh = record["bbox_size"]
                vx, vy = record["velocity_px_frame"]
                pbx1, pby1, pbx2, pby2 = record["predicted_bbox"]
                pcx, pcy = record["predicted_center"]

                frame_writer.writerow([
                    record["frame"], record["timestamp"], record["id"],
                    record["class"], record["state"], record["confidence"],
                    record["tracker_id"], x1, y1, x2, y2, cx, cy, nx, ny,
                    bw, bh, vx, vy,
                    record["motion_confidence"], record["track_stability_score"],
                    record["observation_age_frames"], record["prediction_horizon_frames"],
                    record["prediction_budget_frames"], record["prediction_reliability"],
                    record["spatial_status"],
                    pbx1, pby1, pbx2, pby2, pcx, pcy,
                    record["confirmed"], record["age_frames"], record["missed_frames"],
                    record["max_missed_frames"], record["recoveries"],
                    record["tracker_switches"], record["first_frame"],
                    record["last_seen_frame"], record["state_since_frame"],
                    record["current_observation"]
                ])

            perception_packet = {
                "protocol": "NIDAR_PERCEPTION_V1",
                "frame": frame_no,
                "timestamp": timestamp,
                "camera": {
                    "width": actual_width,
                    "height": actual_height,
                    "source": camera_label,
                },
                "objects": frame_objects,
            }

            jsonl_file.write(json.dumps(perception_packet) + "\n")

            # Non-blocking latest-state handoff to the Raspberry Pi.
            pi_publisher.submit(perception_packet)

            if frame_no % 30 == 0:
                frame_csv.flush()
                jsonl_file.flush()

                if PI_HOST:
                    link = pi_publisher.status()
                    state = "CONNECTED" if link["connected"] else "DISCONNECTED"
                    print(
                        f"[PI LINK] {state} | sent={link['sent_packets']} "
                        f"dropped={link['dropped_packets']}"
                    )

            # ------------------------------------------------
            # DRAW CONFIRMED OBJECTS
            # ------------------------------------------------

            # V9: draw current observations and short-term predicted
            # positions for confirmed objects during temporary visual loss.
            for obj in list(manager.objects.values()):

                if (
                    not obj.confirmed
                    and not SHOW_TENTATIVE
                ):
                    continue

                if obj.last_seen_frame == frame_no:
                    draw_object(frame, obj)
                elif obj.confirmed and obj.state == "TEMP_LOST":
                    predicted = predicted_box_for(
                        obj, frame_no, actual_width, actual_height
                    )
                    draw_object(
                        frame,
                        obj,
                        box_override=predicted,
                        state_override="TEMP_LOST",
                    )

            # ------------------------------------------------
            # FPS
            # ------------------------------------------------

            fps_counter += 1

            elapsed_fps = (
                time.time() - fps_start
            )

            if elapsed_fps >= 1.0:

                fps_display = (
                    fps_counter / elapsed_fps
                )

                fps_counter = 0
                fps_start = time.time()

            confirmed_count = sum(
                1
                for obj in manager.objects.values()
                if obj.confirmed
            )

            state_counts = {
                "TENTATIVE": 0,
                "CONFIRMED": 0,
                "TEMP_LOST": 0,
                "RECOVERED": 0,
            }
            for obj in manager.objects.values():
                if obj.confirmed:
                    state_counts[obj.state] = state_counts.get(obj.state, 0) + 1

            draw_header(
                frame,
                fps_display,
                len(detections),
                confirmed_count,
                ignored_trackerless,
                state_counts,
            )

            cv2.imshow(
                "NIDAR V10 - Adaptive Perception + Spatial Prediction + Sensor Fusion Ready",
                frame,
            )

            key = cv2.waitKey(1) & 0xFF

            if key in (
                ord("q"),
                ord("Q"),
            ):
                break

    finally:

        pi_publisher.stop()
        cap.release()
        frame_csv.close()
        jsonl_file.close()
        cv2.destroyAllWindows()

    # --------------------------------------------------------
    # FINAL REPORT
    # --------------------------------------------------------

    duration = time.time() - start_time

    # --------------------------------------------------------
    # V9 LIFECYCLE EVENT LOG
    # --------------------------------------------------------
    with open(events_csv_path, "w", newline="", encoding="utf-8") as event_file:
        event_writer = csv.writer(event_file)
        event_writer.writerow([
            "frame", "stable_id", "class", "event",
            "old_state", "new_state", "tracker_id"
        ])
        for event in manager.lifecycle_events:
            event_writer.writerow([
                event["frame"],
                event["id"],
                event["class"],
                event["event"],
                event["old_state"],
                event["new_state"],
                event["tracker_id"],
            ])

    avg_fps = save_csv(
        csv_path,
        manager,
        duration,
        frame_no,
        total_detections,
        ignored_trackerless,
    )

    confirmed_count = sum(
        1
        for obj in manager.objects.values()
        if obj.confirmed
    )
    archived_lost_count = sum(
        1
        for obj in manager.history.values()
        if obj.state == "LOST"
    )

    print()
    print("=" * 82)
    print("NIDAR V10 - FINAL REPORT")
    print("=" * 82)

    print(
        f"Test duration           : "
        f"{duration:.1f}s"
    )

    print(
        f"Camera frames           : "
        f"{frame_no}"
    )

    print(
        f"Average FPS             : "
        f"{avg_fps:.2f}"
    )

    print(
        f"Valid detections        : "
        f"{total_detections}"
    )

    print(
        f"Trackerless ignored     : "
        f"{ignored_trackerless}"
    )

    print(
        f"Stable IDs created      : "
        f"{manager.total_new}"
    )

    print(
        f"Complete lifecycle IDs  : "
        f"{len(manager.history)}"
    )

    print(
        f"Objects reached LOST    : "
        f"{archived_lost_count}"
    )

    print(
        f"Currently confirmed     : "
        f"{confirmed_count}"
    )

    print(
        f"Identity recoveries     : "
        f"{manager.total_recoveries}"
    )

    print(
        f"Tracker ID switches     : "
        f"{manager.total_switches}"
    )

    pi_status = pi_publisher.status()
    print(
        f"Pi packets sent         : "
        f"{pi_status['sent_packets']}"
    )
    print(
        f"Pi packets dropped      : "
        f"{pi_status['dropped_packets']}"
    )

    print(
        f"Summary CSV             : "
        f"{csv_path}"
    )

    print(
        f"Frame CSV               : "
        f"{frame_csv_path}"
    )

    print(
        f"JSONL                   : "
        f"{jsonl_path}"
    )

    print(
        f"Lifecycle events CSV    : "
        f"{events_csv_path}"
    )

    print()
    print("Canonical vocabulary:")
    for label in canonical:
        print(f"  - {label}")

    print("=" * 82)


if __name__ == "__main__":
    main()
