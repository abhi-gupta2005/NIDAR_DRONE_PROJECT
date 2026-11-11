import json
import time
import numpy as np

from nidar_lidar_simulator import (
    make_scan,
    SCAN_HZ,
    MEASUREMENTS_PER_SECOND,
)

from lidar_interface import LidarScan
from lidar_obstacle_detector import LidarObstacleDetector
from lidar_obstacle_tracker_v3 import LidarObstacleTrackerV3

from nidar_camera_lidar_fusion_v1 import (
    CameraLidarFusion,
    LidarTrackInput,
    camera_detection_from_nidar_record,
)


PERCEPTION_FILE = "/app/nidar_latest_perception.json"


def load_camera_data():
    try:
        with open(PERCEPTION_FILE, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def build_lidar_scan(scan_id):
    timestamp = time.time()

    angles, distances, hit_types, _ = make_scan(
        (scan_id - 1) / SCAN_HZ
    )

    valid = np.isfinite(distances)

    return LidarScan(
        scan_id=scan_id,
        timestamp=timestamp,
        angle_deg=angles,
        distance_m=distances,
        valid=valid,
        hit_types=hit_types,
        scan_rate_hz=SCAN_HZ,
        measurements_per_second=MEASUREMENTS_PER_SECOND,
    )


def convert_camera_objects(data):
    if data is None:
        return []

    camera = data.get("camera", {})
    objects = data.get("objects", [])

    width = int(camera.get("width", 1280))
    height = int(camera.get("height", 720))

    frame = int(data.get("frame", 0))
    timestamp = float(data.get("timestamp", time.time()))

    detections = []

    for record in objects:
        detection = camera_detection_from_nidar_record(
            record=record,
            image_width=width,
            image_height=height,
            frame=frame,
            timestamp=timestamp,
        )

        if detection is not None:
            detections.append(detection)

    return detections


def convert_lidar_tracks(tracks):
    lidar_inputs = []

    for track in tracks:
        lidar_inputs.append(
            LidarTrackInput(
                track_id=int(track.track_id),
                x_m=float(track.x_m),
                y_m=float(track.y_m),
                distance_m=float(track.distance_m),
                bearing_deg=float(track.bearing_deg),
                speed_mps=float(track.speed_mps),
                state=str(track.state),
                observation_state=str(track.observation_state),
                motion_state=str(track.motion_state),
                confidence=float(track.confidence),
                track_quality=float(track.track_quality),
                missed_scans=int(track.missed_scans),
            )
        )

    return lidar_inputs


def print_fused_objects(fused):
    for obj in fused:
        print(
            "  [FUSED] "
            f"id={obj.fusion_id} "
            f"class={obj.class_name} "
            f"distance={obj.distance_m:.2f}m "
            f"camera_bearing={obj.bearing_camera_deg:.1f}deg "
            f"lidar_bearing={obj.bearing_lidar_deg:.1f}deg "
            f"bearing_error={obj.bearing_error_deg:.1f}deg "
            f"speed={obj.speed_mps:.2f}m/s "
            f"confidence={obj.fusion_confidence:.2f}"
        )


def main():

    print("=" * 70)
    print("NIDAR INTEGRATED PIPELINE")
    print("=" * 70)
    print("Camera source : Pi receiver JSON")
    print("LiDAR source  : Simulator")
    print("Fusion        : Camera + LiDAR")
    print("")

    detector = LidarObstacleDetector()
    tracker = LidarObstacleTrackerV3()
    fusion = CameraLidarFusion()

    scan_id = 0
    last_camera_frame = None

    try:
        while True:

            scan_id += 1

            # --------------------------------------------------
            # 1. Camera
            # --------------------------------------------------

            camera_data = load_camera_data()

            camera_detections = convert_camera_objects(
                camera_data
            )

            camera_frame = (
                int(camera_data.get("frame", 0))
                if camera_data
                else 0
            )

            # --------------------------------------------------
            # 2. LiDAR scan
            # --------------------------------------------------

            scan = build_lidar_scan(scan_id)

            # --------------------------------------------------
            # 3. Obstacle detection
            # --------------------------------------------------

            obstacles = detector.detect(scan)

            # --------------------------------------------------
            # 4. LiDAR tracking
            # --------------------------------------------------

            tracks = tracker.update(
                obstacles,
                time.time(),
            )

            # --------------------------------------------------
            # 5. Convert LiDAR tracks
            # --------------------------------------------------

            lidar_inputs = convert_lidar_tracks(tracks)

            # --------------------------------------------------
            # 6. Fusion
            # --------------------------------------------------

            fused = fusion.associate(
                camera_detections,
                lidar_inputs,
            )

            # --------------------------------------------------
            # 7. Print status when camera frame changes
            # --------------------------------------------------

            if camera_frame != last_camera_frame:

                confirmed = sum(
                    1
                    for track in tracks
                    if track.state == "CONFIRMED"
                )

                print(
                    f"[PIPELINE] "
                    f"camera_frame={camera_frame} "
                    f"camera_objects={len(camera_detections)} "
                    f"lidar_obstacles={len(obstacles)} "
                    f"lidar_tracks={len(tracks)} "
                    f"confirmed={confirmed} "
                    f"fused={len(fused)}"
                )

                if fused:
                    print_fused_objects(fused)

                last_camera_frame = camera_frame

            # --------------------------------------------------
            # 8. Maintain LiDAR scan rate
            # --------------------------------------------------

            time.sleep(1.0 / SCAN_HZ)

    except KeyboardInterrupt:
        print("")
        print("=" * 70)
        print("NIDAR INTEGRATED PIPELINE STOPPED")
        print("=" * 70)


if __name__ == "__main__":
    main()
