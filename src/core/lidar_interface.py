#!/usr/bin/env python3
"""
NIDAR V12.1 - LiDAR Interface Layer

Normalizes STL-19P simulator scans for downstream NIDAR processing.
The downstream code should use LidarScan regardless of whether the
source is the simulator or a future physical UART parser.
"""

from __future__ import annotations

import math
import time

try:
    import serial
except ImportError:
    serial = None
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

# Reuse the exact scan generator from the existing V12.1 simulator.
from src.simulation.nidar_lidar_simulator import (
    SCAN_HZ,
    MEASUREMENTS_PER_SECOND,
    MAX_RANGE_M,
    SAMPLES_PER_SCAN,
    make_scan,
)


@dataclass
class LidarPoint:
    angle_deg: float
    distance_m: float
    valid: bool
    timestamp: float
    scan_id: int
    hit_type: str = "Unknown"


@dataclass
class LidarScan:
    scan_id: int
    timestamp: float
    angle_deg: np.ndarray
    distance_m: np.ndarray
    valid: np.ndarray
    hit_types: List[str]
    scan_rate_hz: float
    measurements_per_second: int

    @property
    def valid_count(self) -> int:
        return int(np.count_nonzero(self.valid))

    @property
    def total_count(self) -> int:
        return int(len(self.distance_m))

    @property
    def nearest_distance_m(self) -> float:
        if not np.any(self.valid):
            return float("nan")
        return float(np.nanmin(self.distance_m[self.valid]))

    def to_xy(self) -> Tuple[np.ndarray, np.ndarray]:
        valid = self.valid
        a = np.deg2rad(self.angle_deg[valid])
        r = self.distance_m[valid]
        return r * np.cos(a), r * np.sin(a)

    def to_points(self) -> List[LidarPoint]:
        return [
            LidarPoint(
                angle_deg=float(a),
                distance_m=float(d),
                valid=bool(v),
                timestamp=self.timestamp,
                scan_id=self.scan_id,
                hit_type=self.hit_types[i],
            )
            for i, (a, d, v) in enumerate(
                zip(self.angle_deg, self.distance_m, self.valid)
            )
        ]


@dataclass
class ObstacleSector:
    start_deg: float
    end_deg: float
    nearest_distance_m: float
    valid_points: int
    blocked: bool


@dataclass
class LidarInterfaceConfig:
    scan_rate_hz: float = SCAN_HZ
    measurements_per_second: int = MEASUREMENTS_PER_SECOND
    max_range_m: float = MAX_RANGE_M
    min_range_m: float = 0.03
    noise_scale: float = 1.0

    # Physical LiDAR UART configuration.
    serial_port: str = "/dev/serial0"
    serial_baudrate: int = 230400
    serial_timeout_s: float = 0.05


class SimulatedSTL19PBackend:
    """Adapter around the existing nidar_v12_gui.py simulator."""

    name = "STL-19P_SIMULATOR"

    def __init__(self, config: Optional[LidarInterfaceConfig] = None):
        self.config = config or LidarInterfaceConfig()
        self.scan_id = 0
        self.sim_time = 0.0

    def reset(self) -> None:
        self.scan_id = 0
        self.sim_time = 0.0

    def next_scan(self, dt: Optional[float] = None) -> LidarScan:
        if dt is None:
            dt = 1.0 / self.config.scan_rate_hz

        self.sim_time += max(0.0, float(dt))

        angles, distances, hits, _person = make_scan(
            self.sim_time,
            self.config.noise_scale,
        )

        angles = np.asarray(angles, dtype=float)
        distances = np.asarray(distances, dtype=float)

        valid = (
            np.isfinite(distances)
            & (distances >= self.config.min_range_m)
            & (distances <= self.config.max_range_m)
        )

        normalized_distance = distances.copy()
        normalized_distance[~valid] = np.nan

        scan = LidarScan(
            scan_id=self.scan_id,
            timestamp=time.time(),
            angle_deg=angles,
            distance_m=normalized_distance,
            valid=valid,
            hit_types=list(hits),
            scan_rate_hz=self.config.scan_rate_hz,
            measurements_per_second=self.config.measurements_per_second,
        )

        self.scan_id += 1
        return scan


class SerialSTL19PBackend:
    """
    Safe protocol-neutral UART backend for the physical STL-19P.

    This backend handles UART connection bring-up only.
    The actual STL-19P packet decoder remains intentionally disabled
    until the physical sensor protocol is verified.
    """

    name = "STL-19P_SERIAL"

    def __init__(
        self,
        config: Optional[LidarInterfaceConfig] = None,
        auto_open: bool = False,
    ):
        self.config = config or LidarInterfaceConfig()
        self.scan_id = 0
        self.serial = None
        self.last_error = None

        if serial is None:
            raise RuntimeError(
                "pyserial is not installed. "
                "Install it with: pip install pyserial"
            )

        if auto_open:
            self.open()

    def open(self) -> bool:
        """Open UART without allowing an unavailable sensor to hang."""
        if self.serial is not None and self.serial.is_open:
            return True

        self.close()
        self.last_error = None

        try:
            self.serial = serial.Serial(
                port=self.config.serial_port,
                baudrate=self.config.serial_baudrate,
                timeout=self.config.serial_timeout_s,
                write_timeout=self.config.serial_timeout_s,
            )

            print(
                f"[LIDAR UART] OPEN "
                f"port={self.config.serial_port} "
                f"baud={self.config.serial_baudrate}"
            )

            return True

        except (serial.SerialException, OSError) as exc:
            self.last_error = str(exc)
            self.serial = None

            print(
                f"[LIDAR UART] NOT AVAILABLE: "
                f"{self.config.serial_port} "
                f"({exc})"
            )

            return False

    def close(self) -> None:
        """Close UART safely."""
        if self.serial is not None:
            try:
                if self.serial.is_open:
                    self.serial.close()
            except Exception:
                pass
            finally:
                self.serial = None

    def reset(self) -> None:
        self.scan_id = 0

        if self.serial is not None and self.serial.is_open:
            try:
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
            except Exception as exc:
                self.last_error = str(exc)

    @property
    def is_open(self) -> bool:
        return bool(
            self.serial is not None
            and self.serial.is_open
        )

    def read_raw(self, size: int = 256) -> bytes:
        """
        Read raw UART bytes.

        Returns b"" when the physical LiDAR is unavailable.
        No packet assumptions are made here.
        """
        if not self.is_open:
            if not self.open():
                return b""

        try:
            return self.serial.read(size)

        except (serial.SerialException, OSError) as exc:
            self.last_error = str(exc)
            self.close()
            return b""

    def next_scan(
        self,
        dt: Optional[float] = None,
    ) -> LidarScan:
        raise NotImplementedError(
            "STL-19P packet decoder is not configured yet. "
            "Use read_raw() for hardware bring-up."
        )

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class LidarProcessor:
    """Hardware-independent processing of normalized LiDAR scans."""

    def __init__(self, config: Optional[LidarInterfaceConfig] = None):
        self.config = config or LidarInterfaceConfig()

    @staticmethod
    def normalize_angle(angle_deg: float) -> float:
        return float(angle_deg % 360.0)

    def sector(
        self,
        scan: LidarScan,
        start_deg: float,
        end_deg: float,
        obstacle_threshold_m: float = 1.0,
    ) -> ObstacleSector:
        start = self.normalize_angle(start_deg)
        end = self.normalize_angle(end_deg)

        angles = scan.angle_deg

        if start <= end:
            mask = (angles >= start) & (angles <= end)
        else:
            mask = (angles >= start) | (angles <= end)

        mask &= scan.valid

        if not np.any(mask):
            nearest = float("nan")
            count = 0
            blocked = False
        else:
            values = scan.distance_m[mask]
            nearest = float(np.nanmin(values))
            count = int(values.size)
            blocked = nearest <= obstacle_threshold_m

        return ObstacleSector(
            start_deg=start,
            end_deg=end,
            nearest_distance_m=nearest,
            valid_points=count,
            blocked=blocked,
        )

    def front_left_right(
        self,
        scan: LidarScan,
        obstacle_threshold_m: float = 1.0,
    ) -> dict:
        return {
            "forward": self.sector(scan, 345, 15, obstacle_threshold_m),
            "left": self.sector(scan, 75, 105, obstacle_threshold_m),
            "rear": self.sector(scan, 165, 195, obstacle_threshold_m),
            "right": self.sector(scan, 255, 285, obstacle_threshold_m),
        }


def run_self_test(scans: int = 5) -> None:
    config = LidarInterfaceConfig(noise_scale=1.0)
    backend = SimulatedSTL19PBackend(config)
    processor = LidarProcessor(config)

    print("=" * 64)
    print("NIDAR V12.1 - LiDAR INTERFACE SELF TEST")
    print("=" * 64)
    print(f"Backend          : {backend.name}")
    print(f"Scan rate        : {config.scan_rate_hz:.1f} Hz")
    print(f"Measurements/sec : {config.measurements_per_second}")
    print(f"Samples/scan     : {SAMPLES_PER_SCAN}")
    print(f"Maximum range    : {config.max_range_m:.1f} m")
    print()

    for _ in range(scans):
        scan = backend.next_scan()
        sectors = processor.front_left_right(scan, obstacle_threshold_m=1.0)

        print(
            f"scan={scan.scan_id:04d} "
            f"points={scan.total_count} "
            f"valid={scan.valid_count} "
            f"nearest={scan.nearest_distance_m:.3f} m"
        )

        for name, sector in sectors.items():
            d = (
                "N/A"
                if math.isnan(sector.nearest_distance_m)
                else f"{sector.nearest_distance_m:.2f} m"
            )
            print(
                f"  {name:7s}: nearest={d:>8s} "
                f"blocked={str(sector.blocked):5s} "
                f"valid={sector.valid_points}"
            )

    x, y = scan.to_xy()
    print()
    print(f"Point cloud size : {len(x)} valid XY points")
    if len(x):
        print(f"Example XY point : ({x[0]:.3f}, {y[0]:.3f}) m")
    print("=" * 64)


if __name__ == "__main__":
    run_self_test()
