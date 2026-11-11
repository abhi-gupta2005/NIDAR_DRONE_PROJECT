#!/usr/bin/env python3

import os
import time

from lidar_interface import (
    LidarInterfaceConfig,
    SerialSTL19PBackend,
)


def check_device(path):
    return os.path.exists(path)


def main():
    config = LidarInterfaceConfig()

    print("=" * 64)
    print("NIDAR HARDWARE CHECK")
    print("=" * 64)

    print(
        f"{'UART device':20s}: "
        f"{config.serial_port:20s} "
        f"{'OK' if check_device(config.serial_port) else 'NOT FOUND'}"
    )

    print(
        f"{'Baudrate':20s}: "
        f"{config.serial_baudrate}"
    )

    print(
        f"{'Timeout':20s}: "
        f"{config.serial_timeout_s:.3f} s"
    )

    print()

    backend = SerialSTL19PBackend(config)

    t0 = time.monotonic()
    opened = backend.open()
    open_time = time.monotonic() - t0

    print(
        f"{'UART open':20s}: "
        f"{'YES' if opened else 'NO'}"
    )

    print(
        f"{'Open time':20s}: "
        f"{open_time:.3f} s"
    )

    if not opened:
        print(
            f"{'UART error':20s}: "
            f"{backend.last_error}"
        )

        backend.close()

        print()
        print("LiDAR status       : NOT AVAILABLE")
        print("SLAM backend       : SIMULATOR")
        print("=" * 64)
        return

    print()
    print("Listening for UART data for 2 seconds...")

    start = time.monotonic()
    total_bytes = 0
    reads = 0

    while time.monotonic() - start < 2.0:
        data = backend.read_raw(512)
        reads += 1

        if data:
            total_bytes += len(data)

    backend.close()

    print()
    print("-" * 64)

    print(
        f"{'UART reads':20s}: "
        f"{reads}"
    )

    print(
        f"{'Incoming bytes':20s}: "
        f"{total_bytes}"
    )

    if total_bytes > 0:
        lidar_status = "DATA DETECTED"
    else:
        lidar_status = "WAITING / NOT CONNECTED"

    print(
        f"{'LiDAR status':20s}: "
        f"{lidar_status}"
    )

    print(
        f"{'Packet decoder':20s}: "
        f"NOT CONFIGURED"
    )

    print(
        f"{'SLAM backend':20s}: "
        f"SIMULATOR"
    )

    print("=" * 64)


if __name__ == "__main__":
    main()
