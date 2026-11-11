import socket
import struct
import time
import cv2
import numpy as np
from ultralytics import YOLOE

HOST = "127.0.0.1"
PORT = 5012
MODEL_PATH = r"D:\NIDAR_DRONE_PROJECT\yoloe-26n-seg.pt"
WINDOW = "NIDAR - Gazebo YOLOE Test"

def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Camera relay disconnected")
        data.extend(chunk)
    return bytes(data)

def main():
    print("Loading YOLOE model...")
    model = YOLOE(MODEL_PATH)
    print("YOLOE loaded.")

    print(f"Connecting to simulated camera at {HOST}:{PORT}...")
    with socket.create_connection((HOST, PORT), timeout=10) as sock:
        sock.settimeout(None)
        print("Connected. Press Q in the video window to quit.")

        count = 0
        start = time.time()
        last_report = start

        while True:
            header = recv_exact(sock, 4)
            size = struct.unpack("!I", header)[0]
            if size <= 0 or size > 10_000_000:
                raise ValueError(f"Invalid frame size: {size}")

            payload = recv_exact(sock, size)
            frame = cv2.imdecode(
                np.frombuffer(payload, dtype=np.uint8),
                cv2.IMREAD_COLOR
            )
            if frame is None:
                continue

            results = model.predict(
                frame,
                imgsz=416,
                conf=0.35,
                verbose=False
            )
            annotated = results[0].plot()

            count += 1
            now = time.time()
            if now - last_report >= 5:
                print(
                    f"YOLO processed {count} frames in "
                    f"{now - start:.1f}s "
                    f"({count / (now - start):.2f} FPS)"
                )
                last_report = now

            cv2.imshow(WINDOW, annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    print("YOLOE test stopped.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        cv2.destroyAllWindows()
