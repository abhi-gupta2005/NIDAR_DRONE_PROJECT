import socket
import struct
import cv2
import numpy as np

HOST = "127.0.0.1"
PORT = 5012

def recv_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Relay disconnected")
        data.extend(chunk)
    return bytes(data)

print(f"Connecting to Gazebo camera relay at {HOST}:{PORT}...")
with socket.create_connection((HOST, PORT), timeout=10) as sock:
    sock.settimeout(None)
    print("Connected! Waiting for camera frames. Press Q to quit.")

    count = 0
    while True:
        header = recv_exact(sock, 4)
        size = struct.unpack("!I", header)[0]

        if size <= 0 or size > 10_000_000:
            raise ValueError(f"Invalid frame size: {size}")

        payload = recv_exact(sock, size)
        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8),
                             cv2.IMREAD_COLOR)

        if frame is None:
            print("Could not decode frame")
            continue

        count += 1
        if count % 30 == 0:
            print(f"Received {count} frames; latest shape: {frame.shape}")

        cv2.imshow("NIDAR - Gazebo Simulated Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
