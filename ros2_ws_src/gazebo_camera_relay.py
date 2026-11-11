import socket
import struct
import threading
import cv2
import numpy as np

from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HOST = "0.0.0.0"
PORT = 5012
TOPIC = "/world/nidar_indoor_rescue/model/x500_nidar_indoor_test/link/camera_link/sensor/camera/image"

client_lock = threading.Lock()
client = None

def accept_clients(server):
    global client
    while True:
        conn, addr = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with client_lock:
            if client:
                try:
                    client.close()
                except OSError:
                    pass
            client = conn
        print(f"[TCP] Windows client connected: {addr}", flush=True)

class CameraRelay(Node):
    def __init__(self):
        super().__init__("gazebo_camera_tcp_relay")
        self.bridge = CvBridge()
        self.count = 0
        self.create_subscription(Image, TOPIC, self.on_image, 1)

    def on_image(self, msg):
        global client
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            )
            if not ok:
                return
            payload = encoded.tobytes()
            packet = struct.pack("!I", len(payload)) + payload

            with client_lock:
                current = client
                if current is not None:
                    try:
                        current.sendall(packet)
                        self.count += 1
                        if self.count % 30 == 0:
                            print(f"[TCP] Sent {self.count} frames", flush=True)
                    except OSError as exc:
                        print(f"[TCP] Client disconnected: {exc}", flush=True)
                        try:
                            current.close()
                        except OSError:
                            pass
                        client = None
        except Exception as exc:
            self.get_logger().error(f"Frame relay error: {exc}")

def main():
    global client
    rclpy.init()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"[TCP] Listening on {HOST}:{PORT}", flush=True)

    threading.Thread(
        target=accept_clients, args=(server,), daemon=True
    ).start()

    node = CameraRelay()
    print(f"[ROS 2] Subscribed to {TOPIC}", flush=True)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        with client_lock:
            if client:
                client.close()
                client = None
        server.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
