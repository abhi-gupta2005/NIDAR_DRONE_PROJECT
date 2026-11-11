import json
import os
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray


PI_HOST = os.environ.get("NIDAR_PI_HOST", "10.218.70.147")
PI_PORT = 5010

DESIRED_RANGE = 3.0

KP_FORWARD = 0.50
KP_LATERAL = 0.80

MAX_FORWARD = 1.5
MAX_LATERAL = 1.5

RANGE_DEADZONE = 0.25
LATERAL_DEADZONE = 0.10

MIN_CONFIDENCE = 0.30
TARGET_TIMEOUT = 1.0


class NIDARBridge(Node):

    def __init__(self):
        super().__init__('nidar_bridge')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/nidar/airmouse_command',
            10
        )

        self.sock = None
        self.buffer = ""

        self.last_target_time = 0.0

        self.get_logger().info('NIDAR AirMouse ROS 2 bridge started')
        self.get_logger().info(
            f'Connecting to Pi: {PI_HOST}:{PI_PORT}'
        )
        self.get_logger().info(
            'Publishing: /nidar/airmouse_command'
        )

    def connect(self):

        if self.sock is not None:
            return True

        try:
            self.sock = socket.create_connection(
                (PI_HOST, PI_PORT),
                timeout=5
            )

            self.sock.settimeout(0.1)

            self.get_logger().info(
                'CONNECTED TO NIDAR PI'
            )

            return True

        except Exception as e:

            self.get_logger().warn(
                f'Pi connection failed: {e}'
            )

            self.sock = None
            return False

    def publish_command(
        self,
        forward,
        lateral,
        vertical,
        valid
    ):

        msg = Float32MultiArray()

        # [forward, lateral, vertical, valid]
        msg.data = [
            float(forward),
            float(lateral),
            float(vertical),
            float(valid)
        ]

        self.publisher.publish(msg)

    def zero_command(self):

        self.publish_command(
            0.0,
            0.0,
            0.0,
            0.0
        )

    def process_target(self, target):

        now = time.time()

        state = target.get('state', '')
        confidence = float(
            target.get('confidence', 0.0)
        )

        x = float(target.get('x_m', 0.0))
        y = float(target.get('y_m', 0.0))
        range_m = float(
            target.get('range_m', 0.0)
        )

        # ------------------------------------------------
        # Safety checks
        # ------------------------------------------------

        if state != 'ACTIVE':

            self.zero_command()

            self.get_logger().warn(
                f'TARGET NOT ACTIVE: {state} -> ZERO'
            )

            return

        if confidence < MIN_CONFIDENCE:

            self.zero_command()

            self.get_logger().warn(
                f'LOW CONFIDENCE {confidence:.2f} -> ZERO'
            )

            return

        self.last_target_time = now

        # ------------------------------------------------
        # NIDAR coordinate system
        #
        # X = lateral
        # Y = forward
        # Range = distance
        # ------------------------------------------------

        forward_error = y - DESIRED_RANGE
        lateral_error = x

        # ------------------------------------------------
        # Forward control
        # ------------------------------------------------

        if abs(forward_error) <= RANGE_DEADZONE:

            forward = 0.0

        else:

            forward = KP_FORWARD * forward_error

        # ------------------------------------------------
        # Lateral control
        # ------------------------------------------------

        if abs(lateral_error) <= LATERAL_DEADZONE:

            lateral = 0.0

        else:

            lateral = KP_LATERAL * lateral_error

        # ------------------------------------------------
        # Limits
        # ------------------------------------------------

        forward = max(
            -MAX_FORWARD,
            min(MAX_FORWARD, forward)
        )

        lateral = max(
            -MAX_LATERAL,
            min(MAX_LATERAL, lateral)
        )

        vertical = 0.0

        self.publish_command(
            forward,
            lateral,
            vertical,
            1.0
        )

        self.get_logger().info(
            f'TARGET #{target.get("stable_id", "?")} | '
            f'RANGE={range_m:.2f}m | '
            f'X={x:+.2f}m | '
            f'Y={y:+.2f}m | '
            f'CMD F={forward:+.2f} '
            f'L={lateral:+.2f} '
            f'V={vertical:+.2f}'
        )

    def receive_data(self):

        if not self.connect():
            self.zero_command()
            return

        try:

            data = self.sock.recv(65536)

            if not data:
                self.sock.close()
                self.sock = None
                self.zero_command()
                return

            self.buffer += data.decode(
                'utf-8',
                errors='ignore'
            )

            while '\n' in self.buffer:

                line, self.buffer = self.buffer.split(
                    '\n',
                    1
                )

                line = line.strip()

                if not line:
                    continue

                try:

                    packet = json.loads(line)

                    target = packet.get(
                        'target'
                    )

                    if target is not None:
                        self.process_target(target)

                except json.JSONDecodeError:

                    self.get_logger().warn(
                        'Invalid JSON received'
                    )

        except socket.timeout:
            pass

        except Exception as e:

            self.get_logger().warn(
                f'Network error: {e}'
            )

            try:
                self.sock.close()
            except Exception:
                pass

            self.sock = None

            self.zero_command()

    def timeout_check(self):

        if (
            self.last_target_time > 0
            and time.time() - self.last_target_time
            > TARGET_TIMEOUT
        ):

            self.zero_command()

            self.get_logger().warn(
                'TARGET TIMEOUT -> ZERO COMMAND'
            )

            self.last_target_time = 0.0

    def run_once(self):

        self.receive_data()
        self.timeout_check()


def main(args=None):

    # 2026-11-02: by default rclpy shuts its context down as soon as
    # Ctrl+C is pressed, so the zero command in `finally` below failed with
    # "publisher's context is invalid" (observed in WSL) and was never sent.
    # With SignalHandlerOptions.NO, Ctrl+C raises KeyboardInterrupt while the
    # context is still valid; the zero command goes out, then we shut down.
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = NIDARBridge()

    timer = node.create_timer(
        0.02,
        node.run_once
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Send the final zero a few times so subscribers receive it before
        # the process exits.
        if rclpy.ok():

            for _ in range(3):

                node.zero_command()

                time.sleep(0.05)

            node.get_logger().info('Shutdown: ZERO command sent')

        if node.sock is not None:

            try:
                node.sock.close()
            except Exception:
                pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
