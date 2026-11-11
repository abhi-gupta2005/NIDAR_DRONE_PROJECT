import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition


class PX4AirMouseSimulation(Node):

    def __init__(self):
        super().__init__('px4_airmouse_sim')

        # ==========================================================
        # PX4 QoS
        # ==========================================================

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==========================================================
        # PX4 TELEMETRY
        # ==========================================================

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        # ==========================================================
        # STATE
        # ==========================================================

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.heading = 0.0

        self.position_received = False

        # ==========================================================
        # SIMULATED AIRMOUSE COMMAND
        # ==========================================================

        # These values represent the corrected AirMouse command.
        #
        # IMPORTANT:
        # These are NOT sent to PX4.
        #
        # They are only used to show what the command would mean.

        self.command_forward = 0.0
        self.command_lateral = 0.0
        self.command_vertical = 0.0

        # ==========================================================
        # TIMER
        # ==========================================================

        self.timer = self.create_timer(
            0.5,
            self.update
        )

        self.get_logger().info(
            'PX4 AirMouse simulation monitor started'
        )

        self.get_logger().info(
            'MODE: SIMULATION ONLY'
        )

        self.get_logger().info(
            'PX4 SETPOINTS: DISABLED'
        )

        self.get_logger().info(
            'NO COMMANDS WILL BE SENT TO PX4'
        )

    # ==========================================================
    # PX4 POSITION CALLBACK
    # ==========================================================

    def position_callback(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

        self.heading = msg.heading

        self.position_received = True

    # ==========================================================
    # SIMULATION UPDATE
    # ==========================================================

    def update(self):

        if not self.position_received:

            print()
            print('=' * 65)
            print('WAITING FOR PX4 TELEMETRY...')
            print('=' * 65)

            return

        # ======================================================
        # CURRENT DRONE STATE
        # ======================================================

        print()
        print('=' * 65)

        print('PX4 / AIRMOUSE SIMULATION')

        print('-' * 65)

        print(
            f'DRONE POSITION'
        )

        print(
            f'X: {self.x:+.3f} m'
        )

        print(
            f'Y: {self.y:+.3f} m'
        )

        print(
            f'Z: {self.z:+.3f} m'
        )

        print()

        print(
            f'DRONE VELOCITY'
        )

        print(
            f'VX: {self.vx:+.3f} m/s'
        )

        print(
            f'VY: {self.vy:+.3f} m/s'
        )

        print(
            f'VZ: {self.vz:+.3f} m/s'
        )

        print()

        print(
            f'HEADING: '
            f'{math.degrees(self.heading):+.2f}°'
        )

        print('-' * 65)

        # ======================================================
        # CURRENT AIRMOUSE COMMAND
        # ======================================================

        print('AIRMOUSE COMMAND')

        print(
            f'FORWARD : '
            f'{self.command_forward:+.3f} m/s'
        )

        print(
            f'LATERAL : '
            f'{self.command_lateral:+.3f} m/s'
        )

        print(
            f'VERTICAL: '
            f'{self.command_vertical:+.3f} m/s'
        )

        print('-' * 65)

        print('PX4 SETPOINTS: DISABLED')

        print('MODE: SIMULATION ONLY')

        print('=' * 65)


def main(args=None):

    rclpy.init(args=args)

    node = PX4AirMouseSimulation()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
