import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude


# =========================================================
# SAFETY SETTINGS
# =========================================================

# Keep both FALSE for this stage.
ALLOW_ARM = False
ALLOW_OFFBOARD = False


class PX4OffboardSafety(Node):

    def __init__(self):

        super().__init__('px4_offboard_safety')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------------------------------------------------
        # PX4 local position
        # -------------------------------------------------

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        # -------------------------------------------------
        # PX4 attitude
        # -------------------------------------------------

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos
        )

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        self.position_received = False
        self.attitude_received = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.heading = 0.0

        self.timer = self.create_timer(
            1.0,
            self.status_report
        )

        self.get_logger().info(
            'PX4 Offboard Safety Controller started'
        )

        self.get_logger().info(
            f'ALLOW_ARM = {ALLOW_ARM}'
        )

        self.get_logger().info(
            f'ALLOW_OFFBOARD = {ALLOW_OFFBOARD}'
        )

        self.get_logger().info(
            'AUTOMATIC ARMING: DISABLED'
        )

        self.get_logger().info(
            'AUTOMATIC OFFBOARD: DISABLED'
        )

        self.get_logger().info(
            'VehicleStatus subscription: DISABLED'
        )

    # =====================================================
    # PX4 position callback
    # =====================================================

    def position_callback(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

        self.heading = msg.heading

        self.position_received = True

    # =====================================================
    # PX4 attitude callback
    # =====================================================

    def attitude_callback(self, msg):

        self.attitude_received = True

    # =====================================================
    # Status report
    # =====================================================

    def status_report(self):

        position_state = (
            'LIVE'
            if self.position_received
            else 'WAITING'
        )

        attitude_state = (
            'LIVE'
            if self.attitude_received
            else 'WAITING'
        )

        print()
        print('=' * 65)
        print('PX4 OFFBOARD SAFETY MONITOR')
        print('-' * 65)

        print(
            f'POSITION TELEMETRY : {position_state}'
        )

        print(
            f'ATTITUDE TELEMETRY  : {attitude_state}'
        )

        if self.position_received:

            print('-' * 65)

            print(
                f'POSITION X : {self.x:+.3f} m'
            )

            print(
                f'POSITION Y : {self.y:+.3f} m'
            )

            print(
                f'POSITION Z : {self.z:+.3f} m'
            )

            print()

            print(
                f'VELOCITY VX : {self.vx:+.3f} m/s'
            )

            print(
                f'VELOCITY VY : {self.vy:+.3f} m/s'
            )

            print(
                f'VELOCITY VZ : {self.vz:+.3f} m/s'
            )

            print()

            print(
                f'HEADING : '
                f'{math.degrees(self.heading):+.2f}°'
            )

        print('-' * 65)

        print(
            f'ARMING REQUEST   : '
            f'{"ENABLED" if ALLOW_ARM else "DISABLED"}'
        )

        print(
            f'OFFBOARD REQUEST : '
            f'{"ENABLED" if ALLOW_OFFBOARD else "DISABLED"}'
        )

        print('-' * 65)

        print(
            'SAFETY STATE: '
            'NO ARM / NO OFFBOARD COMMANDS'
        )

        print('=' * 65)


def main(args=None):

    rclpy.init(args=args)

    node = PX4OffboardSafety()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':

    main()
