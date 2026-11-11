import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude


class NidarTelemetry(Node):

    def __init__(self):
        super().__init__('nidar_telemetry')

        # PX4 uXRCE-DDS OUT topics use BEST_EFFORT.
        # The subscriber must use a compatible QoS.
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # -------------------------------------------------
        # PX4 LOCAL POSITION
        # -------------------------------------------------

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        # -------------------------------------------------
        # PX4 ODOMETRY
        # -------------------------------------------------

        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            px4_qos
        )

        # -------------------------------------------------
        # PX4 ATTITUDE
        # -------------------------------------------------

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos
        )

        # -------------------------------------------------
        # STATE
        # -------------------------------------------------

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.heading = 0.0

        self.q = [1.0, 0.0, 0.0, 0.0]

        self.position_received = False
        self.odometry_received = False
        self.attitude_received = False

        self.timer = self.create_timer(
            0.5,
            self.print_state
        )

        self.get_logger().info(
            'NIDAR AirMouse telemetry node started'
        )

        self.get_logger().info(
            'PX4 QoS: BEST_EFFORT'
        )

    # =====================================================
    # LOCAL POSITION
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
    # ODOMETRY
    # =====================================================

    def odometry_callback(self, msg):

        self.odometry_received = True

    # =====================================================
    # ATTITUDE
    # =====================================================

    def attitude_callback(self, msg):

        self.q = list(msg.q)

        self.attitude_received = True

    # =====================================================
    # DISPLAY
    # =====================================================

    def print_state(self):

        print()
        print("=" * 62)
        print("              NIDAR AIR MOUSE TELEMETRY")
        print("=" * 62)

        print()
        print("PX4 CONNECTION")

        print(
            f"  Local Position : "
            f"{'LIVE' if self.position_received else 'WAITING'}"
        )

        print(
            f"  Odometry       : "
            f"{'LIVE' if self.odometry_received else 'WAITING'}"
        )

        print(
            f"  Attitude       : "
            f"{'LIVE' if self.attitude_received else 'WAITING'}"
        )

        print()
        print("POSITION")

        print(f"  X       : {self.x:+.3f} m")
        print(f"  Y       : {self.y:+.3f} m")
        print(f"  Z       : {self.z:+.3f} m")

        print()
        print("VELOCITY")

        print(f"  VX      : {self.vx:+.3f} m/s")
        print(f"  VY      : {self.vy:+.3f} m/s")
        print(f"  VZ      : {self.vz:+.3f} m/s")

        print()
        print("HEADING")

        print(f"  Heading : {self.heading:+.3f} rad")

        print()
        print("ATTITUDE QUATERNION")

        print(f"  q[0]    : {self.q[0]:+.6f}")
        print(f"  q[1]    : {self.q[1]:+.6f}")
        print(f"  q[2]    : {self.q[2]:+.6f}")
        print(f"  q[3]    : {self.q[3]:+.6f}")

        print()
        print("NIDAR AIR MOUSE")

        print("  Controller : TELEMETRY ONLY")
        print("  Flight     : DISABLED")
        print("  Setpoints  : NOT PUBLISHED")

        print("=" * 62)


def main(args=None):

    rclpy.init(args=args)

    node = NidarTelemetry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
