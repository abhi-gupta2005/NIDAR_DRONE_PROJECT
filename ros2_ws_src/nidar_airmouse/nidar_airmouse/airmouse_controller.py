import math

import rclpy
from rclpy.node import Node


class AirMouseController(Node):

    def __init__(self):
        super().__init__('nidar_airmouse_controller')

        # =================================================
        # SIMULATED NIDAR TARGET
        # =================================================

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_range = 4.0

        # =================================================
        # CONTROL PARAMETERS
        # =================================================

        # Horizontal control
        self.kp_x = 0.8

        # Vertical control
        self.kp_y = 0.8

        # Distance control
        self.kp_range = 0.5

        # Maximum velocities
        self.max_vx = 1.5
        self.max_vy = 1.5
        self.max_vz = 1.0

        # Desired distance from target
        self.desired_range = 3.0

        # Dead zones
        self.x_deadzone = 0.10
        self.y_deadzone = 0.10
        self.range_deadzone = 0.25

        # =================================================
        # COMMAND OUTPUT
        # =================================================

        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0

        # =================================================
        # SIMULATION TIMER
        # =================================================

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info(
            'NIDAR AirMouse Controller started'
        )

        self.get_logger().info(
            'MODE: SIMULATION ONLY'
        )

        self.get_logger().info(
            'PX4 SETPOINTS: DISABLED'
        )

    # =====================================================
    # LIMIT VALUE
    # =====================================================

    def clamp(self, value, minimum, maximum):

        return max(
            minimum,
            min(value, maximum)
        )

    # =====================================================
    # DEADZONE
    # =====================================================

    def apply_deadzone(self, value, deadzone):

        if abs(value) < deadzone:
            return 0.0

        return value

    # =====================================================
    # AIR MOUSE CONTROLLER
    # =====================================================

    def calculate_control(self):

        # -------------------------------------------------
        # TARGET X
        #
        # Positive X = target is right
        # Negative X = target is left
        # -------------------------------------------------

        error_x = self.apply_deadzone(
            self.target_x,
            self.x_deadzone
        )

        # -------------------------------------------------
        # TARGET Y
        #
        # Positive Y = target is above
        # Negative Y = target is below
        # -------------------------------------------------

        error_y = self.apply_deadzone(
            self.target_y,
            self.y_deadzone
        )

        # -------------------------------------------------
        # TARGET RANGE
        #
        # Positive error = target too far
        # Negative error = target too close
        # -------------------------------------------------

        range_error = (
            self.target_range -
            self.desired_range
        )

        range_error = self.apply_deadzone(
            range_error,
            self.range_deadzone
        )

        # -------------------------------------------------
        # VELOCITY COMMANDS
        # -------------------------------------------------

        # Forward/backward
        self.cmd_vx = self.kp_range * range_error

        # Left/right
        self.cmd_vy = self.kp_x * error_x

        # Up/down
        self.cmd_vz = self.kp_y * error_y

        # -------------------------------------------------
        # LIMIT COMMANDS
        # -------------------------------------------------

        self.cmd_vx = self.clamp(
            self.cmd_vx,
            -self.max_vx,
            self.max_vx
        )

        self.cmd_vy = self.clamp(
            self.cmd_vy,
            -self.max_vy,
            self.max_vy
        )

        self.cmd_vz = self.clamp(
            self.cmd_vz,
            -self.max_vz,
            self.max_vz
        )

    # =====================================================
    # PRINT CONTROL STATE
    # =====================================================

    def control_loop(self):

        self.calculate_control()

        print()
        print("=" * 68)
        print("                 NIDAR AIR MOUSE")
        print("=" * 68)

        print()
        print("TARGET")

        print(
            f"  X / Horizontal : "
            f"{self.target_x:+.2f} m"
        )

        print(
            f"  Y / Vertical   : "
            f"{self.target_y:+.2f} m"
        )

        print(
            f"  Range          : "
            f"{self.target_range:.2f} m"
        )

        print()
        print("TARGET ERRORS")

        print(
            f"  X Error        : "
            f"{self.target_x:+.2f} m"
        )

        print(
            f"  Y Error        : "
            f"{self.target_y:+.2f} m"
        )

        print(
            f"  Range Error    : "
            f"{self.target_range - self.desired_range:+.2f} m"
        )

        print()
        print("AIR MOUSE COMMAND")

        print(
            f"  Forward VX     : "
            f"{self.cmd_vx:+.3f} m/s"
        )

        print(
            f"  Right VY       : "
            f"{self.cmd_vy:+.3f} m/s"
        )

        print(
            f"  Up VZ          : "
            f"{self.cmd_vz:+.3f} m/s"
        )

        print()
        print("INTERPRETATION")

        self.print_direction()

        print()
        print("SYSTEM SAFETY")

        print("  PX4 connection : READ ONLY")
        print("  Flight command  : DISABLED")
        print("  Setpoints       : NOT PUBLISHED")
        print("  Mode             : SIMULATION")

        print("=" * 68)

    # =====================================================
    # HUMAN READABLE DIRECTION
    # =====================================================

    def print_direction(self):

        moving = False

        if self.cmd_vx > 0.05:

            print("  → MOVE FORWARD")

            moving = True

        elif self.cmd_vx < -0.05:

            print("  → MOVE BACKWARD")

            moving = True

        if self.cmd_vy > 0.05:

            print("  → MOVE RIGHT")

            moving = True

        elif self.cmd_vy < -0.05:

            print("  → MOVE LEFT")

            moving = True

        if self.cmd_vz > 0.05:

            print("  → MOVE UP")

            moving = True

        elif self.cmd_vz < -0.05:

            print("  → MOVE DOWN")

            moving = True

        if not moving:

            print("  → HOLD POSITION")


def main(args=None):

    rclpy.init(args=args)

    node = AirMouseController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
