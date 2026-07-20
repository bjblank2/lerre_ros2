import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class TankDriveKinematicsNode(Node):
    """Convert /cmd_vel (Twist) to track velocity commands for ros2_control.

    Differential drive kinematics: left_track=ID7, right_track=ID8.
    The left track motor is physically reversed, so its command is negated.
    Publishes [left_omega, right_omega] in rad/s to /track_velocity_controller/commands.
    """

    def __init__(self):
        super().__init__('tank_drive_kinematics_node')

        self.declare_parameter('track_separation', 0.2)
        self.declare_parameter('wheel_radius', 0.05)

        self.track_separation = self.get_parameter('track_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.track_cmd_pub = self.create_publisher(
            Float64MultiArray, '/track_velocity_controller/commands', 10)

    def _cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        omega = msg.angular.z

        half_d = self.track_separation / 2.0
        r = self.wheel_radius

        # Differential drive kinematics → rad/s
        right_omega = (vx + omega * half_d) / r
        # Left motor is physically reversed relative to right, so negate
        left_omega = -(vx - omega * half_d) / r

        out = Float64MultiArray()
        out.data = [left_omega, right_omega]
        self.track_cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TankDriveKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
