#!/usr/bin/env python3

"""
ROS2 LeRRe Node
Unified node that combines:
- SO101 follower arm control
- Tank/tread drive base control (differential drive, 2 tracks)

Uses a single serial port to control both arm and base servos.
"""

import math
import time
from pathlib import Path

import json
import yaml

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

from feetech_python_driver import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class Ros2LeRReNode(Node):
    """
    Unified ROS2 node for controlling both the SO101 follower arm and LeRRe tread base.
    Uses a single serial port to communicate with all servos.
    """

    TRACK_NAMES = ["left_track", "right_track"]
    TRACK_IDS = {"left_track": 7, "right_track": 8}

    def __init__(self):
        super().__init__(
            'lerre_ros2_node',
            automatically_declare_parameters_from_overrides=True,
        )

        # ========== Parameters ==========
        self._declare_parameters({
            'port': '/dev/ttyACM1',
            'baudrate': 1_000_000,
            'arm_id': 'follower_arm',
            'leader_arm_id': 'leader_arm',
            'max_relative_target': 0.0,
            'use_degrees': True,
            'arm_calibration_file': '',
            'max_linear_speed': 1.0,
            'max_angular_speed': 2.0,
            'axis_linear_x': 1,
            'axis_angular_z': 3,
            'track_separation': 0.2,
            'wheel_radius': 0.05,
            'ticks_per_rev': 4096,
            'wheel_control_mode': 'joy',  # 'joy' or 'cmd_vel'
        })

        port_val = self.get_parameter('port').value
        self.port = str(port_val) if port_val is not None else '/dev/ttyACM1'

        baudrate_val = self.get_parameter('baudrate').value
        self.baudrate = int(baudrate_val) if baudrate_val is not None else 1_000_000

        arm_id_val = self.get_parameter('arm_id').value
        self.arm_id = str(arm_id_val) if arm_id_val is not None else 'follower_arm'

        leader_id_val = self.get_parameter('leader_arm_id').value
        self.leader_arm_id = str(leader_id_val) if leader_id_val is not None else 'leader_arm'

        max_target_val = self.get_parameter('max_relative_target').value
        self.max_relative_target_deg = float(max_target_val) if max_target_val is not None else 20.0

        use_degrees_val = self.get_parameter('use_degrees').value
        self.use_degrees = bool(use_degrees_val) if use_degrees_val is not None else True
        if not self.use_degrees:
            raise ValueError(
                "ROS joint_state integration currently assumes degree-normalized servo data;"
                " configure 'use_degrees' true or extend the conversion path."
            )

        arm_calibration_file_val = self.get_parameter('arm_calibration_file').value
        arm_calibration_file = str(arm_calibration_file_val) if arm_calibration_file_val is not None else ''

        max_linear_val = self.get_parameter('max_linear_speed').value
        self.max_linear_speed = float(max_linear_val) if max_linear_val is not None else 1.0

        max_angular_val = self.get_parameter('max_angular_speed').value
        self.max_angular_speed = float(max_angular_val) if max_angular_val is not None else 2.0

        axis_x_val = self.get_parameter('axis_linear_x').value
        self.axis_linear_x = int(axis_x_val) if axis_x_val is not None else 1

        axis_z_val = self.get_parameter('axis_angular_z').value
        self.axis_angular_z = int(axis_z_val) if axis_z_val is not None else 3

        track_sep_val = self.get_parameter('track_separation').value
        self.track_separation = float(track_sep_val) if track_sep_val is not None else 0.2

        wheel_r_val = self.get_parameter('wheel_radius').value
        self.wheel_radius = float(wheel_r_val) if wheel_r_val is not None else 0.05

        ticks_per_rev_val = self.get_parameter('ticks_per_rev').value
        self.ticks_per_rev = int(ticks_per_rev_val) if ticks_per_rev_val is not None else 4096

        wheel_control_mode_val = self.get_parameter('wheel_control_mode').value
        self.wheel_control_mode = str(wheel_control_mode_val) if wheel_control_mode_val is not None else 'joy'
        if self.wheel_control_mode not in ['joy', 'cmd_vel']:
            self.get_logger().warn(
                f'Invalid wheel_control_mode: {self.wheel_control_mode}. Using "joy" as default.'
            )
            self.wheel_control_mode = 'joy'

        # Load arm calibration
        arm_calibration = self.load_calibration_from_parameters('arm_calibration')
        if not arm_calibration:
            arm_calibration = self.load_calibration_from_parameters('calibration')

        if arm_calibration:
            self.get_logger().info('Loaded arm calibration from ROS parameters')
        elif arm_calibration_file:
            try:
                arm_calibration = self.load_calibration_from_file(
                    arm_calibration_file,
                    parameter_namespace='arm_calibration',
                )
                self.get_logger().info(f'Loaded arm calibration from {arm_calibration_file}')
            except Exception as e:
                self.get_logger().warning(f'Failed to load arm calibration: {e}')

        norm_mode_body = MotorNormMode.DEGREES if self.use_degrees else MotorNormMode.RANGE_M100_100

        all_motors = {
            "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
            "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
            "elbow_flex": Motor(3, "sts3215", norm_mode_body),
            "wrist_flex": Motor(4, "sts3215", norm_mode_body),
            "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            "left_track": Motor(self.TRACK_IDS["left_track"], "sts3215", MotorNormMode.RANGE_M100_100),
            "right_track": Motor(self.TRACK_IDS["right_track"], "sts3215", MotorNormMode.RANGE_M100_100),
        }

        self.bus = FeetechMotorsBus(
            port=self.port,
            motors=all_motors,
            calibration=arm_calibration,
        )

        self.joint_name_map = {
            'shoulder_pan': 'shoulder_pan',
            'shoulder_lift': 'shoulder_lift',
            'elbow': 'elbow_flex',
            'wrist_pitch': 'wrist_flex',
            'wrist_roll': 'wrist_roll',
            'wrist_yaw': 'gripper'
        }

        self.teleop_enabled = False
        self.current_arm_joint_states = {}
        self.last_leader_msg = None
        self.current_track_velocities = {track: 0.0 for track in self.TRACK_NAMES}

        # ========== ROS Subscribers ==========
        if self.wheel_control_mode == 'joy':
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
            self.get_logger().info('Wheel control mode: JOYSTICK (/joy)')
        else:
            self.joy_sub = None
            self.get_logger().info('Wheel control mode: CMD_VEL (/cmd_vel)')

        if self.wheel_control_mode == 'cmd_vel':
            self.cmd_vel_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_vel_callback, 10
            )
        else:
            self.cmd_vel_sub = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            f'{self.leader_arm_id}/joint_states',
            self.arm_joint_state_callback,
            30
        )

        # ========== ROS Publishers ==========
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.follower_arm_joint_state_pub = self.create_publisher(
            JointState, f'{self.arm_id}/joint_states', 30
        )

        self.track_joint_state_pub = self.create_publisher(
            JointState, '/track_joint_states', 10
        )

        # ========== ROS Services ==========
        self.teleop_service = self.create_service(
            SetBool,
            f'{self.arm_id}/set_teleop',
            self.teleop_service_callback
        )

        # ========== Timers ==========
        self.arm_state_timer = self.create_timer(0.035, self.publish_arm_states)
        self.track_state_timer = self.create_timer(0.05, self.publish_track_states)

        self.init_servos()

        self.get_logger().info(f'ROS2 LeRRe Node initialized on port {self.port}')
        self.get_logger().info('Arm teleoperation DISABLED by default')
        self.get_logger().info(
            f'To enable: ros2 service call {self.arm_id}/set_teleop std_srvs/srv/SetBool "{{data: true}}"'
        )

    def _declare_parameters(self, defaults: dict):
        for name, default in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

    def load_calibration_from_parameters(self, prefix: str):
        param_entries = self.get_parameters_by_prefix(prefix)
        if not param_entries:
            return None

        raw_mapping = {}
        for key, parameter in param_entries.items():
            parts = key.split('.')
            if len(parts) != 2:
                self.get_logger().warning(
                    f'Ignoring malformed calibration parameter "{prefix}.{key}"'
                )
                continue
            joint_name, field = parts
            raw_mapping.setdefault(joint_name, {})[field] = parameter.value

        return self._build_calibration(raw_mapping)

    def load_calibration_from_file(self, path: str, parameter_namespace: str):
        file_path = Path(path)
        if not file_path.exists():
            raise FileNotFoundError(f'Calibration file "{path}" does not exist')

        with file_path.open('r') as handle:
            if file_path.suffix.lower() in {'.yaml', '.yml'}:
                calib_data = yaml.safe_load(handle)
            else:
                calib_data = json.load(handle)

        if isinstance(calib_data, dict):
            node_key = self.get_name()
            if node_key in calib_data:
                calib_data = calib_data[node_key]
            if isinstance(calib_data, dict) and 'ros__parameters' in calib_data:
                calib_data = calib_data['ros__parameters']
            if isinstance(calib_data, dict) and parameter_namespace in calib_data:
                calib_data = calib_data[parameter_namespace]

        return self._build_calibration(calib_data)

    def _build_calibration(self, data):
        if not isinstance(data, dict):
            raise ValueError('Calibration data must be a mapping of joints')

        calibration = {}
        required_fields = {'id', 'homing_offset', 'range_min', 'range_max'}

        for joint_name, calib_dict in data.items():
            if not isinstance(calib_dict, dict):
                raise ValueError(f'Calibration entry for {joint_name} must be a mapping')

            missing = required_fields - calib_dict.keys()
            if missing:
                raise ValueError(
                    f'Calibration entry for {joint_name} missing fields: {sorted(missing)}'
                )

            calibration[joint_name] = MotorCalibration(
                id=int(calib_dict['id']),
                drive_mode=int(calib_dict.get('drive_mode', 0)),
                homing_offset=int(calib_dict['homing_offset']),
                range_min=int(calib_dict['range_min']),
                range_max=int(calib_dict['range_max']),
            )

        return calibration

    def init_servos(self):
        """Initialize the unified servo connection (arm + tracks)."""
        try:
            self.bus.connect(handshake=True)

            self.bus.disable_torque()
            self.bus.configure_motors()

            arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                          "wrist_flex", "wrist_roll", "gripper"]
            for motor in arm_motors:
                if motor in self.bus.motors:
                    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                    self.bus.write("P_Coefficient", motor, 30)
                    self.bus.write("I_Coefficient", motor, 0)
                    self.bus.write("D_Coefficient", motor, 32)

                    if motor == "gripper":
                        self.bus.write("Max_Torque_Limit", motor, 500)
                        self.bus.write("Protection_Current", motor, 250)
                        self.bus.write("Overload_Torque", motor, 25)

            for track in self.TRACK_NAMES:
                if track in self.bus.motors:
                    self.bus.write("Operating_Mode", track, OperatingMode.VELOCITY.value)

            self.bus.enable_torque(motors=self.TRACK_NAMES)

            self.get_logger().info(f'Successfully connected to unified servo bus on {self.port}')
            self.get_logger().info(
                f'Configured {len(arm_motors)} arm servos and {len(self.TRACK_NAMES)} track servos'
            )
            self.get_logger().info('Arm torque DISABLED by default - tracks are ENABLED')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize servos: {str(e)}')
            raise

    def teleop_service_callback(self, request, response):
        """Service callback to enable/disable arm teleoperation and torque."""
        self.teleop_enabled = request.data

        if self.teleop_enabled:
            try:
                arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                              "wrist_flex", "wrist_roll", "gripper"]
                try:
                    arm_current = self.bus.sync_read("Present_Position", arm_motors, normalize=True)
                    self.bus.sync_write("Goal_Position", arm_current)
                    time.sleep(0.1)
                except Exception as e:
                    self.get_logger().warn(f'Could not read arm current position: {e}')

                self.bus.enable_torque(motors=arm_motors)

                if self.last_leader_msg is not None:
                    try:
                        goal_positions = {}
                        for i, joint_name in enumerate(self.last_leader_msg.name):
                            if i < len(self.last_leader_msg.position):
                                so101_name = self.joint_name_map.get(joint_name)
                                if so101_name:
                                    goal_positions[so101_name] = math.degrees(
                                        float(self.last_leader_msg.position[i])
                                    )
                        if goal_positions:
                            self.bus.sync_write("Goal_Position", goal_positions)
                            self.get_logger().info('Set follower to leader position')
                    except Exception as e:
                        self.get_logger().warn(f'Could not sync to leader position: {e}')

                self.get_logger().info('Arm teleoperation ENABLED')
                response.message = "Arm teleoperation enabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to enable arm torque: {e}')
                response.message = f"Failed to enable arm torque: {e}"
                response.success = False
        else:
            try:
                arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                              "wrist_flex", "wrist_roll", "gripper"]
                self.bus.disable_torque(motors=arm_motors)
                self.get_logger().info('Arm teleoperation DISABLED')
                response.message = "Arm teleoperation disabled"
                response.success = True
            except Exception as e:
                self.get_logger().error(f'Failed to disable arm torque: {e}')
                response.message = f"Failed to disable arm torque: {e}"
                response.success = False

        return response

    def joy_callback(self, msg):
        """Callback for joystick messages - controls track movement."""
        if not self.bus.is_connected:
            return

        if (self.axis_linear_x >= len(msg.axes) or
                self.axis_angular_z >= len(msg.axes)):
            self.get_logger().warn('Axis index out of range for received Joy message')
            return

        linear_x_raw = msg.axes[self.axis_linear_x]
        angular_z_raw = msg.axes[self.axis_angular_z]

        dead_zone = 0.1
        if abs(linear_x_raw) < dead_zone:
            linear_x_raw = 0.0
        if abs(angular_z_raw) < dead_zone:
            angular_z_raw = 0.0

        vx = -linear_x_raw * self.max_linear_speed
        omega = angular_z_raw * self.max_angular_speed

        self.apply_track_velocities(vx, omega)

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages - controls track movement."""
        if not self.bus.is_connected:
            return

        vx = max(-self.max_linear_speed, min(self.max_linear_speed, msg.linear.x))
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, msg.angular.z))

        self.apply_track_velocities(vx, omega)

    def apply_track_velocities(self, vx, omega):
        """
        Apply differential drive kinematics and send track velocity commands.

        Args:
            vx: Linear velocity [m/s]
            omega: Angular velocity [rad/s]
        """
        left_vel, right_vel = self.tank_kinematics(vx, omega)

        if self.wheel_control_mode == 'joy':
            twist_msg = Twist()
            twist_msg.linear.x = vx
            twist_msg.angular.z = omega
            self.twist_pub.publish(twist_msg)

        try:
            left_raw = self.rad_s_to_raw(left_vel / self.wheel_radius)
            right_raw = self.rad_s_to_raw(right_vel / self.wheel_radius)
            self.bus.sync_write("Goal_Velocity", {
                "left_track": -left_raw,
                "right_track": right_raw,
            })
            self.current_track_velocities["left_track"] = left_vel / self.wheel_radius
            self.current_track_velocities["right_track"] = right_vel / self.wheel_radius
        except Exception as e:
            self.get_logger().error(f'Failed to send track velocities: {e}')

    def tank_kinematics(self, vx, omega):
        """
        Differential drive kinematics.

        Args:
            vx: Linear velocity [m/s]
            omega: Angular velocity [rad/s]

        Returns:
            Tuple of (left_track_vel, right_track_vel) [m/s]
        """
        half_sep = self.track_separation / 2.0
        left_vel = vx - omega * half_sep
        right_vel = vx + omega * half_sep
        return left_vel, right_vel

    def rad_s_to_raw(self, rad_s: float) -> int:
        """Convert rad/s to raw servo velocity value."""
        deg_s = rad_s * 180.0 / math.pi
        ticks_per_deg = self.ticks_per_rev / 360.0
        return int(deg_s * ticks_per_deg)

    def arm_joint_state_callback(self, msg):
        """Callback for leader arm joint states."""
        self.last_leader_msg = msg

        if not self.bus.is_connected:
            self.get_logger().warn('Servo bus not connected, attempting to reconnect...')
            try:
                self.init_servos()
            except Exception:
                return
            return

        if not self.teleop_enabled:
            return

        try:
            goal_positions = {}

            for i, joint_name in enumerate(msg.name):
                if i < len(msg.position):
                    so101_name = self.joint_name_map.get(joint_name)
                    if so101_name:
                        goal_positions[so101_name] = math.degrees(float(msg.position[i]))

            if not goal_positions:
                return

            if self.max_relative_target_deg > 0:
                try:
                    arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                                  "wrist_flex", "wrist_roll", "gripper"]
                    present_positions = self.bus.sync_read(
                        "Present_Position", arm_motors, normalize=True
                    )

                    for so101_name, goal_pos in goal_positions.items():
                        if so101_name in present_positions:
                            present_pos = present_positions[so101_name]
                            diff = abs(goal_pos - present_pos)
                            if diff > self.max_relative_target_deg:
                                if goal_pos > present_pos:
                                    goal_positions[so101_name] = present_pos + self.max_relative_target_deg
                                else:
                                    goal_positions[so101_name] = present_pos - self.max_relative_target_deg
                except Exception as e:
                    self.get_logger().warn(f'Could not apply safety limits: {e}')

            self.bus.sync_write("Goal_Position", goal_positions)
            self.current_arm_joint_states = goal_positions.copy()

        except Exception as e:
            self.get_logger().error(f'Error sending command to follower arm: {str(e)}')

    def publish_arm_states(self):
        """Publish current follower arm joint states."""
        if not self.bus.is_connected:
            return

        try:
            arm_motors = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                          "wrist_flex", "wrist_roll", "gripper"]
            positions = self.bus.sync_read("Present_Position", arm_motors, normalize=True)

            self.current_arm_joint_states = positions.copy()

            reverse_map = {v: k for k, v in self.joint_name_map.items()}
            joint_names = []
            joint_positions = []

            for so101_name, position in positions.items():
                our_name = reverse_map.get(so101_name)
                if our_name:
                    joint_names.append(our_name)
                    joint_positions.append(math.radians(float(position)))

            if not joint_positions:
                return

            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = joint_names
            joint_state_msg.position = joint_positions
            joint_state_msg.velocity = [0.0] * len(joint_positions)
            joint_state_msg.effort = [0.0] * len(joint_positions)

            self.follower_arm_joint_state_pub.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing arm joint states: {str(e)}')

    def publish_track_states(self):
        """Publish current track joint states."""
        if not self.bus.is_connected:
            return

        try:
            positions = self.bus.sync_read("Present_Position", self.TRACK_NAMES, normalize=False)
            velocities = self.bus.sync_read("Present_Velocity", self.TRACK_NAMES, normalize=False)

            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [f"{track}_joint" for track in self.TRACK_NAMES]
            joint_state_msg.position = [
                self.raw_pos_to_rad(positions[track]) for track in self.TRACK_NAMES
            ]
            joint_state_msg.velocity = [
                self.raw_vel_to_rad_s(velocities[track]) for track in self.TRACK_NAMES
            ]
            joint_state_msg.effort = [0.0] * len(self.TRACK_NAMES)

            self.track_joint_state_pub.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing track joint states: {str(e)}')

    def raw_pos_to_rad(self, raw: int) -> float:
        deg = raw / (self.ticks_per_rev / 360.0)
        return deg * math.pi / 180.0

    def raw_vel_to_rad_s(self, raw: int) -> float:
        deg_s = raw / (self.ticks_per_rev / 360.0)
        return deg_s * math.pi / 180.0

    def on_shutdown(self):
        if self.bus.is_connected:
            try:
                self.bus.disable_torque()
                self.bus.disconnect(disable_torque=False)
                self.get_logger().info('Servo bus disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting servos: {e}')


def main(args=None):
    rclpy.init(args=args)

    node = Ros2LeRReNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
