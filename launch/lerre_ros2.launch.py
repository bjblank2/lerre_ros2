from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Serial port (shared by all servos)
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for unified servo bus'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Serial communication baudrate'
        ),

        # Arm parameters
        DeclareLaunchArgument(
            'arm_id',
            default_value='follower_arm',
            description='Arm ID for namespace'
        ),
        DeclareLaunchArgument(
            'leader_arm_id',
            default_value='leader_arm',
            description='Leader arm ID to follow'
        ),
        DeclareLaunchArgument(
            'max_relative_target',
            default_value='20.0',
            description='Maximum relative target movement (degrees)'
        ),
        DeclareLaunchArgument(
            'calibration_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('lekiwi_ros2'),
                'params',
                'lekiwi_so101_calibration.yaml',
            ]),
            description='ROS 2 parameter file that defines follower arm calibration data'
        ),

        # Base parameters - joystick mapping
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='1.0',
            description='Maximum linear speed (m/s)'
        ),
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='2.0',
            description='Maximum angular speed (rad/s)'
        ),
        DeclareLaunchArgument(
            'axis_linear_x',
            default_value='1',
            description='Joystick axis for linear X (forward/back)'
        ),
        DeclareLaunchArgument(
            'axis_angular_z',
            default_value='3',
            description='Joystick axis for angular Z (rotation)'
        ),

        # Base parameters - track geometry
        DeclareLaunchArgument(
            'track_seperation',
            default_value='0.2',
            description='Distance between left and right tracks (m)'
        ),
        DeclareLaunchArgument(
            'wheel_radius',
            default_value='0.05',
            description='Drive wheel/sprocket radius (m)'
        ),
        DeclareLaunchArgument(
            'ticks_per_rev',
            default_value='4096',
            description='Encoder ticks per revolution'
        ),
        DeclareLaunchArgument(
            'wheel_control_mode',
            default_value='cmd_vel',
            description='Wheel control mode: "joy" for /joy messages, "cmd_vel" for /cmd_vel messages'
        ),

        Node(
            package='lerre_ros2',
            executable='lerre_ros2_node',
            name='lerre_ros2_node',
            parameters=[
                LaunchConfiguration('calibration_params'),
                {
                    # Serial port
                    'port': LaunchConfiguration('port'),
                    'baudrate': LaunchConfiguration('baudrate'),

                    # Arm parameters
                    'arm_id': LaunchConfiguration('arm_id'),
                    'leader_arm_id': LaunchConfiguration('leader_arm_id'),
                    'max_relative_target': LaunchConfiguration('max_relative_target'),

                    # Base parameters
                    'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                    'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                    'axis_linear_x': LaunchConfiguration('axis_linear_x'),
                    'axis_angular_z': LaunchConfiguration('axis_angular_z'),
                    'track_seperation': LaunchConfiguration('track_seperation'),
                    'wheel_radius': LaunchConfiguration('wheel_radius'),
                    'ticks_per_rev': LaunchConfiguration('ticks_per_rev'),
                    'wheel_control_mode': LaunchConfiguration('wheel_control_mode'),
                },
            ],
            remappings=[
                ('/joy', '/spacemouse/joy'),
                ('/cmd_vel', '/spacemouse/twist'),
            ],
            output='screen'
        ),
    ])
