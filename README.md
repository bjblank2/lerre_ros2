# lerre_ros2

ROS 2 package for LeRRe, a tracked mobile manipulator combining an SO101 follower arm with a tank/tread drive base on a single Feetech servo bus.

**Status: WIP / untested on hardware.** Extracted as-is from `lekiwi_ros2`, where LeRRe support previously lived alongside LeKiwi support.

Two ways to run it:
- `lerre_ros2.launch.py` — direct-serial node (`lerre_ros2_node`) that owns the serial port itself.
- `lerre_ros2_control.launch.py` — ros2_control-based path (via [`feetech_ros2_driver`](https://github.com/bjblank2/feetech_ros2_driver)), matching the pattern used by [`lekiwi_ros2`](https://github.com/bjblank2/lekiwi_ros2) and [`so101_ros2`](https://github.com/bjblank2/so101_ros2). This scaffolding (`urdf/lerre.urdf.xacro`, `config/lerre_controllers.yaml`) already existed prior to extraction but has not been validated on real hardware.

Depends on [`feetech_python_driver`](https://github.com/bjblank2/feetech_python_driver) for Feetech bus communication and calibration, and on `so101_ros2` for the shared SO101 arm URDF.
