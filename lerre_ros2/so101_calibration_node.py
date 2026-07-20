#!/usr/bin/env python3
"""SO101 Calibration Node for the arm mounted on LeRRe.

Thin wrapper around the shared calibration routine in feetech_python_driver.
Determines homing offsets and joint ranges of motion for the SO101 arm
mounted on the LeRRe tracked base. Run this once for the arm.
"""

from feetech_python_driver import So101CalibrationNodeBase, run_calibration_main

import rclpy


class So101CalibrationNode(So101CalibrationNodeBase):
    def __init__(self):
        super().__init__(
            node_name='so101_calibration_node',
            package_name='lerre_ros2',
            default_port='/dev/ttyACM0',
            default_arm_id='arm',
        )


def main(args=None):
    rclpy.init(args=args)
    run_calibration_main(So101CalibrationNode)


if __name__ == '__main__':
    main()
