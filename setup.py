from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lerre_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='brianjblank7@gmail.com',
    description='LeRRe tracked mobile manipulator ROS2 package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lerre_ros2_node = lerre_ros2.lerre_ros2_node:main',
            'tank_drive_kinematics_node = lerre_ros2.tank_drive_kinematics_node:main',
            'arm_teleop_node = lerre_ros2.arm_teleop_node:main',
            'so101_calibration_node = lerre_ros2.so101_calibration_node:main',
        ],
    },
)
