from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'slam_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),

        # config
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.rviz')),

        # urdf
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),

        # maps
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
    ],
    entry_points={
        'console_scripts': [
            # 기존
            'scan_stamp_aligner = slam_bringup.scan_stamp_aligner:main',

            # ★ LiDAR FSM Node 추가 ★
            'lidar_fsm_node = slam_bringup.lidar_fsm_node:main',

            'simple_nav_client = slam_bringup.simple_nav_client:main',
            'odom_stamp_aligner = slam_bringup.odom_stamp_aligner:main',
            'sensor_time_bridge = slam_bringup.sensor_time_bridge:main'
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='slam bringup package',
    license='Apache License 2.0',
)

