#!/usr/bin/env python3
"""
nav.launch.py
--------------
Navigation2 Bringup 실행 파일.

SLAM은 LiDAR Pi에서 수행하므로,
Main Pi에서는 오직:
- AMCL Localization
- Global Planner
- Local Planner
- BT Navigator

만 실행한다.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_share = get_package_share_directory('nav2_bringup')

    # ------------------------------
    # map.yaml 파일 경로 설정
    # Main Pi는 LiDAR Pi가 SLAM한 후 저장된 map.yaml 사용
    # ------------------------------
    map_file = LaunchConfiguration('map')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('project_hybrid_cluster'),
            'config', 'map.yaml'
        ),
        description='Full path to the map YAML file'
    )

    return LaunchDescription([

        # map argument 등록
        declare_map,

        # ================================================
        # Navigation2 Bringup 실행
        # ================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items()
        )
    ])
