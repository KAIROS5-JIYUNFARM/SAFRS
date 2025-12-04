#!/usr/bin/env python3
"""
main_system.launch.py
---------------------
Main Raspberry Pi에서 전체 시스템을 실행하는 메인 Launch 파일.

※ 현재 구조는 SLAM을 LiDAR RPi에서 수행하므로
   Main Pi에서는 SLAM Toolbox를 실행하지 않음.

Main Pi가 실행하는 구성:
1) ZMQ Listener (Camera / LiDAR / Odom ZMQ Bridge)
2) LiDAR TF (base_link ↔ laser)
3) Navigation2 (AMCL + Planner + Controller)
4) RViz2 시각화
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('project_hybrid_cluster')

    return LaunchDescription([

        # =====================================================
        # 1) ZeroMQ Listener (Camera, LiDAR, Odom 브릿지)
        # =====================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'zmq_listener.launch.py')
            )
        ),

        # =====================================================
        # 2) LiDAR TF (Main Pi에서 base_link → laser 위치 정의)
        #   - Navigation2의 로봇 모델 기준 LiDAR 위치
        #   - SLAM은 LiDAR Pi에서 수행하지만, TF는 Main Pi가 유지해야 함
        # =====================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                "--x", "0.10",     # LiDAR의 전방 offset (m)
                "--y", "0.0",
                "--z", "0.12",     # LiDAR의 높이 (m)
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "laser"
            ],
            output='screen'
        ),

        # =====================================================
        # 3) Navigation2 Bringup (AMCL + Planner + Controller)
        #   - SLAM 사용 X (이미 LiDAR Pi에서 SLAM 수행)
        #   - Main Pi는 /map, /scan, /odom을 입력받아 Navigation만 수행
        # =====================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav.launch.py')
            )
        ),

        # =====================================================
        # 4) RViz2 시각화 (default.rviz 설정 로드)
        # =====================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'rviz.launch.py')
            )
        ),
    ])
