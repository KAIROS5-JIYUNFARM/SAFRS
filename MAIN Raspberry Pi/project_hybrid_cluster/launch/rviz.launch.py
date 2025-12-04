#!/usr/bin/env python3
"""
rviz.launch.py
--------------
RViz2 시각화 Launch 파일.
Main Pi에서 Nav2, TF, Scan, Map 상태를 실시간 모니터링할 때 사용.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                # Predefined RViz layout file
                "-d", "/home/ubuntu/ros2_ws/src/project_hybrid_cluster/config/default.rviz"
            ]
        )
    ])
