#!/usr/bin/env python3
"""
zmq_listener.launch.py
----------------------
Main Pi에서 실행되는 ZMQ 기반 ROS2 Bridge 노드들을 실행하는 Launch 파일.

각 노드는 ZMQ(PUB/SUB) 구조를 통해
서브 Raspberry Pi로부터 데이터를 수신하거나,
Motor Pi로 명령을 송신한다.

구성:
- zmq_lidar_sub.py   : LiDAR RPi → /scan
- zmq_camera_sub.py  : Camera RPi → /camera/image_raw
- zmq_odom_sub.py    : Motor RPi → /odom + TF
- zmq_motor_pub.py   : Nav2 /cmd_vel → Motor RPi
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # =====================================================
        # 1) LiDAR ZMQ Subscriber
        #   - LiDAR Pi에서 보내는 LaserScan JSON을 수신
        #   - ROS2 /scan 토픽으로 퍼블리시
        # =====================================================
        Node(
            package='project_hybrid_cluster',
            executable='zmq_lidar_sub.py',
            name='zmq_lidar_sub',
            output='screen'
        ),

        # =====================================================
        # 2) Camera ZMQ Subscriber
        #   - Camera Pi에서 JPEG base64 프레임 수신
        #   - ROS2 /camera/image_raw 토픽 퍼블리시
        # =====================================================
        Node(
            package='project_hybrid_cluster',
            executable='zmq_camera_sub.py',
            name='zmq_camera_sub',
            output='screen'
        ),

        # =====================================================
        # 3) Motor Command ZMQ Publisher
        #   - Nav2에서 생성된 /cmd_vel 메시지를 수신
        #   - Motor Pi로 ZeroMQ 전송
        # =====================================================
        Node(
            package='project_hybrid_cluster',
            executable='zmq_motor_pub.py',
            name='zmq_motor_pub',
            output='screen'
        ),

        # =====================================================
        # 4) Odom ZMQ Subscriber
        #   - Motor Pi 엔코더 기반 x y theta 데이터를 수신
        #   - ROS2 /odom + TF(odom → base_link) 퍼블리시
        # =====================================================
        Node(
            package='project_hybrid_cluster',
            executable='zmq_odom_sub.py',
            name='zmq_odom_sub',
            output='screen'
        ),
    ])
