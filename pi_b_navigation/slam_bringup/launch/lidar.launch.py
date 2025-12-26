from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    x4pro_params = "/home/ubuntu/ros2_ws/src/slam_bringup/config/x4-pro.yaml"

    lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[
            x4pro_params,
            # {
            #         # 🔴 Nav2 / RViz 필수 QoS
            #         'frame_id': 'laser_frame',
            #         'reliability': 'BEST_EFFORT'
            # }
        ]
        # remappings=[
        #     ('/scan', '/scan_raw')
        # ]
    )


    return LaunchDescription([
        lidar_node
    ])
