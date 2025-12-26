from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # =========================
    # Package path
    # =========================
    pkg = get_package_share_directory('slam_bringup')

    # =========================
    # Paths
    # =========================
    map_yaml   = os.path.join(pkg, 'maps', 'map_q.yaml')
    amcl_yaml  = os.path.join(pkg, 'config', 'amcl.yaml')

    lidar_launch = os.path.join(pkg, 'launch', 'lidar.launch.py')
    rsp_launch   = os.path.join(pkg, 'launch', 'robot_state_publisher.launch.py')

    # =========================
    # Robot State Publisher (TF)
    # =========================
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch)
    )

    # =========================
    # LiDAR
    # =========================
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch)
    )
    scan_relay = Node(
        package='slam_bringup',
        executable='scan_relay',
        name='scan_relay',
        output='screen'
    )


    # =========================
    # Map Server
    # =========================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': False
        }]
    )

    # =========================
    # AMCL
    # =========================
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_yaml]
    )

        # =========================
    # Lifecycle Manager (Localization)
    # =========================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )


    return LaunchDescription([
        robot_state_publisher,
        lidar,
        scan_relay,          # ← 반드시 lidar 뒤
        map_server,
        amcl,
        lifecycle_manager
    ])

