from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('slam_bringup')

    # =========================
    # File paths
    # =========================
    map_yaml     = os.path.join(pkg, 'maps', 'map_q.yaml')
    nav2_params  = os.path.join(pkg, 'config', 'nav2_param.yaml')
    amcl_params  = os.path.join(pkg, 'config', 'amcl.yaml')
    lidar_launch = os.path.join(pkg, 'launch', 'lidar.launch.py')
    rsp_launch   = os.path.join(pkg, 'launch', 'robot_state_publisher.launch.py')

    # 원본 navigation_launch.py의 표준 노드 리스트 (순서 준수)
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    return LaunchDescription([
        # 1. Robot State Publisher & LiDAR & Aligner
        IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(lidar_launch)),
        Node(
            package='slam_bringup',
            executable='odom_stamp_aligner',
            name='odom_stamp_aligner',
            output='screen'
        ),

        # 2. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}]
        ),

        # 3. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params, {'use_sim_time': False}]
        ),

        # 4. Controller Server (cmd_vel 리매핑 포함)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # 5. Smoother Server (추가)
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),

        # 6. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),

        # 7. Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),

        # 8. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),

        # 9. Waypoint Follower (추가)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}]
        ),

        # 10. Velocity Smoother (추가 및 리매핑)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': False}],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ),

        # 11. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_nodes
            }]
        ),
    ])