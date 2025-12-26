from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('slam_bringup')
    urdf_path = os.path.join(pkg_path, 'urdf', 'minimal_robot.urdf')

    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'publish_frequency': 30.0},
            {'use_sim_time': False}
        
        ]
    )

    return LaunchDescription([
        rsp_node
    ])
