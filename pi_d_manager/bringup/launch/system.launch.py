from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    mode_manager_node = Node(
        package='system_manager',
        executable='mode_manager',
        name='mode_manager',
        output='screen'
        # 🚫 parameters 제거 (FSM msg 안 쓰므로)
    )

    return LaunchDescription([
        mode_manager_node,
    ])

