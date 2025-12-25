from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # =========================
    # 1) 원격 bringup
    # =========================
    remote_bringup = ExecuteProcess(
        cmd=['bash', 'remote_bringup.sh'],
        cwd='/home/ubuntu/ros2_ws/src/bringup/scripts',
        output='screen'
    )

    # =========================
    # 2) D-Pi system
    # =========================
    d_pi_system_bringup = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'bringup', 'system.launch.py'
        ],
        output='screen'
    )

    return LaunchDescription([
        remote_bringup,
        d_pi_system_bringup
    ])
