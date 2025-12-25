from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os


def generate_launch_description():

    pkg_share = get_package_share_directory("slam_bringup")

    # ---- SLAM params ----
    slam_params = os.path.join(pkg_share, "config", "slam_param.yaml")

    # ---- Robot State Publisher (URDF TF) ----
    rsp_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_state_publisher.launch.py")
        )
    )

    # ---- RViz include ----
    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rviz_view.launch.py")
        )
    )

    # ---- SLAM Toolbox ----
    slam_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    return LaunchDescription([
        rsp_include,     # 먼저 URDF TF 생성
        slam_node,       # 그 다음 SLAM 시작
        rviz_include,    # 마지막에 RViz 실행
    ])
