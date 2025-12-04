launch directory -> Main RPi에서 실행되는 ROS2 + ZeroMQ 기반 분산 로봇 시스템의 launch 파일 모음
각 launch 파일들은 Camera / LiDAR / Motor / Nav2 / Rviz를 통합적으로 실행하는 시스템을 구축하여
main_system.launch.py 파일 하나의 실행만으로 각 RPi들 및 Main RPi의 node들을 실행시킬 수 있도록 제작.

1. Main_system.launch.py
  - Main RPi의 전체 시스템을 실행하는 메인 launch 파일
  - ZMQ Listener, Navigation2 Bringup, Rviz를 실행시켜 각 RPi에서 Camera, Lidar, Odom을 ZMQ로 수신
  - AMCL 기반으로 Localization + Planner + Controller 역할 수행

2. zmq_listener.launch.py
  - 각 RPi와 ZMQ로 통신하는 Bridge 노드를 실행
  - scripts 디렉토리 내부의 zmq_sub/pub 파일들을 실행시키는 역할

3. nav.launch.py
  - Navigation2 Bringup 실행 파일.
  - AMCL Localization
  - Local/Global Planner
  - BT Navigator, Costmap 서버의 기능을 포함

4. rviz.launch.py
  - Rviz2를 실행하여 시각화를 담당하는 목적의 launch 파일.
