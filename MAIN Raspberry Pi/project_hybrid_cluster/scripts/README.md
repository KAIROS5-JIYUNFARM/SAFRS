scripts directory 
- Main RPi에서 실행되는 ZMQ 기반 ROS2 통신 브릿지 담당. 각 스크립트는 Lidar / Camera / Motor Control RPi들과의 메세지 교환을 담당.
- ROS2 Navigation(Nav2) 스택이 정상적으로 동작하기 위한 핵심 데이터 퍼블리시.

1. zmq_camera_sub.py
  - Camera RPi가 전송하는 JPEG base64 image stream을 수신하여 ROS2의 /Camera/image_raw로 publish
  - 출력 ROS 토픽
 
     | Topic | Message Type |
     |--------|--------------|
     | `/camera/image_raw` | `sensor_msgs/msg/Image` |

2. zmq_motor_pub.py
  - ROS2 Navigation2에서 생성한 /cmd_vel을 Motor Control RPi로 전송
  - /cmd_vel -> JSON 변환 담당 (lx, az --> JSON 전송)
  - Nav2가 계산한 주행 명령 실제 바퀴에 전달

3. zmq_lidar_sub.py
  - LiDAR RPi에서 전송하는 스캔 데이터를 수신해 Nav2/SLAM용 /scan topic으로 변환/publish를 수행
  - AMCL Localization / Nav2 경로 생성용 costmap 입력

4. zmq_odom_sub.py
  - Motor Control Pi의 엔코더 기반 위치를 수신해 ROS2 /odom과 TF를 생성하는 노드
  - AMCL의 위치 추정 입력
  - Nav2의 odom 기반 local planner 입력
  - TF tree (odom -> base) 구성

-ZMQ 통신으로 들어오는 외부 센서/기기 데이터를 ROS2 시스템으로 연결하는 통신 브릿지 역할 수행
