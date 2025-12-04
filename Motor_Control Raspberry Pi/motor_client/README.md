Motor_Control Raspberry Pi
- 로봇의 바퀴제어, 엔코더 수집, odom 계산을 담당하는 하위 제어 RPi
- Main RPi로부터 /cmd_vel 수신
- omometry 계산 -> 좌/우 바퀴 엔코더 tick -> 거리(m) 변환

1. motor_command_sub.py
  - Main RPi의 /cmd_vel을 Motor RPi -> mcu로 전달하는 명령 해석기의 역할

| lx | az | Motor 명령 |
|-----|-----|------------|
| > +0.1 | 0 | w (전진) |
| < -0.1 | 0 | s (후진) |
| 0 | > 0.1 | a (좌회전) |
| 0 | < -0.1 | d (우회전) |
| 거의 0 | 거의 0 | x (정지) |

와 같은 방식으로 명령 출력

2. motor_serial_bridge.py
  - Motor RPi <> mcu 사이의 serial bridge 역할
  - 엔코더 데이터 (LF/RF/LR/RR) 수집
  - ZMQ PUB(5002)으로 motor_odom_pub.py에 전달

3. motor_odom_pub.py
  - 로봇의 odometry를 계산하는 노드
  - motor_serial_brige.py -> ZMQ SUB(5002)
  - 엔코더 tick -> meter 변환

- 결과를 x y theta 문자열로 Main RPi에 PUB(5001)
- Main Pi는 위의 결과를 /odom + TF 로 변환하여 Nav2에 전달.
- 시스템 부팅시 자동으로 클라이언트 실행 (motor_client.service)
