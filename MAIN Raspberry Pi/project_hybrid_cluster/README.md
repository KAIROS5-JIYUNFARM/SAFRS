Main Raspberry Pi는 ZeroMQ(ZMQ)를 이용한 Lidar / Camera / Motor Control RPi들의 메시지 허브(Hub) 역할을 하며, 전체 클러스터를 통합 운영하는 핵심 노드로
ROS2 기반 네비게이션, 센서 융합, 동작 명령 전달 등을 모두 Main Pi가 담당.

1. Main Pi는 다음 3대의 Raspberry Pi들과 ZMQ로 연결되어 있음:

### 🔗 ZMQ Port Connection Table (Main Pi)

| 서브 Pi             | 포트                    | 방향(Direction) | 수신/송신 데이터 내용                |
|---------------------|-------------------------|------------------|--------------------------------------|
| **Lidar RPi**       | `tcp://*:6000`          | SUB              | LiDAR Scan 데이터 (`/scan`)         |
| **Motor Control RPi** | `tcp://*:5000`        | SUB              | Odometry (`/odom`)                  |
| **Motor Control RPi** | `tcp://motor_pi_ip:5001` | PUB           | Nav2 `/cmd_vel` 기반 모터 명령 송신 |
| **Camera RPi**      | `tcp://*:7000`          | SUB              | 원본 이미지 스트림 (`/raw_image`)   |

2. Main Pi에서 Nav2 전체를 실행하며 다음 기능을 수행함

  2-1. Localization
  -AMCL 사용
  -LiDAR(/scan) + Odom(/odom) 융합
  -현재 위치(map → odom → base_link) 계산

  2-2. Path Planning & Control
  - NavFn(Global Planner)
  - Pure Pursuit(Local Planner)
  - BT Navigator(Behavior Tree)

  2-3. Motor Command Output
  - Nav2의 /cmd_vel을 받아
  - ZeroMQ PUB으로 Motor RPi에 송신한다.

3. 송/수신 데이터 흐름

  3-1. LiDAR → Main Pi
  - Lidar RPi → ZMQ PUB(6000) → Main Pi ZMQ SUB → /scan → AMCL/SLAM
  
  3-2. Camera → Main Pi
  - Camera RPi → ZMQ PUB(7000) → Main Pi ZMQ SUB → /raw_image
               → 필요 시 YOLO/AI 검출 → Mission Logic
  
  3-3. Motor → Main Pi
  - Motor RPi → ZMQ PUB(5000) → Main Pi ZMQ SUB → /odom → Nav2 Localization
  
  3-4. Nav2 제어 명령 → Motor RPi
  - Nav2 /cmd_vel → Main Pi ZMQ PUB(5001) → Motor RPi → Motor Driver(TB6612)
  
