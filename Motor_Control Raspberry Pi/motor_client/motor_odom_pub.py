#!/usr/bin/env python3
"""
motor_odom_pub.py
-----------------
Motor Control RPi에서 **오도메트리(odometry)**를 계산하는 노드.

🎯 동작 원리
1) motor_serial_bridge.py 가 Arduino 엔코더 데이터를 PUB(5002)
2) motor_odom_pub.py 가 SUB(5002)로 엔코더 문자열 수신
3) 좌/우 바퀴 tick → 거리(m) 변환
4) 차동 구동(differential drive) 공식으로 x,y,theta 적분
5) Main Raspberry Pi로 오돔 데이터 PUB(5001)

🔥 Navigation2에서 사용되는 `/odom` 정보를 MotorPi가 자체 계산하여 제공하는 구조
"""

import zmq
import time
import math

# MainPi에서 수신할 오돔 포트
MAIN_PI_IP = "172.30.1.78"
ZMQ_ODOM_PORT = 5001            # MotorPi → MainPi (x y theta)
ZMQ_ENC_PORT_LOCAL = 5002       # Bridge → MotorPi (encoder ticks)

# -------------------------
# 로봇의 실제 물리 파라미터
# -------------------------
WHEEL_RADIUS = 0.033      # 바퀴 반지름 (m)
TICKS_PER_REV = 1024      # 엔코더 1바퀴 tick
WHEEL_BASE = 0.18         # 좌/우 바퀴 간 거리 (m)

# 로봇 상태 초기화
x, y, theta = 0.0, 0.0, 0.0
last_left = 0
last_right = 0
initialized = False

ctx = zmq.Context()

# -------------------------------------------------------
# (A) Encoder SUB (from motor_serial_bridge.py)
# -------------------------------------------------------
enc_sub = ctx.socket(zmq.SUB)
enc_sub.connect(f"tcp://127.0.0.1:{ZMQ_ENC_PORT_LOCAL}")
enc_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[MotorPi] ENC SUB connected (5002)")

# -------------------------------------------------------
# (B) Odom PUB (to Main Raspberry Pi)
# -------------------------------------------------------
odom_pub = ctx.socket(zmq.PUB)
odom_pub.bind(f"tcp://*:{ZMQ_ODOM_PORT}")
print("[MotorPi] ODOM PUB connected → 5001")

# -------------------------------------------------------
# tick → meter 변환 함수
# -------------------------------------------------------
def ticks_to_distance(ticks):
    rev = ticks / TICKS_PER_REV
    return rev * (2 * math.pi * WHEEL_RADIUS)  # 거리 = 회전수 × 둘레

# =======================================================
# Main Loop (50Hz)
# =======================================================
while True:
    # 엔코더 데이터 수신
    line = enc_sub.recv_string()

    try:
        parts = line.split()
        lf = int(parts[0].split(":")[1])
        rf = int(parts[1].split(":")[1])
        lr = int(parts[2].split(":")[1])
        rr = int(parts[3].split(":")[1])
    except:
        # 형식이 맞지 않을 경우 건너뛰기
        continue

    # 전/후 바퀴 평균값
    left = (lf + lr) / 2
    right = (rf + rr) / 2

    # 첫 수신은 기준값 초기화만 수행
    if not initialized:
        last_left = left
        last_right = right
        initialized = True
        continue

    # delta tick
    dL = left - last_left
    dR = right - last_right
    last_left, last_right = left, right

    # tick → meter 변환
    dL_m = ticks_to_distance(dL)
    dR_m = ticks_to_distance(dR)

    # 차동구동 공식
    dS = (dL_m + dR_m) / 2
    dTheta = (dR_m - dL_m) / WHEEL_BASE

    # Pose 업데이트
    x += dS * math.cos(theta + dTheta / 2)
    y += dS * math.sin(theta + dTheta / 2)
    theta += dTheta

    # 메인 Pi로 전송
    msg = f"{x} {y} {theta}"
    odom_pub.send_string(msg)

    print("[MotorPi] ODOM:", msg)

    time.sleep(0.02)   # 50Hz 주기
