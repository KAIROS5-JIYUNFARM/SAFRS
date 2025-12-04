#!/usr/bin/env python3
"""
motor_odom_pub.py
------------------
Motor Control RPi에서 수행되는 ‘오돔(odometry)’ 계산 노드.

동작 흐름:
1) motor_serial_bridge.py 가 Arduino에서 읽은 엔코더 값을 ZMQ PUB (6002)로 송신
2) motor_odom_pub.py 가 해당 값을 SUB
3) 좌·우 엔코더 tick → 거리(m) 변환
4) 차동구동(differential drive) 공식으로 x, y, theta 적분
5) 메인 Pi에 ‘x y theta’ 문자열을 ZMQ PUB (5001)로 전송

Main Pi는 이것을 받아 /odom + TF로 변환하여 Navigation2가 사용함.
"""

import zmq
import time
import math

# -------- Main Pi 환경 설정 --------
MAIN_PI_IP = "172.30.1.78"
ZMQ_ODOM_PORT = 5001          # Main Pi가 SUB하는 포트
ZMQ_ENC_PORT_LOCAL = 6002     # motor_serial_bridge.py 에서 로컬로 퍼블리시됨

# -------- 로봇 물리 파라미터 --------
WHEEL_RADIUS = 0.033          # 바퀴 반지름 (m)
TICKS_PER_REV = 1024          # 엔코더 1회전 tick 수
WHEEL_BASE = 0.18             # 좌/우 바퀴 간 거리 (m)

# 초기 pose
x, y, theta = 0.0, 0.0, 0.0

last_left = 0
last_right = 0
initialized = False

# -------- ZeroMQ 설정 --------
ctx = zmq.Context()

# (A) Arduino → enc_sub (ENC 읽기 SUB)
enc_sub = ctx.socket(zmq.SUB)
enc_sub.connect(f"tcp://127.0.0.1:{ZMQ_ENC_PORT_LOCAL}")
enc_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[MotorPi] ENC SUB connected (6002)")

# (B) Main Pi → odom_pub (오돔 PUB)
odom_pub = ctx.socket(zmq.PUB)
odom_pub.bind(f"tcp://*:{ZMQ_ODOM_PORT}")
print("[MotorPi] ODOM PUB bind (5001)")


# -------- tick → distance 변환 (m) --------
def ticks_to_distance(ticks):
    rev = ticks / TICKS_PER_REV                  # 회전 수
    return rev * (2 * math.pi * WHEEL_RADIUS)    # 둘레 × 회전수 = 이동거리


# -------- 메인 루프 --------
while True:
    # 1) ZMQ로 엔코더 문자열 수신
    line = enc_sub.recv_string()

    # 2) 문자열 → 각 바퀴 tick 파싱
    try:
        parts = line.split()
        lf = int(parts[0].split(":")[1])
        rf = int(parts[1].split(":")[1])
        lr = int(parts[2].split(":")[1])
        rr = int(parts[3].split(":")[1])
    except:
        continue

    # 좌/우 tick 평균 (전/후 바퀴)
    left = (lf + lr) / 2
    right = (rf + rr) / 2

    # 첫 데이터 초기화
    if not initialized:
        last_left = left
        last_right = right
        initialized = True
        continue

    # delta tick 계산
    dL = left - last_left
    dR = right - last_right
    last_left, last_right = left, right

    # tick → meter 변환
    dL_m = ticks_to_distance(dL)
    dR_m = ticks_to_distance(dR)

    # 차동구동 공식
    dS = (dL_m + dR_m) / 2
    dTheta = (dR_m - dL_m) / WHEEL_BASE

    # pose 업데이트
    x += dS * math.cos(theta + dTheta / 2)
    y += dS * math.sin(theta + dTheta / 2)
    theta += dTheta

    # Main Pi로 송신 (문자열)
    msg = f"{x} {y} {theta}"
    odom_pub.send_string(msg)
    print("[MotorPi] ODOM:", msg)

    time.sleep(0.02)   # 50Hz
