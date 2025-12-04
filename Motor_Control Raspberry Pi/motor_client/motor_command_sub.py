#!/usr/bin/env python3
"""
motor_command_sub.py
--------------------
Main Raspberry Pi의 Navigation2에서 생성하는 `/cmd_vel` 명령을
Motor Control RPi가 수신하여 Arduino 형식(w/a/s/d/x)으로 변환하는 노드.

🎯 동작 원리
1) MainPi: zmq_motor_pub.py → PUB(5000) → {"lx": , "az": }
2) MotorPi: motor_command_sub.py → SUB(5000)
3) linear.x, angular.z 값을 기반으로 모터 명령(w/a/s/d/x) 결정
4) motor_serial_bridge.py 로 ZMQ PUB(5003) 전송
5) motor_serial_bridge → Arduino 로 시리얼 전송

🔥 즉, Navigation2 명령을 실제 모터 드라이버가 이해하는 문자 명령으로 번역하는 역할.
"""

import zmq
import json

# Main Pi의 ZMQ 주소 및 포트
MAIN_PI_IP = "172.30.1.78"
ZMQ_MAIN_CMD_PORT = 5000   # MainPi → MotorPi 명령 전송 포트

ctx = zmq.Context()

# ----------------------------------------------------
# 1) Main Pi → SUB 연결
# ----------------------------------------------------
main_sub = ctx.socket(zmq.SUB)
main_sub.connect(f"tcp://{MAIN_PI_IP}:{ZMQ_MAIN_CMD_PORT}")
main_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[MotorPi] main CMD SUB ready (5000)")

# ----------------------------------------------------
# 2) MotorPi 내부 → motor_serial_bridge.py 로 명령 전달
# ----------------------------------------------------
# motor_serial_bridge.py 는 5003을 SUB함
bridge_pub = ctx.socket(zmq.PUB)
bridge_pub.bind("tcp://*:5003")
print("[MotorPi] bridge CMD PUB bind (5003)")

# ====================================================
# 3) 메인 루프
# ====================================================
while True:
    # ---------- (A) MainPi JSON 명령 수신 ----------
    raw = main_sub.recv_string()
    print("[MotorPi] Received:", raw)

    try:
        data = json.loads(raw)
        lx = data["lx"]   # linear.x
        az = data["az"]   # angular.z
    except:
        # JSON 파싱 실패 시 무시
        continue

    # ---------- (B) ROS2 cmd_vel → 문자 명령 변환 ----------
    if lx > 0.1:
        cmd = "w"        # forward
    elif lx < -0.1:
        cmd = "s"        # backward
    elif az > 0.1:
        cmd = "a"        # turn left
    elif az < -0.1:
        cmd = "d"        # turn right
    else:
        cmd = "x"        # stop (deadzone)

    print("[MotorPi] SEND TO BRIDGE:", cmd)

    # ---------- (C) Bridge로 전송 ----------
    bridge_pub.send_string(cmd)
