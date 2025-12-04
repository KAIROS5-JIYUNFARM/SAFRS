#!/usr/bin/env python3
"""
motor_serial_bridge.py
----------------------
Motor Control RPi 내부에서 실행되는 가장 핵심적인 브릿지 노드.

🎯 역할
1) motor_command_sub.py → (ZMQ:5003) → 단일 문자(w/a/s/d/x) 명령 수신
2) 해당 명령을 USB Serial(Arduino Mega 2560)에 전송
3) Arduino로부터 엔코더 데이터 수신 (LF/RF/LR/RR)
4) 이를 motor_odom_pub.py 가 읽을 수 있도록 ZMQ PUB(5002)으로 재전송

🔥 MotorPi 내부에서 CPU와 Arduino 하드웨어를 연결하는 “중간 허브 역할”
"""

import zmq
import serial
import time
import glob

BAUD = 115200  # Arduino와 통신 속도

# ===============================================================
# 1) Arduino 포트 자동 탐지 함수
#    (USB 연결 시 ttyACM0, ttyACM1 등의 포트를 자동으로 찾는다)
# ===============================================================
def find_port():
    ports = glob.glob("/dev/ttyACM*")
    if len(ports) == 0:
        print("[Bridge] No ACM port found.")  # Arduino 미연결 상태
        return None
    print("[Bridge] Found ports:", ports)
    return ports[0]     # 첫 번째 포트를 사용

# ===============================================================
# 2) Arduino 연결 함수 (연결 실패 시 자동 재시도)
# ===============================================================
def connect_arduino():
    while True:
        port = find_port()
        if port is None:
            time.sleep(1)
            continue

        try:
            s = serial.Serial(port, BAUD, timeout=0.1)
            print(f"[Bridge] Connected to Arduino on {port}")
            return s
        except Exception as e:
            print("[Bridge] Connection failed:", e)
            time.sleep(1)

# 실제 Arduino 연결
ser = connect_arduino()

# ===============================================================
# 3) ZeroMQ 소켓 초기화
# ===============================================================
ctx = zmq.Context()

# ---------------------------------------------------------------
# (A) MotorPi 명령 SUB
#     motor_command_sub.py → PUB(5003) → 여기서 SUB
# ---------------------------------------------------------------
cmd_sub = ctx.socket(zmq.SUB)
cmd_sub.connect("tcp://127.0.0.1:5003")
cmd_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[Bridge] CMD SUB → 5003")

# ---------------------------------------------------------------
# (B) Encoder PUB
#     Arduino 엔코더 데이터를 motor_odom_pub.py 로 전달
#     motor_odom_pub.py 는 5002를 SUB함
# ---------------------------------------------------------------
enc_pub = ctx.socket(zmq.PUB)
enc_pub.bind("tcp://*:5002")
print("[Bridge] ENC PUB → 5002")

# Poller (논블로킹 명령 수신용)
poller = zmq.Poller()
poller.register(cmd_sub, zmq.POLLIN)

# ===============================================================
# 4) 메인 루프
#    명령 수신 → Arduino 전송 → 엔코더 읽기 → ZMQ 재전송
# ===============================================================

while True:

    # -----------------------------------------------------------
    # (1) ZMQ: Motor 명령(w/a/s/d/x) 수신
    # -----------------------------------------------------------
    socks = dict(poller.poll(timeout=5))
    if cmd_sub in socks:
        cmd = cmd_sub.recv_string()
        print("[Bridge] CMD:", cmd)

        # Arduino에 시리얼로 전송
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            print("[Bridge] Serial write error:", e)
            ser = connect_arduino()    # 자동 재연결

    # -----------------------------------------------------------
    # (2) Arduino → 엔코더 데이터 읽기
    # -----------------------------------------------------------
    try:
        line = ser.readline().decode(errors="ignore").strip()

        # 예시 포맷:
        #   "LF:1050 RF:1047 LR:1053 RR:1049"
        if line.startswith("LF:") and "RF:" in line and "LR:" in line and "RR:" in line:
            enc_pub.send_string(line)  # motor_odom_pub.py 로 전달
            # print("[Bridge] ENC:", line)

    except Exception as e:
        print("[Bridge] Serial read error:", e)
        ser = connect_arduino()        # 자동 재연결
