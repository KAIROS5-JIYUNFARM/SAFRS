#!/usr/bin/env python3
"""
lidar_reader.py
----------------
LiDAR Raspberry Pi에서 실행되는 "LiDAR → ZeroMQ PUB" 드라이버.

🎯 역할
1) YDLiDAR / LD06 / LD14 계열과 동일한 프로토콜의 LiDAR 데이터를 /dev/ttyUSB0 에서 수신
2) LiDAR 데이터 패킷(각도 + 거리 배열)을 파싱
3) ZeroMQ PUB(6000)으로 Main Raspberry Pi에 JSON 형태로 전송

🔥 Main Pi에서는 zmq_lidar_sub.py 가 /scan 메시지로 변환해 ROS2에 퍼블리시함.
"""

import serial
import struct
import zmq
import time

# -------------------------
# LiDAR 기본 설정
# -------------------------
PORT = "/dev/ttyUSB0"   # LiDAR USB 시리얼 포트
BAUD = 128000           # LD06/LD14 기본 속도

# LiDAR 패킷 헤더 (0xAA 0x55)
HEADER = b'\xaa\x55'


# ============================================================
# 1) 메인 실행 함수
# ============================================================
def main():

    # --------------------------------------------------------
    # (A) LiDAR 시리얼 포트 오픈
    # --------------------------------------------------------
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print("[LiDAR PI] Serial connected")

    # --------------------------------------------------------
    # (B) ZeroMQ PUB 소켓 생성 → Main Pi로 전송
    # --------------------------------------------------------
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)

    # *:6000 → 어떤 IP든 구독 가능
    sock.bind("tcp://0.0.0.0:6000")
    print("[LiDAR PI] ZMQ PUB @6000")

    # 데이터 누적 버퍼
    buf = b""

    # =======================================================
    # 2) 메인 루프: LiDAR 패킷 읽기 + 파싱
    # =======================================================
    while True:

        # -----------------------------
        # (1) 버퍼에 시리얼 데이터 추가
        # -----------------------------
        buf += ser.read(2048)

        # -----------------------------
        # (2) 헤더(AA 55) 탐색
        # -----------------------------
        idx = buf.find(HEADER)
        if idx == -1:
            buf = b""   # 헤더 못 찾으면 버림
            continue

        # 헤더 이전 데이터 버리기
        if idx > 0:
            buf = buf[idx:]

        # 헤더 + CT + LSN 포함 최소 8바이트 필요
        if len(buf) < 8:
            continue

        # -----------------------------
        # (3) packet length 계산
        # LSN(distance 개수) * 2 + header(8)
        # -----------------------------
        CT = buf[2]    # 패킷 타입
        LSN = buf[3]   # distance sample 개수

        frame_len = 8 + LSN * 2

        # 패킷 전체 데이터가 도착할 때까지 대기
        if len(buf) < frame_len:
            continue

        # -----------------------------
        # (4) 패킷 잘라내기
        # -----------------------------
        packet = buf[:frame_len]
        buf = buf[frame_len:]   # 남은 데이터 다시 버퍼에 저장

        # -----------------------------
        # (5) 헤더 파싱
        # -----------------------------
        CT = packet[2]
        LSN = packet[3]

        # 원시 각도: 0.01도 단위의 정수
        raw_start = struct.unpack("<H", packet[4:6])[0]
        raw_end   = struct.unpack("<H", packet[6:8])[0]

        # 💡 LiDAR 제조사 대부분은 0~35999의 wrap-around 구조를 가짐
        start_angle = (raw_start % 36000) / 100.0
        end_angle   = (raw_end   % 36000) / 100.0

        # -----------------------------
        # (6) 거리(distance) 파싱
        # 2바이트씩 LSN개 존재
        # -----------------------------
        distances = []
        data = packet[8:8 + LSN*2]

        for i in range(LSN):
            # <H = 16bit unsigned little-endian
            dist = struct.unpack("<H", data[i*2:i*2+2])[0] / 1000.0   # mm → m
            distances.append(dist)

        # -----------------------------
        # (7) 전송용 JSON 데이터 구성
        # -----------------------------
        msg = {
            "ct": CT,
            "lsn": LSN,
            "start_angle": start_angle,
            "end_angle": end_angle,
            "distances": distances
        }

        # 디버그 출력
        print(f"[send] {start_angle:.1f} → {end_angle:.1f}, {LSN} samples")

        # -----------------------------
        # (8) ZeroMQ JSON 송신 → MainPi
        # -----------------------------
        sock.send_json(msg)


# ============================================================
# 3) 실행 엔트리 포인트
# ============================================================
if __name__ == "__main__":
    main()
