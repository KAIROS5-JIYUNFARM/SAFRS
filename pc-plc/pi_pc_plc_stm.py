import serial
import time
import socket

# ============================
# 포트 설정
# ============================
PLC_PORT = "COM16"     # PLC RS232 포트
STM32_PORT = "COM5"    # STM32 포트
PLC_BAUD = 9600
STM_BAUD = 115200

# ============================
# 라즈베리파이 UDP 설정
# ============================
RPI_IP = "172.30.1.5"     # 라즈베리파이 IP 주소
RPI_PORT = 5006            # 라즈베리파이 UDP 수신 포트

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# ============================
# 시리얼 포트 오픈
# ============================
plc = serial.Serial(PLC_PORT, PLC_BAUD, timeout=0.05)
stm = serial.Serial(STM32_PORT, STM_BAUD, timeout=0.1)

print("[INFO] PC-PLC-STM32-ROS2 Link Started.")

# ============================
# PLC ENQ 요청 프레임
# ============================
ENQ_FRAME = b'\x05' + b'00RSS0106%MW000' + b'\x04'

buffer = b""


def parse_plc_response(frame: bytes):
    """PLC 응답 ASCII 변환"""
    try:
        ascii_str = frame.decode(errors="ignore")
    except:
        ascii_str = str(frame)

    print("[PLC RX ASCII]:", ascii_str)
    return ascii_str


# ============================
# 메인 루프
# ============================
while True:

    # 1) PLC에 ENQ 요청 전송
    plc.write(ENQ_FRAME)
    print("[PC → PLC] Sent ENQ:", ENQ_FRAME)
    time.sleep(0.02)

    # 2) PLC 응답 수신
    data = plc.read(100)
    if data:
        buffer += data

        # ETX(0x03) 도착하면 한 프레임 완성
        if b'\x03' in buffer:
            frame = buffer
            buffer = b""  # 초기화

            ascii_frame = parse_plc_response(frame)

            # -------------------------------
            # 응답 마지막 WORD 값 추출
            # -------------------------------
            numeric_part = ''.join(filter(str.isdigit, ascii_frame))

            if len(numeric_part) >= 4:
                last4 = numeric_part[-4:]
                print("[PARSE] Last 4 digits =", last4)

                # -------------------------------
                # M0 == ON
                # -------------------------------
                if last4 == "0001":
                    print("[INFO] M0 = ON 감지됨! 5초 뒤 동작 수행")

                    # 5초 대기
                    time.sleep(3)

                    # -------------------------------
                    # 1) STM32로 'S' 전송
                    # -------------------------------
                    stm.write(b"S")
                    print("[PC → STM32] Sent 'S' command")

                    # -------------------------------
                    # 2) 라즈베리파이로 UDP 송신
                    # -------------------------------
                    sock.sendto(b"START", (RPI_IP, RPI_PORT))
                    print(f"[PC → RPI] Sent 'START' via UDP to {RPI_IP}:{RPI_PORT}")

                    print("[INFO] Completed. Exiting loop.")
                    break

    time.sleep(0.1)

print("[INFO] Program finished.")
