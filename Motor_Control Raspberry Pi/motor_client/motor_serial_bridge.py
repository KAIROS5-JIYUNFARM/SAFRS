#!/usr/bin/env python3
import zmq
import serial
import time
import glob

BAUD = 115200

# -------- Auto-detect Arduino port --------
def find_port():
    ports = glob.glob("/dev/ttyACM*")
    if len(ports) == 0:
        print("[Bridge] No ACM port found.")
        return None
    print("[Bridge] Found ports:", ports)
    return ports[0]

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

ser = connect_arduino()

# -------- ZMQ sockets --------
ctx = zmq.Context()

# MotorPi commands SUB (from motor_command_sub)
cmd_sub = ctx.socket(zmq.SUB)
cmd_sub.connect("tcp://127.0.0.1:5003")
cmd_sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("[Bridge] CMD SUB → 5003")

# Encoder PUB (to motor_odom_pub)
enc_pub = ctx.socket(zmq.PUB)
enc_pub.bind("tcp://*:5002")
print("[Bridge] ENC PUB → 5002")

poller = zmq.Poller()
poller.register(cmd_sub, zmq.POLLIN)

while True:
    # Receive cmd
    socks = dict(poller.poll(timeout=5))
    if cmd_sub in socks:
        cmd = cmd_sub.recv_string()
        print("[Bridge] CMD:", cmd)
        try:
            ser.write((cmd + "\n").encode())
        except Exception as e:
            print("[Bridge] Serial write error:", e)
            ser = connect_arduino()

    # Read encoders
    try:
        line = ser.readline().decode(errors="ignore").strip()

        if line.startswith("LF:") and "RF:" in line and "LR:" in line and "RR:" in line:
            enc_pub.send_string(line)

    except Exception as e:
        print("[Bridge] Serial read error:", e)
        ser = connect_arduino()
