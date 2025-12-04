#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import zmq
import time

class MotorZMQ(Node):
    """
    MotorZMQ
    - ROS2 Navigation2에서 생성된 /cmd_vel 메시지를 수신
    - ZeroMQ PUB을 통해 Motor Control RPi에 속도 명령 송신
    """

    def __init__(self):
        super().__init__("motor_zmq_pub")

        # ZMQ Context 및 PUB 소켓 생성
        ctx = zmq.Context()
        self.pub = ctx.socket(zmq.PUB)

        # Motor Pi가 SUB 하는 5000번 포트로 bind
        self.pub.bind("tcp://*:5000")

        # bind 직후 안정화 시간
        time.sleep(0.5)

        # ROS2에서 /cmd_vel 구독
        self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

    def cmd_callback(self, msg):
        """ /cmd_vel 메시지 → JSON 변환 → ZMQ 전송 """
        cmd = {
            "lx": float(msg.linear.x),
            "az": float(msg.angular.z)
        }

        # Motor Pi로 JSON 명령 송신
        self.pub.send_json(cmd)
        self.get_logger().info(f"send ZMQ lx={cmd['lx']:.3f} az={cmd['az']:.3f}")


def main():
    rclpy.init()
    node = MotorZMQ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
