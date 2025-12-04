#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import zmq
from sensor_msgs.msg import LaserScan
import math

class LiDARNode(Node):
    """
    LiDARNode
    - LiDAR Raspberry Pi에서 ZeroMQ로 전송하는 스캔 데이터를 수신
    - LaserScan 메시지로 변환하여 /scan 토픽으로 퍼블리시
    """

    def __init__(self):
        super().__init__('lidar_subscriber')

        # 파라미터에서 LiDAR IP 등록
        self.declare_parameter("lidar_pub_ip", "172.30.1.14")
        lidar_ip = self.get_parameter("lidar_pub_ip").value

        # ZMQ SUB 설정
        ctx = zmq.Context()
        self.sock = ctx.socket(zmq.SUB)
        self.sock.connect(f"tcp://{lidar_ip}:6000")
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")

        # LaserScan 퍼블리셔 생성
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(f"LiDAR subscriber connected to {lidar_ip}:6000")

    def loop(self):
        """ LiDAR 데이터를 JSON으로 수신하여 ROS LaserScan으로 변환 """
        while rclpy.ok():
            data = self.sock.recv_json()

            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = "laser"

            # 시작/끝 각도
            start = math.radians(data["start_angle"])
            end   = math.radians(data["end_angle"])
            count = data["lsn"]

            scan.angle_min = start
            scan.angle_max = end
            scan.angle_increment = (end - start) / max(count - 1, 1)

            # LiDAR 사거리
            scan.range_min = 0.05
            scan.range_max = 12.0

            # 거리 배열
            scan.ranges = data["distances"]

            self.publisher.publish(scan)


def main():
    rclpy.init()
    node = LiDARNode()

    try:
        node.loop()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
