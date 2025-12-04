#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import zmq
import math
import tf2_ros

class OdomZMQSub(Node):
    """
    OdomZMQSub
    - Motor Pi가 전송하는 엔코더 기반 (x, y, theta) 데이터를 수신
    - ROS2의 /odom 메시지와 TF(odom→base_link)를 생성하여 Nav2에 제공
    """

    def __init__(self):
        super().__init__("odom_zmq_sub")

        # /odom 퍼블리셔
        self.pub = self.create_publisher(Odometry, "/odom", 10)

        # TF broadcaster (odom → base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ZMQ SUB 설정
        ctx = zmq.Context()
        self.sub = ctx.socket(zmq.SUB)

        MOTOR_PI_IP = "172.30.1.133"
        self.sub.connect(f"tcp://{MOTOR_PI_IP}:5001")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self.get_logger().info("ODOM ZMQ Subscriber started")

        # 50Hz 주기로 timer_callback 호출
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        """ ZMQ → 'x y theta' 문자열 수신 → Odometry + TF 브로드캐스트 """
        try:
            msg = self.sub.recv_string(flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        try:
            x, y, theta = map(float, msg.split())
        except:
            return

        # ------------------------------ #
        # ODOM 메시지 구성
        # ------------------------------ #
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        q = self.yaw_to_quaternion(theta)
        odom.pose.pose.orientation = q
        odom.pose.covariance = [0.0] * 36

        self.pub.publish(odom)

        # ------------------------------ #
        # TF (odom → base_link)
        # ------------------------------ #
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """ yaw(라디안) → Quaternion 변환 """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main():
    rclpy.init()
    node = OdomZMQSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
