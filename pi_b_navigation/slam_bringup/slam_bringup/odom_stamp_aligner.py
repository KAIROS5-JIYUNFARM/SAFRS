#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomStampAligner(Node):
    def __init__(self):
        super().__init__('odom_stamp_aligner')

        # Publisher: Nav2가 실제로 사용할 /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Subscriber: C파이(STM32)에서 오는 /odom_raw (stamp가 0인 상태)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_raw', self.odom_raw_cb, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_odom = None

        # 20Hz 주기로 TF와 Odom 발행
        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info('Odom Stamp Aligner started: Overwriting stamp with B-Pi time.')

    def odom_raw_cb(self, msg: Odometry):
        # C파이에서 온 데이터 저장 (0.0초 stamp가 들어있음)
        self.latest_odom = msg

    def timer_cb(self):
        if self.latest_odom is None:
            return

        # 🔑 핵심: C파이의 stamp를 완전히 무시하고 B파이의 '현재 시간'을 새로 생성
        # Nav2의 데이터 처리 지연을 방지하기 위해 0.05초 미래 시간을 부여합니다.
        now = self.get_clock().now()
        future_now = now + rclpy.duration.Duration(seconds=0.05)
        current_stamp = future_now.to_msg()
        
        msg = self.latest_odom

        # 1️⃣ /odom 발행 (B파이 시간 주입)
        odom = Odometry()
        odom.header.stamp = current_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        # 2️⃣ TF broadcast (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = current_stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomStampAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()