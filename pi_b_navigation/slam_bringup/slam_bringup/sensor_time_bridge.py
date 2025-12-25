#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

class SensorTimeBridge(Node):
    """
    Nav2-COMPATIBLE SENSOR BRIDGE (STABLE VERSION)
    
    개선 사항:
    1. TF 타임스탬프를 센서 타임스탬프보다 항상 최신으로 유지 (0.01초 오프셋)
    2. AMCL 및 Nav2의 타임스탬프 동기화 오류 방지
    """

    def __init__(self):
        super().__init__('sensor_time_bridge')

        # LiDAR QoS: Best Effort 기반 (Nav2 파라미터와 일치 필요)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', scan_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(LaserScan, '/scan_raw', self.scan_cb, scan_qos)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)

        # State
        self.last_pose = None
        self.time_offset = Duration(seconds=0.01) # 10ms 과거로 센서 시간 설정

        # TF는 50Hz로 안정적으로 발행
        self.create_timer(0.02, self.publish_tf)

        self.get_logger().info('SensorTimeBridge with Time-Offset started')

    # =====================
    # LiDAR (Scan 데이터를 TF보다 살짝 과거로 밀어넣음)
    # =====================
    def scan_cb(self, msg: LaserScan):
        now = self.get_clock().now()
        # 현재 시간보다 10ms 과거로 스탬프를 찍어 TF 버퍼 충돌 방지
        msg.header.stamp = (now - self.time_offset).to_msg()
        msg.header.frame_id = 'laser_frame'
        self.scan_pub.publish(msg)

    # =====================
    # ODOM
    # =====================
    def odom_cb(self, msg: Odometry):
        now = self.get_clock().now()
        
        odom = Odometry()
        # Odom 토픽도 TF와의 정렬을 위해 동일하게 10ms 오프셋 적용
        odom.header.stamp = (now - self.time_offset).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose
        odom.twist = msg.twist

        self.last_pose = msg.pose.pose
        self.odom_pub.publish(odom)

    # =====================
    # TF (가장 최신 시간을 사용하여 브로드캐스팅)
    # =====================
    def publish_tf(self):
        if self.last_pose is None:
            return

        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now  # TF는 '현재' 시간을 그대로 사용
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.last_pose.position.x
        t.transform.translation.y = self.last_pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.last_pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SensorTimeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()