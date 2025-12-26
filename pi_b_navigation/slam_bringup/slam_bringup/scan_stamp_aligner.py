#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ScanStampAligner(Node):
    """
    LaserScan Timestamp Aligner
    /scan_raw -> /scan
    stamp = ROS now()
    """

    def __init__(self):
        super().__init__('scan_stamp_aligner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            qos
        )

        self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_cb,
            qos
        )

        self.get_logger().info('ScanStampAligner started (/scan_raw → /scan)')

    def scan_cb(self, msg: LaserScan):
        scan = LaserScan()
        scan.header = msg.header
        scan.header.stamp.sec = 0
        scan.header.stamp.nanosec = 0
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = msg.ranges
        scan.intensities = msg.intensities

        self.scan_pub.publish(scan)




def main(args=None):
    rclpy.init(args=args)
    node = ScanStampAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
