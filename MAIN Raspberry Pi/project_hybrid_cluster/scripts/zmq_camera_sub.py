#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import zmq, base64, numpy as np, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Camera RPi의 IP 주소
CAMERA_PI_IP = "172.30.1.5"

class CameraZMQ(Node):
    """
    CameraZMQ
    - Camera RPi → ZMQ로 전송되는 base64 JPEG 이미지 수신
    - OpenCV 프레임으로 디코딩 후 ROS2 /camera/image_raw 로 퍼블리시
    """

    def __init__(self):
        super().__init__("camera_zmq_sub")

        # OpenCV ↔ ROS 이미지 변환용 브리지
        self.bridge = CvBridge()

        # ROS2 퍼블리셔 생성
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)

        # ZMQ Subscriber 설정
        ctx = zmq.Context()
        self.sub = ctx.socket(zmq.SUB)

        # Camera Pi의 7000번 포트에 SUB 연결
        self.sub.connect(f"tcp://{CAMERA_PI_IP}:7000")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # 100Hz로 receive() 호출
        self.create_timer(0.01, self.receive)

    def receive(self):
        """ ZMQ 데이터 수신 → base64 decode → ROS Image 퍼블리시 """
        try:
            msg = self.sub.recv_json(flags=zmq.NOBLOCK)
        except:
            return

        # base64 디코딩
        jpg = base64.b64decode(msg["img"])
        arr = np.frombuffer(jpg, dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn("Frame decode failed")
            return

        # OpenCV → ROS Image 변환
        ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        ros_img.header.frame_id = "camera_link"
        self.pub.publish(ros_img)


def main():
    rclpy.init()
    node = CameraZMQ()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
