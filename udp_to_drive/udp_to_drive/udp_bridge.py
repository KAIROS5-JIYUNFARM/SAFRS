import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import String

UDP_IP = "0.0.0.0"
UDP_PORT = 5006   # PC에서 보내는 port


class UDPToROS2Bridge(Node):

    def __init__(self):
        super().__init__("udp_to_ros2_bridge")

        # Publisher
        self.pub = self.create_publisher(String, "/start_drive", 10)

        # UDP 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        self.get_logger().info(f"UDP receiver ready on {UDP_PORT}")

        # 타이머 콜백 (10ms)
        self.timer = self.create_timer(0.01, self.read_udp)


    def read_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().strip()

            self.get_logger().info(f"[UDP] Received: {msg}")

            if msg == "START":
                ros_msg = String()
                ros_msg.data = "start"
                self.pub.publish(ros_msg)
                self.get_logger().info("[ROS2] Published '/start_drive'")

        except BlockingIOError:
            # 수신 데이터 없음
            pass



def main(args=None):
    rclpy.init(args=args)
    node = UDPToROS2Bridge()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
