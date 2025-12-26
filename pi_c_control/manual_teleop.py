# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, tty, termios, select

BANNER = """
---------------------------
Control Your Robot!
---------------------------
 Moving around:
 w : forward
 a : turn right
 s : backward
 d : turn left
 u : increase speed (+5)
 p : decrease speed (-5)
---------------------------
CTRL+C to quit
"""

class ManualTeleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.65
        self.settings = termios.tcgetattr(sys.stdin)
        print(BANNER)
        self.print_speed()

    def print_speed(self):
        sys.stdout.write(f"\rCurrent Speed: {self.speed:.2f}  ")
        sys.stdout.flush()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                tw = Twist()

                if key == 'w':
                    tw.linear.x = self.speed
                elif key == 's':
                    tw.linear.x = -self.speed
                elif key == 'a':
                    tw.angular.z = self.speed * 2.5
                elif key == 'd':
                    tw.angular.z = -self.speed * 2.5
                elif key == 'u':
                    self.speed = min(1.0, self.speed + 0.05)
                    self.print_speed()
                    continue
                elif key == 'p':
                    self.speed = max(0.1, self.speed - 0.05)
                    self.print_speed()
                    continue
                elif key == 'x':
                    tw.linear.x = 0.0
                    tw.angular.z = 0.0
                elif key == '\x03': # CTRL+C
                    break

                self.pub.publish(tw)
                
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = ManualTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()