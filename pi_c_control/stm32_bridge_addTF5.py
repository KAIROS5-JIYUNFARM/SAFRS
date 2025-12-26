#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import serial
import math


class Stm32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')

        # =====================
        # Serial
        # =====================
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("Serial Port Connected!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect serial: {e}")
            raise

        # =====================
        # ROS interfaces
        # =====================
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.serial_timer = self.create_timer(0.01, self.read_serial)

        # =====================
        # Robot parameters
        # =====================
        self.R = 0.040   # wheel radius [m]
        self.L = 0.290   # wheel base  [m]

        self.PWM_MIN, self.PWM_MAX = 70, 100
        self.RPM_MIN, self.RPM_MAX = 10, 330

        # =====================
        # ODOM state
        # =====================
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.wz = 0.0


    # =====================
    # CMD_VEL → Motor
    # =====================
    def rpm_to_pwm(self, rpm):
        abs_rpm = abs(rpm)
        if abs_rpm < self.RPM_MIN:
            return 0
        pwm = self.PWM_MIN + (abs_rpm - self.RPM_MIN) * \
              (self.PWM_MAX - self.PWM_MIN) / (self.RPM_MAX - self.RPM_MIN)
        return int(min(pwm, self.PWM_MAX))


    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z * (1.2 if abs(vx) > 0.1 else 3.0)

        v_l = vx - wz * self.L / 2.0
        v_r = vx + wz * self.L / 2.0

        rpm_l = v_l / (2 * math.pi * self.R) * 60.0
        rpm_r = v_r / (2 * math.pi * self.R) * 60.0

        p_l = self.rpm_to_pwm(rpm_l) * (-1 if rpm_l < 0 else 1)
        p_r = self.rpm_to_pwm(rpm_r) * (-1 if rpm_r < 0 else 1)

        self.ser.write(f"M {p_l} {p_l} {p_r} {p_r}\n".encode())


    # =====================
    # Serial → ODOM_RAW
    # =====================
    def read_serial(self):
        if self.ser.in_waiting <= 0:
            return

        line = self.ser.readline().decode(errors='ignore').strip()
        if not line.startswith("ODOM"):
            return

        parts = line.split(',')
        if len(parts) < 6:
            return

        _, x, y, th, vx, wz = parts

        self.x  = float(x)
        self.y  = float(y)
        self.th = float(th)
        self.vx = float(vx)
        self.wz = float(wz)

        stamp = self.get_clock().now().to_msg()

        # -------- ODOM_RAW --------
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        # pose covariance (x, y, yaw)
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.05 ** 2
        pose_cov[7]  = 0.05 ** 2
        pose_cov[35] = 0.2  ** 2
        odom.pose.covariance = pose_cov

        # twist covariance (vx, wz)
        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.1 ** 2
        twist_cov[35] = 0.3 ** 2
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    rclpy.spin(Stm32Bridge())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    