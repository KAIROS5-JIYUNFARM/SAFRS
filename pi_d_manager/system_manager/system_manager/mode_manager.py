#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        # =========================
        # State
        # =========================
        self.current_mode = 'BOOT'

        # =========================
        # Time stamps
        # =========================
        now = self.get_clock().now()
        self.last_cmd_vel_time = now
        self.last_detect_time = None
        self.last_track_signal_time = None
        self.last_end_time = None

        # Detect counters (STANDBY only)
        self.enemy_detect_count = 0
        self.ally_detect_count = 0

        # =========================
        # Publishers
        # =========================
        self.start_pub = self.create_publisher(Bool, '/system_start', 10)
        self.mode_pub = self.create_publisher(String, '/mode', 10)

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(String, '/detect', self.cb_detect, 10)
        self.create_subscription(String, '/end', self.cb_end, 10)

        # =========================
        # Timer
        # =========================
        self.timer = self.create_timer(0.1, self.update)

        # =========================
        # SYSTEM START
        # =========================
        start_msg = Bool()
        start_msg.data = True
        self.start_pub.publish(start_msg)
        self.get_logger().info('== SYSTEM START ==')

        self.startup_timer = self.create_timer(2.5, self.boot_to_navi)

    # ======================================================
    # Callbacks
    # ======================================================
    def boot_to_navi(self):
        self.startup_timer.cancel()
        self.publish_mode('NAVI')

    def cb_cmd_vel(self, msg: Twist):
        lin_th = 0.01
        ang_th = 0.01

        if (
            abs(msg.linear.x) > lin_th or
            abs(msg.linear.y) > lin_th or
            abs(msg.angular.z) > ang_th
        ):
            self.last_cmd_vel_time = self.get_clock().now()

    def cb_detect(self, msg: String):
        now = self.get_clock().now()
        self.last_detect_time = now

        raw = (msg.data or '').strip()
        if not raw:
            return

        label = raw.split(',')[0].upper()

        if self.current_mode == 'STANDBY':
            if label == 'ENEMY':
                self.enemy_detect_count += 1
                self.ally_detect_count = 0
                if self.enemy_detect_count >= 3:
                    self.publish_mode('TRACK_ENEMY')
                    self.enemy_detect_count = 0

            elif label == 'ALLY':
                self.ally_detect_count += 1
                self.enemy_detect_count = 0
                if self.ally_detect_count >= 3:
                    self.publish_mode('TRACK_ALLY')
                    self.ally_detect_count = 0
            else:
                self.enemy_detect_count = 0
                self.ally_detect_count = 0

        elif self.current_mode in ('TRACK_ALLY', 'TRACK_ENEMY'):
            self.last_track_signal_time = now

    def cb_end(self, msg: String):
        if self.current_mode not in ('TRACK_ALLY', 'TRACK_ENEMY'):
            return

        if (msg.data or '').strip().lower() != 'end':
            return

        self.last_end_time = self.get_clock().now()

        if self.current_mode == 'TRACK_ALLY':
            self.publish_mode('PASS')
            self.get_logger().info('[MODE] PASS')

        elif self.current_mode == 'TRACK_ENEMY':
            self.publish_mode('TRIGGER_ON')
            self.get_logger().warn('[MODE] TRIGGER ON')

    # ======================================================
    # Watchdog Logic
    # ======================================================
    def update(self):
        now = self.get_clock().now()

        # 1️⃣ NAVI → STANDBY (cmd_vel 8초 timeout)
        if self.current_mode == 'NAVI':
            if (now - self.last_cmd_vel_time).nanoseconds > 8e9:
                self.get_logger().warn('cmd_vel timeout → STANDBY')
                self.publish_mode('STANDBY')

        # 2️⃣ STANDBY → NAVI (detect 8초 없음)
        if self.current_mode == 'STANDBY':
            if self.last_detect_time is not None:
                if (now - self.last_detect_time).nanoseconds > 8e9:
                    self.get_logger().info('STANDBY idle → NAVI')
                    self.publish_mode('NAVI')

        # 3️⃣ TRACK / PASS / TRIGGER_ON → STANDBY (end 기준 5초 유지)
        if self.current_mode in ('TRACK_ALLY', 'TRACK_ENEMY', 'PASS', 'TRIGGER_ON'):
            if self.last_end_time is not None:
                if (now - self.last_end_time).nanoseconds > 5e9:
                    self.get_logger().info('ACTION timeout → STANDBY')
                    self.publish_mode('STANDBY')

    # ======================================================
    # Utils
    # ======================================================
    def publish_mode(self, mode: str):
        if mode == self.current_mode:
            return

        now = self.get_clock().now()
        self.current_mode = mode

        if mode == 'NAVI':
            self.last_cmd_vel_time = now
            self.last_detect_time = None
            self.last_track_signal_time = None
            self.last_end_time = None

        elif mode == 'STANDBY':
            self.last_detect_time = now
            self.last_track_signal_time = None
            self.last_end_time = None

        elif mode in ('TRACK_ALLY', 'TRACK_ENEMY'):
            self.last_track_signal_time = now
            self.last_end_time = None

        elif mode in ('PASS', 'TRIGGER_ON'):
            self.last_end_time = now  # 5초 후 STANDBY 복귀

        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)

        self.get_logger().info(f'== MODE → {mode} ==')


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

