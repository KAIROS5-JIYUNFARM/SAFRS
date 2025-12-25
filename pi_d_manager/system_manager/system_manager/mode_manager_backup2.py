#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from system_interfaces.msg import SystemMode, EndSignal, StartSignal, Detect

class ModeManager(Node):
    def __init__(self):
        super().__init__('mode_manager')

        # =========================
        # State
        # =========================
        self.current_mode = 'BOOT'

        # 탐지 카운터 (3회 탐지 논리 구현용)
        self.enemy_detect_count = 0
        self.ally_detect_count = 0

        # 타이머 기준 시각들
        self.last_cmd_vel_time = self.get_clock().now()  # NAVI watchdog
        self.last_end_time = None                        # TRACK watchdog
        self.standby_enter_time = None                   # STANDBY idle watchdog

        # =========================
        # Publishers
        # =========================
        self.start_pub = self.create_publisher(StartSignal, '/system_start', 10)
        self.mode_pub = self.create_publisher(SystemMode, '/mode', 10)

        # =========================
        # Subscribers
        # =========================
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.create_subscription(EndSignal, '/end', self.cb_end, 10)
        # ★ 핵심 수정: 카메라의 탐지 신호를 직접 구독하여 모드 전환 트리거로 사용
        self.create_subscription(Detect, '/detect', self.cb_detect, 10)

        # =========================
        # Timer (10 Hz FSM update)
        # =========================
        self.timer = self.create_timer(0.1, self.update)

        # =========================
        # SYSTEM START (1회)
        # =========================
        start_msg = StartSignal()
        start_msg.start = True
        self.start_pub.publish(start_msg)
        self.get_logger().info('== SYSTEM START (BOOT) ==')

        # BOOT → NAVI (2.5s)
        self.startup_timer = self.create_timer(2.5, self.boot_to_navi)

    # =========================
    # Callbacks
    # =========================
    def boot_to_navi(self):
        self.startup_timer.cancel()
        self.publish_mode('NAVI')
        self.get_logger().info('BOOT complete → NAVI')

    def cb_detect(self, msg: Detect):
        """
        카메라 파이에서 보내는 실시간 탐지 신호를 바탕으로 모드 전환
        (STANDBY나 NAVI 상태에서만 작동)
        """
        if self.current_mode not in ('STANDBY', 'NAVI'):
            return

        label = msg.label.strip().upper()

        if label == 'ENEMY':
            self.enemy_detect_count += 1
            self.ally_detect_count = 0
            if self.enemy_detect_count >= 3:
                self.get_logger().warn('적(ENEMY) 3회 탐지 완료! → TRACK_ENEMY 전환')
                self.publish_mode('TRACK_ENEMY')
                self.send_trigger()
                self.enemy_detect_count = 0

        elif label == 'ALLY':
            self.ally_detect_count += 1
            self.enemy_detect_count = 0
            if self.ally_detect_count >= 3:
                self.get_logger().info('아군(ALLY) 3회 탐지 완료! → TRACK_ALLY 전환')
                self.publish_mode('TRACK_ALLY')
                self.send_pass()
                self.ally_detect_count = 0
        else:
            # 탐지 대상이 없거나 불분명할 경우 카운트 초기화
            self.enemy_detect_count = 0
            self.ally_detect_count = 0

    def cb_cmd_vel(self, msg: Twist):
        now = self.get_clock().now()
        lin_th = 0.01
        ang_th = 0.01

        if (
            abs(msg.linear.x) > lin_th or
            abs(msg.linear.y) > lin_th or
            abs(msg.linear.z) > lin_th or
            abs(msg.angular.x) > ang_th or
            abs(msg.angular.y) > ang_th or
            abs(msg.angular.z) > ang_th
        ):
            self.last_cmd_vel_time = now

    def cb_end(self, msg: EndSignal):
        """
        추적 중 조준 완료 또는 상태 유지 확인 (Watchdog 갱신용)
        """
        now = self.get_clock().now()
        self.last_end_time = now
        # 이미 TRACK 모드인 상태에서 들어오는 신호이므로 로그만 남김
        self.get_logger().debug(f'TRACK Signal Received: {msg.type}')

    # =========================
    # Main FSM Update Loop
    # =========================
    def update(self):
        now = self.get_clock().now()

        # 1) NAVI → STANDBY (cmd_vel timeout 15s)
        if self.current_mode == 'NAVI':
            if (now - self.last_cmd_vel_time).nanoseconds > 15e9:
                self.get_logger().warn('cmd_vel timeout → STANDBY')
                self.publish_mode('STANDBY')

        # 2) TRACK_* → STANDBY (end timeout 5s)
        if self.current_mode in ('TRACK_ALLY', 'TRACK_ENEMY'):
            if self.last_end_time is not None:
                if (now - self.last_end_time).nanoseconds > 5e9:
                    self.get_logger().info('TRACK timeout → STANDBY')
                    self.publish_mode('STANDBY')

        # 3) STANDBY → NAVI (idle 5s)
        if self.current_mode == 'STANDBY':
            if self.standby_enter_time is not None:
                if (now - self.standby_enter_time).nanoseconds > 5e9:
                    self.get_logger().info('STANDBY idle → NAVI')
                    self.publish_mode('NAVI')

    # =========================
    # Utils
    # =========================
    def publish_mode(self, mode: str):
        if mode == self.current_mode:
            return

        now = self.get_clock().now()

        if mode == 'NAVI':
            self.last_cmd_vel_time = now
            self.standby_enter_time = None
            self.last_end_time = None
        elif mode == 'STANDBY':
            self.standby_enter_time = now
            self.last_end_time = None
        elif mode in ('TRACK_ALLY', 'TRACK_ENEMY'):
            self.last_end_time = now
            self.standby_enter_time = None

        self.current_mode = mode

        msg = SystemMode()
        msg.mode = mode
        self.mode_pub.publish(msg)

        self.get_logger().info(f'== MODE → {mode} ==')

    # =========================
    # Motor Commands (stub)
    # =========================
    def send_pass(self):
        self.get_logger().info('[MOTOR] PASS')

    def send_trigger(self):
        self.get_logger().warn('[MOTOR] TRIGGER ON')


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
