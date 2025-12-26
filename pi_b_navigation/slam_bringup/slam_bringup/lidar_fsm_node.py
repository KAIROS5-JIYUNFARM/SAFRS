#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from system_interfaces.msg import SystemMode

class LidarFSMNode(Node):
    def __init__(self):
        super().__init__("lidar_fsm_node")

        # ===== FSM State =====
        self.mode = "BOOT"
        self.lidar_on = False

        # ===== ROS =====
        self.sub_mode = self.create_subscription(
            SystemMode,
            "/mode",
            self.cb_mode,
            10
        )

        self.get_logger().info("LiDAR FSM Node ready.")

    # ----------------------------
    # Mode callback
    # ----------------------------
    def cb_mode(self, msg: SystemMode):
        new_mode = msg.mode.strip().upper()
        if new_mode == self.mode:
            return

        self.get_logger().info(f"[MODE] {self.mode} -> {new_mode}")
        self.mode = new_mode

        # FSM decision
        if self.mode == "NAVI":
            self._lidar_on()
        else:
            self._lidar_off()

    # ----------------------------
    # LiDAR control
    # ----------------------------
    def _lidar_on(self):
        if self.lidar_on:
            return

        self.lidar_on = True
        self.get_logger().info("LiDAR ON (start scan / SLAM)")
        # TODO:
        # - start lidar driver
        # - start SLAM node (or enable)
        # - start scan publish

    def _lidar_off(self):
        if not self.lidar_on:
            return

        self.lidar_on = False
        self.get_logger().info("LiDAR OFF (stop scan / idle)")
        # TODO:
        # - stop lidar driver
        # - stop SLAM
        # - cleanup resources

def main():
    rclpy.init()
    node = LidarFSMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

