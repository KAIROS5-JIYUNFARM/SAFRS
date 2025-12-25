#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose, FollowPath
from geometry_msgs.msg import PoseStamped


class SimpleNavClient(Node):
    def __init__(self):
        super().__init__('simple_nav_client')

        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')

        self.timer = self.create_timer(2.0, self.send_goal)
        self.sent = False

    def send_goal(self):
        if self.sent:
            return
        self.sent = True

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 1.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0

        req = ComputePathToPose.Goal()
        req.goal = goal

        self.path_client.wait_for_server()
        future = self.path_client.send_goal_async(req)
        future.add_done_callback(self.path_response)

    def path_response(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path)

    def follow_path(self, future):
        path = future.result().result.path
        req = FollowPath.Goal()
        req.path = path

        self.follow_client.wait_for_server()
        self.follow_client.send_goal_async(req)


def main():
    rclpy.init()
    node = SimpleNavClient()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
