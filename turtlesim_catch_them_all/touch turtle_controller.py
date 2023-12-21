#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = None
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtle_pose_cb, 10)

    def turtle_pose_cb(self, msg):
        self.pose_ = msg


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
