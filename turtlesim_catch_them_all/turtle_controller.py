#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = Pose()
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        # temp variable to test control loop
        self.target_x = 8.0
        self.target_y = 4.0
        self.goal_pose_threshold = 0.5
        self.linear_vel_p_gain = 2.0
        self.angular_vel_p_gain = 6.0

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtle_pose_cb, 10)

    def turtle_pose_cb(self, msg):
        self.pose_ = msg

    def control_loop(self):
        # if current turtle pose is not none, do not send any cmd_vel
        if self.pose_ == None:
            return
        # compute distance to target
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(math.pow(dist_x, 2.0) + math.pow(dist_y, 2.0))

        cmd_vel_msg = Twist()

        if distance > self.goal_pose_threshold:
            cmd_vel_msg.linear.x = distance * self.linear_vel_p_gain
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            cmd_vel_msg.angular.z = diff * self.angular_vel_p_gain
        else:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0 

        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
