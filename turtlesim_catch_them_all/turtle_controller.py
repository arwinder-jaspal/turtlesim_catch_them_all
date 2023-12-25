#!/usr/bin/env python3
import rclpy
import math
from functools import partial
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ = Pose()
        self.turtles_to_catch_ = None
        self.goal_pose_threshold = 0.5
        self.linear_vel_p_gain = 2.0
        self.angular_vel_p_gain = 6.0

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.turtle_pose_cb, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.alive_turtles_cb, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def turtle_pose_cb(self, msg):
        self.pose_ = msg

    def alive_turtles_cb(self, msg):
        if len(msg.turtles):
            self.turtles_to_catch_ = msg.turtles[0]

    def control_loop(self):
        # if current turtle pose is not none, do not send any cmd_vel
        if self.pose_ == None or self.turtles_to_catch_ == None:
            return
        # compute distance to target
        dist_x = self.turtles_to_catch_.x - self.pose_.x
        dist_y = self.turtles_to_catch_.y - self.pose_.y
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
            self.call_catch_turtle_server(self.turtles_to_catch_.name)
            self.turtles_to_catch_ = None

        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for /catch_turtle service..")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.catch_turtle_done_cb, turtle_name=turtle_name))

    def catch_turtle_done_cb(self, future, turtle_name):

        try:
            response = future.result()
            if not response.success:
                self.get_logger().info("Failed to catch " + turtle_name)

        except Exception as e:
            self.get_logger().error("Failed to call catch_turtle service: %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
