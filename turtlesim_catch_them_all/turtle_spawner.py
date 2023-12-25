#!/usr/bin/env python3

from functools import partial
import rclpy
import random
import math
from rclpy.node import Node

from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_name_prefix_ = "turtle_"
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)
        self.spawn_turtle_timer = self.create_timer(1.0, self.spawn_new_turtle)
        self.catch_turtle_server = self.create_service(
            CatchTurtle, "catch_turtle", self.catch_turtle_callback)

    def catch_turtle_callback(self, request, response):
        kill_turtle_client = self.create_client(Kill, "kill")
        while not kill_turtle_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for kill service...")

        kill_request = Kill.Request()

        kill_request.name = request.name
        future = kill_turtle_client.call_async(kill_request)

        future.add_done_callback(
            partial(self.killed_turtle_done_cb, turtle_name=request.name))

        response.success = True
        return response

    def killed_turtle_done_cb(self, future, turtle_name):
        try:
            future.result()
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Failed to call kill service..")

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        x = random.uniform(0.5, 10.5)
        y = random.uniform(0.5, 10.5)
        theta = random.uniform(-2*math.pi, 2*math.pi)

        self.call_spawn_turtle_server(name, x, y, theta)

    def call_spawn_turtle_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server..")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(
            self.done_spawn_callback, x=x, y=y, theta=theta))

    def done_spawn_callback(self, future, x, y, theta):
        try:
            response = future.result()
            if response.name is not None:
                new_turtle = Turtle()
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                new_turtle.name = response.name
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()

                self.get_logger().info("Spawned Turtle: " + response.name +
                                       " at: (" + "%.2f" % x + ", " + "%.2f" % y + ", " + "%.2f" % theta + ").")
        except Exception as e:
            self.get_logger().error("Service call failed: " + str(e))


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
