#!/usr/bin/env python3

from functools import partial
import rclpy
import random
import math
from rclpy.node import Node

from turtlesim.srv import Spawn


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.turtle_name_prefix_ = "turtle_"
        self.turtle_counter_ = 0
        self.spwan_turtle_timer = self.create_timer(2.0, self.spawn_new_turtle)

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
            self.done_spawn_callback, turtle_name=turtle_name, x=x, y=y, theta=theta))

    def done_spawn_callback(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name is not None:
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
