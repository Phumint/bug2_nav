#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.state = 'FIND_WALL'  # states: FIND_WALL, TURN_LEFT, FOLLOW_WALL
        self.stop_distance = 0.4  # stop if something is this close in front
        self.desired_wall_distance = 0.5  # maintain this distance from the wall (left side)

        self.forward_speed = 0.15
        self.turn_speed = 0.5
        self.timer = self.create_timer(0.1, self.control_loop)

        self.front_dist = float('inf')
        self.left_dist = float('inf')

    def scan_callback(self, msg: LaserScan):
        def get_range_at_angle(angle_deg):
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            index = max(0, min(index, len(msg.ranges) - 1))
            value = msg.ranges[index]
            return value if not math.isinf(value) and not math.isnan(value) else float('inf')

        self.front_dist = get_range_at_angle(0)
        self.left_dist = get_range_at_angle(90)

    def control_loop(self):
        cmd = Twist()

        if self.state == 'FIND_WALL':
            self.get_logger().info('State: FIND_WALL')
            if self.front_dist < self.stop_distance:
                self.state = 'TURN_LEFT'
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == 'TURN_LEFT':
            self.get_logger().info('State: TURN_LEFT')
            if self.left_dist < self.desired_wall_distance + 0.1:
                self.state = 'FOLLOW_WALL'
            else:
                cmd.angular.z = self.turn_speed

        elif self.state == 'FOLLOW_WALL':
            self.get_logger().info('State: FOLLOW_WALL')
            error = self.desired_wall_distance - self.left_dist
            kp = 1.0
            cmd.linear.x = self.forward_speed
            cmd.angular.z = kp * error  # simple P-control

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
