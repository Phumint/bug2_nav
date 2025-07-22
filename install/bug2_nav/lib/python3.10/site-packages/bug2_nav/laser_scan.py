#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarZoneDetector(Node):
    def __init__(self):
        super().__init__('lidar_zone_detector')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distance_threshold = 1.0  # meters
        self.get_logger().info("LidarZoneDetector started!")

    def scan_callback(self, msg: LaserScan):
        # Clear zone flags
        zone_flags = {
            'front': False,
            'left': False,
            'right': False,
            'back': False,
        }

        for i, r in enumerate(msg.ranges):
            # Skip invalid readings
            if math.isinf(r) or math.isnan(r):
                continue

            # Calculate angle in degrees
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)

            # Normalize angle to [0, 360)
            angle = angle % 360

            # Classify based on angle
            if (330 <= angle < 360) or (0 <= angle <= 30):
                if r < self.distance_threshold:
                    zone_flags['front'] = True
            elif 30 < angle <= 150:
                if r < self.distance_threshold:
                    zone_flags['left'] = True
            elif 210 <= angle < 330:
                if r < self.distance_threshold:
                    zone_flags['right'] = True
            elif 150 < angle < 210:
                if r < self.distance_threshold:
                    zone_flags['back'] = True

        # Print detected zones
        detected_zones = [zone for zone, detected in zone_flags.items() if detected]
        if detected_zones:
            self.get_logger().info(f"Obstacle detected in zone(s): {', '.join(detected_zones)}")
        else:
            self.get_logger().info("No obstacle detected within threshold.")


def main(args=None):
    rclpy.init(args=args)
    node = LidarZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
