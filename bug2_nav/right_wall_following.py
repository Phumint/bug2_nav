#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class WallFollower(Node):
    def __init__(self):
        # Initialize the ROS2 node with a name
        super().__init__('turtlebot3_wall_follower')

        # Publisher for robot's velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Robot's current state
        self.state = "FORWARD" # Possible states: "FORWARD", "TURN_TO_WALL", "FOLLOW_WALL", "TURN_CORNER", "SEARCH_FOR_WALL"

        # --- Configuration Parameters ---
        self.linear_speed = 0.15  # Linear speed of the robot (m/s)
        self.angular_speed_turn = 0.3 # Angular speed for turning (rad/s)
        self.angular_speed_corner = 0.5 # Angular speed for sharp turns at corners

        self.search_linear_speed = 0.05 # Linear speed when actively searching for a wall
        self.search_angular_speed = 0.2 # Angular speed when actively searching for a wall (e.g., slow turn)

        self.obstacle_threshold = 0.5 # Distance (m) to consider an obstacle in front
        self.desired_wall_distance = 0.7 # Desired distance (m) from the wall to maintain

        # Proportional gain for wall following control (Kp)
        self.kp_angular = 1.5

        # Threshold for detecting an "open space" to trigger wall search
        self.open_space_threshold = 3.0 # meters, if front and right are both beyond this, search for wall

        # Define the angles for laser scan regions (in degrees relative to robot front)
        self.region_angles = {
            "front": (0, 15),
            "right_front": (300, 330),
            "right_mid": (260, 280),
            "right_rear": (240, 255),
            "left_front": (30, 60),
            "left_mid": (80, 100),
            "left_rear": (105, 120),
        }

        self.get_logger().info("TurtleBot3 Wall Follower Node Initialized (ROS2).")

    def get_min_distance_in_region(self, ranges, start_angle_deg, end_angle_deg):
        """
        Calculates the minimum distance in a specified angular region from the laser scan data.
        Handles 'inf' values by treating them as a very large distance.
        """
        # Convert angles to indices. Assuming 360 readings for 360 degrees.
        # LaserScan ranges are typically ordered from 0 to 359 degrees (front is 0).
        
        # Adjust angles to be within 0-359 range
        start_idx = int(start_angle_deg) % 360
        end_idx = int(end_angle_deg) % 360

        region_ranges = []
        if start_idx <= end_idx:
            # Normal case: region does not cross the 0/359 boundary
            region_ranges = ranges[start_idx:end_idx + 1]
        else:
            # Region crosses the 0/359 boundary (e.g., front region 345-15)
            region_ranges = ranges[start_idx:] + ranges[:end_idx + 1]
        
        # Filter out 'inf' values and replace with a large number for min calculation
        filtered_ranges = [r if not math.isinf(r) else 100.0 for r in region_ranges]
        
        if not filtered_ranges:
            return 100.0 # Return a large value if no valid readings in region

        return min(filtered_ranges)

    def scan_callback(self, msg):
        """
        Callback function for the /scan topic.
        Processes laser scan data and controls robot movement based on the current state.
        """
        twist = Twist() # Initialize Twist message for publishing velocity

        # Get minimum distances in defined regions
        front_dist = min(
            self.get_min_distance_in_region(msg.ranges, 0, 15),
            self.get_min_distance_in_region(msg.ranges, 345, 359)
        )
        right_front_dist = self.get_min_distance_in_region(msg.ranges, 300, 330)
        right_mid_dist = self.get_min_distance_in_region(msg.ranges, 260, 280)
        
        valid_right_distances = [d for d in [right_front_dist, right_mid_dist] if d < 100.0]
        if valid_right_distances:
            current_wall_distance = sum(valid_right_distances) / len(valid_right_distances)
        else:
            current_wall_distance = 100.0 # No valid wall detected on the right

        self.get_logger().info(f"State: {self.state}, Front: {front_dist:.2f}m, Right Wall: {current_wall_distance:.2f}m")

        # --- State Machine Logic ---
        if self.state == "FORWARD":
            # Move straight until an obstacle is detected in front
            if front_dist < self.obstacle_threshold:
                self.get_logger().info("Obstacle detected in front. Changing state to TURN_TO_WALL.")
                self.state = "TURN_TO_WALL"
            elif current_wall_distance > self.open_space_threshold and front_dist > self.open_space_threshold:
                # If in a very open space, transition to search for a wall
                self.get_logger().info("In open space, searching for wall. Changing state to SEARCH_FOR_WALL.")
                self.state = "SEARCH_FOR_WALL"
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0

        elif self.state == "SEARCH_FOR_WALL":
            # Move slowly and turn to find a wall
            twist.linear.x = self.search_linear_speed
            # Turn right to sweep for a wall on the right side
            twist.angular.z = -self.search_angular_speed 

            # If a wall is detected on the right, transition to align with it
            if current_wall_distance < (self.desired_wall_distance * 2.0): # Wall found
                self.get_logger().info("Wall detected during search. Changing state to TURN_TO_WALL.")
                self.state = "TURN_TO_WALL"
            # If an obstacle appears in front while searching, avoid it
            elif front_dist < self.obstacle_threshold:
                self.get_logger().info("Obstacle in front during search. Changing state to TURN_TO_WALL.")
                self.state = "TURN_TO_WALL"


        elif self.state == "TURN_TO_WALL":
            # Turn left to align with the right wall or avoid the obstacle
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed_turn # Turn left
            
            # If the front is clear enough and a wall is detected on the right
            if front_dist > (self.obstacle_threshold * 1.5) and current_wall_distance < (self.desired_wall_distance * 1.8):
                self.get_logger().info("Front clear, wall detected on right. Changing state to FOLLOW_WALL.")
                self.state = "FOLLOW_WALL"
            # If still blocked in front after turning a bit, keep turning
            elif front_dist < self.obstacle_threshold:
                self.get_logger().info("Still blocked in front, continuing to turn.")
                # Stay in TURN_TO_WALL state
            else:
                # If front is clear but no wall on right (e.g., turned too much), go back to searching
                self.get_logger().info("Front clear but no wall on right. Changing state to SEARCH_FOR_WALL.")
                self.state = "SEARCH_FOR_WALL"


        elif self.state == "FOLLOW_WALL":
            # Check for sudden obstacles in front while wall-following
            if front_dist < self.obstacle_threshold:
                self.get_logger().info("Obstacle directly in front while wall-following. Changing state to TURN_TO_WALL.")
                self.state = "TURN_TO_WALL"
                
            # Check for corners or end of wall (large gap on the right)
            elif current_wall_distance > (self.desired_wall_distance * 3.0): # If wall disappears significantly
                self.get_logger().info("Wall lost on right (possible corner/gap). Changing state to TURN_CORNER.")
                self.state = "TURN_CORNER"
                
            else:
                # Proportional control for wall following
                error = self.desired_wall_distance - current_wall_distance
                twist.linear.x = self.linear_speed
                twist.angular.z = self.kp_angular * error
                self.get_logger().info(f"Following wall. Error: {error:.2f}, Angular Z: {twist.angular.z:.2f}")

        elif self.state == "TURN_CORNER":
            # Primarily turn right in place to find the next wall segment
            twist.linear.x = 0.01 # Very small forward speed to prevent getting stuck, effectively turning in place
            twist.angular.z = -self.angular_speed_corner # Turn right more aggressively
            
            # Once a wall is re-detected on the right and front is clear, resume following
            if current_wall_distance < (self.desired_wall_distance * 1.8) and front_dist > self.obstacle_threshold:
                self.get_logger().info("Wall re-detected after corner. Changing state to FOLLOW_WALL.")
                self.state = "FOLLOW_WALL"
            # If it turns too much and hits something in front, go back to turn_to_wall
            elif front_dist < self.obstacle_threshold:
                self.get_logger().info("Hit obstacle while turning corner. Changing state to TURN_TO_WALL.")
                self.state = "TURN_TO_WALL"
            # If after turning, it's still in an open space, go to search for wall
            elif current_wall_distance > self.open_space_threshold:
                self.get_logger().info("Still in open space after corner turn. Changing state to SEARCH_FOR_WALL.")
                self.state = "SEARCH_FOR_WALL"


        # Publish the calculated velocity commands
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 client library
    node = WallFollower() # Create the node
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        pass # Handle Ctrl+C
    finally:
        node.destroy_node() # Destroy the node
        rclpy.shutdown() # Shutdown ROS2 client library

if __name__ == '__main__':
    main()

