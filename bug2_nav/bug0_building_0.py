#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class Bug0Navigator(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('turtlebot3_bug0_navigator')

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Robot State Variables ---
        self.state = "GO_TO_GOAL" # Main states: "GO_TO_GOAL", "BUG_FOLLOW"
        # Sub-state for BUG_FOLLOW, similar to WallFollower's states
        self.bug_sub_state = "BUG_FOLLOW_ALIGN" # Possible: "BUG_FOLLOW_ALIGN", "BUG_FOLLOW_WALL", "BUG_FOLLOW_CORNER"

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0 # Orientation in radians

        # --- Goal Position (Fixed for this example) ---
        self.goal_x = -3.0
        self.goal_y = 0.5

        # --- Configuration Parameters ---
        self.linear_speed_go_to_goal = 0.5  # Linear speed when moving towards goal (m/s)
        self.angular_speed_go_to_goal = 0.7 # Angular adjustment speed for goal seeking (rad/s)

        # Parameters for BUG_FOLLOW sub-states (adapted from WallFollower)
        self.linear_speed_bug_follow = 0.1  # Linear speed when following boundary (m/s)
        self.angular_speed_bug_align = 0.3 # Angular speed for aligning with obstacle
        self.angular_speed_bug_corner = 0.5 # Angular speed for sharp turns at corners

        self.obstacle_detection_distance = 0.7 # Distance (m) to consider an obstacle in front for GO_TO_GOAL
        self.wall_follow_distance = 1.0 # Desired distance (m) to maintain from the obstacle during BUG_FOLLOW

        self.kp_angular_goal_seek = 1.25 # Proportional gain for angular control in GO_TO_GOAL
        self.kp_angular_wall_follow = 1.5 # Proportional gain for wall following control

        # Threshold for clearing the path to goal
        self.clear_path_threshold = 0.8 # If min distance towards goal is greater than this, path is clear

        # Define the angles for laser scan regions (in degrees relative to robot front)
        self.region_angles = {
            "front": (0, 15), # 0 degrees and +/- 15 degrees
            "right_front": (300, 330), # 300 to 330 degrees (relative to front)
            "right_mid": (260, 280),  # 260 to 280 degrees (around 270 degrees)
            "left_front": (30, 60),
        }

        self.get_logger().info("Bug0 Navigator Node Initialized (ROS2).")
        self.get_logger().info(f"Goal set to: ({self.goal_x}, {self.goal_y})")

    def get_min_distance_in_region(self, ranges, start_angle_deg, end_angle_deg):
        """
        Calculates the minimum distance in a specified angular region from the laser scan data.
        Handles 'inf' values by treating them as a very large distance.
        """
        # Convert angles to indices. Assuming 360 readings for 360 degrees.
        # LaserScan ranges are typically ordered from 0 to 359 degrees (front is 0).
        
        start_idx = int(start_angle_deg) % 360
        end_idx = int(end_angle_deg) % 360

        region_ranges = []
        if start_idx <= end_idx:
            region_ranges = ranges[start_idx:end_idx + 1]
        else:
            # Region crosses the 0/359 boundary (e.g., front region 345-15)
            region_ranges = ranges[start_idx:] + ranges[:end_idx + 1]
        
        # Filter out 'inf' values and replace with a large number for min calculation
        filtered_ranges = [r for r in region_ranges if not math.isinf(r) and r > 0.01] # Filter out 0.0 readings too
        
        if not filtered_ranges:
            return 100.0 # Return a large value if no valid readings in region

        return min(filtered_ranges)

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def odom_callback(self, msg):
        """
        Callback function for the /odom topic. Updates robot's current position and orientation.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_yaw = yaw # Yaw is the rotation around Z-axis

    def scan_callback(self, msg):
        """
        Callback function for the /scan topic. Implements the Bug0 algorithm logic.
        """
        twist = Twist() # Initialize Twist message for publishing velocity

        # Get minimum distances in defined regions
        # For front, consider a wider range to detect obstacles more reliably
        front_dist = min(
            self.get_min_distance_in_region(msg.ranges, 0, 30), # Wider front view
            self.get_min_distance_in_region(msg.ranges, 330, 359)
        )
        right_front_dist = self.get_min_distance_in_region(msg.ranges, 300, 330)
        right_mid_dist = self.get_min_distance_in_region(msg.ranges, 260, 280)
        
        # Calculate average right wall distance for wall following
        valid_right_distances = [d for d in [right_front_dist, right_mid_dist] if d < 100.0]
        if valid_right_distances:
            current_wall_distance = sum(valid_right_distances) / len(valid_right_distances)
        else:
            current_wall_distance = 100.0 # No valid wall detected on the right

        # Calculate distance and angle to goal
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) # Angle of the goal relative to robot's global X-axis

        # Angle error to goal relative to robot's current heading
        angle_error_to_goal = self.normalize_angle(angle_to_goal - self.robot_yaw)

        self.get_logger().info(
            f"State: {self.state}, Sub-state: {self.bug_sub_state if self.state == 'BUG_FOLLOW' else 'N/A'}, "
            f"Dist to Goal: {distance_to_goal:.2f}m, "
            f"Angle Error: {math.degrees(angle_error_to_goal):.2f}deg, "
            f"Front Dist: {front_dist:.2f}m, "
            f"Right Wall Dist: {current_wall_distance:.2f}m"
        )

        # --- Bug0 State Machine Logic ---
        if distance_to_goal < 0.2: # If close enough to goal, stop
            self.get_logger().info("Goal Reached!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return # Stop processing and return

        if self.state == "GO_TO_GOAL":
            # If an obstacle is detected in front, switch to BUG_FOLLOW
            if front_dist < self.obstacle_detection_distance:
                self.get_logger().info("Obstacle detected in GO_TO_GOAL. Switching to BUG_FOLLOW.")
                self.state = "BUG_FOLLOW"
                self.bug_sub_state = "BUG_FOLLOW_ALIGN" # Initialize sub-state for wall alignment
            else:
                # Move towards the goal
                twist.linear.x = self.linear_speed_go_to_goal
                twist.angular.z = self.kp_angular_goal_seek * angle_error_to_goal

        elif self.state == "BUG_FOLLOW":
            # Check if the path to the goal is clear (OVERRIDE for all sub-states)
            robot_centric_angle_to_goal_deg = math.degrees(self.normalize_angle(angle_to_goal - self.robot_yaw))
            cone_half_width_deg = 10 
            cone_start_angle = (robot_centric_angle_to_goal_deg - cone_half_width_deg + 360) % 360
            cone_end_angle = (robot_centric_angle_to_goal_deg + cone_half_width_deg + 360) % 360
            min_dist_towards_goal = self.get_min_distance_in_region(msg.ranges, cone_start_angle, cone_end_angle)
            
            # Condition to switch back to GO_TO_GOAL:
            # Path towards goal is clear AND robot is not currently too close to an obstacle on its right
            # The second condition prevents immediate re-entry into BUG_FOLLOW if it just left an obstacle
            if min_dist_towards_goal > self.clear_path_threshold and current_wall_distance > (self.wall_follow_distance * 1.5):
                self.get_logger().info("Path to goal clear. Switching to GO_TO_GOAL.")
                self.state = "GO_TO_GOAL"
                # Reset sub-state for next BUG_FOLLOW encounter
                self.bug_sub_state = "BUG_FOLLOW_ALIGN" 
            else:
                # If path to goal is NOT clear, execute wall-following sub-states
                if self.bug_sub_state == "BUG_FOLLOW_ALIGN":
                    # Turn left to align with the right wall or avoid the obstacle
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed_bug_align # Turn left
                    
                    # If the front is clear enough and a wall is detected on the right
                    if front_dist > (self.obstacle_detection_distance * 1.5) and current_wall_distance < (self.wall_follow_distance * 1.8):
                        self.get_logger().info("Front clear, wall detected on right. Changing sub-state to BUG_FOLLOW_WALL.")
                        self.bug_sub_state = "BUG_FOLLOW_WALL"
                    # If still blocked in front after turning a bit, keep turning
                    elif front_dist < self.obstacle_detection_distance:
                        self.get_logger().info("Still blocked in front, continuing to align.")
                        # Stay in BUG_FOLLOW_ALIGN sub-state
                    else:
                        # If front is clear but no wall on right (e.g., turned too much), try to find a wall by turning right slightly
                        self.get_logger().info("Front clear but no wall on right. Turning right to find wall.")
                        twist.linear.x = self.linear_speed_bug_follow / 2.0
                        twist.angular.z = -self.angular_speed_bug_align / 2.0 # Small right turn to find wall
                        
                elif self.bug_sub_state == "BUG_FOLLOW_WALL":
                    # Check for sudden obstacles in front while wall-following
                    if front_dist < self.obstacle_detection_distance:
                        self.get_logger().info("Obstacle directly in front while wall-following. Changing sub-state to BUG_FOLLOW_ALIGN.")
                        self.bug_sub_state = "BUG_FOLLOW_ALIGN"
                        
                    # Check for corners or end of wall (large gap on the right)
                    elif current_wall_distance > (self.wall_follow_distance * 3.0): # If wall disappears significantly
                        self.get_logger().info("Wall lost on right (possible corner/gap). Changing sub-state to BUG_FOLLOW_CORNER.")
                        self.bug_sub_state = "BUG_FOLLOW_CORNER"
                        
                    else:
                        # Proportional control for wall following
                        error = self.wall_follow_distance - current_wall_distance
                        twist.linear.x = self.linear_speed_bug_follow
                        twist.angular.z = self.kp_angular_wall_follow * error
                        self.get_logger().info(f"Following wall. Error: {error:.2f}, Angular Z: {twist.angular.z:.2f}")

                elif self.bug_sub_state == "BUG_FOLLOW_CORNER":
                    # Primarily turn right in place to find the next wall segment
                    twist.linear.x = 0.01 # Very small forward speed to prevent getting stuck, effectively turning in place
                    twist.angular.z = -self.angular_speed_bug_corner # Turn right more aggressively
                    
                    # Once a wall is re-detected on the right and front is clear, resume following
                    if current_wall_distance < (self.wall_follow_distance * 1.8) and front_dist > self.obstacle_detection_distance:
                        self.get_logger().info("Wall re-detected after corner. Changing sub-state to BUG_FOLLOW_WALL.")
                        self.bug_sub_state = "BUG_FOLLOW_WALL"
                    # If it turns too much and hits something in front, go back to align
                    elif front_dist < self.obstacle_detection_distance:
                        self.get_logger().info("Hit obstacle while turning corner. Changing sub-state to BUG_FOLLOW_ALIGN.")
                        self.bug_sub_state = "BUG_FOLLOW_ALIGN"
                    # If after turning, it's still in an open space, try to align again
                    elif current_wall_distance > (self.wall_follow_distance * 3.0): # Still no wall after turning
                        self.get_logger().info("Still in open space after corner turn. Changing sub-state to BUG_FOLLOW_ALIGN.")
                        self.bug_sub_state = "BUG_FOLLOW_ALIGN"


        # Publish the calculated velocity commands
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 client library
    node = Bug0Navigator() # Create the node
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        pass # Handle Ctrl+C
    finally:
        node.destroy_node() # Destroy the node
        rclpy.shutdown() # Shutdown ROS2 client library

if __name__ == '__main__':
    main()
