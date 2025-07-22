#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class Bug1Navigator(Node):
    def __init__(self):
        # Initialize the ROS2 node
        super().__init__('turtlebot3_bug1_navigator')

        # --- Publishers and Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- Robot State Variables ---
        self.state = "GO_TO_GOAL" # Main states: "GO_TO_GOAL", "BUG_FOLLOW"
        # Sub-state for BUG_FOLLOW, managing obstacle circumvention
        self.bug_sub_state = "BUG_FOLLOW_INIT" # Possible: "BUG_FOLLOW_INIT", "BUG_FOLLOW_ALIGN", "BUG_FOLLOW_WALL", "BUG_FOLLOW_CORNER", "BUG_FOLLOW_RETURN_TO_LEAVE"

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0 # Orientation in radians
        self.last_robot_x = 0.0 # For path length calculation
        self.last_robot_y = 0.0 # For path length calculation

        # --- Goal Position (Fixed) ---
        self.goal_x = -3.0
        self.goal_y = 0.5

        # --- Bug1 Specific Variables ---
        self.hit_point_x = 0.0
        self.hit_point_y = 0.0
        self.min_dist_to_goal_on_boundary = float('inf') # Qm's distance to goal
        self.leave_point_x = 0.0 # Qm's x-coordinate (R1)
        self.leave_point_y = 0.0 # Qm's y-coordinate (R1)
        self.circumference_complete = False # Flag for full loop around obstacle
        self.initial_hit_dist_to_goal = float('inf') # Distance from hit point to goal

        # Path length registers (R2 and R3)
        self.path_length_hi_to_qm = 0.0 # R2: Length from Hi to Qm
        self.current_boundary_path_length = 0.0 # Accumulates total path length while following boundary
        self.path_length_qm_to_hi = 0.0 # R3: Length from Qm back to Hi (after full loop)

        # --- Configuration Parameters ---
        self.linear_speed_go_to_goal = 0.5  # Linear speed when moving towards goal (m/s)
        self.angular_speed_go_to_goal = 0.7 # Angular adjustment speed for goal seeking (rad/s)

        # Parameters for BUG_FOLLOW sub-states (ADJUSTED TO MATCH BUG0 REFERENCE)
        self.linear_speed_bug_follow = 0.15  # Linear speed when following boundary (m/s) - From Bug0 linear_speed
        self.angular_speed_bug_align = 0.4 # Angular speed for aligning with obstacle (Increased)
        self.angular_speed_bug_corner = 0.5 # Angular speed for sharp turns at corners - From Bug0 angular_speed_corner

        self.obstacle_detection_distance = 0.5 # Distance (m) to consider an obstacle in front for GO_TO_GOAL (Increased)
        self.wall_follow_distance = 0.6 # Desired distance (m) from the wall to maintain

        self.kp_angular_goal_seek = 1.0 # Proportional gain for angular control in GO_TO_GOAL
        self.kp_angular_wall_follow = 1.5 # Proportional gain for wall following control

        # Thresholds
        self.goal_reach_threshold = 0.2 # Distance to goal to consider it reached
        self.hit_point_return_threshold = 0.15 # Distance to hit point to consider full circumvention (Stricter)
        self.clear_path_threshold = 0.8 # If min distance towards goal is greater than this, path is clear
        self.min_move_away_from_hit_point = 1.0 # Robot must move this far from hit point before checking return (Increased)

        # Define the angles for laser scan regions (in degrees relative to robot front)
        # For clockwise circumvention, we keep the obstacle on the LEFT.
        self.region_angles = {
            "front": (0, 15), # 0 degrees and +/- 15 degrees
            "left_front": (30, 60),
            "left_mid": (80, 100),
            "left_rear": (105, 120),
            "right_front": (300, 330), # Used for general obstacle detection/clearing, not primary wall follow
        }

        self.get_logger().info("Bug1 Navigator Node Initialized (ROS2).")
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
        # Store previous position for path length calculation
        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y

        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.robot_yaw = yaw # Yaw is the rotation around Z-axis

    def scan_callback(self, msg):
        """
        Callback function for the /scan topic. Implements the Bug1 algorithm logic.
        """
        twist = Twist() # Initialize Twist message for publishing velocity

        # Get minimum distances in defined regions
        front_dist = min(
            self.get_min_distance_in_region(msg.ranges, 0, 30), # Wider front view
            self.get_min_distance_in_region(msg.ranges, 330, 359)
        )
        left_front_dist = self.get_min_distance_in_region(msg.ranges, 30, 60)
        left_mid_dist = self.get_min_distance_in_region(msg.ranges, 80, 100)
        
        # Calculate average left wall distance for wall following (obstacle on LEFT)
        valid_left_distances = [d for d in [left_front_dist, left_mid_dist] if d < 100.0]
        if valid_left_distances:
            current_wall_distance = sum(valid_left_distances) / len(valid_left_distances)
        else:
            current_wall_distance = 100.0 # No valid wall detected on the left

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
            f"Left Wall Dist: {current_wall_distance:.2f}m"
        )

        # --- Bug1 State Machine Logic ---
        if distance_to_goal < self.goal_reach_threshold: # If close enough to goal, stop
            self.get_logger().info("Goal Reached! Stopping algorithm.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return # Stop processing and return

        if self.state == "GO_TO_GOAL":
            # Step 1: Move toward Target along a straight line
            if front_dist < self.obstacle_detection_distance:
                self.get_logger().info("Obstacle encountered. Defining Hit Point Hi and proceeding to Step 2.")
                self.state = "BUG_FOLLOW"
                self.bug_sub_state = "BUG_FOLLOW_INIT" # Initialize sub-state for Bug1
                self.hit_point_x = self.robot_x # Record hit point Hi
                self.hit_point_y = self.robot_y
                self.initial_hit_dist_to_goal = distance_to_goal # Record distance to goal at hit point
                
                # Initialize Qm (leave point) and path lengths
                self.min_dist_to_goal_on_boundary = distance_to_goal 
                self.leave_point_x = self.robot_x 
                self.leave_point_y = self.robot_y
                self.circumference_complete = False
                self.current_boundary_path_length = 0.0 # Reset path length for new circumvention
                self.path_length_hi_to_qm = 0.0
                self.path_length_qm_to_hi = 0.0
                self.get_logger().info(f"Hit point recorded (Hi): ({self.hit_point_x:.2f}, {self.hit_point_y:.2f})")
                self.get_logger().info(f"Initial Qm (Li) set to Hi: ({self.leave_point_x:.2f}, {self.leave_point_y:.2f})")
            else:
                twist.linear.x = self.linear_speed_go_to_goal
                twist.angular.z = self.kp_angular_goal_seek * angle_error_to_goal

        elif self.state == "BUG_FOLLOW":
            # Step 2: Follow the obstacle boundary
            
            # Calculate distance from current position to hit point
            dist_to_hit_point = math.sqrt((self.robot_x - self.hit_point_x)**2 + (self.robot_y - self.hit_point_y)**2)

            # Accumulate path length (approximated)
            distance_traveled_since_last_scan = math.sqrt((self.robot_x - self.last_robot_x)**2 + (self.robot_y - self.last_robot_y)**2)
            self.current_boundary_path_length += distance_traveled_since_last_scan

            # Log circumference status for debugging
            self.get_logger().info(f"Current Path Length: {self.current_boundary_path_length:.2f}m, Dist to Hi: {dist_to_hit_point:.2f}m, Min Move Away: {self.min_move_away_from_hit_point:.2f}m, Circumference Complete: {self.circumference_complete}")

            # Check for circumference completion (return to Hi after moving away)
            # This is part of Step 2(b) - traverse boundary and return to Hi
            if not self.circumference_complete and \
               dist_to_hit_point < self.hit_point_return_threshold and \
               self.current_boundary_path_length > self.min_move_away_from_hit_point: # Ensure it moved away first
                
                self.get_logger().info("Circumference completed! Preparing to return to leave point (Qm).")
                self.circumference_complete = True
                self.path_length_qm_to_hi = self.current_boundary_path_length - self.path_length_hi_to_qm # Calculate R3
                self.get_logger().info(f"R2 (Hi to Qm): {self.path_length_hi_to_qm:.2f}m, R3 (Qm to Hi): {self.path_length_qm_to_hi:.2f}m")
                self.bug_sub_state = "BUG_RETURN_TO_LEAVE_POINT" # Proceed to Step 3 (simplified)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist) # Stop before changing state
                return # Exit scan_callback to avoid further processing in current cycle

            # --- BUG_FOLLOW Sub-state Logic ---
            if self.bug_sub_state == "BUG_FOLLOW_INIT":
                # Initial alignment to start following the obstacle boundary (clockwise)
                # Turn right to put obstacle on left
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed_bug_align # Turn right
                
                # Transition to actual wall following once aligned or hit obstacle
                if front_dist < self.obstacle_detection_distance or current_wall_distance < (self.wall_follow_distance * 1.5):
                    self.get_logger().info("Aligned with obstacle. Starting BUG_FOLLOW_ALIGN.")
                    self.bug_sub_state = "BUG_FOLLOW_ALIGN"
                
            elif self.bug_sub_state == "BUG_FOLLOW_ALIGN":
                # Prioritize clearing the front if an obstacle is very close
                if front_dist < self.obstacle_detection_distance:
                    self.get_logger().info("Obstacle in front during align. Turning left to clear.")
                    twist.linear.x = 0.0 # Stop linear motion
                    twist.angular.z = self.angular_speed_bug_align # Turn left (away from obstacle for left-hand wall following)
                # If front is clear, try to align with the left wall
                elif current_wall_distance < (self.wall_follow_distance * 1.8): # Wall detected on left
                    self.get_logger().info("Front clear, wall detected on left. Changing sub-state to BUG_FOLLOW_WALL.")
                    self.bug_sub_state = "BUG_FOLLOW_WALL"
                    twist.linear.x = 0.0 # Stop for state transition
                    twist.angular.z = 0.0
                else:
                    # Front is clear, but no wall on left or wall is too far.
                    # Continue turning right (clockwise) to sweep for the wall.
                    self.get_logger().info("Front clear but no wall on left. Continuing to turn right to find wall (BUG_FOLLOW_ALIGN).")
                    twist.linear.x = self.linear_speed_bug_follow / 2.0
                    twist.angular.z = -self.angular_speed_bug_align / 2.0 # Small right turn to find wall
                    
            elif self.bug_sub_state == "BUG_FOLLOW_WALL":
                # Update Qm (leave point) if current distance to goal is minimal
                if distance_to_goal < self.min_dist_to_goal_on_boundary:
                    self.min_dist_to_goal_on_boundary = distance_to_goal
                    self.leave_point_x = self.robot_x
                    self.leave_point_y = self.robot_y
                    self.path_length_hi_to_qm = self.current_boundary_path_length # R2
                    self.get_logger().info(f"New Qm (Li) recorded: ({self.leave_point_x:.2f}, {self.leave_point_y:.2f}), Min Dist to Goal: {self.min_dist_to_goal_on_boundary:.2f}m, R2: {self.path_length_hi_to_qm:.2f}m")

                # Check for sudden obstacles in front while wall-following
                if front_dist < self.obstacle_detection_distance:
                    self.get_logger().info("Obstacle directly in front while wall-following. Changing sub-state to BUG_FOLLOW_ALIGN.")
                    self.bug_sub_state = "BUG_FOLLOW_ALIGN"
                    
                # Check for corners or end of wall (large gap on the left)
                elif (current_wall_distance > (self.wall_follow_distance * 3.0) or \
                     left_front_dist > (self.wall_follow_distance * 2.0)) and \
                     front_dist > self.obstacle_detection_distance: # Combined condition for earlier detection
                    self.get_logger().info("Wall lost on left (possible corner/gap). Changing sub-state to BUG_FOLLOW_CORNER.")
                    self.bug_sub_state = "BUG_FOLLOW_CORNER"
                    
                else:
                    # Proportional control for wall following (keep wall_follow_distance on LEFT)
                    error = current_wall_distance - self.wall_follow_distance # Error for left wall
                    twist.linear.x = self.linear_speed_bug_follow
                    twist.angular.z = -self.kp_angular_wall_follow * error # Negative for clockwise turn
                    self.get_logger().info(f"Following wall. Error: {error:.2f}, Angular Z: {twist.angular.z:.2f}")

            elif self.bug_sub_state == "BUG_FOLLOW_CORNER":
                # Primarily turn left in place to find the next wall segment (keeping obstacle on left)
                twist.linear.x = 0.01 # Reintroduced small forward speed (from Bug0 reference)
                twist.angular.z = self.angular_speed_bug_corner # Turn left more aggressively
                
                # Once a wall is re-detected on the left and front is clear, resume following
                if current_wall_distance < (self.wall_follow_distance * 1.5) and front_dist > self.obstacle_detection_distance: # Consistent with Bug0, tighter re-acquisition
                    self.get_logger().info("Wall re-detected after corner. Changing sub-state to BUG_FOLLOW_WALL.")
                    self.bug_sub_state = "BUG_FOLLOW_WALL"
                # If it turns too much and hits something in front, go back to align
                elif front_dist < self.obstacle_detection_distance:
                    self.get_logger().info("Hit obstacle while turning corner. Changing sub-state to BUG_FOLLOW_ALIGN.")
                    self.bug_sub_state = "BUG_FOLLOW_ALIGN"
                # If after turning, it's still in an open space (no wall on left), try to align again
                elif current_wall_distance > (self.wall_follow_distance * 3.0): # Consistent with Bug0 open_space_threshold
                    self.get_logger().info("Still in open space after corner turn. Changing sub-state to BUG_FOLLOW_ALIGN.")
                    self.bug_sub_state = "BUG_FOLLOW_ALIGN"

            elif self.bug_sub_state == "BUG_RETURN_TO_LEAVE_POINT":
                # Step 3 (simplified): Navigate towards the leave point (Qm)
                # The algorithm specifies "determine a shorter way along the boundary to Li"
                # For this simulation, we simplify to direct navigation to Qm.
                dx_leave = self.leave_point_x - self.robot_x
                dy_leave = self.leave_point_y - self.robot_y
                dist_to_leave_point = math.sqrt(dx_leave**2 + dy_leave**2)
                angle_to_leave_point = math.atan2(dy_leave, dx_leave)
                angle_error_to_leave_point = self.normalize_angle(angle_to_leave_point - self.robot_yaw)

                self.get_logger().info(f"Returning to Qm (Li). Dist: {dist_to_leave_point:.2f}m, Angle Error: {math.degrees(angle_error_to_leave_point):.2f}deg")

                if dist_to_leave_point < self.goal_reach_threshold: # Reached leave point Qm
                    self.get_logger().info("Reached Qm (Li). Switching to GO_TO_GOAL for next iteration (Step 1).")
                    self.state = "GO_TO_GOAL"
                    self.bug_sub_state = "BUG_FOLLOW_INIT" # Reset sub-state for next encounter
                else:
                    twist.linear.x = self.linear_speed_go_to_goal # Use goal-seeking speed
                    twist.angular.z = self.kp_angular_goal_seek * angle_error_to_leave_point


        # Publish the calculated velocity commands
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 client library
    node = Bug1Navigator() # Create the node
    try:
        rclpy.spin(node) # Keep the node alive
    except KeyboardInterrupt:
        pass # Handle Ctrl+C
    finally:
        node.destroy_node() # Destroy the node
        rclpy.shutdown() # Shutdown ROS2 client library

if __name__ == '__main__':
    main()
