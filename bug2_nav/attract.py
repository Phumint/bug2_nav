import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry # Import Odometry message
import math
from tf_transformations import euler_from_quaternion # For converting quaternion to Euler angles

class PotentialFieldNavigator(Node):
    """
    A ROS2 node that implements a potential field method for navigation
    and obstacle avoidance using known obstacle locations and odometry.

    Subscribes to:
        - /odom (nav_msgs/msg/Odometry): For robot's current pose.

    Publishes to:
        - /cmd_vel (geometry_msgs/msg/Twist): For sending velocity commands to the robot.
    """

    def __init__(self):
        super().__init__('potential_field_navigator')

        # Declare parameters for tuning the potential field
        self.declare_parameter('goal_x', 0.0)  # X-coordinate of the goal
        self.declare_parameter('goal_y', 0.0)  # Y-coordinate of the goal
        self.declare_parameter('attractive_gain', 0.5)  # Gain for attractive force
        self.declare_parameter('repulsive_gain', 1.0)   # Gain for repulsive force
        self.declare_parameter('obstacle_influence_radius', 0.8) # Robot's influence radius for obstacles
        self.declare_parameter('max_linear_velocity', 0.3) # Maximum linear speed
        self.declare_parameter('max_angular_velocity', 0.8) # Maximum angular speed
        self.declare_parameter('goal_threshold', 0.2) # Distance to goal to consider reached

        # Get parameter values
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.attractive_gain = self.get_parameter('attractive_gain').get_parameter_value().double_value
        self.repulsive_gain = self.get_parameter('repulsive_gain').get_parameter_value().double_value
        self.obstacle_influence_radius = self.get_parameter('obstacle_influence_radius').get_parameter_value().double_value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.goal_threshold = self.get_parameter('goal_threshold').get_parameter_value().double_value

        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y})")
        self.get_logger().info(f"Attractive Gain: {self.attractive_gain}")
        self.get_logger().info(f"Repulsive Gain: {self.repulsive_gain}")
        self.get_logger().info(f"Obstacle Influence Radius: {self.obstacle_influence_radius}")

        # Current robot pose, updated by odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0 # Radians

        # Define known obstacles as a list of (x, y, radius) tuples
        # These are world coordinates of circular obstacles
        self.known_obstacles = [
            #left side
            (-2.470090, 1.551270, 0.5),  # Obstacle 1: (x, y, radius)
            (-4.569400, 4.021650, 0.5), # Obstacle 2
            (-1.012044, -4.577737, 0.5),# Obstacle 3
            (-2.331216, 7.346490, 0.5),  # Obstacle 4
            (-6.071654, 7.920873, 0.5), # Obstacle 5
            #right side 
            (3.467945, 1.790815, 0.5),  # Obstacle 1: (x, y, radius)
            (6.586999, 1.299347, 0.5), # Obstacle 2
            (8.752687, 3.075527, 0.5),# Obstacle 3
            (2.293440, 4.930690, 0.5),  # Obstacle 4
            (4.776393, 4.977253, 0.5), # Obstacle 5   
            (2.136877, 8.334687, 0.5),# Obstacle 6
            (4.612366, 7.636077, 0.5),  # Obstacle 7
            (8.071814, 6.527067, 0.5), # Obstacle 8
        ]
        self.get_logger().info(f"Known Obstacles: {self.known_obstacles}")

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber for odometry data
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)
        self.odom_subscription  # prevent unused variable warning

        # Timer to periodically calculate and publish velocity commands
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz control loop

    def odometry_callback(self, msg):
        """
        Callback function for receiving odometry data.
        Updates the robot's current pose (x, y, theta).
        """
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to Euler angles to get yaw (theta)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_theta = yaw
        # self.get_logger().info(f"Current Pose: ({self.current_x:.2f}, {self.current_y:.2f}, {math.degrees(self.current_theta):.2f} deg)")

    def calculate_attractive_force(self):
        """
        Calculates the attractive force towards the goal.
        The force magnitude is proportional to the distance to the goal.
        """
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        if distance_to_goal < self.goal_threshold:
            self.get_logger().info("Goal reached!")
            return 0.0, 0.0 # No force if goal is reached

        # Calculate force components
        fx_attractive = self.attractive_gain * dx
        fy_attractive = self.attractive_gain * dy

        return fx_attractive, fy_attractive

    def calculate_repulsive_force(self):
        """
        Calculates the repulsive force from known obstacles.
        The force magnitude is inversely proportional to the distance to the obstacle's edge.
        """
        fx_repulsive = 0.0
        fy_repulsive = 0.0

        for ox, oy, o_radius in self.known_obstacles:
            # Calculate vector from robot to obstacle center
            dx_obs = ox - self.current_x
            dy_obs = oy - self.current_y
            
            # Distance from robot center to obstacle center
            dist_to_obs_center = math.sqrt(dx_obs**2 + dy_obs**2)

            # Distance from robot's edge to obstacle's edge (rho)
            # Assuming robot is a point for simplicity in this calculation,
            # or considering 'o_radius' as the effective radius including robot's body
            rho = dist_to_obs_center - o_radius

            # Check if obstacle is within influence radius and not too close (rho > 0)
            if rho < self.obstacle_influence_radius and rho > 0:
                # To avoid division by zero or very large forces when rho is very small,
                # add a small epsilon or clamp rho.
                effective_rho = max(rho, 0.01) # Ensure rho is not zero or negative

                # Calculate the magnitude of the repulsive force
                # Formula: F_rep = k_rep * (1/rho - 1/rho_0) * (1/rho^2)
                # Where rho_0 is obstacle_influence_radius
                force_magnitude = self.repulsive_gain * (1.0 / effective_rho - 1.0 / self.obstacle_influence_radius) / (effective_rho**2)

                # Calculate angle from robot to obstacle center
                angle_to_obs_center = math.atan2(dy_obs, dx_obs)

                # Repulsive force components (pushing away from obstacle)
                # Direction is opposite to the vector from robot to obstacle
                fx_repulsive += force_magnitude * -math.cos(angle_to_obs_center)
                fy_repulsive += force_magnitude * -math.sin(angle_to_obs_center)
            elif rho <= 0: # Robot is inside or very close to the obstacle
                # Apply a very strong repulsive force to push out
                self.get_logger().warn(f"Robot too close to obstacle at ({ox:.2f}, {oy:.2f}). Applying strong repulsion.")
                # Push directly away from obstacle center
                angle_to_obs_center = math.atan2(dy_obs, dx_obs)
                fx_repulsive += 10.0 * -math.cos(angle_to_obs_center) # Strong force
                fy_repulsive += 10.0 * -math.sin(angle_to_obs_center)
                
        return fx_repulsive, fy_repulsive

    def control_loop(self):
        """
        Main control loop that calculates forces, combines them,
        and publishes velocity commands.
        """
        # Calculate attractive force
        fx_attractive, fy_attractive = self.calculate_attractive_force()

        # Calculate repulsive force
        fx_repulsive, fy_repulsive = self.calculate_repulsive_force()

        # Combine forces
        fx_total = fx_attractive + fx_repulsive
        fy_total = fy_attractive + fy_repulsive

        # Convert total force into linear and angular velocities
        # Linear velocity is proportional to the magnitude of the force in the robot's forward direction
        # Angular velocity is proportional to the angle of the force relative to the robot's forward direction

        # Calculate the angle of the total force vector in the world frame
        force_angle_world = math.atan2(fy_total, fx_total)

        # Calculate the magnitude of the total force vector
        force_magnitude = math.sqrt(fx_total**2 + fy_total**2)

        # Desired linear velocity (in robot's forward direction)
        # Project the total force onto the robot's forward axis
        # The angle of the force relative to the robot's heading
        angle_relative_to_robot = force_angle_world - self.current_theta
        linear_x = force_magnitude * math.cos(angle_relative_to_robot)
        # Cap linear velocity
        linear_x = max(min(linear_x, self.max_linear_velocity), -self.max_linear_velocity)

        # Desired angular velocity (to align with the force direction)
        # This is the difference between the force angle and the robot's current heading
        angular_z = angle_relative_to_robot
        # Normalize angle to be between -pi and pi
        angular_z = math.atan2(math.sin(angular_z), math.cos(angular_z))
        # Cap angular velocity
        angular_z = max(min(angular_z, self.max_angular_velocity), -self.max_angular_velocity)

        # Create and publish Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        # Stop if goal is reached
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        if distance_to_goal < self.goal_threshold:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info("Goal reached. Stopping robot.")
            # Optionally, you might want to shut down the node here or change state
            # rclpy.shutdown() # Uncomment to shut down node after reaching goal

        self.publisher_.publish(twist_msg)
        # self.get_logger().info(f"Published Twist: Linear.x={twist_msg.linear.x:.2f}, Angular.z={twist_msg.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    navigator = PotentialFieldNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()