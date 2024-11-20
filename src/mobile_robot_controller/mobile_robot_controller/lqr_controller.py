import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import time
from scipy.linalg import solve_continuous_are
import csv

class LQRTunedController(Node):
    def __init__(self):
        super().__init__('lqr_tuned_controller')
        self.body_radius = 0.5  # Radius of the robot body
        self.dt = 0.1  # Time step in seconds
        self.max_linear_velocity = 1.0  # Maximum linear velocity
        self.max_angular_velocity = 2.0  # Maximum angular velocity

        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.waypoints = []
        self.trajectory = []  # To store trajectory data
        self.obstacle_penalty = 0.0
        self.min_obstacle_distance = float('inf')
        self.angle_to_nearest_obstacle = 0.0

        # Publishers and Subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_pose_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.waypoint_count_publisher = self.create_publisher(
            PoseStamped, '/waypoint_count', 10)

        # Add predefined waypoints directly
        self.add_predefined_waypoints()

        # Timer to update control every dt seconds
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Safety distance for obstacle avoidance (increased from 1.0 to 1.5)
        self.safety_distance = 1.5  # You can adjust this value as needed

    def add_predefined_waypoints(self):
        predefined_waypoints = [
            (1.0, 8.0),
            (7.0, 10.0),
            (11.0, 10.0),
            (7.0, 8.0),
            (9.0, 2.0),
            (4.0, 4.0),
            (0.0, 0.0)
        ]

        for waypoint in predefined_waypoints:
            self.waypoints.append(waypoint)
            self.get_logger().info(f"Added predefined waypoint: {waypoint}")

        self.publish_waypoint_count()

    def odom_callback(self, msg):
        # Extract pose information from the Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to Euler angle for simplicity
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

        # Save the current state to the trajectory list
        self.trajectory.append((self.x, self.y, self.theta))

    def lidar_callback(self, msg):
        # Process LiDAR data to find the nearest obstacle
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=np.inf, posinf=np.inf, neginf=np.inf)
        self.min_obstacle_distance = np.min(ranges)
        min_index = np.argmin(ranges)
        angle_increment = msg.angle_increment
        self.angle_to_nearest_obstacle = min_index * angle_increment + msg.angle_min

        # Normalize the angle
        self.angle_to_nearest_obstacle = self.normalize_angle(self.angle_to_nearest_obstacle)

        # Log obstacle information
        self.get_logger().info(
            f"Min obstacle distance: {self.min_obstacle_distance:.2f} m, "
            f"Angle: {np.degrees(self.angle_to_nearest_obstacle):.2f} deg"
        )

    def goal_pose_callback(self, msg):
        # Append the goal pose as a waypoint (without orientation)
        waypoint = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Received new waypoint: {waypoint}")
        self.publish_waypoint_count()

    def control_loop(self):
        if not self.waypoints:
            # If there are no waypoints, stop the robot and save trajectory
            self.stop_robot()
            self.save_trajectory()
            return

        # Log the minimum obstacle distance
        self.get_logger().info(f"Minimum obstacle distance: {self.min_obstacle_distance:.2f} m")

        # Check for obstacle proximity with increased safety distance
        if self.min_obstacle_distance < self.safety_distance:
            self.get_logger().info(f"Obstacle within {self.safety_distance}m detected.")
            # Obstacle avoidance override
            v, omega = self.circular_trajectory_control()
            self.get_logger().info("Obstacle avoidance override activated.")
        else:
            self.get_logger().info("No close obstacle detected. Using LQR control.")
            # Use LQR control inputs
            v, omega = self.lqr_control(self.x, self.y, self.theta, self.waypoints[0])

        # Clip velocities
        v = np.clip(v, -self.max_linear_velocity, self.max_linear_velocity)
        omega = np.clip(omega, -self.max_angular_velocity, self.max_angular_velocity)

        self.get_logger().info(
            f"Control inputs: linear velocity = {v:.2f}, angular velocity = {omega:.2f}"
        )

        # Publish velocities
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.velocity_publisher.publish(twist)

        # Check if the robot is close to the current waypoint (position only)
        distance_to_waypoint = np.sqrt((self.waypoints[0][0] - self.x) ** 2 + (self.waypoints[0][1] - self.y) ** 2)
        if distance_to_waypoint < 0.5:
            self.get_logger().info(
                f"Reached waypoint: {self.waypoints[0]}, "
                f"remaining waypoints: {len(self.waypoints) - 1}"
            )
            self.waypoints.pop(0)
            self.publish_waypoint_count()
            time.sleep(1)

        # If there are no remaining waypoints, stop the robot and save trajectory
        if not self.waypoints:
            self.stop_robot()
            self.save_trajectory()

    def stop_robot(self):
        # Publish zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Robot has stopped.")

    def publish_waypoint_count(self):
        # Create and publish a PoseStamped message to indicate the number of waypoints remaining
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'waypoint_count'
        msg.pose.position.x = float(len(self.waypoints))
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        self.waypoint_count_publisher.publish(msg)
        self.get_logger().info(f"Published waypoint count: {len(self.waypoints)}")

    def lqr_control(self, x, y, theta, waypoint):
        # Extract waypoint coordinates
        x_goal, y_goal = waypoint

        # Compute desired orientation towards the waypoint
        theta_desired = np.arctan2(y_goal - y, x_goal - x)

        # Compute orientation error
        theta_error = self.normalize_angle(theta - theta_desired)

        # Linearized system matrices (A, B)
        A = np.array([
            [0, 0, -np.sin(theta) * self.dt],
            [0, 0, np.cos(theta) * self.dt],
            [0, 0, 0]
        ])
        B = np.array([
            [np.cos(theta) * self.dt, 0],
            [np.sin(theta) * self.dt, 0],
            [0, self.dt]
        ])

        # Tuning parameters
        position_penalty = 50  # Base penalty for position errors
        orientation_penalty = 1  # Penalty for orientation errors

        # LQR cost matrices
        Q = np.diag([position_penalty, position_penalty, orientation_penalty])
        R = np.diag([5, 5])  # Penalize control effort for smoother inputs

        # Solve the continuous-time algebraic Riccati equation (CARE)
        P = solve_continuous_are(A, B, Q, R)

        # Compute LQR gain
        K = np.linalg.inv(R) @ B.T @ P

        # Compute state error
        state = np.array([x, y, theta])
        target_state = np.array([x_goal, y_goal, theta_desired])
        error = state - target_state

        # Normalize orientation error
        error[2] = self.normalize_angle(error[2])

        # Calculate control input
        u = -K @ error

        # Return control inputs
        return u[0], u[1]

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def circular_trajectory_control(self):
        # Parameters for circular trajectory
        desired_radius = self.safety_distance  # Use the safety distance as desired radius
        linear_speed = 0.2  # Reduced speed for better control
        # Compute angular speed to maintain circular motion
        angular_speed = linear_speed / desired_radius

        # Determine direction to rotate based on obstacle position
        theta_obstacle = self.angle_to_nearest_obstacle

        # Avoid obstacles behind the robot differently
        if np.abs(theta_obstacle) > np.pi / 2:
            # Obstacle is behind, maintain current heading
            omega = 0.0
            self.get_logger().info("Obstacle is behind. Maintaining current heading.")
        elif theta_obstacle >= 0:
            # Obstacle is on the left, turn right
            omega = -angular_speed
            self.get_logger().info("Obstacle on the left. Turning right.")
        else:
            # Obstacle is on the right, turn left
            omega = angular_speed
            self.get_logger().info("Obstacle on the right. Turning left.")

        v = linear_speed

        # Log the control inputs
        self.get_logger().info(
            f"Circular Trajectory Control: v = {v:.2f}, omega = {omega:.2f}, "
            f"Obstacle angle: {np.degrees(theta_obstacle):.2f} deg"
        )

        return v, omega

    def save_trajectory(self):
        # Save the trajectory data to a CSV file
        with open('lqr_tuned_robot_trajectory.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'theta'])
            for data in self.trajectory:
                writer.writerow(data)
        self.get_logger().info("Trajectory data saved to 'lqr_tuned_robot_trajectory.csv'")

def main(args=None):
    rclpy.init(args=args)
    lqr_tuned_controller = LQRTunedController()
    rclpy.spin(lqr_tuned_controller)
    lqr_tuned_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
