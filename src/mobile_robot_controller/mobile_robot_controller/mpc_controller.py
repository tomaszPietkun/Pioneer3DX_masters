import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.optimize import minimize
import time
import csv

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.body_radius = 0.5  # Radius of the robot body
        self.prediction_horizon = 10  # Prediction horizon for MPC
        self.dt = 0.1  # Time step in seconds
        self.delta = 500.0  # Scaling factor for obstacle avoidance cost
        self.Q = np.eye(2)  # Weight matrix for tracking cost (assuming 2 state variables: x, y)
        self.max_linear_velocity = 1.0  # Maximum linear velocity
        self.max_angular_velocity = 2.0  # Maximum angular velocity

        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.waypoints = []
        self.trajectory = []  # To store trajectory data

        # Obstacle detection
        self.min_obstacle_distance = float('inf')  # Will be updated with LiDAR data
        self.safe_distance_threshold = 1.0  # Safe distance to maintain from obstacles

        # Publishers and Subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.waypoint_count_publisher = self.create_publisher(PoseStamped, '/waypoint_count', 10)

        # Add predefined waypoints directly
        self.add_predefined_waypoints()

        # Timer to update control every dt seconds
        self.timer = self.create_timer(self.dt, self.control_loop)

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

    def goal_pose_callback(self, msg):
        # Append the goal pose as a waypoint (only x and y)
        waypoint = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints.append(waypoint)
        self.get_logger().info(f"Received new waypoint: {waypoint}")
        self.publish_waypoint_count()

    def lidar_callback(self, msg):
        # Calculate the minimum distance to any object
        self.min_obstacle_distance = min(msg.ranges)
        self.get_logger().info(f'Minimum distance to an obstacle: {self.min_obstacle_distance:.2f} m')

    def control_loop(self):
        if not self.waypoints:
            # If there are no waypoints, stop the robot and save trajectory
            self.stop_robot()
            self.save_trajectory()
            return

        # Calculate the control inputs using MPC, considering obstacle avoidance
        v, omega = self.mpc_control(self.x, self.y, self.theta, self.waypoints, self.body_radius, self.dt,
                                    self.prediction_horizon)

        # Log the control input values
        self.get_logger().info(f"Control inputs: linear velocity = {v:.2f}, angular velocity = {omega:.2f}")

        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.velocity_publisher.publish(twist)

        # If the robot is close to the current waypoint, pop it from the list
        if np.sqrt((self.waypoints[0][0] - self.x) ** 2 + (self.waypoints[0][1] - self.y) ** 2) < 0.4:
            self.waypoints.pop(0)
            self.get_logger().info(f"Reached waypoint, remaining waypoints: {len(self.waypoints)}")
            self.publish_waypoint_count()
            self.stop_robot()
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

    def mpc_control(self, x, y, theta, waypoints, body_radius, dt, prediction_horizon):
        def objective(u):
            cost = 0.0
            x_pred, y_pred, theta_pred = x, y, theta
            for i in range(prediction_horizon):
                v = u[i]
                omega = u[prediction_horizon + i]
                x_pred += v * np.cos(theta_pred) * dt
                y_pred += v * np.sin(theta_pred) * dt
                theta_pred += omega * dt

                target_x, target_y = waypoints[0]
                state_error = np.array([x_pred - target_x, y_pred - target_y])
                cost += state_error.T @ self.Q @ state_error

                # Penalize movement if an obstacle is too close
                if self.min_obstacle_distance < self.safe_distance_threshold:
                    cost += self.delta / (self.min_obstacle_distance + 1e-6) ** 2 # Higher cost for closer obstacles

                # Encourage some movement to avoid completely stopping near obstacles
                cost += 0.1 * (v ** 2 + omega ** 2)
            return cost

        def constraints(u):
            return []

        max_v = self.max_linear_velocity if self.min_obstacle_distance > self.safe_distance_threshold else 0.2

        u0 = [0.0] * (2 * prediction_horizon)
        bounds = [(-max_v, max_v)] * prediction_horizon + [(-self.max_angular_velocity, self.max_angular_velocity)] * prediction_horizon
        result = minimize(objective, u0, bounds=bounds, constraints={'type': 'ineq', 'fun': constraints})
        return result.x[0], result.x[prediction_horizon]

    def save_trajectory(self):
        # Save the trajectory data to a CSV file
        with open('mpc_robot_trajectory.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'theta'])
            for data in self.trajectory:
                writer.writerow(data)
        self.get_logger().info("Trajectory data saved to 'mpc_robot_trajectory.csv'")

def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()