import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
import csv

class EKFSLAM(Node):
    def __init__(self):
        super().__init__('ekf_slam')

        # Define parameters
        self.dt = 0.1  # Time step in seconds
        self.state_dim = 3  # State dimension (x, y, theta for the robot)
        self.state = np.zeros((self.state_dim, 1))  # Initial state vector
        self.covariance = np.eye(self.state_dim)  # Initial covariance matrix
        self.Q = np.diag([0.1, 0.1, np.deg2rad(1)])  # Process noise covariance
        self.R = np.diag([0.5, 0.5])  # Measurement noise covariance

        # For landmarks
        self.landmarks = []
        self.landmark_covariances = []
        self.add_predefined_landmarks()

        # For waypoints
        self.waypoints = [
            (5.0, 5.0),
            (10.0, 10.0),
            (15.0, 5.0)
        ]

        # Publishers and Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/ekf_slam_pose', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to update control every dt seconds
        self.timer = self.create_timer(self.dt, self.ekf_slam_step)

    def add_predefined_landmarks(self):
        # Use the same waypoints as in the MPC and LQR controllers for landmarks
        predefined_landmarks = [
            (5.0, 5.0),
            (10.0, 10.0),
            (15.0, 5.0)
        ]

        for landmark in predefined_landmarks:
            self.landmarks.append(np.array([[landmark[0]], [landmark[1]]]))
            self.landmark_covariances.append(np.eye(2) * 1.0)
            self.get_logger().info(f"Added predefined landmark: {landmark}")

    def odom_callback(self, msg):
        # Update the state prediction based on odometry data
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        theta = self.state[2, 0]

        # Predict the next state (motion model)
        delta_x = linear_vel * np.cos(theta) * self.dt
        delta_y = linear_vel * np.sin(theta) * self.dt
        delta_theta = angular_vel * self.dt

        # Limit changes to prevent divergence due to incorrect scaling
        delta_x = np.clip(delta_x, -0.5, 0.5)
        delta_y = np.clip(delta_y, -0.5, 0.5)
        delta_theta = np.clip(delta_theta, -np.pi / 4, np.pi / 4)

        # Update state with the delta values
        self.state[0, 0] += delta_x
        self.state[1, 0] += delta_y
        self.state[2, 0] += delta_theta

        # Normalize angle to be between -pi and pi
        self.state[2, 0] = np.arctan2(np.sin(self.state[2, 0]), np.cos(self.state[2, 0]))

        # Predict covariance using linearized model
        G = np.eye(self.state_dim)
        G[0, 2] = -linear_vel * np.sin(theta) * self.dt
        G[1, 2] = linear_vel * np.cos(theta) * self.dt

        self.covariance = G @ self.covariance @ G.T + self.Q

    def scan_callback(self, msg):
        # Process the Lidar scan data to detect landmarks and update state estimate
        # Here, we assume known correspondences for simplicity.
        for idx, range_value in enumerate(msg.ranges):
            if range_value < msg.range_max:  # Valid range data
                angle = msg.angle_min + idx * msg.angle_increment
                lx = self.state[0, 0] + range_value * np.cos(self.state[2, 0] + angle)
                ly = self.state[1, 0] + range_value * np.sin(self.state[2, 0] + angle)

                # Assume we can associate the landmark (this would normally require a data association step)
                if len(self.landmarks) <= idx:
                    self.landmarks.append(np.array([[lx], [ly]]))
                    self.landmark_covariances.append(np.eye(2) * 1.0)
                else:
                    # EKF Update Step for Existing Landmarks
                    z = np.array([[lx], [ly]])
                    z_hat = self.landmarks[idx]  # Predicted measurement
                    y = z - z_hat  # Innovation

                    H = np.array([[1, 0, 0], [0, 1, 0]])  # Measurement matrix

                    S = H @ self.covariance @ H.T + self.R  # Innovation covariance
                    K = self.covariance @ H.T @ np.linalg.inv(S)  # Kalman gain

                    # Update state
                    self.state += K @ y

                    # Update covariance
                    self.covariance = (np.eye(self.state_dim) - K @ H) @ self.covariance

    def ekf_slam_step(self):
        # Publish the estimated robot pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.state[0, 0]
        pose_msg.pose.position.y = self.state[1, 0]
        pose_msg.pose.orientation.z = np.sin(self.state[2, 0] / 2.0)
        pose_msg.pose.orientation.w = np.cos(self.state[2, 0] / 2.0)

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info(f"Published EKF pose: x={self.state[0, 0]:.2f}, y={self.state[1, 0]:.2f}, theta={np.rad2deg(self.state[2, 0]):.2f}°")

        # Control logic to move towards waypoints
        if self.waypoints:
            waypoint = self.waypoints[0]
            error_x = waypoint[0] - self.state[0, 0]
            error_y = waypoint[1] - self.state[1, 0]
            distance = np.sqrt(error_x**2 + error_y**2)

            # Simple proportional control to drive towards waypoint
            v = 0.5 * distance
            omega = 1.5 * (np.arctan2(error_y, error_x) - self.state[2, 0])

            # Limit velocities
            v = np.clip(v, -1.0, 1.0)
            omega = np.clip(omega, -1.0, 1.0)

            # Publish control command
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = omega
            self.velocity_publisher.publish(twist)

            # If close to the waypoint, proceed to the next
            if distance < 0.2:
                self.get_logger().info(f"Reached waypoint: {waypoint}")
                self.waypoints.pop(0)

def main(args=None):
    rclpy.init(args=args)
    ekf_slam = EKFSLAM()
    rclpy.spin(ekf_slam)
    ekf_slam.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
