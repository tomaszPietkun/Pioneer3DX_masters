import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math


class PathFollowingAndObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('path_following_and_obstacle_avoidance')

        # Publisher to control velocity (cmd_vel)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to read laser scan data for obstacle detection
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Subscriber to read odometry data to determine the robot's current position
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Define a timer to regularly call the path following method
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.follow_path)

        # Obstacle distance threshold (in meters)
        self.obstacle_distance_threshold = 0.5

        # Flag to indicate obstacle detection
        self.obstacle_detected = False

        # Default linear and angular speed for path following
        self.linear_speed = 0.2
        self.angular_speed = 0.0

        # List of waypoints (x, y) to follow
        self.waypoints = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
        self.current_waypoint_index = 0

        # Current position and orientation of the robot
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0

    def laser_callback(self, msg):
        # Check for obstacles in the front (e.g., within 90 degrees of front view)
        front_ranges = msg.ranges[-45:] + msg.ranges[:45]
        front_ranges = [distance for distance in front_ranges if not math.isinf(distance)]

        if front_ranges:
            min_distance = min(front_ranges)
        else:
            min_distance = float('inf')

        # If an obstacle is detected within threshold distance, stop forward movement
        if min_distance < self.obstacle_distance_threshold:
            self.get_logger().info('Obstacle detected at distance: %.2f m' % min_distance)
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def odom_callback(self, msg):
        # Update the current position and yaw of the robot
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def follow_path(self):
        # Create a Twist message to control the robot
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Stop moving forward and rotate to avoid obstacle
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Rotate to avoid the obstacle
        else:
            # Follow the planned path by moving towards the current waypoint
            if self.current_waypoint_index < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_waypoint_index]
                distance_to_waypoint = math.sqrt(
                    (target_x - self.current_position[0]) ** 2 + (target_y - self.current_position[1]) ** 2)
                target_angle = math.atan2(target_y - self.current_position[1], target_x - self.current_position[0])
                angle_diff = target_angle - self.current_yaw

                # Normalize angle difference to the range [-pi, pi]
                angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

                if distance_to_waypoint > 0.1:
                    # Move towards the waypoint
                    cmd_vel.linear.x = self.linear_speed
                    cmd_vel.angular.z = 1.0 * angle_diff  # Proportional control for angular velocity
                else:
                    # Reached the waypoint, move to the next one
                    self.get_logger().info('Reached waypoint %d' % self.current_waypoint_index)
                    self.current_waypoint_index += 1
            else:
                # All waypoints have been reached, stop the robot
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.get_logger().info('All waypoints reached, stopping the robot.')

        # Publish the command velocity to move the robot
        self.velocity_publisher.publish(cmd_vel)

    def shutdown(self):
        # Stop the robot on shutdown
        cmd_vel = Twist()
        self.velocity_publisher.publish(cmd_vel)
        self.get_logger().info('Shutting down, stopping the robot.')


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the node
    path_following_node = PathFollowingAndObstacleAvoidance()

    try:
        # Keep the node running
        rclpy.spin(path_following_node)
    except KeyboardInterrupt:
        path_following_node.shutdown()
    finally:
        # Destroy the node explicitly to release resources
        path_following_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
