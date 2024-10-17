import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
from scipy.linalg import solve_continuous_are

class LQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('LQRController has been started.')

        # Subskrybent odometrii do aktualizacji stanu
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.state = np.array([[0.0], [0.0], [0.0], [0.0]])  # [x, y, v_x, v_y]

        # Definicja macierzy LQR
        self.A = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]])  # Macierz stanu
        self.B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]])                         # Macierz wejść
        self.Q = np.eye(4) * 10  # Macierz kosztów stanu
        self.R = np.eye(2)       # Macierz kosztów wejść
        self.K = self.lqr(self.A, self.B, self.Q, self.R)  # Obliczenie macierzy wzmocnień

        # Trajektoria do podążania: przykładowe punkty (x, y)
        self.trajectory = [[1, 1], [2, 2], [3, 1], [4, 0]]
        self.current_target_idx = 0

    def lqr(self, A, B, Q, R):
        # Rozwiązanie równania Riccatiego za pomocą scipy
        P = solve_continuous_are(A, B, Q, R)
        # Obliczenie macierzy wzmocnień LQR
        K = np.dot(np.linalg.inv(R), np.dot(B.T, P))
        return K

    def odom_callback(self, msg):
        # Aktualizacja stanu na podstawie danych z odometrii
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        velocity_x = msg.twist.twist.linear.x
        velocity_y = msg.twist.twist.linear.y
        self.state = np.array([[position_x], [position_y], [velocity_x], [velocity_y]])

    def control_loop(self):
        if self.current_target_idx >= len(self.trajectory):
            self.get_logger().info('Trajectory complete')
            return

        # Pobierz aktualny cel
        target = self.trajectory[self.current_target_idx]
        target_state = np.array([[target[0]], [target[1]], [0], [0]])

        # Oblicz błąd stanu
        error = self.state - target_state

        # Obliczenie sterowania przy użyciu LQR
        u = -np.dot(self.K, error)

        # Publikowanie wiadomości sterującej
        msg = Twist()
        msg.linear.x = float(u[0])  # Prędkość liniowa w osi x
        msg.angular.z = float(u[1])  # Przyjmijmy uproszczenie, że sterowanie w y przekłada się na kąt
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear X: %.2f, Angular Z: %.2f' % (msg.linear.x, msg.angular.z))

        # Sprawdzenie, czy osiągnięto cel (prosty warunek odległości)
        distance_to_target = math.sqrt((self.state[0, 0] - target[0])**2 + (self.state[1, 0] - target[1])**2)
        if distance_to_target < 0.1:
            self.current_target_idx += 1
            self.get_logger().info('Reached target %d, moving to next target' % self.current_target_idx)

def main(args=None):
    rclpy.init(args=args)
    node = LQRController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()