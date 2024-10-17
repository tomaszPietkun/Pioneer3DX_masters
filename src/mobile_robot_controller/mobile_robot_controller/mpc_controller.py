import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from scipy.optimize import minimize
import math
import matplotlib.pyplot as plt

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('MPCController has been started.')

        # Subskrybent odometrii do aktualizacji stanu
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, v_x, v_y]

        # Trajektoria do podążania: przykładowe punkty (x, y)
        self.trajectory = [[1, 1], [2, 2], [3, 1], [4, 0]]
        self.current_target_idx = 0
        self.horizon = 10  # Długość horyzontu predykcyjnego

        # Przechowywanie historii pozycji dla wykresu
        self.position_history = []

    def odom_callback(self, msg):
        # Aktualizacja stanu na podstawie danych z odometrii
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        velocity_x = msg.twist.twist.linear.x
        velocity_y = msg.twist.twist.linear.y
        self.state = np.array([position_x, position_y, velocity_x, velocity_y])
        # Dodawanie bieżącej pozycji do historii
        self.position_history.append((position_x, position_y))

    def objective_function(self, u):
        # Funkcja celu do minimalizacji w MPC
        cost = 0.0
        state = self.state.copy()
        target = self.trajectory[self.current_target_idx]
        for k in range(self.horizon):
            state[0] += state[2] * 0.1  # Aktualizacja pozycji x
            state[1] += state[3] * 0.1  # Aktualizacja pozycji y
            state[2] = u[2 * k]         # Aktualizacja prędkości x
            state[3] = u[2 * k + 1]     # Aktualizacja prędkości y
            # Koszt to suma kwadratów błędów stanu i sygnałów sterujących
            cost += (state[0] - target[0])**2 + (state[1] - target[1])**2
            cost += 0.1 * (u[2 * k]**2 + u[2 * k + 1]**2)  # Dodaj koszt energii sterowania
        return cost

    def control_loop(self):
        if self.current_target_idx >= len(self.trajectory):
            self.get_logger().info('Trajectory complete')
            self.plot_position_history()
            return

        # Optymalizacja sterowania za pomocą metody minimalizacji
        u0 = np.zeros(2 * self.horizon)  # Początkowe wartości wejść sterujących
        bounds = [(-1.0, 1.0) for _ in range(2 * self.horizon)]  # Ograniczenia sterowania
        result = minimize(self.objective_function, u0, bounds=bounds, method='SLSQP')

        # Publikowanie optymalnych sygnałów sterujących
        if result.success:
            u_opt = result.x
            msg = Twist()
            msg.linear.x = u_opt[0]  # Prędkość liniowa
            msg.angular.z = u_opt[1]  # Prędkość kątowa
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: Linear X: %.2f, Angular Z: %.2f' % (msg.linear.x, msg.angular.z))

        # Sprawdzenie, czy osiągnięto cel (prosty warunek odległości)
        target = self.trajectory[self.current_target_idx]
        distance_to_target = math.sqrt((self.state[0] - target[0])**2 + (self.state[1] - target[1])**2)
        if distance_to_target < 0.1:
            self.current_target_idx += 1
            self.get_logger().info('Reached target %d, moving to next target' % self.current_target_idx)

    def plot_position_history(self):
        # Tworzenie wykresu z historii pozycji
        x_vals, y_vals = zip(*self.position_history)
        plt.figure()
        plt.plot(x_vals, y_vals, label='Robot Path')
        trajectory_x, trajectory_y = zip(*self.trajectory)
        plt.plot(trajectory_x, trajectory_y, 'ro-', label='Target Trajectory')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.legend()
        plt.title('Robot Trajectory vs Target Trajectory')
        plt.grid()
        plt.savefig('robot_trajectory.png')  # Zapisz wykres do pliku
        self.get_logger().info('Saved position history plot to robot_trajectory.png')

def main(args=None):
    rclpy.init(args=args)
    node = MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()