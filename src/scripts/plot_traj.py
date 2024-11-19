import matplotlib.pyplot as plt
import csv
import matplotlib.patches as patches

# Function to read the trajectory data from a CSV file
def read_trajectory_data(filename):
    x_data = []
    y_data = []
    with open(filename, mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            x_data.append(float(row[0]))
            y_data.append(float(row[1]))
        return x_data, y_data

# Function to plot the trajectory with waypoints and obstacles
def plot_trajectories(lqr_filename='lqr_tuned_robot_trajectory.csv', save_filename='lqr_robot_trajectory.png'):
    # Read LQR trajectory data
    lqr_x, lqr_y = read_trajectory_data(lqr_filename)

    # Waypoints (same as those defined in your controller)
    waypoints = [
        (1.0, 8.0),
        (7.0, 10.0),
        (11.0, 7.0),
        (4.0, 6.0),
        (7.0, 2.0),
        (0.0, 0.0)
    ]

    # Obstacles (positions and sizes from your world file)
    obstacles = [
        {'position': (3, 7), 'size': (0.5, 0.5), 'color': 'red'},
        {'position': (7, 5), 'size': (0.5, 0.5), 'color': 'green'},
        {'position': (9, 8), 'size': (0.5, 0.5), 'color': 'blue'},
        {'position': (5, 2), 'size': (0.5, 0.5), 'color': 'yellow'},
        {'position': (2, 9), 'size': (0.5, 0.5), 'color': 'magenta'}
    ]

    # Plotting the trajectory
    plt.figure(figsize=(10, 6))

    # Plot the robot's trajectory
    plt.plot(lqr_x, lqr_y, linestyle='-', color='b', label='Trajektoria robota')

    # Plot waypoints
    waypoint_x = [wp[0] for wp in waypoints]
    waypoint_y = [wp[1] for wp in waypoints]
    plt.scatter(waypoint_x, waypoint_y, marker='x', color='k', s=100, label='Punkty')

    # Add labels to waypoints (optional)
    for i, (x, y) in enumerate(waypoints):
        plt.text(x + 0.1, y + 0.1, f'WP{i+1}', fontsize=9, color='black')

    # Plot obstacles as squares
    ax = plt.gca()
    for obs in obstacles:
        obs_x, obs_y = obs['position']
        obs_width, obs_height = obs['size']
        # Create a rectangle patch
        rect = patches.Rectangle(
            (obs_x - obs_width / 2, obs_y - obs_height / 2),  # Lower-left corner
            obs_width,
            obs_height,
            linewidth=1,
            edgecolor=obs['color'],
            facecolor=obs['color'],
            alpha=0.5,
            label='Przeszkoda' if 'Przeszkoda' not in ax.get_legend_handles_labels()[1] else ''
        )
        # Add the rectangle to the plot
        ax.add_patch(rect)

    # Set axis limits
    plt.xlim(-0.5, 12)  # Adjust these values as needed
    plt.ylim(-0.5, 12)  # Adjust these values as needed

    # Alternatively, set both axes at once
    # plt.axis([-1, 12, -1, 12])

    # Plot Formatting
    plt.xlabel('Pozycja X (m)')
    plt.ylabel('Pozycja Y (m)')
    plt.title('Trajektoria Robota na mapie z przeszkodami')
    plt.legend()
    plt.grid(True)
    #plt.axis('equal')  # To keep x and y scales equal for proper visualization

    # Save the plot to a file
    plt.savefig(save_filename, dpi=300)  # Adjust dpi for resolution as needed

    # Show the plot
    plt.show()

# Main script to read the data and plot it
if __name__ == '__main__':
    plot_trajectories()
