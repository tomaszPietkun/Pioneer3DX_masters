import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Define paths for URDF, RViz config, and Gazebo world
    urdf_path = PathJoinSubstitution(
        [get_package_share_directory('mobile_robot_description'), 'urdf', 'robot.urdf.xacro']
    )

    config_rviz_path = PathJoinSubstitution(
        [get_package_share_directory('mobile_robot_bringup'), 'rviz', 'p3dx_config.rviz']
    )

    world_path = PathJoinSubstitution(
        [get_package_share_directory('mobile_robot_bringup'), 'worlds', 'my_custom_world.world']
    )

    # Node for publishing robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )


    # Include Gazebo launch file with custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # Node to spawn the entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    '''
    # Node to start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', config_rviz_path],
        output='screen'
    )
    '''

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_entity_node,
        #rviz_node
    ])
