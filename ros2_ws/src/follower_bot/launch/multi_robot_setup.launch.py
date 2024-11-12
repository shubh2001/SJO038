import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the paths to the necessary launch files
    multi_turtlebot_sim_path = get_package_share_directory('multi_turtlebot_sim')

    # Launch the world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_turtlebot_sim_path, 'launch', 'standalone_world.launch.py')
        ),
        launch_arguments={
            'world_name':'turtlebot3_world.world'
        }.items()
    )

    # Spawn the first robot
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_turtlebot_sim_path, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'robot1',
            'x_pose': '0.5',
            'y_pose': '0.5'
        }.items()
    )

    # Spawn the follower robot
    spawn_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(multi_turtlebot_sim_path, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'robot_prefix': 'follower',
            'x_pose': '-0.5',
            'y_pose': '-0.5'
        }.items()
    )

    # Static transform publisher
    static_transform = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', 
             '0', '0', '0', '0', '0', '0', 'robot1/odom', 'follower/odom'],
        output='screen'
    )

    # Run the follower bot node
    follower_node = ExecuteProcess(
        cmd=['ros2', 'run', 'follower_bot', 'turtlebot_follower_node'],
        output='screen'
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        world_launch,
        spawn_robot1,
        spawn_follower,
        static_transform,
        follower_node
    ])
