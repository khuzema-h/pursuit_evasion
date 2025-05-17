import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    # Base path to models
    base_model_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models'
    )

    # Define robot configurations, each with its own SDF file
    robots = [
        {
            'name': 'Pursuer',
            'sdf_file': os.path.join(base_model_path, 'turtlebot3_waffle', 'pursuer.sdf'),
            'x_pose': '0.0', 'y_pose': '0.0', 'z_pose': '0.01', 'yaw': '3.14'
        },
        {
            'name': 'Evader',
            'sdf_file': os.path.join(base_model_path, 'turtlebot3_waffle', 'evader.sdf'),
            'x_pose': '10.5', 'y_pose': '6.5', 'z_pose': '0.01', 'yaw': '3.14'
        },
        # You can add more robots with different SDFs
    ]

    ld = LaunchDescription()

    for robot in robots:
        robot_group = GroupAction([
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot['name'],
                    '-file', robot['sdf_file'],
                    '-x', robot['x_pose'],
                    '-y', robot['y_pose'],
                    '-z', robot['z_pose'],
                    '-Y', robot['yaw'],
                    '-robot_namespace', robot['name']
                ],
                output='screen',
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot['name'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ])
        
        ld.add_action(robot_group)

    return ld
