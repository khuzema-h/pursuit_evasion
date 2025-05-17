from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    namespace = 'Pursuer'  # Change this to your desired namespace
    
    return LaunchDescription([
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        ThisLaunchFileDir(),
                        '/slam.launch.py'
                    ]),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'base_frame': f'{namespace}/base_footprint',
                        'odom_frame': f'{namespace}/odom',
                        'map_frame': f'{namespace}/map',
                    }.items()
                )
            ]
        )
    ])