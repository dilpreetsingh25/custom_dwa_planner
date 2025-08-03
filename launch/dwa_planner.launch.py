from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner',
            name='dwa_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x': 2.0,
                'goal_y': 0.5
            }]
        )
    ])