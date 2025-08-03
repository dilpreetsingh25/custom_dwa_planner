import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    # Get the path to turtlebot3_gazebo launch directory
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
    # turtlebot3_world_launch = os.path.join(get_package_share_directory('custom_dwa_planner'), 'launch', 'turtlebot3_world.launch.py')

    rviz_config_path = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'rviz', 'dwa_config.rviz'
    )

    return LaunchDescription([
        # Declare 'world' argument
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_world',
            description='Gazebo world name'
        ),

        # Declare 'rviz_config' argument
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('custom_dwa_planner'),
                'rviz',
                'dwa_config.rviz'
            ),
            description='RViz configuration file'
        ),
        DeclareLaunchArgument('goal_x', default_value='1.5'),
        DeclareLaunchArgument('goal_y', default_value='0.5'),

        # Include TurtleBot3 Gazebo simulation launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch),
            launch_arguments={
                'world': LaunchConfiguration('world')
            }.items()
        ),

        # Custom DWA Planner Node
        Node( 
            package='custom_dwa_planner',
            executable='dwa_planner',
            name='dwa_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }]
        ),


        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
