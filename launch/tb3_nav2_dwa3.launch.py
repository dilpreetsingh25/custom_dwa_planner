import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_dwa_local_planner = get_package_share_directory('custom_dwa_planner')

    # Configurations
    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    nav2_params = os.path.join(pkg_dwa_local_planner, 'config', 'nav2_params.yaml')
    # map_file = os.path.join(pkg_dwa_local_planner, 'config', 'my_hex6.yaml')
    # To this:
    map_file = os.path.join(
        get_package_share_directory('custom_dwa_planner'),
        'config', 'my_hex6.yaml'
    )
    rviz_config = os.path.join(pkg_dwa_local_planner, 'rviz', 'nav2_dwa.rviz')

    # Navigation nodes
    navigation_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    return LaunchDescription([     
        # Declare launch arguments  
        DeclareLaunchArgument('goal_x', default_value='1.5'),
        DeclareLaunchArgument('goal_y', default_value='0.5'), 
        # Set environment variables
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle_pi'),
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(
            pkg_tb3_gazebo, 'models')),
        
        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')
            ]),
            launch_arguments={'world': world,
                              'use_sim_time': 'true'}.items()
        ),
        
        # Use nav2_bringup's localization launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
            ]),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'true'
            }.items()
        ),


        # Navigation lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': navigation_nodes}
            ]
        ),
        
        # Navigation nodes
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params]
        ),
        
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'smoothed_cmd_vel'), 
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])