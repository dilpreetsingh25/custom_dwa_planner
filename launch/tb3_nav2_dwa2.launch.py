import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch.actions

def generate_launch_description():
    # Package directories
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_dwa_local_planner = get_package_share_directory('custom_dwa_planner')

    # Configurations
    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    nav2_params = os.path.join(pkg_dwa_local_planner, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dwa_local_planner, 'config', 'my_hex6.yaml')
    rviz_config = os.path.join(pkg_dwa_local_planner, 'rviz', 'nav2_dwa.rviz')

    # Define separate lifecycle managers
    localization_nodes = ['map_server', 'amcl']
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
        # Set TurtleBot3 model
        # ExecuteProcess(
        #     cmd=['export', 'TURTLEBOT3_MODEL=waffle_pi'],
        #     shell=True
        # ),
        
        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'))
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
            launch_arguments={'world': world}.items()
        ),
        
        # Localization nodes
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params, {'yaml_filename': map_file}],
            # Add remappings for transforms
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),
        
        # Localization lifecycle manager
        launch.actions.TimerAction(
           period=2.0,  # Wait 2 seconds
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[
                        {'autostart': True},
                        {'node_names': localization_nodes}
                    ],
                    # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
                )
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
            remappings=[('cmd_vel', 'smoothed_cmd_vel'), ('cmd_vel_smoothed', 'cmd_vel')]
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
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # Teleop (optional)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e'
        )
    ])