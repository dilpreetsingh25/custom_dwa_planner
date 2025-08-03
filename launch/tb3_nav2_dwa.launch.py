import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


lifecycle_nodes = ['controller_server',
                   'amcl',
                   'map_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

def generate_launch_description():
    # Package directories
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    # pkg_nav2_bringup = get_package_share_directory('turtlebot3_navigation2')
    pkg_dwa_local_planner = get_package_share_directory('custom_dwa_planner')  # Your custom package

    # Configurations
    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'turtlebot3_world.world')
    nav2_params = os.path.join(pkg_dwa_local_planner, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dwa_local_planner, 'config', 'my_hex6.yaml')
    rviz_config = os.path.join(pkg_dwa_local_planner, 'rviz', 'nav2_dwa.rviz')

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        )
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3_gazebo, 'launch', 'turtlebot3_world.launch.py')),
            launch_arguments={'world': world}.items()
        ),
        
        # Launch Nav2
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),
        
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[nav2_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params],
        ),
        
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
        ),      
        


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
                nav2_params
                ]
        ),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}, 
                        {'use_sim_time': True},
                        nav2_params]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True},
                        nav2_params]
            
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
                nav2_params
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
        
        # Teleop (for manual control testing)
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix='xterm -e'
        )
    ])