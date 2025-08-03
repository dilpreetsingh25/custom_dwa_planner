# Custom DWA Planner for Nav2

A custom Dynamic Window Approach (DWA) local planner plugin for the Nav2 navigation system, optimized for TurtleBot3 and compatible with ROS 2 Humble.

## Features
- 🚀 Full DWA implementation with trajectory simulation and scoring

- 🤖 Optimized for TurtleBot3 differential drive robots

- 🗺️ Nav2 plugin architecture integration

- 📊 Real-time trajectory visualization

- ⚙️ Configurable cost function weights (goal distance, obstacle avoidance, smoothness)

- 🛑 Dynamic collision avoidance using laser scan data

- 🎯 RVIZ integration for goal setting and visualization

  ## Installation
 ### Prerequisites
- ROS 2 Humble

- TurtleBot3 packages

- Nav2 navigation stack

```
# Create workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Clone repository
git clone https://github.com/dilpreetsingh25/custom_dwa_planner.git

# Build package
cd ~/turtlebot3_ws
```

## Usage
```
export TURTLEBOT3_MODEL=waffle_pi
rosdep install -y --from-paths src --ignore-src
colcon build --packages-select custom_dwa_planner
source install/setup.bash
ros2 launch custom_dwa_planner tb3_nav2_dwa3.launch.py

```



In RViz:

- Set initial pose using "2D Pose Estimate"

- Set navigation goal using "2D Nav Goal"

Parameters

| Parameter         | Default | Description                          |
|------------------|---------|--------------------------------------|
| `max_vel`        | 0.5     | Maximum linear velocity (m/s)        |
| `min_vel`        | -0.2    | Minimum linear velocity (m/s)        |
| `max_rot_vel`    | 1.5     | Maximum angular velocity (rad/s)     |
| `min_rot_vel`    | -1.5    | Minimum angular velocity (rad/s)     |
| `acc_lim_x`      | 0.5     | Linear acceleration limit (m/s²)     |
| `acc_lim_theta`  | 1.0     | Angular acceleration limit (rad/s²)  |
| `sim_time`       | 1.5     | Trajectory simulation time (s)       |
| `dt`             | 0.1     | Simulation time step (s)             |
| `v_samples`      | 20      | Linear velocity samples              |
| `w_samples`      | 20      | Angular velocity samples             |
| `robot_radius`   | 0.1     | Collision radius (m)                 |
| `goal_tolerance` | 0.2     | Goal tolerance radius (m)            |
| `alpha`          | 0.6     | Goal distance weight                 |
| `beta`           | 0.3     | Obstacle avoidance weight            |

## Visualization

https://drive.google.com/file/d/1fS5PQTOW9zgqv2A3Q03w_oVkzSKTfWGt/view?usp=sharing

