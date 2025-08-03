// #include "custom_dwa_planner.hpp"
// #include "nav2_util/node_utils.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// using namespace std::chrono_literals;

// namespace custom_dwa_planner {

// void CustomDWAPlanner::configure(
//     const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//     std::string name,
//     std::shared_ptr<tf2_ros::Buffer> /*tf*/,
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
//     node_ = parent.lock();
//     plugin_name_ = "DwaPlanner";
//     costmap_ros_ = costmap_ros;
//     costmap_ = costmap_ros_->getCostmap();
//     global_frame_ = costmap_ros_->getGlobalFrameID();

//     // Declare and load parameters
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".max_vel", rclcpp::ParameterValue(0.26));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".min_vel", rclcpp::ParameterValue(0.0));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".max_rot_vel", rclcpp::ParameterValue(1.82));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".min_rot_vel", rclcpp::ParameterValue(-1.82));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(2.84));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.5));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".dt", rclcpp::ParameterValue(0.1));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".v_samples", rclcpp::ParameterValue(20));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".w_samples", rclcpp::ParameterValue(20));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".robot_radius", rclcpp::ParameterValue(0.1));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.15));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".goal_dist_weight", rclcpp::ParameterValue(0.6));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".obstacle_weight", rclcpp::ParameterValue(0.3));
//     nav2_util::declare_parameter_if_not_declared(
//         node_, plugin_name_ + ".speed_weight", rclcpp::ParameterValue(0.1));

//     params_.max_vel = node_->get_parameter(plugin_name_ + ".max_vel").as_double();
//     params_.min_vel = node_->get_parameter(plugin_name_ + ".min_vel").as_double();
//     params_.max_rot_vel = node_->get_parameter(plugin_name_ + ".max_rot_vel").as_double();
//     params_.min_rot_vel = node_->get_parameter(plugin_name_ + ".min_rot_vel").as_double();
//     params_.acc_lim_x = node_->get_parameter(plugin_name_ + ".acc_lim_x").as_double();
//     params_.acc_lim_theta = node_->get_parameter(plugin_name_ + ".acc_lim_theta").as_double();
//     params_.sim_time = node_->get_parameter(plugin_name_ + ".sim_time").as_double();
//     params_.dt = node_->get_parameter(plugin_name_ + ".dt").as_double();
//     params_.v_samples = node_->get_parameter(plugin_name_ + ".v_samples").as_int();
//     params_.w_samples = node_->get_parameter(plugin_name_ + ".w_samples").as_int();
//     params_.robot_radius = node_->get_parameter(plugin_name_ + ".robot_radius").as_double();
//     params_.goal_tolerance = node_->get_parameter(plugin_name_ + ".goal_tolerance").as_double();
//     params_.goal_dist_weight = node_->get_parameter(plugin_name_ + ".goal_dist_weight").as_double();
//     params_.obstacle_weight = node_->get_parameter(plugin_name_ + ".obstacle_weight").as_double();
//     params_.speed_weight = node_->get_parameter(plugin_name_ + ".speed_weight").as_double();

//     RCLCPP_INFO(node_->get_logger(), "DWA Planner configured");
// }

// void CustomDWAPlanner::activate() {
//     RCLCPP_INFO(node_->get_logger(), "DWA Planner activated");
// }

// void CustomDWAPlanner::deactivate() {
//     RCLCPP_INFO(node_->get_logger(), "DWA Planner deactivated");
// }

// void CustomDWAPlanner::cleanup() {
//     RCLCPP_INFO(node_->get_logger(), "DWA Planner cleaned up");
// }

// void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path) {
//     if (path.poses.empty()) {
//         RCLCPP_WARN(node_->get_logger(), "Received empty plan");
//         return;
//     }
//     goal_ = path.poses.back();
//     RCLCPP_INFO(node_->get_logger(), "New goal set: (%.2f, %.2f)",
//                 goal_.pose.position.x, goal_.pose.position.y);
// }

// void CustomDWAPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage) {
//     if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
//         speed_limit_ = params_.max_vel;
//     } else {
//         speed_limit_ = percentage ? params_.max_vel * speed_limit / 100.0 : speed_limit;
//     }
//     RCLCPP_INFO(node_->get_logger(), "Speed limit set to: %.2f m/s", speed_limit_);
// }

// geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
//     const geometry_msgs::msg::PoseStamped & pose,
//     const geometry_msgs::msg::Twist & velocity,
//     nav2_core::GoalChecker * /*goal_checker*/)
// {
//     geometry_msgs::msg::TwistStamped cmd_vel;
//     cmd_vel.header.stamp = node_->now();
//     cmd_vel.header.frame_id = "base_link";

//     // Check if goal is set
//     if (goal_.header.frame_id.empty()) {
//         RCLCPP_WARN(node_->get_logger(), "No goal set");
//         return cmd_vel;
//     }

//     // Check goal distance
//     const double dx = pose.pose.position.x - goal_.pose.position.x;
//     const double dy = pose.pose.position.y - goal_.pose.position.y;
//     const double distance = std::hypot(dx, dy);

//     if (distance <= params_.goal_tolerance) {
//         RCLCPP_INFO(node_->get_logger(), "Goal reached!");
//         return cmd_vel;  // Return zero velocity
//     }

//     // Calculate dynamic window
//     const double v_min = std::max(
//         params_.min_vel,
//         velocity.linear.x - params_.acc_lim_x * params_.dt
//     );
//     const double v_max = std::min(
//         speed_limit_ > 0.0 ? speed_limit_ : params_.max_vel,
//         velocity.linear.x + params_.acc_lim_x * params_.dt
//     );
//     const double w_min = std::max(
//         params_.min_rot_vel,
//         velocity.angular.z - params_.acc_lim_theta * params_.dt
//     );
//     const double w_max = std::min(
//         params_.max_rot_vel,
//         velocity.angular.z + params_.acc_lim_theta * params_.dt
//     );

//     // Generate velocity samples
//     std::vector<double> v_samples;
//     std::vector<double> w_samples;
//     for (int i = 0; i < params_.v_samples; ++i) {
//         v_samples.push_back(v_min + i * (v_max - v_min) / std::max(1, params_.v_samples - 1));
//     }
//     for (int i = 0; i < params_.w_samples; ++i) {
//         w_samples.push_back(w_min + i * (w_max - w_min) / std::max(1, params_.w_samples - 1));
//     }

//     // Evaluate trajectories
//     double best_cost = std::numeric_limits<double>::max();
//     double best_v = 0.0;
//     double best_w = 0.0;

//     for (const double v : v_samples) {
//         for (const double w : w_samples) {
//             // Simulate trajectory
//             auto trajectory = simulateTrajectory(v, w, pose.pose);
            
//             // Skip unsafe trajectories
//             if (!isTrajectorySafe(trajectory)) continue;
            
//             // Calculate cost components
//             double goal_dist, obstacle_dist;
//             const double cost = calculateCost(trajectory, v, w, goal_dist, obstacle_dist);
            
//             // Select best trajectory
//             if (cost < best_cost) {
//                 best_cost = cost;
//                 best_v = v;
//                 best_w = w;
//             }
//         }
//     }

//     // Set command velocity
//     if (best_cost < std::numeric_limits<double>::max()) {
//         cmd_vel.twist.linear.x = best_v;
//         cmd_vel.twist.angular.z = best_w;
//         RCLCPP_DEBUG(node_->get_logger(), "Cmd_vel: (%.2f, %.2f)", best_v, best_w);
//     } else {
//         RCLCPP_WARN(node_->get_logger(), "No valid trajectory found - stopping");
//     }

//     return cmd_vel;
// }

// std::vector<std::pair<double, double>> CustomDWAPlanner::simulateTrajectory(
//     double v, double w, const geometry_msgs::msg::Pose & start_pose)
// {
//     std::vector<std::pair<double, double>> trajectory;
//     double x = start_pose.position.x;
//     double y = start_pose.position.y;
//     double theta = tf2::getYaw(start_pose.orientation);
//     double time = 0.0;

//     while (time < params_.sim_time) {
//         // Update position
//         x += v * std::cos(theta) * params_.dt;
//         y += v * std::sin(theta) * params_.dt;
//         theta += w * params_.dt;
        
//         // Store point
//         trajectory.push_back({x, y});
//         time += params_.dt;
//     }
//     return trajectory;
// }

// bool CustomDWAPlanner::isTrajectorySafe(
//     const std::vector<std::pair<double, double>> & trajectory)
// {
//     for (const auto & point : trajectory) {
//         // Convert to map coordinates
//         unsigned int mx, my;
//         if (!costmap_->worldToMap(point.first, point.second, mx, my)) {
//             return false;  // Outside map bounds
//         }

//         // Check for obstacles
//         if (costmap_->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
//             return false;
//         }
//     }
//     return true;
// }

// double CustomDWAPlanner::calculateCost(
//     const std::vector<std::pair<double, double>> & trajectory,
//     double v, double w,
//     double & goal_dist, double & obstacle_dist)
// {
//     if (trajectory.empty()) {
//         goal_dist = obstacle_dist = std::numeric_limits<double>::max();
//         return goal_dist;
//     }

//     // Goal distance (from endpoint)
//     const auto & endpoint = trajectory.back();
//     goal_dist = std::hypot(
//         endpoint.first - goal_.pose.position.x,
//         endpoint.second - goal_.pose.position.y
//     );

//     // Obstacle distance (minimum along trajectory)
//     obstacle_dist = std::numeric_limits<double>::max();
//     for (const auto & point : trajectory) {
//         unsigned int mx, my;
//         if (costmap_->worldToMap(point.first, point.second, mx, my)) {
//             const double dist = costmap_->getCost(mx, my) / 255.0;
//             if (dist < obstacle_dist) obstacle_dist = dist;
//         }
//     }

//     // Normalize costs
//     const double goal_cost = goal_dist / params_.goal_dist_weight;
//     const double obstacle_cost = 1.0 / (obstacle_dist + 1e-5) * params_.obstacle_weight;
//     const double speed_cost = (params_.max_vel - v) / params_.max_vel * params_.speed_weight;

//     return goal_cost + obstacle_cost + speed_cost;
// }

// }  // namespace custom_dwa_planner

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//     custom_dwa_planner::CustomDWAPlanner, 
//     nav2_core::Controller)







#include "custom_dwa_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace custom_dwa_planner {

void CustomDWAPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> /*tf*/,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    plugin_name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    
    // Initialize state variables
    current_pose_ = geometry_msgs::msg::PoseStamped();
    current_velocity_ = geometry_msgs::msg::Twist();
    goal_reached_ = false;
    
    // Declare parameters with default values
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_vel", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".min_vel", rclcpp::ParameterValue(-0.2));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_rot_vel", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".min_rot_vel", rclcpp::ParameterValue(-1.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(1.0));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".sim_time", rclcpp::ParameterValue(1.5));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".dt", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".v_samples", rclcpp::ParameterValue(20));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".w_samples", rclcpp::ParameterValue(20));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".robot_radius", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".alpha", rclcpp::ParameterValue(0.6));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".beta", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".gamma", rclcpp::ParameterValue(0.2));

    // Load parameters
    params_.max_vel = node_->get_parameter(plugin_name_ + ".max_vel").as_double();
    params_.min_vel = node_->get_parameter(plugin_name_ + ".min_vel").as_double();
    params_.max_rot_vel = node_->get_parameter(plugin_name_ + ".max_rot_vel").as_double();
    params_.min_rot_vel = node_->get_parameter(plugin_name_ + ".min_rot_vel").as_double();
    params_.acc_lim_x = node_->get_parameter(plugin_name_ + ".acc_lim_x").as_double();
    params_.acc_lim_theta = node_->get_parameter(plugin_name_ + ".acc_lim_theta").as_double();
    params_.sim_time = node_->get_parameter(plugin_name_ + ".sim_time").as_double();
    params_.dt = node_->get_parameter(plugin_name_ + ".dt").as_double();
    params_.v_samples = node_->get_parameter(plugin_name_ + ".v_samples").as_int();
    params_.w_samples = node_->get_parameter(plugin_name_ + ".w_samples").as_int();
    params_.robot_radius = node_->get_parameter(plugin_name_ + ".robot_radius").as_double();
    params_.goal_tolerance = node_->get_parameter(plugin_name_ + ".goal_tolerance").as_double();
    params_.alpha = node_->get_parameter(plugin_name_ + ".alpha").as_double();
    params_.beta = node_->get_parameter(plugin_name_ + ".beta").as_double();
    params_.gamma = node_->get_parameter(plugin_name_ + ".gamma").as_double();

    RCLCPP_INFO(node_->get_logger(), "DWA Planner configured");
}

void CustomDWAPlanner::activate()
{
    // Create subscribers with proper QoS
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        rclcpp::SensorDataQoS(),
        std::bind(&CustomDWAPlanner::scanCallback, this, _1));
        
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&CustomDWAPlanner::odomCallback, this, _1));
    
    // Create publisher for velocity commands
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // Create visualization publishers
    trajectories_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/trajectories", 10);
    best_trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/best_trajectory", 10);
    goal_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/goal_marker", 10);
    
    RCLCPP_INFO(node_->get_logger(), "DWA Planner activated");
}

void CustomDWAPlanner::deactivate()
{
    scan_sub_.reset();
    odom_sub_.reset();
    cmd_vel_pub_.reset();
    trajectories_pub_.reset();
    best_trajectory_pub_.reset();
    goal_marker_pub_.reset();
    
    RCLCPP_INFO(node_->get_logger(), "DWA Planner deactivated");
}

void CustomDWAPlanner::cleanup()
{
    scan_sub_.reset();
    odom_sub_.reset();
    cmd_vel_pub_.reset();
    trajectories_pub_.reset();
    best_trajectory_pub_.reset();
    goal_marker_pub_.reset();
    
    RCLCPP_INFO(node_->get_logger(), "DWA Planner cleaned up");
}

void CustomDWAPlanner::setPlan(const nav_msgs::msg::Path & path)
{
    if (path.poses.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received empty plan");
        return;
    }
    goal_ = path.poses.back();
    goal_reached_ = false;
    publishGoalMarker();
    RCLCPP_INFO(node_->get_logger(), "New goal set: (%.2f, %.2f)", 
                goal_.pose.position.x, goal_.pose.position.y);
}

constexpr double NO_SPEED_LIMIT = -1.0;

void CustomDWAPlanner::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
    if (speed_limit == NO_SPEED_LIMIT) {
        speed_limit_ = params_.max_vel;
    } else {
        speed_limit_ = percentage ? params_.max_vel * speed_limit / 100.0 : speed_limit;
    }
    RCLCPP_INFO(node_->get_logger(), "Speed limit set to: %.2f m/s", speed_limit_);
}

geometry_msgs::msg::TwistStamped CustomDWAPlanner::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/)
{
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_link";
    
    // Update current pose from controller input
    current_pose_ = pose;
    
    // Check if we have necessary data
    if (obstacles_.empty() || current_pose_.header.frame_id.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Waiting for sensor data...");
        return cmd_vel;
    }
    
    // Check if goal is reached
    const double dx = current_pose_.pose.position.x - goal_.pose.position.x;
    const double dy = current_pose_.pose.position.y - goal_.pose.position.y;
    const double distance = std::hypot(dx, dy);
    
    if (distance <= params_.goal_tolerance) {
        if (!goal_reached_) {
            RCLCPP_INFO(node_->get_logger(), "Goal reached!");
            goal_reached_ = true;
        }
        return cmd_vel;  // Return zero velocity
    }
    
    // Reset goal reached flag if moved away
    if (goal_reached_ && distance > params_.goal_tolerance * 1.5) {
        goal_reached_ = false;
    }
    
    // Generate velocity samples within dynamic window
    const double v_min = std::max(
        params_.min_vel,
        current_velocity_.linear.x - params_.acc_lim_x * params_.dt
    );
    const double v_max = std::min(
        speed_limit_ > 0.0 ? speed_limit_ : params_.max_vel,
        current_velocity_.linear.x + params_.acc_lim_x * params_.dt
    );
    const double w_min = std::max(
        params_.min_rot_vel,
        current_velocity_.angular.z - params_.acc_lim_theta * params_.dt
    );
    const double w_max = std::min(
        params_.max_rot_vel,
        current_velocity_.angular.z + params_.acc_lim_theta * params_.dt
    );
    
    // Evaluate trajectories
    double best_cost = std::numeric_limits<double>::max();
    double best_v = 0.0;
    double best_w = 0.0;
    std::vector<std::vector<std::pair<double, double>>> all_trajectories;
    std::vector<double> costs;
    std::vector<std::pair<double, double>> best_trajectory;
    
    for (int i = 0; i < params_.v_samples; ++i) {
        const double v = v_min + i * (v_max - v_min) / std::max(1, params_.v_samples - 1);
        
        for (int j = 0; j < params_.w_samples; ++j) {
            const double w = w_min + j * (w_max - w_min) / std::max(1, params_.w_samples - 1);
            
            // Simulate trajectory
            auto trajectory = simulateTrajectory(v, w);
            
            // Calculate cost
            double goal_dist, obstacle_dist;
            const double cost = calculateCost(trajectory, v, w, goal_dist, obstacle_dist);
            
            // Store for visualization
            all_trajectories.push_back(trajectory);
            costs.push_back(cost);
            
            // Check if best valid trajectory
            if (cost < best_cost && isTrajectorySafe(trajectory)) {
                best_cost = cost;
                best_v = v;
                best_w = w;
                best_trajectory = trajectory;
            }
        }
    }
    
    // Publish best velocity command
    cmd_vel.twist.linear.x = best_v;
    cmd_vel.twist.angular.z = best_w;
    cmd_vel_pub_->publish(cmd_vel.twist);
    
    // Publish trajectories for visualization
    publishTrajectories(all_trajectories, costs, best_trajectory);
    
    RCLCPP_DEBUG(node_->get_logger(), 
                 "Best vel: (%.2f, %.2f), Cost: %.2f, Dist: %.2f", 
                 best_v, best_w, best_cost, distance);
    
    return cmd_vel;
}

void CustomDWAPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_velocity_ = msg->twist.twist;
}

void CustomDWAPlanner::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    obstacles_.clear();
    const double angle_min = msg->angle_min;
    const double angle_increment = msg->angle_increment;
    
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        const double range = msg->ranges[i];
        
        // Skip invalid measurements
        if (!std::isfinite(range)) continue ;
        
        // Calculate angle for this ray
        const double angle = angle_min + i * angle_increment;
        
        // Convert to Cartesian in robot frame
        const double x = range * std::cos(angle);
        const double y = range * std::sin(angle);
        
        // Skip points too close to robot
        if (std::hypot(x, y) < params_.robot_radius) continue;
        
        // Transform to world frame
        const double world_x = current_pose_.pose.position.x + 
                              x * std::cos(tf2::getYaw(current_pose_.pose.orientation)) - 
                              y * std::sin(tf2::getYaw(current_pose_.pose.orientation));
        const double world_y = current_pose_.pose.position.y + 
                              x * std::sin(tf2::getYaw(current_pose_.pose.orientation)) + 
                              y * std::cos(tf2::getYaw(current_pose_.pose.orientation));
        
        obstacles_.push_back({world_x, world_y});
    }
}

std::vector<std::pair<double, double>> CustomDWAPlanner::simulateTrajectory(
    double v, double w)
{
    std::vector<std::pair<double, double>> trajectory;
    double x = current_pose_.pose.position.x;
    double y = current_pose_.pose.position.y;
    double theta = tf2::getYaw(current_pose_.pose.orientation);
    double time = 0.0;
    
    while (time < params_.sim_time) {
        // Update position
        x += v * std::cos(theta) * params_.dt;
        y += v * std::sin(theta) * params_.dt;
        theta += w * params_.dt;
        
        // Normalize orientation
        theta = std::atan2(std::sin(theta), std::cos(theta));
        
        trajectory.push_back({x, y});
        time += params_.dt;
    }
    return trajectory;
}

double CustomDWAPlanner::calculateCost(
    const std::vector<std::pair<double, double>> & trajectory,
    double v, 
    double w,
    double & goal_dist, double & min_obstacle_dist)
{
    if (trajectory.empty()) {
        goal_dist = min_obstacle_dist = std::numeric_limits<double>::max();
        return goal_dist;
    }
    
    // Goal distance from endpoint
    const auto& endpoint = trajectory.back();
    goal_dist = std::hypot(
        endpoint.first - goal_.pose.position.x,
        endpoint.second - goal_.pose.position.y
    );
    
    // Find minimum obstacle distance along trajectory
    min_obstacle_dist = std::numeric_limits<double>::max();
    for (const auto& point : trajectory) {
        for (const auto& obstacle : obstacles_) {
            const double dist = std::hypot(
                point.first - obstacle.first,
                point.second - obstacle.second
            );
            if (dist < min_obstacle_dist) {
                min_obstacle_dist = dist;
            }
        }
    }
    
    // Smoothness cost (change in angular velocity)
    const double smoothness_cost = std::abs(w - current_velocity_.angular.z);
    
    return params_.alpha * goal_dist +
           params_.beta * (1.0 / std::max(min_obstacle_dist, 0.01)) +
           params_.gamma * smoothness_cost;
}

bool CustomDWAPlanner::isTrajectorySafe(
    const std::vector<std::pair<double, double>> & trajectory)
{
    for (const auto& point : trajectory) {
        for (const auto& obstacle : obstacles_) {
            const double dist = std::hypot(
                point.first - obstacle.first,
                point.second - obstacle.second
            );
            if (dist < params_.robot_radius) {
                return false;
            }
        }
    }
    return true;
}

void CustomDWAPlanner::publishTrajectories(
    const std::vector<std::vector<std::pair<double, double>>> & all_trajectories,
    const std::vector<double> & costs,
    const std::vector<std::pair<double, double>> & best_trajectory)
{
    // Create marker array
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "trajectories";
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.pose.orientation.w = 1.0;
    
    // Find min/max cost for color scaling
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double max_cost = *std::max_element(costs.begin(), costs.end());
    double cost_range = std::max(max_cost - min_cost, 0.01);
    
    for (size_t i = 0; i < all_trajectories.size(); ++i) {
        marker.id = i;
        marker.points.clear();
        marker.colors.clear();
        
        // Set color based on normalized cost
        const double normalized_cost = (costs[i] - min_cost) / cost_range;
        marker.color.r = normalized_cost;
        marker.color.g = 1.0 - normalized_cost;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        for (const auto& point : all_trajectories[i]) {
            geometry_msgs::msg::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = 0.0;
            marker.points.push_back(p);
            marker.colors.push_back(marker.color);
        }
        marker_array.markers.push_back(marker);
    }
    
    trajectories_pub_->publish(marker_array);
    
    // Publish best trajectory
    if (!best_trajectory.empty()) {
        visualization_msgs::msg::Marker best_marker;
        best_marker.header = marker.header;
        best_marker.ns = "best_trajectory";
        best_marker.id = 0;
        best_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        best_marker.action = visualization_msgs::msg::Marker::ADD;
        best_marker.scale.x = 0.05;
        best_marker.color.r = 0.0;
        best_marker.color.g = 0.0;
        best_marker.color.b = 1.0;
        best_marker.color.a = 1.0;
        
        for (const auto& point : best_trajectory) {
            geometry_msgs::msg::Point p;
            p.x = point.first;
            p.y = point.second;
            p.z = 0.0;
            best_marker.points.push_back(p);
        }
        best_trajectory_pub_->publish(best_marker);
    }
}

void CustomDWAPlanner::publishGoalMarker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = goal_.pose.position.x;
    marker.pose.position.y = goal_.pose.position.y;
    marker.pose.position.z = 0.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    goal_marker_pub_->publish(marker);
}

}  // namespace custom_dwa_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    custom_dwa_planner::CustomDWAPlanner, 
    nav2_core::Controller)