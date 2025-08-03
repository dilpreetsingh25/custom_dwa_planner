// #ifndef CUSTOM_DWA_PLANNER_HPP
// #define CUSTOM_DWA_PLANNER_HPP

// #include <vector>
// #include <string>
// #include <memory>
// #include <limits>
// #include <utility>
// #include <cmath>
// #include <algorithm>

// #include "rclcpp/rclcpp.hpp"
// #include "nav2_core/controller.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "geometry_msgs/msg/twist_stamped.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include "tf2/utils.h"
// #include "pluginlib/class_list_macros.hpp"

// namespace custom_dwa_planner {

// class CustomDWAPlanner : public nav2_core::Controller {
// public:
//     CustomDWAPlanner() = default;
//     ~CustomDWAPlanner() override = default;

//     void configure(
//         const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
//         std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
//         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

//     void activate() override;
//     void deactivate() override;
//     void cleanup() override;

//     geometry_msgs::msg::TwistStamped computeVelocityCommands(
//         const geometry_msgs::msg::PoseStamped & pose,
//         const geometry_msgs::msg::Twist & velocity,
//         nav2_core::GoalChecker * goal_checker) override;

//     void setPlan(const nav_msgs::msg::Path & path) override;
    
//     void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

// private:
//     struct DWAParams {
//         double max_vel;
//         double min_vel;
//         double max_rot_vel;
//         double min_rot_vel;
//         double acc_lim_x;
//         double acc_lim_theta;
//         double sim_time;
//         double dt;
//         int v_samples;
//         int w_samples;
//         double robot_radius;
//         double goal_tolerance;
//         double goal_dist_weight;
//         double obstacle_weight;
//         double speed_weight;
//     } params_;

//     double speed_limit_{0.0};  // Add speed limit member


//     std::vector<std::pair<double, double>> simulateTrajectory(
//         double v, double w, const geometry_msgs::msg::Pose & start_pose);

//     double calculateCost(
//         const std::vector<std::pair<double, double>> & trajectory,
//         double v, double w, double & goal_dist);

//     bool isTrajectorySafe(const std::vector<std::pair<double, double>> & trajectory);

//     void publishTrajectories(
//         const std::vector<std::vector<std::pair<double, double>>> & all_trajectories,
//         const std::vector<double> & costs,
//         const std::vector<std::pair<double, double>> & best_trajectory);

//     void publishGoalMarker();

//     rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
//     std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
//     nav2_costmap_2d::Costmap2D * costmap_;
//     std::string plugin_name_;
//     geometry_msgs::msg::PoseStamped goal_;
//     bool goal_marker_published_;

//     // Publishers
//     rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectories_pub_;
//     rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr best_trajectory_pub_;
//     rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
// };

// }  // namespace custom_dwa_planner

// #endif  // CUSTOM_DWA_PLANNER_HPP





#ifndef CUSTOM_DWA_PLANNER_HPP
#define CUSTOM_DWA_PLANNER_HPP

#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "pluginlib/class_list_macros.hpp"

namespace custom_dwa_planner {

class CustomDWAPlanner : public nav2_core::Controller {
public:
    CustomDWAPlanner() = default;
    ~CustomDWAPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path & path) override;

    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped & pose,
        const geometry_msgs::msg::Twist & velocity,
        nav2_core::GoalChecker * goal_checker) override;

private:
    struct Params {
        double max_vel;
        double min_vel;
        double max_rot_vel;
        double min_rot_vel;
        double acc_lim_x;
        double acc_lim_theta;
        double sim_time;
        double dt;
        int v_samples;
        int w_samples;
        double robot_radius;
        double goal_tolerance;
        double alpha;
        double beta;
        double gamma;
    } params_;

    std::vector<std::pair<double, double>> simulateTrajectory(double v, double w);
    double calculateCost(
        const std::vector<std::pair<double, double>> & trajectory,
        double v,
         double w,
        double & goal_dist, double & min_obstacle_dist);
    bool isTrajectorySafe(const std::vector<std::pair<double, double>> & trajectory);
    void publishTrajectories(
        const std::vector<std::vector<std::pair<double, double>>> & all_trajectories,
        const std::vector<double> & costs,
        const std::vector<std::pair<double, double>> & best_trajectory);
    void publishGoalMarker();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Member variables
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string plugin_name_;
    std::string global_frame_;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    std::vector<std::pair<double, double>> obstacles_;
    double speed_limit_ = 0.0;
    bool goal_reached_ = false;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectories_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr best_trajectory_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
};

}  // namespace custom_dwa_planner

#endif  // CUSTOM_DWA_PLANNER_HPP