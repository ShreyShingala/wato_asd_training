
#include "control_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>


PurePursuitController::PurePursuitController()
    : Node("pure_pursuit_controller"), core_(this->get_logger()) {
    lookahead_distance_ = 1.0;
    goal_tolerance_ = 0.1;
    linear_speed_ = 0.5;

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void PurePursuitController::controlLoop() {
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        return;
    }
    // Loosen pruning: only remove points far behind, always keep at least the closest point
    nav_msgs::msg::Path pruned_path = *current_path_;
    const auto &robot_pose = robot_odom_->pose.pose;
    double robot_yaw = core_.extractYaw(robot_pose.orientation);
    const auto &robot_pos = robot_pose.position;
    auto &poses = pruned_path.poses;
    // Find the first point that is not far behind (dot > -0.5)
    auto keep_from = poses.begin();
    for (auto it = poses.begin(); it != poses.end(); ++it) {
        double dx = it->pose.position.x - robot_pos.x;
        double dy = it->pose.position.y - robot_pos.y;
        double heading_x = std::cos(robot_yaw);
        double heading_y = std::sin(robot_yaw);
        double dot = dx * heading_x + dy * heading_y;
        if (dot > -0.5) {
            keep_from = it;
            break;
        }
    }
    // Always keep at least the closest point
    if (keep_from != poses.begin()) {
        poses.erase(poses.begin(), keep_from);
    }
    if (poses.empty()) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }
    auto lookahead_point = core_.findLookaheadPoint(pruned_path, *robot_odom_, lookahead_distance_);
    if (!lookahead_point) {
        // Stop if no valid lookahead point
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }
    // Check if goal reached
    const auto &goal = pruned_path.poses.back().pose.position;
    const auto &robot = robot_odom_->pose.pose.position;
    if (core_.computeDistance(goal, robot) < goal_tolerance_) {
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }
    auto cmd_vel = core_.computeVelocity(*lookahead_point, *robot_odom_, linear_speed_, lookahead_distance_);
    cmd_vel_pub_->publish(cmd_vel);
}



int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
