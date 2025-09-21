
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
    auto lookahead_point = core_.findLookaheadPoint(*current_path_, *robot_odom_, lookahead_distance_);
    if (!lookahead_point) {
        // Stop if no valid lookahead point
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);
        return;
    }
    // Check if goal reached
    const auto &goal = current_path_->poses.back().pose.position;
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
