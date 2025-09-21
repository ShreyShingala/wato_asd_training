#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>

namespace robot {

class ControlCore {
public:
  ControlCore(const rclcpp::Logger& logger);

  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
    const nav_msgs::msg::Path &path,
    const nav_msgs::msg::Odometry &odom,
    double lookahead_distance) const;

  geometry_msgs::msg::Twist computeVelocity(
    const geometry_msgs::msg::PoseStamped &target,
    const nav_msgs::msg::Odometry &odom,
    double linear_speed,
    double lookahead_distance) const;

  double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const;
  double extractYaw(const geometry_msgs::msg::Quaternion &quat) const;

private:
  rclcpp::Logger logger_;
};

} // namespace robot

#endif
