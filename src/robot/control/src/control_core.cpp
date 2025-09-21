
#include "control_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robot {

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path &path,
  const nav_msgs::msg::Odometry &odom,
  double lookahead_distance) const {
  if (path.poses.empty()) return std::nullopt;
  const auto &robot = odom.pose.pose.position;
  for (const auto &pose_stamped : path.poses) {
    if (computeDistance(robot, pose_stamped.pose.position) > lookahead_distance) {
      return pose_stamped;
    }
  }
  // If no point found, return last point (goal)
  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped &target,
  const nav_msgs::msg::Odometry &odom,
  double linear_speed,
  double lookahead_distance) const {
  geometry_msgs::msg::Twist cmd_vel;
  const auto &robot_pose = odom.pose.pose;
  double robot_yaw = extractYaw(robot_pose.orientation);
  double dx = target.pose.position.x - robot_pose.position.x;
  double dy = target.pose.position.y - robot_pose.position.y;
  double target_yaw = std::atan2(dy, dx);
  double alpha = target_yaw - robot_yaw;
  // Normalize angle to [-pi, pi]
  while (alpha > M_PI) alpha -= 2 * M_PI;
  while (alpha < -M_PI) alpha += 2 * M_PI;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = 2.0 * linear_speed * std::sin(alpha) / lookahead_distance;
  return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) const {
  tf2::Quaternion q(
    quat.x,
    quat.y,
    quat.z,
    quat.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

} // namespace robot
