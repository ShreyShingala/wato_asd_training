#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner_core {
// Returns true if robot has reached the goal
bool goalReached(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::PointStamped& goal);
// Logs occupancy at start and goal
void logStartGoalOccupancy(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::PointStamped& goal, rclcpp::Logger logger);
// Plans path and publishes result
void planPath(
  bool goal_received,
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& robot_pose,
  const geometry_msgs::msg::PointStamped& goal,
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Logger logger);
}
#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const { return (x == other.x && y == other.y); }
  bool operator!=(const CellIndex &other) const { return !(*this == other); }
};

// Hash for CellIndex
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Node for open set (min-heap by f)
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) const
  {
    return a.f_score > b.f_score;
  }
};

namespace planner_core
{
  // Conversions
  CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid &map);
  geometry_msgs::msg::Pose indexToPose(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map);

  // Occupancy helpers
  bool isFree(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c, int8_t occ_thresh = 50);
  bool findNearestFree(const nav_msgs::msg::OccupancyGrid &map,
                       const CellIndex &seed, int radius,
                       CellIndex &out, int8_t occ_thresh = 50);

  // A* path planning (goal is a Point; start is a Pose for orientation compatibility if needed)
  nav_msgs::msg::Path aStarSearch(const nav_msgs::msg::OccupancyGrid &map,
                                  const geometry_msgs::msg::Pose &start_pose,
                                  const geometry_msgs::msg::Point &goal_point,
                                  int8_t occ_thresh = 50);
}

#endif  // PLANNER_CORE_HPP_
