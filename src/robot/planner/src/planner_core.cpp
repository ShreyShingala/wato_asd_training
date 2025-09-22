
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>


#include "planner_core.hpp"
#include <algorithm>
#include <limits>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace planner_core {

bool goalReached(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::PointStamped& goal) {
  constexpr double xy_tol = 0.2; // meters
  double dx = robot_pose.position.x - goal.point.x;
  double dy = robot_pose.position.y - goal.point.y;
  return (dx * dx + dy * dy) < (xy_tol * xy_tol);
}

void logStartGoalOccupancy(const nav_msgs::msg::OccupancyGrid& map, const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::PointStamped& goal, rclcpp::Logger logger) {
  // Optionally log occupancy values at start and goal for debugging
  // Not strictly required for core logic
}

void planPath(
  bool goal_received,
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::Pose& robot_pose,
  const geometry_msgs::msg::PointStamped& goal,
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Logger logger)
{
  if (!goal_received) return;
  logStartGoalOccupancy(map, robot_pose, goal, logger);
  nav_msgs::msg::Path path = aStarSearch(map, robot_pose, goal.point, 50); // 50 occupancy threshold
  path.header.stamp = clock->now();
  path_pub->publish(path);
}

} // namespace planner_core

namespace planner_core
{

static inline bool inBounds(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c)
{
  return c.x >= 0 && c.y >= 0 &&
         c.x < static_cast<int>(map.info.width) &&
         c.y < static_cast<int>(map.info.height);
}

static inline int flatIndex(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c)
{
  return c.y * static_cast<int>(map.info.width) + c.x;
}

CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid &map)
{
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;

  // Use floor to handle negative coordinates correctly
  int mx = static_cast<int>(std::floor((x - ox) / res));
  int my = static_cast<int>(std::floor((y - oy) / res));
  return CellIndex(mx, my);
}

geometry_msgs::msg::Pose indexToPose(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map)
{
  geometry_msgs::msg::Pose pose;
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;

  pose.position.x = ox + (static_cast<double>(idx.x) + 0.5) * res;
  pose.position.y = oy + (static_cast<double>(idx.y) + 0.5) * res;
  pose.position.z = 0.0;
  return pose;
}

bool isFree(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c, int8_t occ_thresh)
{
  if (!inBounds(map, c)) return false;
  const int idx = flatIndex(map, c);
  const int8_t v = map.data[idx];
  // Treat unknown (-1) as free, and anything below threshold as free
  // v is signed int8_t; unknown is -1; occupied values are 0..100
  return (v < 0) || (v < occ_thresh);
}

bool findNearestFree(const nav_msgs::msg::OccupancyGrid &map,
                     const CellIndex &seed, int radius,
                     CellIndex &out, int8_t occ_thresh)
{
  if (isFree(map, seed, occ_thresh)) {
    out = seed;
    return true;
  }
  for (int r = 1; r <= radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        // Only check the perimeter of the square ring
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        CellIndex c{seed.x + dx, seed.y + dy};
        if (isFree(map, c, occ_thresh)) {
          out = c;
          return true;
        }
      }
    }
  }
  return false;
}

nav_msgs::msg::Path aStarSearch(const nav_msgs::msg::OccupancyGrid &map,
                                const geometry_msgs::msg::Pose &start_pose,
                                const geometry_msgs::msg::Point &goal_point,
                                int8_t occ_thresh)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    return path;
  }

  const CellIndex start_seed = worldToMap(start_pose.position.x, start_pose.position.y, map);
  const CellIndex goal_seed  = worldToMap(goal_point.x,             goal_point.y,             map);

  if (!inBounds(map, start_seed) || !inBounds(map, goal_seed)) {
    return path;
  }

  // A* search setup
  CellIndex start = start_seed;
  CellIndex goal = goal_seed;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  auto heuristic = [](const CellIndex& a, const CellIndex& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };
  struct CompareF {
    bool operator()(const AStarNode& a, const AStarNode& b) const {
      return a.f_score > b.f_score;
    }
  };
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

  // Relocate start/goal if in obstacle
  CellIndex start_free, goal_free;
  if (!findNearestFree(map, start, 5, start_free, occ_thresh)) return path;
  if (!findNearestFree(map, goal, 5, goal_free, occ_thresh)) return path;
  start = start_free;
  goal = goal_free;

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  std::unordered_set<CellIndex, CellIndexHash> closed;

  const std::vector<CellIndex> directions = {
    {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {-1,1}, {1,-1}, {-1,-1}
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();
    if (current == goal) break;
    if (closed.count(current)) continue;
    closed.insert(current);
    for (const auto& d : directions) {
      CellIndex nb{current.x + d.x, current.y + d.y};
      if (!inBounds(map, nb) || !isFree(map, nb, occ_thresh)) continue;
      double tentative_g = g_score[current] + heuristic(current, nb);
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        came_from[nb] = current;
        g_score[nb] = tentative_g;
        double f = tentative_g + heuristic(nb, goal);
        open_set.emplace(nb, f);
      }
    }
  }

  // Reconstruct path if we reached the goal (or relocated goal == start)
  if (came_from.find(goal) == came_from.end() && !(start == goal)) {
    // Could not reach goal
    return path;
  }

  // Build reverse path
  std::vector<CellIndex> rev;
  CellIndex cur = goal;
  rev.push_back(cur);
  while (cur != start) {
    auto it = came_from.find(cur);
    if (it == came_from.end()) break;
    cur = it->second;
    rev.push_back(cur);
  }
  std::reverse(rev.begin(), rev.end());

  // Convert to PoseStamped
  path.poses.reserve(rev.size());
  for (const auto &c : rev) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map.header.frame_id;
    ps.pose = indexToPose(c, map);
    path.poses.push_back(ps);
  }

  return path;
}


} // namespace planner_core
