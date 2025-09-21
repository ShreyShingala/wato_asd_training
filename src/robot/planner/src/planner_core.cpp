#include "planner_core.hpp"
#include <algorithm>
#include <limits>

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
    // Either start or goal outside the grid
    return path;
  }

  // Relocate start/goal to nearest free cell within a small radius
  CellIndex start = start_seed, goal = goal_seed;
  if (!findNearestFree(map, start_seed, 3, start, occ_thresh)) return path;
  if (!findNearestFree(map, goal_seed,  3, goal,  occ_thresh)) return path;

  // A* setup
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_set<int> closed; // track by flat index

  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    // Euclidean in grid units
    return std::hypot(double(a.x - b.x), double(a.y - b.y));
  };

  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  // 8-connected neighbors (diagonals allowed)
  const std::vector<std::pair<CellIndex, double>> dirs = {
    {{1,0}, 1.0}, {{-1,0}, 1.0}, {{0,1}, 1.0}, {{0,-1}, 1.0},
    {{1,1}, std::sqrt(2.0)}, {{-1,1}, std::sqrt(2.0)}, {{1,-1}, std::sqrt(2.0)}, {{-1,-1}, std::sqrt(2.0)}
  };

  CellIndex reached = start;
  double min_goal_dist = std::numeric_limits<double>::max();
  while (!open_set.empty())
  {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    double goal_dist = heuristic(current, goal);
    if (goal_dist < min_goal_dist) {
      min_goal_dist = goal_dist;
      reached = current;
    }

    // If reached goal cell, stop exploring
    if (current == goal) {
      reached = goal;
      break;
    }

    const int cFlat = flatIndex(map, current);
    if (closed.count(cFlat)) continue;
    closed.insert(cFlat);

    for (const auto &dir : dirs)
    {
      const CellIndex &d = dir.first;
      double move_cost = dir.second;
      CellIndex nb{current.x + d.x, current.y + d.y};
      if (!inBounds(map, nb)) continue;

      if (!isFree(map, nb, occ_thresh)) continue;

      // For diagonal moves, check that both adjacent cardinal cells are also free (no corner cutting)
      if (std::abs(d.x) == 1 && std::abs(d.y) == 1) {
        CellIndex adj1{current.x + d.x, current.y};
        CellIndex adj2{current.x, current.y + d.y};
        if (!isFree(map, adj1, occ_thresh) || !isFree(map, adj2, occ_thresh)) continue;
      }

      const double tentative_g = g_score[current] + move_cost;
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        came_from[nb] = current;
        g_score[nb] = tentative_g;
        const double f = tentative_g + heuristic(nb, goal);
        open_set.emplace(nb, f);
      }
    }
  }


  // Reconstruct path from the actual reached cell (may not be the goal if unreachable)
  if (came_from.find(reached) == came_from.end() && !(start == reached)) {
    // Could not reach any cell
    return path;
  }

  std::vector<CellIndex> rev;
  CellIndex cur = reached;
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
