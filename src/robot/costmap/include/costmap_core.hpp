#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace robot
{

class CostmapCore {
public:
  explicit CostmapCore(const rclcpp::Logger& logger);

  // main API
  void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

  // getters for external use (read-only)
  float getResolution() const { return resolution_; }
  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  float getOriginX() const { return origin_x_; }
  float getOriginY() const { return origin_y_; }
  const std::vector<std::vector<int>>& getGrid() const { return grid_; }

private:
  rclcpp::Logger logger_;

  // Costmap parameters (can later be made rclcpp::Parameters)
  float resolution_ = 0.1f; // meters per cell
  int width_ = 100;        // number of cells
  int height_ = 100;       // number of cells
  float origin_x_ = -5.0f; // meters (world)
  float origin_y_ = -5.0f; // meters (world)
  int max_cost_ = 100;
  int unknown_cost_ = -1;
  int default_value_ = 0;

  // the grid: grid_[y][x]
  std::vector<std::vector<int>> grid_;

  // helpers
  void InitializeGrid();
  void markObstacle(int x_idx, int y_idx);
  bool inBounds(int x_idx, int y_idx) const;
  void inflateObstacles();
  void ConvertToGrid(double range, double angle, int& x_grid, int& y_grid) const;
};

} // namespace robot

#endif
