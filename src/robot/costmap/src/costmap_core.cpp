#include "costmap_core.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) { //INITIALIZES GRID
    grid_.resize(height_, std::vector<int>(width_, default_value_));
}

void CostmapCore::InitializeGrid() { //SETS ALL VALYES TO 0
    for (auto& row : grid_) {
        std::fill(row.begin(), row.end(), default_value_);
    }
}

void CostmapCore::ConvertToGrid(double range, double angle, int& x_grid, int& y_grid) const { //CONVERTS TO GRID COORDINATES
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

bool CostmapCore::inBounds(int x_idx, int y_idx) const { //CHECKS IF IN BOUNDS
    return x_idx >= 0 && x_idx < width_ && y_idx >= 0 && y_idx < height_;
}

void CostmapCore::markObstacle(int x_idx, int y_idx) { //MARKS OBSTACLE IN GRID
    grid_[y_idx][x_idx] = max_cost_;
}

void CostmapCore::processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan) { //PROCESSES LASER SCAN
    //MAYBE RESET EACH INCOMING SCAN
    InitializeGrid();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
        double range = static_cast<double>(scan->ranges[i]);

        if (!std::isfinite(range)) continue;
        if (range < static_cast<double>(scan->range_min) || range > static_cast<double>(scan->range_max)) continue;

        int x_grid = 0, y_grid = 0;
        ConvertToGrid(range, angle, x_grid, y_grid);
        if (inBounds(x_grid, y_grid)) {
            markObstacle(x_grid, y_grid);
        }
    }

    inflateObstacles();
    RCLCPP_DEBUG(logger_, "Processed LaserScan and updated costmap.");
}

void CostmapCore::inflateObstacles() {
    std::vector<std::vector<int>> inflatedGrid = grid_;

    const double inflation_radius = 1.0; // meters
    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
    const int max_inflate_cost = max_cost_;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (grid_[y][x] == max_cost_) {
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;

                        if (!inBounds(nx, ny)) continue;

                        double dist = std::hypot(dx * resolution_, dy * resolution_);
                        if (dist > inflation_radius) continue;

                        int cost = static_cast<int>(std::round(max_inflate_cost * (1.0 - dist / inflation_radius)));
                        cost = std::clamp(cost, 0, max_inflate_cost);

                        if (cost > inflatedGrid[ny][nx]) {
                            inflatedGrid[ny][nx] = cost;
                        }
                    }
                }
            }
        }
    }

    grid_.swap(inflatedGrid);
}
} // namespace robot
