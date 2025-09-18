#include <chrono>
#include <memory>
#include <cstdint>
#include <algorithm>
#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Step 1: Process the scan and update the costmap
    costmap_.processLaserScan(msg);

    // Step 2: Convert costmap to OccupancyGrid and publish immediately
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->get_clock()->now();
    grid_msg.header.frame_id = "map";

    float resolution = costmap_.getResolution();
    int width = costmap_.getWidth();
    int height = costmap_.getHeight();
    float origin_x = costmap_.getOriginX();
    float origin_y = costmap_.getOriginY();

    grid_msg.info.resolution = resolution;
    grid_msg.info.width = static_cast<unsigned int>(width);
    grid_msg.info.height = static_cast<unsigned int>(height);
    grid_msg.info.origin.position.x = origin_x;
    grid_msg.info.origin.position.y = origin_y;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    const auto& grid = costmap_.getGrid();

    grid_msg.data.clear();
    grid_msg.data.reserve(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int v = grid[y][x];
            // convert to int8_t in valid occupancy range: -1..100
            if (v < -1) v = -1;
            if (v > 100) v = 100;
            grid_msg.data.push_back(static_cast<int8_t>(v));
        }
    }

    costmap_pub_->publish(grid_msg);
    //RCLCPP_INFO(this->get_logger(), "Published costmap %d x %d", width, height); // Commented out to reduce log spam
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}