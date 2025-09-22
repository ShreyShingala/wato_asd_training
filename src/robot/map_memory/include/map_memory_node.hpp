#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/utils.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "map_memory_core.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapMemoryNode : public rclcpp::Node {
  // Store latest odometry
  nav_msgs::msg::Odometry latest_odom_;
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    double computeDistance(double x1, double y1, double x2, double y2) const;
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

     // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Data storage
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;

    // Robot position tracking
    double last_x_ = 0.0;
    double last_y_ = 0.0;
    double robot_yaw_ = 0.0;
    geometry_msgs::msg::Quaternion heading_;
    bool first_odom_ = true;
    double distance_threshold_ = 0.1; // meters
    int initialCount = 0;

    //Addtional Variables
    const double sideLength = 30.0; // meters
    const double resolution = 0.2;  // meters
    const int occupied = 100;
    const int partially_occupied = 50;
    const int totalSideUnits = static_cast<int>(sideLength / resolution);

    // Additional data storage
    bool should_update_map_ = false;
    bool costmap_updated_ = false;


    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

  // threshold for movement (use only one definition)

    void integrateCostmap();
};

#endif