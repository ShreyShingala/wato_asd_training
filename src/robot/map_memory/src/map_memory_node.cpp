#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Subscribe to /costmap
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  
  // Subscribe to /odom/filtered
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  
  // Publisher for /map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  
  // Timer for periodic map update
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));


  // --- Map initialization to match testing.cpp ---
  global_map_ = nav_msgs::msg::OccupancyGrid();
  global_map_.header.frame_id = "map";
  global_map_.header.stamp = this->get_clock()->now();
  global_map_.info.resolution = resolution;
  global_map_.info.origin.position.x = -(sideLength / 2);
  global_map_.info.origin.position.y = -(sideLength / 2);
  global_map_.info.width = totalSideUnits;
  global_map_.info.height = totalSideUnits;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.resize(global_map_.info.width * global_map_.info.height, 0); // 0 as unknown

  // Add initialCount for forced updates
  initialCount = 0;

  integrateCostmap();
  map_pub_->publish(global_map_);
  RCLCPP_INFO(this->get_logger(), "MapMemoryNode initialized and published");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  costmap_updated_ = true;
  if (initialCount < 5) {
    should_update_map_ = true; // force update for first few messages
  }
  //RCLCPP_INFO(this->get_logger(), "Received costmap update");
}

double MapMemoryNode::computeDistance(double x1, double y1, double x2, double y2) const {
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = *msg;
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (first_odom_) { //first time initialization
        last_x_ = x;
        last_y_ = y;
        first_odom_ = false;
        RCLCPP_INFO(this->get_logger(), "Initialized odometry tracking at x=%.2f, y=%.2f", x, y);
        return;
    }

    double distance = computeDistance(x, y, last_x_, last_y_);

    //RCLCPP_INFO(this->get_logger(), "Odometry update: x=%.2f, y=%.2f, distance since last=%.2f", new_x, new_y, distance);


    if (distance >= distance_threshold_) {
      last_x_ = x;
      last_y_ = y;
      robot_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
      should_update_map_ = true;
      //RCLCPP_INFO(this->get_logger(), "Robot moved %.2f meters since last update.", distance);
    }
}

void MapMemoryNode::updateMap() { //Bound to timer to also auto update
  if (should_update_map_ && costmap_updated_) {
    integrateCostmap();
    map_pub_->publish(global_map_);
    should_update_map_ = false;
    costmap_updated_ = false;
    initialCount++;
    RCLCPP_INFO(this->get_logger(), "Published updated global map.");
  }
}

void MapMemoryNode::integrateCostmap() {
  RCLCPP_INFO(this->get_logger(), "Integrating latest costmap into global map...");

  if (latest_costmap_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "No latest costmap to integrate.");
    return;
  }

  // local costmap params
  const double local_res = latest_costmap_.info.resolution;
  const int local_w = static_cast<int>(latest_costmap_.info.width);
  const int local_h = static_cast<int>(latest_costmap_.info.height);
  const double local_origin_x = latest_costmap_.info.origin.position.x;
  const double local_origin_y = latest_costmap_.info.origin.position.y;

  // global map params
  const double global_res = global_map_.info.resolution;
  const int global_w = static_cast<int>(global_map_.info.width);
  const int global_h = static_cast<int>(global_map_.info.height);
  const double global_origin_x = global_map_.info.origin.position.x;
  const double global_origin_y = global_map_.info.origin.position.y;

  //RCLCPP_INFO(this->get_logger(),"costmap frame='%s' local_origin=(%.3f,%.3f) local_res=%.3f size=%d x %d",local_frame.c_str(), local_origin_x, local_origin_y, local_res, local_w, local_h);
  //RCLCPP_INFO(this->get_logger(), "global_origin=(%.3f,%.3f) global_res=%.3f size=%d x %d robot=(%.3f,%.3f) yaw=%.3f",global_origin_x, global_origin_y, global_res, global_w, global_h, robot_x, robot_y, yaw);

  // diagnostics counters
  int64_t in_bounds = 0;
  int64_t out_of_bounds = 0;
  int64_t updated_cells = 0;
  int64_t considered_cells = 0;

  // for each cell in the costmap
  for (int ly = 0; ly < local_h; ++ly) {
    for (int lx = 0; lx < local_w; ++lx) {
      int costmap_idx = ly * local_w + lx;
      int8_t costmap_value = latest_costmap_.data[costmap_idx];

      if (costmap_value == -1){
        continue;
      }

      // compute local cell center in local costmap coordinates (meters)
      double cell_local_x = local_origin_x + lx * local_res + local_res / 2.0;
      double cell_local_y = local_origin_y + ly * local_res + local_res / 2.0;

      double world_x, world_y;

      // transform to global frame using robot pose
      world_x = last_x_ + std::cos(robot_yaw_) * cell_local_x - std::sin(robot_yaw_) * cell_local_y;
      world_y = last_y_ + std::sin(robot_yaw_) * cell_local_x + std::cos(robot_yaw_) * cell_local_y;

      // global coordinates to global map indices
      int gx = static_cast<int>((world_x - global_origin_x) / global_res);
      int gy = static_cast<int>((world_y - global_origin_y) / global_res);

      if (gy >= 0 && gy < global_h && gx >= 0 && gx < global_w) {
        int global_idx = gy * global_w + gx;
        global_map_.data[global_idx] = costmap_value;
      }else {
        ++out_of_bounds;
        continue;
      }
      ++in_bounds;
      ++considered_cells;
    }
  }
  global_map_.header.stamp = this->get_clock()->now();
  //RCLCPP_INFO(this->get_logger(),
  //  "Integration summary: considered=%lld in_bounds=%lld out_of_bounds=%lld updated=%lld",
  //  (long long)considered_cells, (long long)in_bounds, (long long)out_of_bounds, (long long)updated_cells);
}


/* //THIS IS THE OLD FUNCTION I MADE WHICH APPARENTLY WAS BAD

void MapMemoryNode::integrateCostmap() {
  RCLCPP_INFO(this->get_logger(), "INTEGRATING COSTMAP TO THEN PUBLISH LATER");


  if (latest_costmap_.data.empty()) {   // Check if the latest costmap is empty
    RCLCPP_INFO(this->get_logger(), "DA HECK??? Why empty?");
    return;
  }


  double robot_x = latest_odom_.pose.pose.position.x;   // Get the robot's current position
  double robot_y = latest_odom_.pose.pose.position.y;

  // Find offset and aligns the costmap's origin with the robot's current position
  int global_origin_x = static_cast<int>((robot_x - latest_costmap_.info.origin.position.x - global_map_.info.origin.position.x) / global_map_.info.resolution);
  int global_origin_y = static_cast<int>((robot_y - latest_costmap_.info.origin.position.y - global_map_.info.origin.position.y) / global_map_.info.resolution);

  // Loop through each cell in the local costmap
  for (auto y = 0; y < latest_costmap_.info.height; ++y) {
    for (auto x = 0; x < latest_costmap_.info.width; ++x) {
      
      // Compute the 1D index for the costmap cell
      int costmap_index = y * latest_costmap_.info.width + x;

      // Transform the local costmap cell (x, y) into the global map frame, accounting for robot yaw
      int globalx = global_origin_x + static_cast<int>(x * std::cos(robot_yaw_) - y * std::sin(robot_yaw_));
      int globaly = global_origin_y + static_cast<int>(x * std::sin(robot_yaw_) + y * std::cos(robot_yaw_));

      // Check bounds for the global map
      if (globalx < 0 || globaly < 0 || globalx >= static_cast<int>(global_map_.info.width) || globaly >= static_cast<int>(global_map_.info.height)){
        continue;
      }

      // Compute the 1D index for the global map cell
      int global_idx = globaly * global_map_.info.width + globalx;
      int costmap_val = latest_costmap_.data[costmap_index];

      // Only update the global map if the costmap cell is known (not -1)
      if (costmap_val != -1) {
        global_map_.data[global_idx] = costmap_val;
      }
    }
  }
} 

*/

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
