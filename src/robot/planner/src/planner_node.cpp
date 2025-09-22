#include "planner_node.hpp"
#include <cmath>

PlannerNode::PlannerNode()
: Node("planner_node"),
  state_(State::WAITING_FOR_GOAL),
  goal_received_(false),
  timeout_sec_(10.0)
{
  // Subscribers
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  // Publisher
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  // Timer (1Hz checker)
  replan_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&PlannerNode::replanTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PlannerNode initialized.");
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = *msg;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
    RCLCPP_INFO(this->get_logger(), "Map updated. Replanning...");
    planner_core::planPath(goal_received_, current_map_, robot_pose_, goal_, path_pub_, this->get_clock(), this->get_logger());
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "New goal received (%.2f, %.2f).",
              goal_.point.x, goal_.point.y);

  planner_core::planPath(goal_received_, current_map_, robot_pose_, goal_, path_pub_, this->get_clock(), this->get_logger());
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::replanTimerCallback()
{
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL)
  {
  if (planner_core::goalReached(robot_pose_, goal_))
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      state_ = State::WAITING_FOR_GOAL;
      goal_received_ = false;
      // Publish an empty/stop path is optional; controller stops by tolerance
      return;
    }

    const double elapsed = (this->now() - goal_start_time_).seconds();
    if (elapsed > timeout_sec_)
    {
      RCLCPP_WARN(this->get_logger(), "Timeout reached. Replanning...");
  planner_core::planPath(goal_received_, current_map_, robot_pose_, goal_, path_pub_, this->get_clock(), this->get_logger());
      goal_start_time_ = this->now();
    }
  }
}



// ---- main ----
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
