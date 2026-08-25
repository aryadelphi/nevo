#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

namespace nevo_robot_api
{

class PositionSensorNode : public rclcpp::Node
{
public:
  explicit PositionSensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_sim_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr position_pub_;

  std::string robot_id_;
  std::string ui_id_;
};

} // namespace nevo_robot_api
