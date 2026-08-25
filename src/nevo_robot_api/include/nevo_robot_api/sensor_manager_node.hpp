#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <thread>
#include <atomic>
#include <string>

namespace nevo_robot_api
{

class SensorManagerNode : public rclcpp::Node
{
public:
  explicit SensorManagerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SensorManagerNode();

private:
  // ===== BATTERY =====
  void batteryReadLoop();
  void handleBatteryJson(const std::string & raw);

  // ===== ODOM =====
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ===== ROS =====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_pub_;
  

  // ===== THREAD =====
  std::thread battery_thread_;
  std::atomic<bool> battery_running_;

  // ===== PARAM =====
  std::string robot_id_;
  std::string ui_id_;
  std::string battery_device_;
};

} // namespace nevo_robot_api
