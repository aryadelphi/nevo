#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <thread>
#include <string>

namespace nevo_robot_api
{

class BatterySensorNode : public rclcpp::Node
{
public:
  explicit BatterySensorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BatterySensorNode();

private:
  void batteryReadLoop();
  void handleBatteryJson(const std::string & raw);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_pub_;

  std::string robot_id_;
  std::string ui_id_;
  std::string battery_device_;

  std::thread battery_thread_;
  std::atomic<bool> running_;
};

} // namespace nevo_robot_api
