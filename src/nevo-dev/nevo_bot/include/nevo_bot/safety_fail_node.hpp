#ifndef NEVO_BOT__SAFETY_FAIL_NODE_HPP_
#define NEVO_BOT__SAFETY_FAIL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class SafetyFailNode : public rclcpp::Node
{
public:
  SafetyFailNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void watchdogLoop();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // Jazzy diff_drive_controller only accepts TwistStamped on ~/cmd_vel
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Duration cmd_timeout_;
  bool received_cmd_;
};

#endif
