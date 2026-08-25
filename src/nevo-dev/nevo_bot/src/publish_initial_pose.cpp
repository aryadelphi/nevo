// src/publish_initial_pose.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (argc >= 4) {
    try {
      x = std::stod(argv[1]);
      y = std::stod(argv[2]);
      yaw = std::stod(argv[3]);
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("initpose"), "Invalid args, using defaults 0,0,0");
    }
  }

  auto node = rclcpp::Node::make_shared("one_shot_initialpose");
  auto pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;

  double qz = std::sin(yaw * 0.5);
  double qw = std::cos(yaw * 0.5);
  msg.pose.pose.orientation.z = qz;
  msg.pose.pose.orientation.w = qw;

  RCLCPP_INFO(node->get_logger(),
              "Initialpose node started (x=%.3f y=%.3f yaw=%.3f). Waiting for /clock (if using sim time)...",
              x, y, yaw);

  // Wait for /clock to be published (if using sim time). Timeout after N seconds.
  const int timeout_seconds = 20;
  auto start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && node->get_clock()->now().nanoseconds() == 0) {
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count();
    if (elapsed > timeout_seconds) {
      RCLCPP_WARN(node->get_logger(), "/clock not available after %d seconds — continuing anyway", timeout_seconds);
      break;
    }
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(100ms);
  }

  // Publish multiple times to ensure subscribers (AMCL) receive the initial pose
  const int publish_count = 30;
  const auto publish_interval = 200ms; // 5 Hz (6 seconds total)

  for (int i = 0; i < publish_count && rclcpp::ok(); ++i) {
    msg.header.stamp = node->now();
    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(),
                "Published /initialpose attempt %d/%d (stamp=%u.%u)",
                i+1, publish_count,
                static_cast<unsigned int>(msg.header.stamp.sec),
                static_cast<unsigned int>(msg.header.stamp.nanosec));
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(publish_interval);
  }

  RCLCPP_INFO(node->get_logger(), "Initialpose publishing finished, shutting down.");

  rclcpp::spin_some(node);
  std::this_thread::sleep_for(200ms);
  rclcpp::shutdown();
  return 0;
}
