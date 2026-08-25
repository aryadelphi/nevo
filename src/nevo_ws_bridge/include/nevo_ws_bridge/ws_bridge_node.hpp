#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nevo_ws_bridge/ws_client.hpp"
#include "nevo_ws_bridge/ws_server.hpp"

namespace nevo_ws_bridge
{

class WsBridgeNode : public rclcpp::Node
{
public:
  explicit WsBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void wsOutCallback(
    const std_msgs::msg::String::SharedPtr msg);

  void handleCloudMessage(
    const std::string & payload);

  void handleStatusChange(
    ConnectionState state);

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ws_in_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr network_status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ws_out_sub_;

  // WebSocket
  std::unique_ptr<WsClient> ws_client_;
  std::unique_ptr<WsServer> ws_server_;

  // Params
  std::string ws_uri_primary_;
  std::string ws_uri_backup_;
  std::string robot_id_;
  std::string network_mode_;
  double reconnect_interval_sec_{3.0};

  bool enable_local_ws_{true};
  int local_ws_port_{8765};

  bool cloud_enabled_{false};
};

}  // namespace nevo_ws_bridge
