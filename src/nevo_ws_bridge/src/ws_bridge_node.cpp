#include "nevo_ws_bridge/ws_bridge_node.hpp"

#include <cstdlib>
#include <algorithm>

namespace nevo_ws_bridge
{

WsBridgeNode::WsBridgeNode(
  const rclcpp::NodeOptions & options)
: Node("nevo_ws_bridge", options)
{
  // ================================
  // PARAMETERS
  // ================================

  robot_id_ = this->declare_parameter<std::string>(
    "robot_id","TFRB2");

  ws_uri_primary_ = this->declare_parameter<std::string>(
    "ws_uri_primary", "ws://localhost:3001");

  ws_uri_backup_ = this->declare_parameter<std::string>(
    "ws_uri_backup", "");

  reconnect_interval_sec_ = this->declare_parameter<double>(
    "reconnect_interval_sec", 3.0);

  enable_local_ws_ = this->declare_parameter<bool>(
    "enable_local_ws", true);

  local_ws_port_ = this->declare_parameter<int>(
    "local_ws_port", 8765);

  network_mode_ = this->declare_parameter<std::string>(
    "network_mode", "auto"); // offline | online | auto

  cloud_max_msgs_per_sec_ = std::max(1, static_cast<int>(
    this->declare_parameter<int>("cloud_max_msgs_per_sec", 8)));

  RCLCPP_INFO(get_logger(), "=== nevo_ws_bridge ===");
  RCLCPP_INFO(get_logger(), "robot_id      : %s", robot_id_.c_str());
  RCLCPP_INFO(get_logger(), "network_mode  : %s", network_mode_.c_str());
  RCLCPP_INFO(get_logger(), "local_ws      : %s",
              enable_local_ws_ ? "ENABLED" : "DISABLED");

  // ================================
  // ROS INTERFACES
  // ================================

  ws_in_pub_ = this->create_publisher<std_msgs::msg::String>(
    "nevo/ws_in", rclcpp::QoS(10));

  network_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "nevo/network_status", rclcpp::QoS(10));

  cloud_drain_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&WsBridgeNode::drainCloudQueue, this));

  ws_out_sub_ = this->create_subscription<std_msgs::msg::String>(
    "nevo/ws_out",
    rclcpp::QoS(10),
    std::bind(&WsBridgeNode::wsOutCallback,
              this,
              std::placeholders::_1));

  // ================================
  // LOCAL SERVER (ALWAYS SAFE)
  // ================================

  if (enable_local_ws_)
  {
    ws_server_ = std::make_unique<WsServer>(local_ws_port_);

    ws_server_->setMessageCallback(
      [this](const std::string& payload)
      {
        std_msgs::msg::String msg;
        msg.data = payload;

        RCLCPP_INFO(this->get_logger(),
                    "LOCAL WS → ROS: %s",
                    payload.c_str());

        ws_in_pub_->publish(msg);
      });

    ws_server_->start();
  }

  // ================================
  // CLOUD MANAGEMENT
  // ================================

  if (network_mode_ == "offline")
  {
    RCLCPP_INFO(get_logger(),
                "Cloud disabled (OFFLINE mode)");
    cloud_enabled_ = false;
    return;
  }

  if (network_mode_ == "offline")
  {
    RCLCPP_INFO(get_logger(),
                "Cloud disabled (OFFLINE mode)");
    cloud_enabled_ = false;
    return;
  }

  // "online" and "auto" both start the cloud client unconditionally. The
  // WsClient reconnect loop (retry every reconnect_interval_sec) rides out any
  // network delay — e.g. the robot booting before the hotspot's internet route
  // exists, or an internet drop/recovery later. Gating the client on a single
  // startup ping permanently disabled the cloud if that ping failed at boot,
  // so the robot could never come online until the stack was restarted.
  cloud_enabled_ = true;

  if (cloud_enabled_)
  {
    RCLCPP_INFO(get_logger(),
                "Cloud enabled");

    ws_client_ = std::make_unique<WsClient>();

    ws_client_->configure(
      ws_uri_primary_,
      ws_uri_backup_,
      robot_id_,
      reconnect_interval_sec_);

    ws_client_->setMessageCallback(
      [this](const std::string & payload)
      {
        this->handleCloudMessage(payload);
      });

    ws_client_->setStatusCallback(
      [this](ConnectionState state)
      {
        this->handleStatusChange(state);
      });

    ws_client_->start();
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "Cloud not started (no internet detected)");
  }
}

//
// ROS → OUTGOING
//
void WsBridgeNode::wsOutCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (cloud_enabled_ && ws_client_) {
    if (cloud_queue_.size() >= cloud_queue_limit_) {
      cloud_queue_.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Cloud send queue full (%zu), dropping oldest message", cloud_queue_limit_);
    }
    cloud_queue_.push_back(msg->data);
    drainCloudQueue();
  }

  if (enable_local_ws_ && ws_server_)
    ws_server_->sendToAll(msg->data);
}

void WsBridgeNode::drainCloudQueue()
{
  // Callbacks share the node's default (mutually exclusive) group, so this and
  // wsOutCallback never run concurrently.
  if (!cloud_enabled_ || !ws_client_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  while (!cloud_sent_times_.empty() &&
         now - cloud_sent_times_.front() >= std::chrono::seconds(1)) {
    cloud_sent_times_.pop_front();
  }
  while (!cloud_queue_.empty() &&
         static_cast<int>(cloud_sent_times_.size()) < cloud_max_msgs_per_sec_) {
    ws_client_->send(cloud_queue_.front());
    cloud_queue_.pop_front();
    cloud_sent_times_.push_back(now);
  }
}

//
// CLOUD → ROS
//
void WsBridgeNode::handleCloudMessage(
  const std::string & payload)
{
  std_msgs::msg::String msg;
  msg.data = payload;

  RCLCPP_INFO(this->get_logger(),
              "CLOUD WS → ROS: %s",
              payload.c_str());

  ws_in_pub_->publish(msg);
}

//
// CLOUD STATUS
//
void WsBridgeNode::handleStatusChange(
  ConnectionState state)
{
  std_msgs::msg::String msg;

  switch (state)
  {
    case ConnectionState::DISCONNECTED:
      msg.data = "DISCONNECTED";
      break;

    case ConnectionState::CONNECTING:
      msg.data = "CONNECTING";
      break;

    case ConnectionState::CONNECTED:
      msg.data = "CONNECTED";
      break;

    case ConnectionState::FALLBACK:
      msg.data = "FALLBACK";
      break;

    default:
      msg.data = "UNKNOWN";
      break;
  }

  network_status_pub_->publish(msg);
}
}  // namespace nevo_ws_bridge


// ======================================
// MAIN
// ======================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<nevo_ws_bridge::WsBridgeNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
