#include "nevo_robot_api/position_sensor_node.hpp"

#include <nlohmann/json.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using json = nlohmann::json;

namespace nevo_robot_api
{

PositionSensorNode::PositionSensorNode(const rclcpp::NodeOptions & options)
: Node("position_sensor_node", options)
{
  robot_id_ = this->declare_parameter<std::string>("robot_id", "TFRB2");
  ui_id_    = this->declare_parameter<std::string>("ui_id", "TFUI2");

  position_pub_ = this->create_publisher<std_msgs::msg::String>(
    "nevo/sensor/position", 20);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/diff_cont/odom",
    rclcpp::QoS(20),
    std::bind(&PositionSensorNode::odomCallback, this, std::placeholders::_1));

  // odom_sub_sim_ = this->create_subscription<nav_msgs::msg::Odometry>(
  //   "/diff_drive_base_controller/odom",
  //   rclcpp::QoS(20),
  //   std::bind(&PositionSensorNode::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "PositionSensorNode started");
}

void PositionSensorNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double yaw = tf2::getYaw(msg->pose.pose.orientation);

  json j;
  j["code"] = "POSITION";
  j["data"] = {
    {"robot_id", robot_id_},
    {"ui_id", ui_id_},
    {"x", x},
    {"y", y},
    {"yaw", yaw}
  };

  std_msgs::msg::String out;
  out.data = j.dump();
  position_pub_->publish(out);
}

} // namespace nevo_robot_api

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nevo_robot_api::PositionSensorNode>());
  rclcpp::shutdown();
  return 0;
}
