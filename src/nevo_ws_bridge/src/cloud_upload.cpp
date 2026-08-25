#include "nevo_ws_bridge/cloud_upload.hpp"

#include <cstdlib>
#include <sstream>

namespace nevo_ws_bridge
{

CloudUpload::CloudUpload(
    const rclcpp::NodeOptions & options) 
: Node("cloud_upload", options)
{
    robot_id_ = this->declare_parameter<std::string>(
        "robot_id","TFRB2");
    ui_id_ = this->declare_parameter<std::string>(
        "ui_id","TFUI2");
    
    ws_out_pub = this->create_publisher<std_msgs::msg::String>(
        "nevo/ws_out", 10);
    
    battery_sub_ = this->create_subscription<std_msgs::msg::String>(
        "nevo/sensor/battery",
        rclcpp::QoS(10),
        std::bind(&CloudUpload::getData,
                  this,
                  std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::seconds(60),
        std::bind(&CloudUpload::uploadData, this)
    );
    
    
    RCLCPP_INFO(this->get_logger(), "Cloud upload sensor data");      
}

void CloudUpload::getData(
    const std_msgs::msg::String::SharedPtr msg)
{
    latest_battery_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Data stored");
}

void CloudUpload::uploadData()
{
    if(latest_battery_.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No Battery data yet!");
        return;
    }

    std_msgs::msg::String msg;
    msg.data = latest_battery_;

    ws_out_pub->publish(msg);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<nevo_ws_bridge::CloudUpload>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}