#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nevo_ws_bridge/ws_client.hpp"
#include "nevo_ws_bridge/ws_server.hpp"


namespace nevo_ws_bridge 
{
class CloudUpload : public rclcpp::Node
{
public:
    explicit CloudUpload(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    );

private:
    void uploadData();
    
    void getData(
        const std_msgs::msg::String::SharedPtr msg
    );

    std::string robot_id_;
    std::string ui_id_;
    std::string latest_battery_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ws_out_pub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_sub_ ;


    rclcpp::TimerBase::SharedPtr timer_;
};

}