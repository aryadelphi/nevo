#ifndef KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
#define KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_

extern "C"
{
  #include <libfreenect.h>
}
#include "rclcpp/rclcpp.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <thread>
#include <atomic>
#include <mutex>

namespace kinect_ros2
{

class KinectRosComponent : public rclcpp::Node
{
public:
  KinectRosComponent(const rclcpp::NodeOptions & options);
  ~KinectRosComponent();

private:
  freenect_context * fn_ctx_;
  freenect_device * fn_dev_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread freenect_thread_;
  std::atomic<bool> running_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> rgb_info_manager_, depth_info_manager_;
  sensor_msgs::msg::CameraInfo rgb_info_, depth_info_, depth_small_info_;

  image_transport::CameraPublisher depth_pub_, depth_small_pub_, rgb_pub_;

  static void depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp);
  static void rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp);

  void publish_callback();
  bool enable_rgb_ = false;   // RGB stream is not used by the nav stack
  int downscale_ = 8;         // depth decimation for the point cloud (8 -> 80x60)
  int full_depth_every_n_ = 1;  // full-res depth/image_raw: publish every Nth frame
  unsigned int full_depth_count_ = 0;
};

}
#endif  // KINECT_ROS2__KINECT_ROS2_COMPONENT_HPP_
