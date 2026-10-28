#include "kinect_ros2/kinect_ros2_component.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>

using namespace std::chrono_literals;

namespace kinect_ros2
{
static cv::Mat _depth_image(cv::Mat::zeros(cv::Size(640, 480), CV_16UC1));
static cv::Mat _rgb_image(cv::Mat::zeros(cv::Size(640, 480), CV_8UC3));

static uint16_t * _freenect_depth_pointer = nullptr;
static uint8_t * _freenect_rgb_pointer = nullptr;

// Written by the libfreenect thread, read by the publish timer.
static std::mutex _frame_mutex;
static bool _depth_flag = false;
static bool _rgb_flag = false;
static int _rgb_frame_count = 0;
static rclcpp::Clock _capture_clock(RCL_SYSTEM_TIME);
static rclcpp::Time _depth_stamp;

KinectRosComponent::KinectRosComponent(const rclcpp::NodeOptions & options)
: Node("kinect_ros2", options), running_(false)
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory("kinect_ros2");

  //todo: use parameters
  depth_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_depth.yaml");

  rgb_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    this, "kinect",
    "file://" + pkg_share + "/cfg/calibration_rgb.yaml");

  rgb_info_ = rgb_info_manager_->getCameraInfo();
  rgb_info_.header.frame_id = "kinect_rgb";
  depth_info_ = depth_info_manager_->getCameraInfo();
  depth_info_.header.frame_id = "kinect_depth";

  enable_rgb_ = declare_parameter<bool>("enable_rgb", false);
  downscale_ = std::max(1, static_cast<int>(declare_parameter<int>("depth_downscale", 8)));
  // Thins only the full-res stream (recording / survey); the decimated /points
  // stream for STVL keeps the full rate. Compression (image_transport) runs per
  // published frame, so this is where the CPU is saved.
  full_depth_every_n_ = std::max(1, static_cast<int>(
    declare_parameter<int>("full_depth_every_n", 1)));
  const double DOWNSCALE = downscale_;
  depth_small_info_ = depth_info_;
  depth_small_info_.width = static_cast<uint32_t>(depth_info_.width / DOWNSCALE);
  depth_small_info_.height = static_cast<uint32_t>(depth_info_.height / DOWNSCALE);
  depth_small_info_.k[0] /= DOWNSCALE;
  depth_small_info_.k[2] /= DOWNSCALE;
  depth_small_info_.k[4] /= DOWNSCALE;
  depth_small_info_.k[5] /= DOWNSCALE;
  depth_small_info_.p[0] /= DOWNSCALE;
  depth_small_info_.p[2] /= DOWNSCALE;
  depth_small_info_.p[5] /= DOWNSCALE;
  depth_small_info_.p[6] /= DOWNSCALE;

  depth_pub_ = image_transport::create_camera_publisher(this, "depth/image_raw");
  depth_small_pub_ = image_transport::create_camera_publisher(this, "depth/small/image_raw");
  rgb_pub_ = image_transport::create_camera_publisher(this, "image_raw");

  int ret = freenect_init(&fn_ctx_, NULL);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "ERROR INIT");
    rclcpp::shutdown();
  }

  freenect_set_log_level(fn_ctx_, FREENECT_LOG_FATAL);
  freenect_select_subdevices(fn_ctx_, FREENECT_DEVICE_CAMERA);

  int num_devices = ret = freenect_num_devices(fn_ctx_);
  if (ret < 0) {
    RCLCPP_ERROR(get_logger(), "FREENECT - ERROR GET DEVICES");
    rclcpp::shutdown();
  }
  if (num_devices == 0) {
    RCLCPP_ERROR(get_logger(), "FREENECT - NO DEVICES");
    freenect_shutdown(fn_ctx_);
    rclcpp::shutdown();
  }
  ret = freenect_open_device(fn_ctx_, &fn_dev_, 0);
  if (ret < 0) {
    freenect_shutdown(fn_ctx_);
    RCLCPP_ERROR(get_logger(), "FREENECT - ERROR OPEN");
    rclcpp::shutdown();
  }
  ret =
    freenect_set_depth_mode(
    fn_dev_, freenect_find_depth_mode(
      FREENECT_RESOLUTION_MEDIUM,
      FREENECT_DEPTH_MM));
  if (ret < 0) {
    freenect_shutdown(fn_ctx_);
    RCLCPP_ERROR(get_logger(), "FREENECT - ERROR SET DEPTH");
    rclcpp::shutdown();
  }

  freenect_set_depth_callback(fn_dev_, depth_cb);
  freenect_set_video_callback(fn_dev_, rgb_cb);
  freenect_set_video_mode(fn_dev_, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_YUV_RGB));

  ret = freenect_start_depth(fn_dev_);
  if (ret < 0) {
    freenect_shutdown(fn_ctx_);
    RCLCPP_ERROR(get_logger(), "FREENECT - ERROR START DEPTH");
    rclcpp::shutdown();
  }

  ret = enable_rgb_ ? freenect_start_video(fn_dev_) : 0;
  if (ret < 0) {
    RCLCPP_WARN(get_logger(), "RGB unavailable, depth only");
  }

  running_ = true;
  freenect_thread_ = std::thread([this]() {
    while (running_ && rclcpp::ok()) {
      freenect_process_events(fn_ctx_);
    }
  });

  timer_ = create_wall_timer(33ms, std::bind(&KinectRosComponent::publish_callback, this));
}

KinectRosComponent::~KinectRosComponent()
{
  RCLCPP_INFO(get_logger(), "stoping kinnect");
  running_ = false;
  if (freenect_thread_.joinable()) {
    freenect_thread_.join();
  }
  freenect_stop_depth(fn_dev_);
  if (enable_rgb_) {
    freenect_stop_video(fn_dev_);
  }
  freenect_close_device(fn_dev_);
  freenect_shutdown(fn_ctx_);
}


/* The freenect lib stores the depth data in a static region of the memory, so the best way to
create a cv::Mat is passing the pointer of that region, avoiding the need of copying the data
to a new cv::Mat. This way, the callback only used to set a flag that indicates that a new image
has arrived. The flag is unset when a msg is published */
void KinectRosComponent::depth_cb(freenect_device * dev, void * depth_ptr, uint32_t timestamp)
{
  (void)dev;
  (void)timestamp;
  std::lock_guard<std::mutex> lock(_frame_mutex);
  if (_depth_flag) {
    return;
  }

  _freenect_depth_pointer = (uint16_t *)depth_ptr;
  memcpy(_depth_image.data, depth_ptr, 640 * 480 * 2);
  _depth_stamp = _capture_clock.now();  // capture time, not publish time
  _depth_flag = true;
}

void KinectRosComponent::rgb_cb(freenect_device * dev, void * rgb_ptr, uint32_t timestamp)
{
  (void)dev;
  (void)timestamp;
  std::lock_guard<std::mutex> lock(_frame_mutex);
  if (_rgb_flag) {
    return;
  }

  _freenect_rgb_pointer = (uint8_t *)rgb_ptr;
  memcpy(_rgb_image.data, rgb_ptr, 640 * 480 * 3);
  _rgb_flag = true;
}

void KinectRosComponent::publish_callback()
{
  cv::Mat depth, rgb;
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(_frame_mutex);
    if (_depth_flag) {
      depth = _depth_image.clone();
      stamp = _depth_stamp;
      _depth_flag = false;
    }
    if (_rgb_flag) {
      if (++_rgb_frame_count % 2 == 0) {
        rgb = _rgb_image.clone();
      }
      _rgb_flag = false;
    }
  }

  if (!depth.empty()) {
    auto header = std_msgs::msg::Header();
    header.frame_id = "kinect_depth";
    header.stamp = stamp;

    // Full-resolution depth only when someone listens (e.g. rviz, the survey
    // recorder), and only every full_depth_every_n-th frame.
    if (depth_pub_.getNumSubscribers() > 0 &&
      full_depth_count_++ % static_cast<unsigned int>(full_depth_every_n_) == 0)
    {
      depth_info_.header.stamp = stamp;
      auto msg = cv_bridge::CvImage(header, "16UC1", depth).toImageMsg();
      depth_pub_.publish(*msg, depth_info_);
    }

    // Decimated depth feeds depth_image_proc -> /points (STVL).
    cv::Mat small;
    cv::resize(depth, small, cv::Size(), 1.0 / downscale_, 1.0 / downscale_, cv::INTER_NEAREST);
    depth_small_info_.header.stamp = stamp;
    auto small_msg = cv_bridge::CvImage(header, "16UC1", small).toImageMsg();
    depth_small_pub_.publish(*small_msg, depth_small_info_);
  }

  if (!rgb.empty() && rgb_pub_.getNumSubscribers() > 0) {
    auto rgb_header = std_msgs::msg::Header();
    rgb_header.frame_id = "kinect_rgb";
    rgb_header.stamp = now();
    rgb_info_.header.stamp = rgb_header.stamp;
    auto msg = cv_bridge::CvImage(rgb_header, "rgb8", rgb).toImageMsg();
    rgb_pub_.publish(*msg, rgb_info_);
  }
}
}  // namespace kinect_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinect_ros2::KinectRosComponent)
