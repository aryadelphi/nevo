#include "nevo_bot/safety_fail_node.hpp"

using std::placeholders::_1;

SafetyFailNode::SafetyFailNode()
: Node("safety_fail_node"),
  cmd_timeout_(rclcpp::Duration::from_seconds(0.5)),  // <<< WAJIB ADA
  received_cmd_(false)
{
  int timeout_ms = this->declare_parameter("cmd_timeout_ms", 500);
  cmd_timeout_ = rclcpp::Duration::from_seconds(timeout_ms / 1000.0);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/diff_cont/cmd_vel_unstamped_raw", 10,
    std::bind(&SafetyFailNode::cmdVelCallback, this, _1));

  safe_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/diff_cont/cmd_vel", 10);

  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&SafetyFailNode::watchdogLoop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "cmd_vel freshness safety node started (timeout=%d ms)",
    timeout_ms
  );
}

void SafetyFailNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;
  last_cmd_time_ = this->now();
  received_cmd_ = true;
  // Forward immediately instead of waiting up to 50 ms for the next timer tick.
  publish(last_cmd_, last_cmd_time_);
}

void SafetyFailNode::publish(const geometry_msgs::msg::Twist & twist, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = stamp;
  output.header.frame_id = "base_link";
  output.twist = twist;
  safe_cmd_pub_->publish(output);
  last_pub_time_ = stamp;
}

void SafetyFailNode::watchdogLoop()
{
  auto now = this->now();

  if (!received_cmd_ || (now - last_cmd_time_) > cmd_timeout_) {
    // FAIL SAFE: stale or no command -> zero (keeps diff_cont fed at 20 Hz)
    publish(geometry_msgs::msg::Twist(), now);
  } else if ((now - last_pub_time_) > rclcpp::Duration::from_seconds(0.1)) {
    // Source publishes slower than 10 Hz: repeat the last fresh command so
    // diff_cont's own cmd_vel_timeout doesn't stop the robot.
    publish(last_cmd_, now);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SafetyFailNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}