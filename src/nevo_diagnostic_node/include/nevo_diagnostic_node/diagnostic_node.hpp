#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace nevo_diagnostic
{

// ─────────────────────────────────────────────────────────────────────────────
// State machine states
// ─────────────────────────────────────────────────────────────────────────────
enum class DiagState
{
  INIT,      // Node just started, waiting for grace period to pass
  CHECKING,  // Actively checking whether all topics are healthy
  READY,     // All conditions met; switch to lightweight monitoring
  FAILED     // A critical condition was not met; will trigger reset
};

// ─────────────────────────────────────────────────────────────────────────────
// DiagnosticNode
// ─────────────────────────────────────────────────────────────────────────────
class DiagnosticNode : public rclcpp::Node
{
public:
  explicit DiagnosticNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~DiagnosticNode() override;

private:
  int ready_publish_counter_ = 0;
  void publishChecking(double elapsed, bool map_ok, bool amcl_ok, bool odom_ok);
  
  std::string mode_{"navigation"};
  bool require_amcl_{true};
  bool require_map_{true};
  bool require_odom_{true};
  // ── ROS interfaces ──────────────────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  ws_out_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr                 map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;

  // Main evaluation timer (runs every 1 s)
  rclcpp::TimerBase::SharedPtr eval_timer_;

  // ── Parameters ──────────────────────────────────────────────────────────────
  std::string robot_id_;
  std::string ui_id_;
  int         grace_period_sec_;
  int         global_timeout_sec_;
  double      odom_timeout_sec_;

  // ── State machine ───────────────────────────────────────────────────────────
  DiagState state_{DiagState::INIT};

  // Timestamp when the node started (used for grace period & global timeout)
  rclcpp::Time node_start_time_;

  // ── Topic health tracking ───────────────────────────────────────────────────
  bool   map_received_{false};                    // /map seen at least once
  bool   amcl_received_{false};                   // /amcl_pose seen at least once
  rclcpp::Time last_amcl_time_;                   // last time /amcl_pose arrived
  rclcpp::Time last_odom_time_;                   // last time /odometry/filtered arrived
  bool   odom_received_{false};                   // /odometry/filtered seen at least once

  // Debounce counters: conditions must fail for N consecutive cycles before
  // being treated as "hard fail" (1 cycle = 1 s eval timer tick)
  int odom_stale_cycles_{0};
  static constexpr int kDebounceFailCycles{3};    // 3 s debounce before FAILED

  // ── Subscription callbacks ──────────────────────────────────────────────────
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ── Timer callback ──────────────────────────────────────────────────────────
  void onEvalTimer();

  // ── State transitions ───────────────────────────────────────────────────────
  void transitionToChecking();
  void transitionToReady();
  void transitionToFailed(const std::string & reason);

  // ── Publishing helpers ──────────────────────────────────────────────────────
  void publishInit(const std::string & message);
  void publishReady();
  void publishError(const std::string & error_message);

  // ── Utility ─────────────────────────────────────────────────────────────────
  double elapsedSec() const;
  std::string buildInitJson(const std::string & status,
                            const std::string & init_message) const;
  std::string buildErrorJson(const std::string & error_message) const;

  // ── Single-instance stack guard (newest-launch wins) ──────────────────────
  // A process-wide lock file (/tmp/nevo_stack.lock) is held via flock for the
  // lifetime of the node. If another stack already holds it, we tear that
  // stack down (kill its `ros2 launch nevo_bot` ancestor) and take over, so a
  // manual `ros2 launch` while an auto-restarted stack is running can never
  // cause DUPLICATE_ROBOT_ID storms.
  void acquireStackLock();
  bool tryAcquireLock();                 // flock NB + stamp lock file with our PID
  int  readLockHolderPid() const;        // PID from lock file, or -1
  int  findStackLaunchPid(int pid) const; // walk ancestors looking for launch cmdline
  bool isInOurAncestry(int pid) const;
  void killOtherStacks();                // sweep /proc for other `ros2 launch nevo_bot`
  void terminateProcess(int pid, int signal) const;

  int  lock_fd_{-1};                     // fd holding the flock (kept for node lifetime)
  static constexpr const char * kLockFilePath = "/tmp/nevo_stack.lock";
};

}  // namespace nevo_diagnostic
