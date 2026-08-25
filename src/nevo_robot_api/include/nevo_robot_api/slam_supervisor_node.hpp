#pragma once

#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace nevo_robot_api
{

struct CoverageSnapshot {
  rclcpp::Time stamp;
  double coverage_ratio;  // [0.0 – 1.0]
};

class SlamSupervisorNode : public rclcpp::Node
{
public:
  explicit SlamSupervisorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── ROS interfaces ──────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         ws_in_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            ws_out_pub_;

  // ── Parameters ──────────────────────────────────────────────
  std::string robot_id_;
  std::string ui_id_;

  // coverage window: if coverage doesn't grow by min_coverage_delta_
  // within window_duration_sec_, mapping is declared finished
  double window_duration_sec_;   // default 30 s
  double min_coverage_delta_;    // default 0.01 (1 % of map cells)

  // frontier detection: a free cell adjacent to unknown is a frontier.
  // if frontier_ratio_ < frontier_threshold_, frontiers are exhausted
  double frontier_threshold_;    // default 0.005 (0.5 % of map cells)

  // ── State ────────────────────────────────────────────────────
  std::atomic<bool> mapping_done_{false};
  std::atomic<bool> mapping_active_{true};   // can be toggled via ws_in START/STOP
  std::mutex        coverage_mutex_;

  std::deque<CoverageSnapshot> coverage_window_;
  double                       last_frontier_ratio_{1.0};

  // ── Map callback ─────────────────────────────────────────────
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // ── Coverage / frontier helpers ───────────────────────────────
  double computeCoverageRatio(const nav_msgs::msg::OccupancyGrid & grid) const;
  double computeFrontierRatio(const nav_msgs::msg::OccupancyGrid & grid) const;
  bool   isCoverageStagnant() const;

  // ── Notification helpers ──────────────────────────────────────
  void publishMappingDone();
  void publishMappingStatus(const std::string & status,
                            double coverage_ratio,
                            double frontier_ratio);
};

}  // namespace nevo_robot_api