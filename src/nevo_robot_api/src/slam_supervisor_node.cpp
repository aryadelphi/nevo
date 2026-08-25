#include "nevo_robot_api/slam_supervisor_node.hpp"

#include <algorithm>
#include <cstdint>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std::chrono_literals;

namespace nevo_robot_api
{

// ══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ══════════════════════════════════════════════════════════════════════════════

SlamSupervisorNode::SlamSupervisorNode(const rclcpp::NodeOptions & options)
: Node("slam_supervisor_node", options)
{
  // ── Parameters ────────────────────────────────────────────────────────────
  robot_id_ = this->declare_parameter<std::string>("robot_id", "TFRB2");
  ui_id_    = this->declare_parameter<std::string>("ui_id",    "TFUI2");

  // How long (seconds) to watch for coverage growth before declaring "stagnant"
  window_duration_sec_ = this->declare_parameter<double>(
    "coverage_window_sec", 30.0);

  // Minimum coverage ratio increment (0–1) required within the window
  min_coverage_delta_ = this->declare_parameter<double>(
    "min_coverage_delta", 0.01);

  // Frontier ratio (frontier cells / total known cells) below which
  // we consider exploration exhausted (mirrors explore_lite behaviour)
  frontier_threshold_ = this->declare_parameter<double>(
    "frontier_threshold", 0.1);

  RCLCPP_INFO(this->get_logger(),
    "[SlamSupervisor] robot_id=%s  window=%.0fs  "
    "delta=%.3f  frontier_thr=%.4f",
    robot_id_.c_str(),
    window_duration_sec_,
    min_coverage_delta_,
    frontier_threshold_);

  // ── Subscribers ───────────────────────────────────────────────────────────
  // /map  — published by slam_toolbox (OccupancyGrid)
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(10),
    std::bind(&SlamSupervisorNode::mapCallback, this, std::placeholders::_1));

  // ── Publisher ─────────────────────────────────────────────────────────────
  ws_out_pub_ = this->create_publisher<std_msgs::msg::String>(
    "nevo/ws_out", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
    "[SlamSupervisor] node started — waiting for /map ...");
}

void SlamSupervisorNode::mapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (mapping_done_.load()) {
    return;   // already declared done — no-op
  }

  if (!mapping_active_.load()) {
    return;   // supervisor paused via ws_in STOP_MAPPING
  }

  const double coverage_ratio  = computeCoverageRatio(*msg);
  const double frontier_ratio  = computeFrontierRatio(*msg);
  last_frontier_ratio_          = frontier_ratio;

  // ── Push to rolling window ───────────────────────────────────────────────
  {
    std::lock_guard<std::mutex> lock(coverage_mutex_);

    CoverageSnapshot snap;
    snap.stamp          = this->now();
    snap.coverage_ratio = coverage_ratio;
    coverage_window_.push_back(snap);

    // Evict snapshots older than window_duration_sec_
    const rclcpp::Duration window(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(window_duration_sec_)));

    while (coverage_window_.size() > 1 &&
           (snap.stamp - coverage_window_.front().stamp) > window)
    {
      coverage_window_.pop_front();
    }
  }
  
  // ── Completion check (same dual-condition as m_explore) ──────────────────
  const bool stagnant  = isCoverageStagnant();
  const bool exhausted = (frontier_ratio < frontier_threshold_);
  
  if (stagnant && exhausted) {
    mapping_done_.store(true);
    RCLCPP_INFO(this->get_logger(),
      "[SlamSupervisor] MAPPING COMPLETE — "
      "coverage=%.3f  frontiers=%.4f",
      coverage_ratio, frontier_ratio);

    publishMappingDone();
  } else {
    // Periodic status update (DEBUG level to avoid flooding)
    publishMappingStatus(
      stagnant ? "STAGNANT_WAIT_FRONTIER" :
      exhausted ? "FRONTIER_DONE_WAIT_STAGNANT" : "EXPLORING",
      coverage_ratio,
      frontier_ratio);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "[SlamSupervisor] coverage=%.3f  frontiers=%.4f  stagnant=%s  exhausted=%s",
    coverage_ratio, frontier_ratio,
    stagnant ? "YES" : "no",
    exhausted ? "YES" : "no");
}

double SlamSupervisorNode::computeCoverageRatio(
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  if (grid.data.empty()) return 0.0;

  std::size_t known = 0;
  for (const auto & cell : grid.data) {
    if (cell != -1) ++known;
  }
  return static_cast<double>(known) /
         static_cast<double>(grid.data.size());
}

double SlamSupervisorNode::computeFrontierRatio(
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  if (grid.data.empty()) return 1.0;   // assume still exploring

  const int width  = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);

  std::size_t known_count    = 0;
  std::size_t frontier_count = 0;

  // 4-connectivity offsets: up, down, left, right
  const int offsets[4] = {-width, +width, -1, +1};

  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const int idx  = row * width + col;
      const int8_t v = grid.data[static_cast<std::size_t>(idx)];

      if (v == -1) continue;   // unknown — skip
      ++known_count;

      if (v != 0) continue;    // occupied — not a frontier candidate

      // FREE cell — check 4-neighbours for unknown
      bool is_frontier = false;
      for (const int off : offsets) {
        const int nidx = idx + off;
        // boundary check
        if (nidx < 0 || nidx >= width * height) continue;
        // avoid wrap-around on left/right edges
        if (off == -1 && col == 0)       continue;
        if (off == +1 && col == width-1) continue;

        if (grid.data[static_cast<std::size_t>(nidx)] == -1) {
          is_frontier = true;
          break;
        }
      }
      if (is_frontier) ++frontier_count;
    }
  }

  if (known_count == 0) return 1.0;
  return static_cast<double>(frontier_count) /
         static_cast<double>(known_count);
}

bool SlamSupervisorNode::isCoverageStagnant() const
{
  std::lock_guard<std::mutex> lock(
    const_cast<std::mutex &>(coverage_mutex_));

  if (coverage_window_.size() < 2) return false;

  const double oldest = coverage_window_.front().coverage_ratio;
  const double newest = coverage_window_.back().coverage_ratio;
  const double delta  = newest - oldest;

  return (delta < min_coverage_delta_);
}

void SlamSupervisorNode::publishMappingDone()
{
  double cov = 0.0;
  {
    std::lock_guard<std::mutex> lock(coverage_mutex_);
    if (!coverage_window_.empty()) {
      cov = coverage_window_.back().coverage_ratio;
    }
  }

  json j;
  j["code"] = "MAPPING_DONE";
  j["data"] = {
    {"robot_id",       robot_id_},
    {"ui_id",          ui_id_},
    {"coverage",       cov},
    {"frontier_ratio", last_frontier_ratio_},
    {"method",         "coverage_and_frontier"}
  };

  std_msgs::msg::String out;
  out.data = j.dump();
  ws_out_pub_->publish(out);

  RCLCPP_INFO(this->get_logger(),
    "[SlamSupervisor] WS_OUT MAPPING_DONE: %s", out.data.c_str());
}

void SlamSupervisorNode::publishMappingStatus(
  const std::string & status,
  double              coverage_ratio,
  double              frontier_ratio)
{
  json j;
  j["code"] = "MAPPING_STATUS";
  j["data"] = {
    {"robot_id",       robot_id_},
    {"ui_id",          ui_id_},
    {"status",         status},
    {"coverage",       coverage_ratio},
    {"frontier_ratio", frontier_ratio}
  };

  std_msgs::msg::String out;
  out.data = j.dump();
  ws_out_pub_->publish(out);

  RCLCPP_DEBUG(this->get_logger(),
    "[SlamSupervisor] WS_OUT MAPPING_STATUS: %s", out.data.c_str());
}

}  // namespace nevo_robot_api

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nevo_robot_api::SlamSupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}