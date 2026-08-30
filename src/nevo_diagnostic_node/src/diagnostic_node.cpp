/**
 * @file diagnostic_node.cpp
 * @brief NEVO Diagnostic Node — monitors robot boot and publishes readiness
 *        status as JSON to /nevo/ws_out.
 *
 * State machine:
 *
 *   INIT ──(grace period elapsed)──► CHECKING ──(all topics OK)──► READY
 *                                          │
 *                                          └──(timeout / topic fail)──► FAILED
 *                                                                          │
 *                                                          (publishes ERROR + restarts service)
 *
 * The node is purely passive: it only subscribes and publishes.
 * It does NOT call any lifecycle services or modify other nodes.
 */

#include "nevo_diagnostic_node/diagnostic_node.hpp"

#include <cstdlib>   // std::system
#include <sstream>
#include <iomanip>

#include <cerrno>
#include <csignal>
#include <cstring>
#include <cctype>
#include <fstream>
#include <thread>
#include <chrono>
#include <dirent.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace nevo_diagnostic
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
DiagnosticNode::DiagnosticNode(const rclcpp::NodeOptions & options)
: Node("nevo_diagnostic_node", options)
{
  // Tambah declare parameter
  this->declare_parameter<std::string>("mode", "navigation");
  mode_ = this->get_parameter("mode").as_string();

  RCLCPP_INFO(get_logger(),
    "[NEVO DIAG] Mode: %s", mode_.c_str());

  // Take over the single-instance stack lock BEFORE anything else: if the user
  // manually launched while another stack was already running, the newest
  // launch must win, otherwise both stacks publish the same robot_id and the
  // cloud UI gets DUPLICATE_ROBOT_ID storms.
  acquireStackLock();

  // Set requirement berdasarkan mode
  if (mode_ == "mapping") {
    require_amcl_ = false;  // AMCL tidak jalan saat mapping
    require_map_  = false;  // Map belum ada, sedang dibangun
    require_odom_ = true;   // Odom tetap diperlukan
    
    RCLCPP_INFO(get_logger(),
      "[NEVO DIAG] Mapping mode: only checking /odometry/filtered");
  } else {
    require_amcl_ = true;
    require_map_  = true;
    require_odom_ = true;
    
    RCLCPP_INFO(get_logger(),
      "[NEVO DIAG] Navigation mode: checking /map + /amcl_pose + /odometry/filtered");
  }
  // ── Declare & get parameters ───────────────────────────────────────────────
  this->declare_parameter<std::string>("robot_id",           "TFRB2");
  this->declare_parameter<std::string>("ui_id",              "TFUI2");
  this->declare_parameter<int>        ("grace_period_sec",   10);
  this->declare_parameter<int>        ("global_timeout_sec", 60);
  this->declare_parameter<double>     ("odom_timeout_sec",   3.0);

  robot_id_           = this->get_parameter("robot_id").as_string();
  ui_id_              = this->get_parameter("ui_id").as_string();
  grace_period_sec_   = this->get_parameter("grace_period_sec").as_int();
  global_timeout_sec_ = this->get_parameter("global_timeout_sec").as_int();
  odom_timeout_sec_   = this->get_parameter("odom_timeout_sec").as_double();

  RCLCPP_INFO(get_logger(),
    "[NEVO DIAG] Starting — robot_id=%s, ui_id=%s, grace=%ds, timeout=%ds",
    robot_id_.c_str(), ui_id_.c_str(), grace_period_sec_, global_timeout_sec_);

  // ── Publisher ──────────────────────────────────────────────────────────────
  ws_out_pub_ = this->create_publisher<std_msgs::msg::String>("/nevo/ws_out", 10);

  // ── Subscribers ────────────────────────────────────────────────────────────
  // /map uses reliable+transient_local (matches map_server publisher QoS)
  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { onMap(msg); });

  amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", qos,
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { onAmclPose(msg); });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", qos,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdometry(msg); });

  // ── Record start time ──────────────────────────────────────────────────────
  node_start_time_ = this->now();

  // ── Initialise time trackers to "never" (epoch) ───────────────────────────
  last_amcl_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_odom_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // ── Evaluation timer — fires every 1 second ────────────────────────────────
  eval_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() { onEvalTimer(); });

  // Publish initial INITIALIZING status
  publishInit("System boot started. Waiting for grace period.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Destructor  — release the single-instance stack lock
// ─────────────────────────────────────────────────────────────────────────────
DiagnosticNode::~DiagnosticNode()
{
  if (lock_fd_ >= 0) {
    (void)::flock(lock_fd_, LOCK_UN);
    (void)::close(lock_fd_);
    lock_fd_ = -1;
    RCLCPP_INFO(get_logger(), "[NEVO DIAG] Stack lock released.");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Single-instance stack guard  (newest launch wins)
// ─────────────────────────────────────────────────────────────────────────────
namespace {

std::string procCmdline(int pid)
{
  std::ifstream f("/proc/" + std::to_string(pid) + "/cmdline");
  std::string s((std::istreambuf_iterator<char>(f)),
                std::istreambuf_iterator<char>());
  for (char & c : s) { if (c == '\0') c = ' '; }
  return s;
}

std::string procComm(int pid)
{
  std::ifstream f("/proc/" + std::to_string(pid) + "/comm");
  std::string s;
  std::getline(f, s);
  return s;
}

int procPpid(int pid)
{
  std::ifstream f("/proc/" + std::to_string(pid) + "/stat");
  std::string line;
  if (!std::getline(f, line)) return 0;
  const size_t open_p  = line.find('(');
  const size_t close_p = line.rfind(')');
  if (open_p == std::string::npos || close_p == std::string::npos) return 0;
  std::istringstream ss(line.substr(close_p + 1));
  char state = 0;
  int  ppid  = 0;
  ss >> state >> ppid;
  return ppid;
}

}  // namespace

void DiagnosticNode::acquireStackLock()
{
  RCLCPP_INFO(get_logger(), "[NEVO DIAG] Acquiring single-instance stack lock...");

  lock_fd_ = ::open(kLockFilePath, O_CREAT | O_RDWR | O_CLOEXEC, 0644);
  if (lock_fd_ < 0) {
    RCLCPP_WARN(get_logger(),
      "[NEVO DIAG] Cannot open %s: %s — proceeding without lock.",
      kLockFilePath, std::strerror(errno));
    return;
  }

  if (tryAcquireLock()) {
    RCLCPP_INFO(get_logger(), "[NEVO DIAG] Stack lock acquired (no conflicting stack).");
    return;
  }

  // Another stack holds the lock → newest launch wins: tear it down first.
  const int holder = readLockHolderPid();
  int launch_pid = -1;
  if (holder > 0 && !isInOurAncestry(holder)) {
    launch_pid = findStackLaunchPid(holder);
    if (launch_pid > 0) {
      RCLCPP_WARN(get_logger(),
        "[NEVO DIAG] Conflicting stack launch found (pid %d) — terminating it (newest wins).",
        launch_pid);
      terminateProcess(launch_pid, SIGTERM);
    } else {
      RCLCPP_WARN(get_logger(),
        "[NEVO DIAG] Conflicting stack holder pid %d — terminating it.", holder);
      terminateProcess(holder, SIGTERM);
    }
  }

  // Belt-and-suspenders: sweep /proc for any other `ros2 launch nevo_bot`
  // process that is not part of this stack's own ancestry.
  killOtherStacks();

  // Wait for the old holder to exit (flock is auto-released on process exit),
  // then acquire the lock ourselves. Escalate to SIGKILL if it lingers.
  constexpr int kMaxWaitMs = 15000;
  for (int waited = 0; waited < kMaxWaitMs; waited += 100) {
    if (tryAcquireLock()) {
      RCLCPP_INFO(get_logger(),
        "[NEVO DIAG] Stack lock acquired after %.1f s (old stack terminated).",
        waited / 1000.0);
      return;
    }
    if (waited == 8000) {
      if (launch_pid > 0) {
        RCLCPP_WARN(get_logger(),
          "[NEVO DIAG] Old launch still alive — escalating to SIGKILL.");
        terminateProcess(launch_pid, SIGKILL);
        (void)::kill(-launch_pid, SIGKILL);  // best-effort process-group kill
      }
      if (holder > 0) terminateProcess(holder, SIGKILL);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_ERROR(get_logger(),
    "[NEVO DIAG] Could not acquire stack lock after %.1f s — old stack may "
    "still be shutting down. Continuing anyway (watch for duplicate topics).",
    kMaxWaitMs / 1000.0);
}

bool DiagnosticNode::tryAcquireLock()
{
  if (lock_fd_ < 0) return false;
  if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      RCLCPP_WARN(get_logger(), "[NEVO DIAG] flock error: %s", std::strerror(errno));
    }
    return false;
  }
  // Acquired — stamp the lock file with our PID.
  if (::ftruncate(lock_fd_, 0) == 0) {
    (void)::lseek(lock_fd_, 0, SEEK_SET);
    const std::string pid = std::to_string(::getpid()) + "\n";
    (void)::write(lock_fd_, pid.c_str(), pid.size());
    (void)::fsync(lock_fd_);
  }
  return true;
}

int DiagnosticNode::readLockHolderPid() const
{
  std::ifstream f(kLockFilePath);
  long pid = 0;
  f >> pid;
  return (pid > 0) ? static_cast<int>(pid) : -1;
}

int DiagnosticNode::findStackLaunchPid(int pid) const
{
  int cur = pid;
  for (int hop = 0; hop < 32 && cur > 1; ++hop) {
    const std::string cl = procCmdline(cur);
    if (cl.find("ros2 launch") != std::string::npos &&
        cl.find("nevo_bot")   != std::string::npos) {
      return cur;
    }
    cur = procPpid(cur);
  }
  return -1;
}

bool DiagnosticNode::isInOurAncestry(int pid) const
{
  int cur = ::getppid();
  for (int hop = 0; hop < 64 && cur > 1; ++hop) {
    if (cur == pid) return true;
    cur = procPpid(cur);
  }
  return false;
}

void DiagnosticNode::killOtherStacks()
{
  DIR * dir = ::opendir("/proc");
  if (!dir) return;
  while (dirent * e = ::readdir(dir)) {
    if (!e->d_name[0] || !std::isdigit(static_cast<unsigned char>(e->d_name[0]))) {
      continue;
    }
    const int pid = std::atoi(e->d_name);
    if (pid <= 1 || pid == ::getpid() || isInOurAncestry(pid)) continue;
    const std::string cl = procCmdline(pid);
    if (cl.find("ros2 launch") == std::string::npos) continue;
    if (cl.find("nevo_bot")   == std::string::npos) continue;
    // Skip shell wrappers (`bash -c "ros2 launch nevo_bot ..."` etc.): their
    // cmdline contains the launch text but they are not the launch itself.
    // Only the real `ros2 launch` python process is worth killing.
    const std::string comm = procComm(pid);
    if (comm == "bash" || comm == "sh" || comm == "dash" ||
        comm == "zsh" || comm == "fish" || comm == "tmux" ||
        comm == "tmux: server") {
      continue;
    }
    RCLCPP_WARN(get_logger(),
      "[NEVO DIAG] Terminating stale stack launch (pid %d).", pid);
    terminateProcess(pid, SIGTERM);
  }
  ::closedir(dir);
}

void DiagnosticNode::terminateProcess(int pid, int signal) const
{
  if (pid <= 1) return;
  if (::kill(pid, signal) != 0) {
    RCLCPP_WARN(get_logger(), "[NEVO DIAG] kill(%d, %d) failed: %s",
      pid, signal, std::strerror(errno));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Subscription callbacks  (just record arrival; no logic here)
// ─────────────────────────────────────────────────────────────────────────────

void DiagnosticNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr /*msg*/)
{
  if (!map_received_) {
    RCLCPP_INFO(get_logger(), "[NEVO DIAG] /map received for the first time.");
    map_received_ = true;
  }
}

void DiagnosticNode::onAmclPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr /*msg*/)
{
  if (!amcl_received_) {
    RCLCPP_INFO(get_logger(), "[NEVO DIAG] /amcl_pose received for the first time.");
    amcl_received_ = true;
  }
  last_amcl_time_ = this->now();
}

void DiagnosticNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr /*msg*/)
{
  if (!odom_received_) {
    RCLCPP_INFO(get_logger(), "[NEVO DIAG] /odometry/filtered received for the first time.");
    odom_received_ = true;
  }
  last_odom_time_ = this->now();
  odom_stale_cycles_ = 0;  // reset debounce counter on fresh data
}

// ─────────────────────────────────────────────────────────────────────────────
// Main evaluation timer  (runs every 1 s)
// ─────────────────────────────────────────────────────────────────────────────
void DiagnosticNode::onEvalTimer()
{
  const double elapsed = elapsedSec();

  // ── Nothing to do once we have reached a terminal state ───────────────────
  if (state_ == DiagState::READY) {
    // Lightweight monitoring: only check for odom freeze after READY
    // (we stay in READY; no reset is triggered from here)
    if (odom_received_) {
      double odom_age = (this->now() - last_odom_time_).seconds();
      if (odom_age > odom_timeout_sec_) {
        RCLCPP_WARN(get_logger(),
          "[NEVO DIAG] /odometry/filtered stale for %.1f s (monitoring only, no reset).",
          odom_age);
      }
    }
    return;
  }

  if (state_ == DiagState::FAILED) {
    return;  // terminal — nothing more to evaluate
  }

  // ── INIT: wait for grace period ────────────────────────────────────────────
  if (state_ == DiagState::INIT) {
    RCLCPP_INFO(get_logger(),
      "[NEVO DIAG] INIT — elapsed %.1f s / grace %d s", elapsed, grace_period_sec_);
    if (elapsed >= static_cast<double>(grace_period_sec_)) {
      transitionToChecking();
    }
    return;
  }

  // ── CHECKING: evaluate all readiness conditions ────────────────────────────
  // (we are now past the grace period)

  // 1) Global timeout guard
  if (elapsed >= static_cast<double>(global_timeout_sec_)) {
    // Build a descriptive reason from what is still missing
    std::string reason;
    if (!map_received_)   reason += "No /map received. ";
    if (!amcl_received_)  reason += "No /amcl_pose received. ";
    if (!odom_received_)  reason += "No /odometry/filtered received. ";
    if (reason.empty())   reason = "System did not reach READY within timeout.";

    transitionToFailed("Global timeout (" +
      std::to_string(global_timeout_sec_) + " s). " + reason);
    return;
  }

  // 2) Evaluate odom staleness with debounce
  if (odom_received_) {
    double odom_age = (this->now() - last_odom_time_).seconds();
    if (odom_age > odom_timeout_sec_) {
      odom_stale_cycles_++;
      RCLCPP_WARN(get_logger(),
        "[NEVO DIAG] /odometry/filtered stale for %.1f s (debounce %d/%d)",
        odom_age, odom_stale_cycles_, kDebounceFailCycles);
    }
    // Note: odom_stale_cycles_ is reset to 0 in onOdometry()
  }

  // 3) Hard-fail: odom was seen but is now consistently stale
  if (odom_received_ && odom_stale_cycles_ >= kDebounceFailCycles) {
    transitionToFailed(
      "/odometry/filtered stopped updating for >" +
      std::to_string(static_cast<int>(odom_timeout_sec_)) + " s.");
    return;
  }


  // 4) READY check: sesuai mode
  const bool map_ok  = !require_map_  || map_received_;
  const bool amcl_ok = !require_amcl_ || (
    amcl_received_ &&
    ((this->now() - last_amcl_time_).seconds() < 5.0)
  );
  const bool odom_ok = !require_odom_ || (
    odom_received_ && (odom_stale_cycles_ == 0)
  );

  RCLCPP_INFO(get_logger(),
    "[NEVO DIAG] CHECKING (%.1f s) — map:%s  amcl:%s  odom:%s  [mode:%s]",
    elapsed,
    require_map_  ? (map_ok  ? "OK" : "WAIT") : "SKIP",
    require_amcl_ ? (amcl_ok ? "OK" : "WAIT") : "SKIP",
    require_odom_ ? (odom_ok ? "OK" : "WAIT") : "SKIP",
    mode_.c_str());

  if (map_ok && amcl_ok && odom_ok) {
    transitionToReady();
  } else {
    // Republish INITIALIZING so the UI stays updated
    publishInit("Checking system topics… elapsed " +
      std::to_string(static_cast<int>(elapsed)) + " s.");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// State transitions
// ─────────────────────────────────────────────────────────────────────────────

void DiagnosticNode::transitionToChecking()
{
  RCLCPP_INFO(get_logger(), "[NEVO DIAG] → CHECKING");
  state_ = DiagState::CHECKING;
  publishInit("Grace period elapsed. Checking system readiness.");
}

void DiagnosticNode::transitionToReady()
{
  RCLCPP_INFO(get_logger(), "[NEVO DIAG] → READY");
  state_ = DiagState::READY;
  publishReady();
}

void DiagnosticNode::transitionToFailed(const std::string & reason)
{
  RCLCPP_ERROR(get_logger(), "[NEVO DIAG] → FAILED: %s", reason.c_str());
  state_ = DiagState::FAILED;
  publishError(reason);

  rclcpp::sleep_for(std::chrono::seconds(2));

  // ✅ Restart service sesuai mode — only if the service is actually running
  // (when launched manually via launch_robot.launch.py the services are
  //  disabled, and `systemctl restart` would wrongly start them)
  std::string service_name = (mode_ == "mapping") 
    ? "nevo-mapping.service" 
    : "nevo.service";

  std::string is_active_cmd =
    "/usr/bin/systemctl is-active --quiet " + service_name;
  if (std::system(is_active_cmd.c_str()) != 0) {
    RCLCPP_WARN(get_logger(),
      "[NEVO DIAG] %s is not active — skipping service restart "
      "(manual launch mode)", service_name.c_str());
    return;
  }

  RCLCPP_WARN(get_logger(),
    "[NEVO DIAG] Restarting %s...", service_name.c_str());

  std::string cmd = "sudo /usr/bin/systemctl restart " + service_name;
  int ret = std::system(cmd.c_str());

  if (ret != 0) {
    RCLCPP_ERROR(get_logger(),
      "[NEVO DIAG] Failed to restart %s", service_name.c_str());
  }
}



// ─────────────────────────────────────────────────────────────────────────────
// JSON builders  (hand-rolled to avoid external dependencies)
// ─────────────────────────────────────────────────────────────────────────────

// std::string DiagnosticNode::buildInitJson(const std::string & status,
//                                           const std::string & init_message) const
// {
//   // Determine top-level "code" field
//   const std::string code = (status == "READY") ? "READY" : "INIT";

//   std::ostringstream oss;
//   oss << "{"
//       << "\"code\":\"" << code << "\","
//       << "\"data\":{"
//         << "\"robot_id\":\""    << robot_id_    << "\","
//         << "\"ui_id\":\""       << ui_id_       << "\","
//         << "\"status\":\""      << status       << "\","
//         << "\"init_message\":\"" << init_message << "\""
//       << "}"
//       << "}";
//   return oss.str();
// }

// std::string DiagnosticNode::buildErrorJson(const std::string & error_message) const
// {
//   std::ostringstream oss;
//   oss << "{"
//       << "\"code\":\"ERROR\","
//       << "\"data\":{"
//         << "\"robot_id\":\""     << robot_id_                    << "\","
//         << "\"ui_id\":\""        << ui_id_                       << "\","
//         << "\"error_code\":\""   << "SYSTEM_BOOT_FAIL"           << "\","
//         << "\"error_message\":\"" << error_message               << "\","
//         << "\"severity\":\""     << "CRITICAL"                   << "\""
//       << "}"
//       << "}";
//   return oss.str();
// }

// ─────────────────────────────────────────────────────────────────────────────
// Utility
// ─────────────────────────────────────────────────────────────────────────────

double DiagnosticNode::elapsedSec() const
{
  return (this->now() - node_start_time_).seconds();
}


void DiagnosticNode::publishChecking(
    double elapsed, bool map_ok, bool amcl_ok, bool odom_ok)
{
  // Buat pesan yang informatif tentang apa yang masih ditunggu
  std::string waiting_for;
  if (!map_ok)  waiting_for += "map ";
  if (!amcl_ok) waiting_for += "amcl ";
  if (!odom_ok) waiting_for += "odom ";

  std::ostringstream init_message;
  init_message << "Checking system readiness ("
               << static_cast<int>(elapsed)
               << "s) — waiting for: "
               << (waiting_for.empty() ? "none" : waiting_for);

  std::ostringstream oss;
  oss << "{"
      << "\"code\":\"INIT\","
      << "\"data\":{"
        << "\"robot_id\":\""     << robot_id_            << "\","
        << "\"ui_id\":\""        << ui_id_               << "\","
        << "\"status\":\"INITIALIZING\","
        << "\"init_message\":\"" << init_message.str()   << "\""
      << "}"
      << "}";

  auto msg = std_msgs::msg::String{};
  msg.data = oss.str();
  ws_out_pub_->publish(msg);
}

void DiagnosticNode::publishReady()
{
  std::ostringstream oss;
  oss << "{"
      << "\"code\":\"INIT\","
      << "\"data\":{"
        << "\"robot_id\":\""     << robot_id_                                      << "\","
        << "\"ui_id\":\""        << ui_id_                                         << "\","
        << "\"status\":\"READY\","
        << "\"init_message\":\"" << "All systems operational. Robot is ready."     << "\""
      << "}"
      << "}";

  auto msg = std_msgs::msg::String{};
  msg.data = oss.str();
  ws_out_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "[NEVO DIAG] READY status published to websocket.");
}


void DiagnosticNode::publishInit(const std::string & message)
{
  std::ostringstream oss;
  oss << "{"
      << "\"code\":\"INIT\","
      << "\"data\":{"
        << "\"robot_id\":\""    << robot_id_ << "\","
        << "\"ui_id\":\""       << ui_id_    << "\","
        << "\"status\":\"INITIALIZING\","
        << "\"init_message\":\"" << message  << "\""
      << "}"
      << "}";

  auto msg = std_msgs::msg::String{};
  msg.data = oss.str();
  ws_out_pub_->publish(msg);
}


void DiagnosticNode::publishError(const std::string & error_message)
{
  std::ostringstream oss;
  oss << "{"
      << "\"code\":\"INIT\","
      << "\"data\":{"
        << "\"robot_id\":\""     << robot_id_     << "\","
        << "\"ui_id\":\""        << ui_id_        << "\","
        << "\"status\":\"FAILED\","
        << "\"init_message\":\"" << error_message << "\""
      << "}"
      << "}";

  auto msg = std_msgs::msg::String{};
  msg.data = oss.str();
  ws_out_pub_->publish(msg);
}


}  // namespace nevo_diagnostic

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nevo_diagnostic::DiagnosticNode>());
  rclcpp::shutdown();
  return 0;
}
