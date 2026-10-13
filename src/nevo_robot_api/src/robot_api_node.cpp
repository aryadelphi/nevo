#include "nevo_robot_api/robot_api_node.hpp"

#include <stdexcept>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"


// JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <chrono>
#include <thread>
#include <sstream>
#include <atomic>
#include <curl/curl.h>
#include <optional>
#include <filesystem>
#include <zip.h>
#include <fstream>
#include <array>
#include <unordered_map>
#include <regex>

using namespace std::chrono_literals;

// Simple scope guard — executes callable on destruction
template<typename F>
struct ScopeGuard {
    F func_;
    bool active_ = true;
    explicit ScopeGuard(F f) : func_(std::move(f)) {}
    ~ScopeGuard() { if (active_) func_(); }
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;
    void dismiss() { active_ = false; }
};

namespace nevo_robot_api
{

RobotApiNode::RobotApiNode(const rclcpp::NodeOptions & options)
: Node("nevo_robot_api", options),
  goal_in_progress_(false),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_, this)
{
  home_base_pose_ = buildPoseStamped(0.0, 0.0, 0.0);  // default aman
  has_home_base_ = true;
  
  default_robot_id_ = this->declare_parameter<std::string>("robot_id", "TFRB2");
  default_ui_id_ = this->declare_parameter<std::string>("ui_id", "TFUI2");

  // Initial mode — set from the launch file so a manually-launched
  // mapping stack (hw_mapping.launch.py mode:=mapping) starts in MAPPING.
  // Without this, MAPPING_FLAG would be rejected ("robot not in MAPPING mode").
  std::string initial_mode = this->declare_parameter<std::string>("mode", "nav");
  if (initial_mode == "mapping") {
    sm_.setMode("MAPPING");
    ready_mode_ = "mapping";
    RCLCPP_INFO(this->get_logger(),
      "Initial mode: MAPPING (mapping stack)");
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Initial mode: IDLE (navigation stack)");
  }

  ws_in_sub_ = this->create_subscription<std_msgs::msg::String>(
    "nevo/ws_in", rclcpp::QoS(10),
    std::bind(&RobotApiNode::wsInCallback, this, std::placeholders::_1));

  // The mode swap is fire-and-forget: this node is killed a few seconds after
  // ACKing MAPPING_START/STOP. The NEW stack must tell the cloud it is up,
  // otherwise the UI hangs on the swap. We announce READY (a) whenever the
  // ws_bridge reports a (re)connection, and (b) periodically for the first
  // 90s after startup in case the CONNECTED event is missed.
  network_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "nevo/network_status", rclcpp::QoS(10),
    std::bind(&RobotApiNode::networkStatusCallback, this, std::placeholders::_1));

  ready_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), [this]() {
      if (ready_send_count_ >= 18) {
        ready_timer_->cancel();
        return;
      }
      ready_send_count_++;
      sendReady();
    });

  read_sensor_odom_ = this->create_subscription<std_msgs::msg::String>(
    "nevo/sensor/position",
    rclcpp::QoS(10),
    std::bind(&RobotApiNode::cacheLastPose, this, std::placeholders::_1)
  );

  // Sesudah — default mengikuti HOME user yang sedang login:
  const char* home_dir = std::getenv("HOME");
  std::string default_map_path = home_dir 
      ? std::string(home_dir) + "/nevo_maps"
      : "/tmp/nevo_maps";

  map_base_path_ = this->declare_parameter<std::string>(
    "map_base_path", default_map_path);

  RCLCPP_INFO(this->get_logger(),
    "map_base_path: %s", map_base_path_.c_str());

  uploaded_by_ = this->declare_parameter<std::string>("uploaded_by", "1");

  upload_server_url_ = this->declare_parameter<std::string>(
    "upload_server_url",
    "https://tifa-be.forgixrobotic.com/api/maps/upload/full");

  RCLCPP_INFO(this->get_logger(),
    "Upload server URL: %s", upload_server_url_.c_str());

  // Which map does map_server run? launch_robot reads the same file.
  enforce_map_id_ = this->declare_parameter<bool>("enforce_map_id", false);
  {
    std::string sel_file = home_dir ? std::string(home_dir) + "/nevo_selected_map.txt" : "";
    std::ifstream f(sel_file);
    std::string sel_path;
    std::smatch m;
    if (f && std::getline(f, sel_path) &&
        std::regex_search(sel_path, m, std::regex("map_([0-9]+)")))
    {
      active_map_id_ = std::stoi(m[1].str());
    }
    RCLCPP_INFO(this->get_logger(), "Active map_id: %d (enforce_map_id=%s)",
      active_map_id_, enforce_map_id_ ? "true" : "false");
  }

  // =====================
  // Publisher
  // ===================== 

  diff_cont_pub = this->create_publisher<geometry_msgs::msg::Twist>("/diff_cont", rclcpp::QoS(10));

  ws_out_pub_ = this->create_publisher<std_msgs::msg::String>(
    "nevo/ws_out", rclcpp::QoS(10));

  // persistent action client for Nav2 (no namespace)
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  
  slam_save_map_client_ = this->create_client<slam_toolbox::srv::SaveMap>(
    "slam_toolbox/save_map");

  RCLCPP_INFO(this->get_logger(),
    "nevo_robot_api started. default_robot_id=%s",
    default_robot_id_.c_str());

  // Cancel any stale navigation goals from a previous session (e.g. after mode swap).
  // Without this, the planner_server keeps trying to plan to a goal with a timestamp
  // that predates the current TF buffer, causing permanent extrapolation errors.
  if (nav_client_->wait_for_action_server(2s)) {
    nav_client_->async_cancel_all_goals();
    RCLCPP_INFO(this->get_logger(), "Cancelled all stale goals from previous session");
  }
}

void RobotApiNode::cacheLastPose(
  const std_msgs::msg::String::SharedPtr msg)
{
  try {
    json j = json::parse(msg->data);

    if (j.contains("code") && j["code"] == "POSITION") {
      last_position_json_ = j;

      if (!has_last_position_) {
        RCLCPP_INFO(this->get_logger(),
          "First sensor position received: x=%.3f y=%.3f yaw=%.3f",
          j["data"]["x"].get<double>(),
          j["data"]["y"].get<double>(),
          j["data"]["yaw"].get<double>());
      }

      has_last_position_ = true;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(),
      "cacheLastPose parse error: %s | raw: %s",
      e.what(), msg->data.c_str());
  }
}

void RobotApiNode::sendLastPoseOnce()
{
  if (!has_last_position_) {
    RCLCPP_WARN(this->get_logger(),
      "No cached POSITION to send");
    return;
  }

  std_msgs::msg::String out;
  out.data = last_position_json_.dump();
  ws_out_pub_->publish(out);

  RCLCPP_INFO(this->get_logger(),
    "WS_OUT POSITION (event-based): %s",
    out.data.c_str());
}


void RobotApiNode::wsInCallback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & payload = msg->data;
  RCLCPP_INFO(this->get_logger(), "WS_IN: %s", payload.c_str());

  try {
    json j = json::parse(payload);

    if (!j.contains("code") || !j.contains("data")) {
      throw std::runtime_error("Missing 'code' or 'data'");
    }

    std::string code = j.at("code").get<std::string>();
    json data = j.at("data");

    // Determine robot_id from incoming payload (if present)
    std::string incoming_robot_id = "";
    if (data.contains("robot_id")) {
      incoming_robot_id = data.at("robot_id").get<std::string>();
    }

    // FILTER: if incoming_robot_id is present and does not match this node's robot id, IGNORE
    if (!incoming_robot_id.empty() && incoming_robot_id != default_robot_id_) {
      RCLCPP_INFO(this->get_logger(), "Message for robot_id=%s ignored (this robot=%s)",
                  incoming_robot_id.c_str(), default_robot_id_.c_str());
      // send ACK IGNORED so sender knows message was received but not for this robot
      sendAck(incoming_robot_id, code, "IGNORED");
      return;
    }

    // If no robot_id in payload, treat as intended for this robot (backwards compatible)

    //start logic pergerakan robot
    std::string robot_id_for_handling = default_robot_id_;
    if (!incoming_robot_id.empty()) {
      robot_id_for_handling = incoming_robot_id;
    }

    if (code == "MOVE") {
      handleMove(robot_id_for_handling, payload);
    }
    else if (code == "OP") {
      handleOp(robot_id_for_handling, payload);
    }
    else if (code == "MODE") {
      handleMode(robot_id_for_handling, payload);
    }
    else if(code == "INTERRUPT") {
      handleINT(robot_id_for_handling, payload);
    }
    else if(code == "INTERRUPT_CONFIRM") {
      handleICM(robot_id_for_handling, payload); //interrupt confirm
    }else if(code == "INTERRUPT_CANCEL") {
      handleICL(robot_id_for_handling, payload); //interrupt cancel
    }else if(code == "PICKUP_CONFIRM"){
      handlePickUp(payload);
    }else if(code == "TELEOP"){
      handleTeleop(payload);
    }else if(code == "MAPPING_START"){
      navToMapping(robot_id_for_handling, payload);
    }else if(code == "MAPPING_SAVE"){
      mapUpload(robot_id_for_handling, payload);
    }else if(code == "MAPPING_STOP"){
      mappingToNav(robot_id_for_handling, payload);
    }else if(code == "MAPPING_FLAG"){
      // update the x,y,z,yaw data
      flagCoordinate(robot_id_for_handling, payload);
    }else if (code == "MAP_DATA") {
      handleMapData(payload);             // terima & simpan zip map
    } else if (code == "MAP_SELECTED"){
      handleMapSelected(payload);
    } else if (code == "ERROR") {
      if (data.contains("error") && data.at("error").get<std::string>() == "DUPLICATE_ROBOT_ID") {
        RCLCPP_WARN(this->get_logger(), "DUPLICATE_ROBOT_ID received — WsClient handles recovery internally");
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Unhandled code='ERROR' (ignored for now)");
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Unhandled code='%s' (ignored for now)", code.c_str());
    }

    //end logic pergerakan robot

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    sendError(std::string("JSON error: ") + e.what(),
              "ERROR",
              "HIGH");
  }
}

// ============================================================
//                    MAPPING SECTION START
// ============================================================

void RobotApiNode::navToMapping(const std::string & robot_id,
                                const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    bool status = j.at("data").at("status").get<bool>();

    // ✅ CORRECTED LOGIC: true = start mapping, false = reject
    if(!status) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_START rejected: status=false (already in navigation mode or mapping)");
      sendAck(robot_id, "MAPPING_START", "REJECTED");
      return;
    }

    // Already mapping — idempotent ACK so the UI can re-request freely
    if (sm_.getMode() == RobotMode::MAPPING) {
      RCLCPP_INFO(this->get_logger(),
        "MAPPING_START: already in MAPPING mode");
      sendAck(robot_id, "MAPPING_START", "STARTED");
      return;
    }

    // ✅ Only allow MAPPING_START when robot is IDLE
    if (sm_.getMode() != RobotMode::IDLE) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_START rejected: robot not IDLE (current=%s)",
        sm_.modeToString(sm_.getMode()).c_str());
      sendAck(robot_id, "MAPPING_START", "REJECTED");
      return;
    }

    // Send the ACK first so the cloud UI sees STARTED before the stack
    // restarts (the restart will kill this node a few seconds later).
    sendAck(robot_id, "MAPPING_START", "STARTED");

    // Trigger the auto-restart: kill the current launch_robot stack and
    // relaunch it in mapping mode inside a detached tmux session.
    RCLCPP_INFO(this->get_logger(),
      "MAPPING_START: restarting stack into MAPPING mode (nevo_restart.sh mapping)");
    // Spawn the restart as a transient systemd unit that escapes nevo.service's
    // cgroup, and run it with KillMode=process so the tmux stack it launches is
    // NOT killed when the unit's main process (the script) exits. A plain
    // `nohup bash ... &` keeps the script inside the service's control-group,
    // so `systemctl stop nevo.service` (KillMode=control-group) kills the swap
    // mid-flight. A `--scope` does NOT escape either (scopes created from
    // inside a service are nested under it). A transient unit with
    // KillMode=process escapes AND survives: systemd explicitly leaves the
    // remaining tmux/ros processes running after the unit stops.
    int rc = std::system(
        "sudo -n systemd-run --no-block --collect --property=KillMode=process "
        "--unit=nevo-restart-mapping "
        "--uid=nevo --gid=nevo --setenv=HOME=/home/nevo "
        "bash /home/nevo/nevo_restart.sh mapping "
        ">>/home/nevo/nevo_restart.log 2>&1 &");
    if (rc != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "MAPPING_START: failed to spawn nevo_restart.sh (rc=%d)", rc);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(),
      "navToMapping parse error: %s", e.what());
    sendError(std::string("JSON parse error: ") + e.what(), "ERROR", "HIGH");
  }
}

long long RobotApiNode::generateMapId(){
    static std::atomic<int> counter{0};

    auto now = std::chrono::system_clock::now();

    auto millis =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()).count();

    int seq = counter.fetch_add(1);

    return millis * 100000 + seq;
}

// ============================================================
// MAPPING SECTION - FINAL CORRECTED IMPLEMENTATION
// ============================================================

void RobotApiNode::mapUpload(const std::string & robot_id, const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    bool status = j.at("data").at("status").get<bool>();

    if(!status) {
      sendAck(robot_id, "MAPPING_SAVE", "REJECTED");
      return;
    }

    map_name_ = j.at("data").at("map_name").get<std::string>();
    std::replace(map_name_.begin(), map_name_.end(), ' ', '_');
    category_name_ = j.at("data").at("category").get<std::string>();
    category_type_ = j.at("data").at("category_type").get<std::string>();

    if (!std::filesystem::exists(map_base_path_)) {
      std::filesystem::create_directories(map_base_path_);
    }

    map_path_pgm  = map_base_path_ + "/" + map_name_ + ".pgm";
    map_path_yaml = map_base_path_ + "/" + map_name_ + ".yaml";

    // ✅ FIX 2: Hapus file lama jika ada (hindari false positive dari run sebelumnya)
    if (std::filesystem::exists(map_path_pgm)) {
      std::filesystem::remove(map_path_pgm);
      RCLCPP_INFO(this->get_logger(), "Removed old PGM: %s", map_path_pgm.c_str());
    }
    if (std::filesystem::exists(map_path_yaml)) {
      std::filesystem::remove(map_path_yaml);
      RCLCPP_INFO(this->get_logger(), "Removed old YAML: %s", map_path_yaml.c_str());
    }

    category_id = generateMapId();

    if(!slam_save_map_client_->wait_for_service(std::chrono::seconds(5))) {
      sendError("SLAM service not available", "SLAM_ERROR", "HIGH");
      return;
    }

    auto req = std::make_shared<slam_toolbox::srv::SaveMap::Request>();

    // ✅ FIX 3: Kirim FULL PATH tanpa ekstensi, bukan hanya nama file
    // slam_toolbox akan append .pgm dan .yaml sendiri
    std::string full_map_path = map_base_path_ + "/" + map_name_;
    req->name.data = full_map_path;

    RCLCPP_INFO(this->get_logger(),
      "Calling SLAM save_map with full path: '%s'", full_map_path.c_str());

    slam_save_map_client_->async_send_request(req,
      [this, robot_id](rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture future) {
        try {
          auto response = future.get();
          RCLCPP_INFO(this->get_logger(), "SLAM save_map service completed");

          auto pgm_path   = map_path_pgm;
          auto yaml_path  = map_path_yaml;
          int  max_retries = 20;  // ✅ FIX 4: Tambah retries (20 x 500ms = 10 detik)
          auto retry_count = std::make_shared<int>(0);

          if(delay_timer_) {
            delay_timer_->cancel();
            delay_timer_.reset();
          }

          delay_timer_ = this->create_wall_timer(
            500ms,  // ✅ FIX 5: Interval 500ms lebih aman untuk disk I/O
            [this, robot_id, pgm_path, yaml_path, max_retries, retry_count]() {

              (*retry_count)++;

              bool pgm_exists  = std::filesystem::exists(pgm_path);
              bool yaml_exists = std::filesystem::exists(yaml_path);

              // ✅ FIX 6: Cek ukuran file > 0 (bukan hanya exists)
              bool pgm_valid  = pgm_exists  && std::filesystem::file_size(pgm_path)  > 0;
              bool yaml_valid = yaml_exists && std::filesystem::file_size(yaml_path) > 0;

              RCLCPP_INFO(this->get_logger(),
                "File check #%d/%d: PGM=%s(%s) YAML=%s(%s)",
                *retry_count, max_retries,
                pgm_exists  ? "EXISTS" : "MISSING",
                pgm_valid   ? "VALID"  : "EMPTY",
                yaml_exists ? "EXISTS" : "MISSING",
                yaml_valid  ? "VALID"  : "EMPTY");

              if (pgm_valid && yaml_valid) {
                RCLCPP_INFO(this->get_logger(),
                  "Map files verified OK: %s", pgm_path.c_str());

                uploadMapToServer(robot_id);
                sendAck(robot_id, "MAPPING_SAVE", "SAVED");

                if(delay_timer_) {
                  delay_timer_->cancel();
                  delay_timer_.reset();
                }
              }
              else if (*retry_count >= max_retries) {
                RCLCPP_ERROR(this->get_logger(),
                  "Map files not found after %d retries (%.1fs). "
                  "Check slam_toolbox has write permission to: %s",
                  max_retries,
                  max_retries * 0.5,
                  pgm_path.c_str());

                // ✅ FIX 7: Debug — list isi direktori untuk diagnosis
                try {
                  std::string dir = std::filesystem::path(pgm_path).parent_path().string();
                  RCLCPP_ERROR(this->get_logger(), "Contents of %s:", dir.c_str());
                  for (const auto & entry : std::filesystem::directory_iterator(dir)) {
                    RCLCPP_ERROR(this->get_logger(), "  %s", entry.path().string().c_str());
                  }
                } catch (...) {}

                sendError("Map files not created by SLAM", "FILE_ERROR", "HIGH");

                if(delay_timer_) {
                  delay_timer_->cancel();
                  delay_timer_.reset();
                }
              }
            }
          );

        } catch (const std::exception & e) {
          RCLCPP_ERROR(this->get_logger(), "SLAM service error: %s", e.what());
          sendError(std::string("SLAM error: ") + e.what(), "SLAM_ERROR", "HIGH");
        }
      }
    );

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "mapUpload error: %s", e.what());
    sendError(std::string("mapUpload error: ") + e.what(), "ERROR", "HIGH");
  }
}
// ============================================================

void RobotApiNode::uploadMapToServer(const std::string & robot_id)
{
  try {
    // ✅ Validasi file ada dan tidak kosong SEBELUM upload
    if (!std::filesystem::exists(map_path_pgm) || 
        std::filesystem::file_size(map_path_pgm) == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "PGM file missing or empty: %s", map_path_pgm.c_str());
      sendError("PGM file not found: " + map_path_pgm, "UPLOAD_ERROR", "HIGH");
      return;
    }
    if (!std::filesystem::exists(map_path_yaml) || 
        std::filesystem::file_size(map_path_yaml) == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "YAML file missing or empty: %s", map_path_yaml.c_str());
      sendError("YAML file not found: " + map_path_yaml, "UPLOAD_ERROR", "HIGH");
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Uploading: PGM=%s (%.1f KB), YAML=%s (%.1f KB)",
      map_path_pgm.c_str(),
      std::filesystem::file_size(map_path_pgm) / 1024.0,
      map_path_yaml.c_str(),
      std::filesystem::file_size(map_path_yaml) / 1024.0);

    std::unique_ptr<CURL, decltype(&curl_easy_cleanup)> curl(
        curl_easy_init(), &curl_easy_cleanup);

    if(!curl) {
      sendError("CURL initialization failed", "UPLOAD_ERROR", "HIGH");
      return;
    }

    std::unique_ptr<curl_mime, decltype(&curl_mime_free)> form(
        curl_mime_init(curl.get()), &curl_mime_free);

    curl_mimepart *field;

    json categories = json::array({{
      {"category_id",   category_id},
      {"category_name", category_name_},
      {"category_type", category_type_},
      {"color",         "#3b82f6"},
      {"sort_order",    0},
      {"is_active",     1}
    }});

    json destinations = json::array();
    {
      std::lock_guard<std::mutex> lock(flags_mutex_);
      for(const auto & flag : flagged_coordinates_) {
        destinations.push_back({
          {"destination_id",   flag.flag_id},
          {"category_id",      category_id},
          {"destination_name", flag.label},
          {"destination_code", "FLAG_" + std::to_string(flag.flag_id)},
          {"x",                flag.x},
          {"y",                flag.y},
          {"yaw",              flag.yaw},
          {"zone",             category_name_},
          {"is_home",          0}
        });
      }
    }

    destinations.push_back({
      {"destination_id",   99},
      {"category_id",      nullptr},
      {"destination_name", "Home"},
      {"destination_code", "HOME_BASE"},
      {"x",                home_base_pose_.pose.position.x},
      {"y",                home_base_pose_.pose.position.y},
      {"yaw",              tf2::getYaw(home_base_pose_.pose.orientation)},
      {"zone",             "Base"},
      {"is_home",          1}
    });

    std::string cat_str  = categories.dump();
    std::string dest_str = destinations.dump();

    // Build form fields
    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "mapName");
    curl_mime_data(field, map_name_.c_str(), CURL_ZERO_TERMINATED);

    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "uploadedBy");
    curl_mime_data(field, uploaded_by_.c_str(), CURL_ZERO_TERMINATED);

    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "pgmFile");
    curl_mime_filedata(field, map_path_pgm.c_str());

    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "yamlFile");
    curl_mime_filedata(field, map_path_yaml.c_str());

    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "categories");
    curl_mime_data(field, cat_str.c_str(), CURL_ZERO_TERMINATED);

    field = curl_mime_addpart(form.get());
    curl_mime_name(field, "destinations");
    curl_mime_data(field, dest_str.c_str(), CURL_ZERO_TERMINATED);

    // ✅ Capture response body dari server
    std::string response_body;
    curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION,
      +[](char* ptr, size_t size, size_t nmemb, void* userdata) -> size_t {
        auto* body = static_cast<std::string*>(userdata);
        body->append(ptr, size * nmemb);
        return size * nmemb;
      });
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response_body);

    curl_easy_setopt(curl.get(), CURLOPT_URL,     upload_server_url_.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_MIMEPOST, form.get());
    curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT,  30L);

    CURLcode res       = curl_easy_perform(curl.get());
    long     http_code = 0;
    curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &http_code);

    if(res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(),
        "CURL failed: %s", curl_easy_strerror(res));
      sendError(std::string("Upload failed: ") + curl_easy_strerror(res),
                "UPLOAD_ERROR", "HIGH");
      return;
    }

    // ✅ Log HTTP status code + response body untuk debug
    RCLCPP_INFO(this->get_logger(),
      "Upload HTTP %ld — response: %s",
      http_code, response_body.c_str());

    if(http_code != 200 && http_code != 201) {
      RCLCPP_ERROR(this->get_logger(),
        "Server rejected upload (HTTP %ld): %s",
        http_code, response_body.c_str());
      sendError("Server error HTTP " + std::to_string(http_code) + ": " + response_body,
                "UPLOAD_ERROR", "HIGH");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Map uploaded successfully ✅");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "uploadMapToServer error: %s", e.what());
    sendError(std::string("Upload error: ") + e.what(), "UPLOAD_ERROR", "HIGH");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(flags_mutex_);
    flagged_coordinates_.clear();
  }
}

std::string RobotApiNode::sanitizeMapName(const std::string & raw_name)
{
  // ✅ Check 1: Empty string
  if (raw_name.empty()) {
    throw std::invalid_argument("Map name cannot be empty");
  }

  // ✅ Check 2: Length limit
  if (raw_name.length() > 100) {
    throw std::invalid_argument("Map name too long (max 100 characters)");
  }

  std::string sanitized = raw_name;

  // ✅ Check 3: Replace spaces with underscores
  std::replace(sanitized.begin(), sanitized.end(), ' ', '_');

  // ✅ Check 4: Remove invalid filesystem characters
  const std::string invalid_chars = "/\\:*?\"<>|";
  sanitized.erase(
    std::remove_if(sanitized.begin(), sanitized.end(),
      [&invalid_chars](char c) { 
        return invalid_chars.find(c) != std::string::npos;
      }),
    sanitized.end()
  );

  // ✅ Check 5: Prevent path traversal
  if (sanitized.find("..") != std::string::npos) {
    throw std::invalid_argument(
      "Map name contains invalid path pattern '..'");
  }

  // ✅ Check 6: Ensure name valid after sanitization
  if (sanitized.find_first_not_of("_.") == std::string::npos) {
    throw std::invalid_argument(
      "Map name invalid after sanitization");
  }

  RCLCPP_DEBUG(this->get_logger(),
    "Sanitized map name: '%s' → '%s'",
    raw_name.c_str(), sanitized.c_str());

  return sanitized;
}

double RobotApiNode::getJsonDouble(
    const json & obj,
    const std::vector<std::string> & keys,
    double default_value)
{
  try {
    json current = obj;
    for (const auto & key : keys) {
      if (!current.contains(key)) {
        throw std::out_of_range("Key not found: " + key);
      }
      current = current.at(key);
    }
    return current.get<double>();
  } catch (const std::exception & e) {
    std::string key_path;
    for (const auto & k : keys) {
      key_path += k + ".";
    }
    RCLCPP_DEBUG(this->get_logger(), 
      "JSON key path '%s' not found: %s, using default %.2f",
      key_path.c_str(), e.what(), default_value);
    return default_value;
  }
}

// Update handleTeleop() to use safe access
void RobotApiNode::handleTeleop(const std::string & raw_json)
{
  try{
    json j = json::parse(raw_json);

    RCLCPP_INFO(this->get_logger(), "Received TELEOP command");
  
    geometry_msgs::msg::Twist cmd_diff;
    
    // ✅ USE SAFE JSON ACCESS
    cmd_diff.linear.x = getJsonDouble(j, {"data", "linear", "x"}, 0.0);
    cmd_diff.linear.y = getJsonDouble(j, {"data", "linear", "y"}, 0.0);
    cmd_diff.linear.z = getJsonDouble(j, {"data", "linear", "z"}, 0.0);
  
    cmd_diff.angular.x = getJsonDouble(j, {"data", "angular", "x"}, 0.0);
    cmd_diff.angular.y = getJsonDouble(j, {"data", "angular", "y"}, 0.0);
    cmd_diff.angular.z = getJsonDouble(j, {"data", "angular", "z"}, 0.0);

    diff_cont_pub->publish(cmd_diff);
    
    RCLCPP_INFO(this->get_logger(), 
      "Published TELEOP: linear=(%.2f,%.2f,%.2f) angular=(%.2f,%.2f,%.2f)",
      cmd_diff.linear.x, cmd_diff.linear.y, cmd_diff.linear.z,
      cmd_diff.angular.x, cmd_diff.angular.y, cmd_diff.angular.z);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "TELEOP parse error: %s", e.what());
    sendError(std::string("TELEOP error: ") + e.what(),
              "ERROR",
              "HIGH");
  }
}

bool RobotApiNode::isValidSequence(int sequence)
{
  // Allow -1 (home base) and 1-10000 for regular goals
  return (sequence == -1) || (sequence > 0 && sequence <= MAX_SEQUENCE);
}

void RobotApiNode::handleMove(const std::string & robot_id,
                              const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    json data = j.at("data");

    // ===== STEP 1: Extract & Validate Sequence =====
    int sequence = data.value("sequence", 0);
    
    // ✅ ADD VALIDATION
    if (!isValidSequence(sequence)) {
      RCLCPP_WARN(this->get_logger(), 
        "handleMove: Invalid sequence=%d (expected -1 or 1-%d)",
        sequence, MAX_SEQUENCE);
      sendError("Invalid sequence number", 
               "INVALID_SEQUENCE", "MEDIUM");
      return;
    }

    // ===== STEP 2: Extract Type =====
    std::string type = data.value("type", "GENERIC");
    
    RCLCPP_INFO(this->get_logger(), 
      "handleMove: sequence=%d, type=%s",
      sequence, type.c_str());

    // ===== STEP 3: Handle HOMEBASE Interrupt Recovery =====
    if (type == "HOMEBASE" && sm_.getMode() == RobotMode::INTERRUPTED) {
      RCLCPP_INFO(this->get_logger(), 
        "handleMove: Recovering from INTERRUPTED, clearing paused goal");
      
      paused_goal_.reset();
      goal_in_progress_ = false;
      current_move_sequence_ = -1;
      current_move_type_.clear();
    }

    // ===== STEP 4: Update Current Move Context =====
    current_move_sequence_ = sequence;
    current_move_type_ = type;

    // ===== STEP 5: Update Home Base (if provided) =====
    if (data.contains("home_base")) {
      auto hb = data.at("home_base");
      double hb_x = hb.at("x").get<double>();
      double hb_y = hb.at("y").get<double>();
      double hb_yaw = hb.at("yaw").get<double>();
      
      home_base_pose_ = buildPoseStamped(hb_x, hb_y, hb_yaw);
      has_home_base_ = true;
      
      RCLCPP_INFO(this->get_logger(), 
        "handleMove: Updated home_base to (%.2f, %.2f, %.2f)",
        hb_x, hb_y, hb_yaw);
    }

    // ===== STEP 6: Extract & Build Destination Pose =====
    if (!data.contains("dest")) {
      throw std::runtime_error("MOVE missing 'dest' field");
    }
    
    auto dest = data.at("dest");
    double dest_x = dest.at("x").get<double>();
    double dest_y = dest.at("y").get<double>();
    double dest_yaw = dest.at("yaw").get<double>();
    
    auto pose = buildPoseStamped(dest_x, dest_y, dest_yaw);
    
    RCLCPP_INFO(this->get_logger(), 
      "handleMove: destination (%.2f, %.2f, %.2f) for sequence %d",
      dest_x, dest_y, dest_yaw, sequence);

    // ===== STEP 7: Validate Robot State =====
    if (sm_.getMode() != RobotMode::IDLE && type != "HOMEBASE") {
      RCLCPP_WARN(this->get_logger(),
        "handleMove: Robot not IDLE (current=%s), rejecting move seq=%d",
        sm_.modeToString(sm_.getMode()).c_str(), sequence);
      
      sendError("MOVE rejected: robot not IDLE",
                "MOTOR_FAIL", "MEDIUM");
      return;
    }

    // ===== STEP 8: Set State & Send Goal =====
    sm_.setMode("MOVING");
    RCLCPP_INFO(this->get_logger(), 
      "STATE_MACHINE: MODE = MOVING (from handleMove seq=%d)", sequence);
    
    sendSingleNavGoal(sequence, pose);

    // ===== STEP 9: Send Acknowledgement =====
    sendAck(robot_id, "MOVE", "ACCEPTED");
    
    RCLCPP_INFO(this->get_logger(), 
      "handleMove: MOVE seq=%d accepted and goal sent", sequence);

  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(this->get_logger(), 
      "handleMove: Invalid argument: %s", e.what());
    sendError(std::string("Invalid argument: ") + e.what(),
              "MOTOR_FAIL", "MEDIUM");
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), 
      "handleMove: Runtime error: %s", e.what());
    sendError(std::string("Runtime error: ") + e.what(),
              "MOTOR_FAIL", "MEDIUM");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), 
      "handleMove: Exception: %s", e.what());
    sendError(std::string("handleMove error: ") + e.what(),
              "MOTOR_FAIL", "MEDIUM");
  }
}

// Update handleOp() to validate
void RobotApiNode::handleOp(const std::string & robot_id,
                            const std::string & raw_json)
{
  if (sm_.getMode() != RobotMode::IDLE){
    RCLCPP_WARN(this->get_logger(),
      "OP rejected: robot not IDLE");
    sendError("OP rejected: robot not IDLE",
              "MOTOR_FAIL", "MEDIUM");
    return;
  }  

  try {
    json j = json::parse(raw_json);
    json data = j.at("data");

    // Goals are coordinates in one specific map: refuse them if the dashboard
    // means a different map than the one map_server is running.
    if (data.contains("map_id") && active_map_id_ >= 0) {
      int op_map_id = data.at("map_id").is_string()
        ? std::stoi(data.at("map_id").get<std::string>())
        : data.at("map_id").get<int>();
      if (op_map_id != active_map_id_) {
        std::string msg = "OP map_id " + std::to_string(op_map_id) +
          " does not match the loaded map " + std::to_string(active_map_id_);
        if (enforce_map_id_) {
          RCLCPP_ERROR(this->get_logger(), "OP rejected: %s", msg.c_str());
          sendError("OP rejected: " + msg, "MAP_MISMATCH", "HIGH");
          return;
        }
        RCLCPP_WARN(this->get_logger(), "%s (enforce_map_id=false, continuing)", msg.c_str());
      }
    }

    op_sequence_counter_ = 0;

    if (data.contains("home_base")) {
      auto hb = data.at("home_base");
      home_base_pose_ = buildPoseStamped(
        hb.at("x").get<double>(),
        hb.at("y").get<double>(),
        hb.at("yaw").get<double>()
      );
      has_home_base_ = true;
    }

    if (data.contains("type")) {
      std::string type = data.at("type").get<std::string>();
      RCLCPP_INFO(this->get_logger(), "OP type=%s", type.c_str());
    }

    if (!data.contains("tray_tasks")) {
      throw std::runtime_error("OP missing 'tray_tasks'");
    }

    auto tray_tasks = data.at("tray_tasks");
    if (!tray_tasks.is_array() || tray_tasks.empty()) {
      throw std::runtime_error("OP 'tray_tasks' must be non-empty array");
    }

    int enq_count = 0;

    for (const auto & task : tray_tasks) {
      if (!task.contains("dest")) {
        throw std::runtime_error("tray_task missing 'dest'");
      }

      auto dest = task.at("dest");
      double x = dest.at("x").get<double>();
      double y = dest.at("y").get<double>();
      double yaw = dest.at("yaw").get<double>();

      auto pose = buildPoseStamped(x, y, yaw);

      op_sequence_counter_++;
      int sequence = op_sequence_counter_;
      
      // ✅ ADD VALIDATION
      if (!isValidSequence(sequence)) {
        RCLCPP_WARN(this->get_logger(), 
          "OP: Sequence overflow, stopping at %d", enq_count);
        break;
      }
      
      enqueueGoal(sequence, pose);
      enq_count++;

      RCLCPP_INFO(this->get_logger(),
        "OP: enqueued seq=%d x=%.3f y=%.3f yaw=%.3f",
        sequence, x, y, yaw);
    }

    sm_.setMode("MOVING");
    processNextGoal();

    sendAck(robot_id, "OP", "ENQUEUED");

    RCLCPP_INFO(this->get_logger(),
      "OP handled: %d tray_tasks enqueued", enq_count);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "handleOp error: %s", e.what());
    sendError(std::string("handleOp error: ") + e.what(),
              "MOTOR_FAIL", "MEDIUM");
  }
}


void RobotApiNode::mappingToNav(const std::string & robot_id,
                                const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    bool status = j.at("data").at("status").get<bool>();

    // ✅ CORRECTED LOGIC: false = stop mapping, true = reject
    if(status) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_STOP rejected: status=true (mapping not active)");
      sendAck(robot_id, "MAPPING_STOP", "REJECTED");
      return;
    }

    // Already IDLE — idempotent ACK
    if (sm_.getMode() == RobotMode::IDLE) {
      RCLCPP_INFO(this->get_logger(),
        "MAPPING_STOP: already in IDLE mode");
      sendAck(robot_id, "MAPPING_STOP", "STOPPED");
      return;
    }

    if (sm_.getMode() != RobotMode::MAPPING) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_STOP rejected: current mode = %s",
        sm_.modeToString(sm_.getMode()).c_str());
      sendAck(robot_id, "MAPPING_STOP", "REJECTED");
      return;
    }

    // Send the ACK first so the cloud UI sees STOPPED before the stack
    // restarts (the restart will kill this node a few seconds later).
    sendAck(robot_id, "MAPPING_STOP", "STOPPED");

    // Trigger the auto-restart: kill the current launch_robot stack and
    // relaunch it in nav mode inside a detached tmux session.
    RCLCPP_INFO(this->get_logger(),
      "MAPPING_STOP: restarting stack into NAV mode (nevo_restart.sh nav)");
    int rc = std::system(
        "sudo -n systemd-run --no-block --collect --property=KillMode=process "
        "--unit=nevo-restart-nav "
        "--uid=nevo --gid=nevo --setenv=HOME=/home/nevo "
        "bash /home/nevo/nevo_restart.sh nav "
        ">>/home/nevo/nevo_restart.log 2>&1 &");
    if (rc != 0) {
      RCLCPP_ERROR(this->get_logger(),
        "MAPPING_STOP: failed to spawn nevo_restart.sh (rc=%d)", rc);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(),
      "mappingToNav parse error: %s", e.what());
    sendError(std::string("JSON parse error: ") + e.what(), "ERROR", "HIGH");
  }
}

void RobotApiNode::flagCoordinate(const std::string & robot_id,
                                  const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    bool status = j.at("data").at("status").get<bool>();

    if(!status) {
      sendAck(robot_id, "MAPPING_FLAG", "REJECTED");
      return;
    }

    // ✅ BUG 4 FIX: Only allow flags when in MAPPING mode
    if (sm_.getMode() != RobotMode::MAPPING) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_FLAG rejected: robot not in MAPPING mode (current=%s)",
        sm_.modeToString(sm_.getMode()).c_str());
      sendAck(robot_id, "MAPPING_FLAG", "REJECTED");
      return;
    }

    double x = 0.0, y = 0.0, yaw = 0.0;
    std::string source = "";

    // ✅ FIX: Flags were recorded in the raw odom frame, but the saved map
    // uses the map frame (with an origin offset, e.g. origin: [-6.92, -10.37]).
    // Convert the robot pose to the MAP frame via TF so flags stay aligned
    // with the saved/reloaded map and navigation destinations line up.
    try {
      auto t = tf_buffer_.lookupTransform(
        "map", "base_link", rclcpp::Time(0));
      x   = t.transform.translation.x;
      y   = t.transform.translation.y;
      yaw = tf2::getYaw(t.transform.rotation);
      source = "tf_map_frame";

      RCLCPP_INFO(this->get_logger(),
        "MAPPING_FLAG: map-frame pos x=%.3f y=%.3f yaw=%.3f", x, y, yaw);

    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(),
        "MAPPING_FLAG: map->base_link unavailable (%s); "
        "falling back to odom-frame position", e.what());

      if(has_last_position_) {
        try {
          // ✅ Struktur aktual: data.x, data.y, data.yaw (tanpa nested "pose")
          x   = last_position_json_.at("data").at("x").get<double>();
          y   = last_position_json_.at("data").at("y").get<double>();
          yaw = last_position_json_.at("data").at("yaw").get<double>();
          source = "sensor_position";

          RCLCPP_INFO(this->get_logger(),
            "MAPPING_FLAG: sensor pos x=%.3f y=%.3f yaw=%.3f", x, y, yaw);

        } catch (const std::exception & e) {
          RCLCPP_WARN(this->get_logger(),
            "Cannot parse position: %s", e.what());
          if(has_home_base_) {
            x   = home_base_pose_.pose.position.x;
            y   = home_base_pose_.pose.position.y;
            yaw = tf2::getYaw(home_base_pose_.pose.orientation);
            source = "home_base_fallback";
          }
        }
      } else {
        RCLCPP_WARN(this->get_logger(),
          "MAPPING_FLAG: has_last_position_=false. "
          "Pastikan topic 'nevo/sensor/position' aktif dan publish data.");

        if(has_home_base_) {
          x   = home_base_pose_.pose.position.x;
          y   = home_base_pose_.pose.position.y;
          yaw = tf2::getYaw(home_base_pose_.pose.orientation);
          source = "home_base_fallback";
        }
      }
    }

    FlaggedCoordinate flag;
    flag.flag_id  = ++flag_counter_;
    flag.x        = x;
    flag.y        = y;
    flag.yaw      = yaw;
    flag.timestamp = this->now();

    // ✅ FIX: Baca "goal_name" dulu, fallback ke "label", lalu ke default
    auto & data = j.at("data");
    if (data.contains("goal_name") && !data["goal_name"].get<std::string>().empty()) {
      flag.label = data["goal_name"].get<std::string>();
    } else if (data.contains("label") && !data["label"].get<std::string>().empty()) {
      flag.label = data["label"].get<std::string>();
    } else {
      flag.label = "FLAG_" + std::to_string(flag.flag_id);
    }

    {
      std::lock_guard<std::mutex> lock(flags_mutex_);
      flagged_coordinates_.push_back(flag);
    }

    RCLCPP_INFO(this->get_logger(),
      "MAPPING_FLAG buffered: ID=%d, pos=(%.3f, %.3f, %.3f), "
      "source=%s, label='%s', total=%zu",
      flag.flag_id, x, y, yaw,
      source.c_str(), flag.label.c_str(),
      flagged_coordinates_.size());

    sendAck(robot_id, "MAPPING_FLAG", "FLAGGED");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "flagCoordinate error: %s", e.what());
    sendError(std::string("JSON parse error: ") + e.what(), "ERROR", "HIGH");
  }
}

// ============================================================
//                    MAPPING SECTION END
// ============================================================


void RobotApiNode::handlePickUp(const std::string & raw_json)
{
  json j = json::parse(raw_json);
  int seq = j.at("data").at("sequence").get<int>();

  if(sm_.getMode() != RobotMode::PAUSED){
    RCLCPP_INFO(this->get_logger(), 
    "PICKUP_CONFIRM ignored, not in waiting pickup");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if(!goal_queue_.empty()){
      goal_queue_.pop();
    }
    goal_in_progress_ = false;
  }

  RCLCPP_INFO(this->get_logger(), 
              "Pickup confirmed for seq=%d", seq);

  if(goal_queue_.empty()){
    // Single MOVE (bukan OP) → langsung IDLE, tidak perlu pulang
    if(!current_move_type_.empty()){
      RCLCPP_INFO(this->get_logger(),
                  "Single MOVE confirmed, returning to IDLE");
      sendLastPoseOnce();
      sm_.setMode("IDLE");
      current_move_type_.clear();
      current_move_sequence_ = -1;
    }else if(has_home_base_) {
      RCLCPP_INFO(this->get_logger(),
                  "All goals finished, returning to home base");

      sm_.setMode("MOVING");
      sendSingleNavGoal(-1, home_base_pose_);
    }else{
      sm_.setMode("IDLE");
    }
  }else{
    RCLCPP_INFO(this->get_logger(), 
                  "go to next goal");
    sm_.setMode("MOVING");
    processNextGoal();
  }
}




void RobotApiNode::handleMode(const std::string & robot_id,
                              const std::string & raw_json)
{
  try {
    json j = json::parse(raw_json);
    std::string mode_str = j.at("data").at("mode").get<std::string>();

    bool ok = sm_.setMode(mode_str);

    if (!ok) {
      RCLCPP_WARN(this->get_logger(),
        "Invalid mode transition request to '%s'", mode_str.c_str());
      sendError("Invalid mode transition to: " + mode_str,
                "ERROR",
                "MEDIUM");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "STATE_MACHINE: MODE = %s (internal)", mode_str.c_str());
    sendAck(robot_id, "MODE", "ACCEPTED");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "handleMode error: %s", e.what());
    sendError(std::string("handleMode error: ") + e.what(),
              "ERROR",
              "MEDIUM");
  }
}

geometry_msgs::msg::PoseStamped RobotApiNode::buildPoseStamped(
  double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = this->now();
  pose.header.frame_id = "map";

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);

  return pose;
}

//custom code

void RobotApiNode::handleINT(const std::string & robot_id,
                             const std::string &)
{
  if (sm_.getMode() != RobotMode::MOVING) {
    RCLCPP_WARN(this->get_logger(),
      "INTERRUPT rejected: current mode = %s",
      sm_.modeToString(sm_.getMode()).c_str());

    sendAck(robot_id, "INTERRUPT", "REJECTED");
    return;
  }

  if (!sm_.setMode("INTERRUPTED")) {
    RCLCPP_WARN(this->get_logger(),
      "INTERRUPT failed: cannot transition the mode");

    sendAck(robot_id, "INTERRUPT", "REJECTED");
    return;
  }

  RCLCPP_WARN(this->get_logger(), "STATE_MACHINE: mode = INTERRUPTED");

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // ===== CASE 1: INTERRUPT MOVE =====
    if (current_move_sequence_ >= 0) {
      paused_goal_ = GoalItem{
        current_move_sequence_,
        last_sent_pose_   // <-- pose MOVE terakhir yang dikirim ke Nav2
      };

      RCLCPP_WARN(this->get_logger(),
        "Paused MOVE seq=%d",
        paused_goal_->sequence);
    }
    // ===== CASE 2: INTERRUPT OP =====
    else if (goal_in_progress_ && !goal_queue_.empty()) {
      paused_goal_ = goal_queue_.front();

      RCLCPP_WARN(this->get_logger(),
        "Paused OP goal seq=%d",
        paused_goal_->sequence);
    }
  }

  // Cancel semua goal Nav2
  if (nav_client_) {
    RCLCPP_WARN(this->get_logger(), "INTERRUPT: canceling all Nav2 goals");
    nav_client_->async_cancel_all_goals();
  }

  if (delay_timer_) {
    delay_timer_->cancel();
    delay_timer_.reset();
  }

  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }

  sendAck(robot_id, "INTERRUPT", "ACCEPTED");
}


void RobotApiNode::handleICL(const std::string & robot_id,
                             const std::string &)
{
  if (sm_.getMode() != RobotMode::INTERRUPTED) {
    sendAck(robot_id, "INTERRUPT_CANCEL", "REJECTED");
    return;
  }

  if (!paused_goal_) {
    sendAck(robot_id, "INTERRUPT_CANCEL", "NO_PAUSED_GOAL");
    return;
  }

  // ✅ FIX: Save paused goal BEFORE any modification
  GoalItem resume_goal = *paused_goal_;
  bool is_move_type = (current_move_sequence_ >= 0);

  sm_.setMode("MOVING");

  // ✅ FIX: Clear state AFTER saving
  paused_goal_.reset();

  // ✅ FIX: Proper type-based resume
  if (is_move_type) {
    // ===== RESUME MOVE =====
    sendSingleNavGoal(resume_goal.sequence, resume_goal.pose);
    sendAck(robot_id, "INTERRUPT_CANCEL", "RESUMED");
  } else {
    // ===== RESUME OP (Queue) =====
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      goal_queue_.push(resume_goal);
    }
    processNextGoal();
    sendAck(robot_id, "INTERRUPT_CANCEL", "RESUMED");
  }
}



void RobotApiNode::handleICM(const std::string & robot_id,
                             const std::string &)
{
  if (sm_.getMode() != RobotMode::INTERRUPTED) {
    sendAck(robot_id, "INTERRUPT_CONFIRM", "REJECTED");
    return;
  }

  if (!has_home_base_) {
    RCLCPP_WARN(this->get_logger(),
      "No home_base, forcing IDLE");

    paused_goal_.reset();
    goal_in_progress_ = false;
    current_move_sequence_ = -1;
    current_move_type_.clear();

    sm_.setMode("IDLE");
    return;
  }


  if (nav_client_) {
    nav_client_->async_cancel_all_goals();
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!goal_queue_.empty()) goal_queue_.pop();
    goal_in_progress_ = false;
    paused_goal_.reset();
  }

  // CLEAR MOVE CONTEXT
  current_move_sequence_ = -1;
  current_move_type_.clear();

  home_timer_ = this->create_wall_timer(
    300ms,
    [this, robot_id]() {
      home_timer_->cancel();
      home_timer_.reset();

      sm_.setMode("MOVING");
      sendSingleNavGoal(-1, home_base_pose_);
    }
  );

  sendAck(robot_id, "INTERRUPT_CONFIRM", "RETURNING_HOME");
}




void RobotApiNode::sendAck(
  const std::string & robot_id,
  const std::string & command_id,
  const std::string & status)
{
  json j;
  j["code"] = "ACK";
  j["data"] = {
    {"robot_id", robot_id},
    {"ui_id", default_ui_id_},
    {"command_id", command_id},
    {"status", status}
  };

  auto out = j.dump();

  std_msgs::msg::String msg;
  msg.data = out;
  ws_out_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "WS_OUT ACK: %s", out.c_str());
}

// Announce to the cloud that THIS stack is up and in a given mode. Sent after
// every (re)connection and periodically during startup so that a mode swap
// (which kills the requesting node) has a positive completion signal instead
// of the UI spinning forever.
void RobotApiNode::sendReady()
{
  json j;
  j["code"] = "READY";
  j["data"] = {
    {"robot_id", default_robot_id_},
    {"ui_id", default_ui_id_},
    {"mode", ready_mode_},
    {"ready", true}
  };

  auto out = j.dump();

  std_msgs::msg::String msg;
  msg.data = out;
  ws_out_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "WS_OUT READY: %s", out.c_str());
}

void RobotApiNode::networkStatusCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "CONNECTED" || msg->data == "FALLBACK") {
    if (ready_timer_) {
      ready_timer_->cancel();
    }
    sendReady();
  }
}

void RobotApiNode::sendError(const std::string & message, 
                            const std::string & err_code,
                            const std::string & prior)
{
  // rclcpp::Time now = this->get_clock()->now();

  json j;
  j["code"] = "ERROR";
  j["data"] = {
    {"robot_id", default_robot_id_},
    {"ui_id", default_ui_id_},
    {"error_code", err_code}, 
    {"error_message", message} ,
    {"severity", prior}
  };

  auto out = j.dump();

  std_msgs::msg::String msg;
  msg.data = out;
  ws_out_pub_->publish(msg);

  RCLCPP_WARN(this->get_logger(), "WS_OUT ERROR: %s", out.c_str());
}

void RobotApiNode::sendSingleNavGoal(
  int sequence,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Jangan kirim goal baru kalau sedang INTERRUPTED
  if (sm_.getMode() == RobotMode::INTERRUPTED) {
    RCLCPP_INFO(this->get_logger(), "Sequence %d canceled (INTERRUPTED)", sequence);
    return;
  }

  // Simpan pose terakhir (WAJIB untuk resume MOVE)
  last_sent_pose_ = pose;

  // Pastikan action client ada
  if (!nav_client_) {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  // Pastikan server tersedia
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_WARN(this->get_logger(),
      "navigate_to_pose action server not available (single MOVE)");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    goal_in_progress_ = true;
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  RCLCPP_INFO(this->get_logger(),
    "Sending single MOVE goal seq=%d", sequence);

  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // ===== goal_response_callback =====
  send_goal_options.goal_response_callback =
    [this, sequence](GoalHandleNavigate::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(),
          "MOVE goal seq=%d rejected", sequence);

        std::lock_guard<std::mutex> lock(queue_mutex_);
        goal_in_progress_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(),
          "MOVE goal seq=%d accepted", sequence);
      }
    };

  // ===== feedback_callback (opsional) =====
  send_goal_options.feedback_callback =
    [this](GoalHandleNavigate::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (feedback) {
        RCLCPP_DEBUG(this->get_logger(),
          "MOVE feedback — dist remaining: %.3f",
          feedback->distance_remaining);
      }
    };

  // ===== result_callback =====
  send_goal_options.result_callback =
    [this, sequence](const GoalHandleNavigate::WrappedResult & result) {

      // INTERRUPT → abaikan result
      
      if (result.code == rclcpp_action::ResultCode::CANCELED &&
          sm_.getMode() == RobotMode::INTERRUPTED) {

        std::lock_guard<std::mutex> lock(queue_mutex_);
        goal_in_progress_ = false;
        return;
      }

      {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        goal_in_progress_ = false;
      }

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(),
            "MOVE SUCCEEDED (seq=%d)", sequence);

          if (sequence >= 0) {
            sendGoalComplete(sequence);
          }

          // Tunggu konfirmasi UI sebelum kembali ke IDLE
          sm_.setMode("PAUSED");
          RCLCPP_INFO(this->get_logger(),
            "Waiting for PICKUP CONFIRM (seq=%d)", sequence);
          break;

        case rclcpp_action::ResultCode::ABORTED:
          sendError(std::string("MOVE ABORTED — auto-recovering"),
            "NAV_ABORTED", "HIGH");
          RCLCPP_WARN(this->get_logger(),
            "MOVE ABORTED (seq=%d) — triggering auto-recovery", sequence);
      
          // ── TAMBAHAN: jangan panggil resetToIdle jika sudah dalam proses reset
          if (!is_resetting_) {
            resetToIdle("MOVE ABORTED seq=" + std::to_string(sequence));
          } else {
            RCLCPP_WARN(this->get_logger(),
              "MOVE ABORTED result ignored — reset already in progress");
            std::lock_guard<std::mutex> lock(queue_mutex_);
            goal_in_progress_ = false;
          }
          return;

        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(this->get_logger(),
            "MOVE CANCELED (seq=%d)", sequence);
          break;

        default:
          RCLCPP_WARN(this->get_logger(),
            "MOVE UNKNOWN RESULT (seq=%d)", sequence);
          break;
      }

      // Home goal (-1) tetap safety net
      if (sequence == -1 &&
        result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        sendLastPoseOnce();
        sm_.setMode("IDLE");
      }
    };

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}


void RobotApiNode::enqueueGoal(int sequence, 
                              const geometry_msgs::msg::PoseStamped & pose)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // ✅ ADD SIZE CHECK
    if (goal_queue_.size() >= MAX_QUEUE_SIZE) {
      RCLCPP_WARN(this->get_logger(), 
        "Goal queue full (max %zu), rejecting seq=%d",
        MAX_QUEUE_SIZE, sequence);
      sendError("Goal queue full", "QUEUE_FULL", "MEDIUM");
      return;
    }
    
    GoalItem item;
    item.sequence = sequence;
    item.pose = pose;
    goal_queue_.push(item);
    
    RCLCPP_DEBUG(this->get_logger(),
      "Goal enqueued, queue size=%zu/%zu",
      goal_queue_.size(), MAX_QUEUE_SIZE);
  }
  
  if (sm_.getMode() == RobotMode::MOVING){
    processNextGoal();
  }
}

void RobotApiNode::sendGoalComplete(int sequence)
{
  json j;
  j["code"] = "GOAL_COMPLETE";
  j["data"] = {
    {"robot_id", default_robot_id_},
    {"ui_id", default_ui_id_},
    {"sequence", sequence}
  };

  auto out = j.dump();

  std_msgs::msg::String msg;
  msg.data = out;
  ws_out_pub_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "WS_OUT GOAL_COMPLETE: %s", out.c_str());
}

void RobotApiNode::sendRobotStuck(
  int sequence,
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & reason)
{
  // ambil yaw dari quaternion
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  json j;
  j["code"] = "ROBOT_STUCK";
  j["data"] = {
    {"robot_id", default_robot_id_},
    {"ui_id", default_ui_id_},
    {"sequence", sequence},
    {"reason", reason},
    {
      "position",
      {
        {"x", pose.pose.position.x},
        {"y", pose.pose.position.y},
        {"yaw", yaw}
      }
    }
  };

  auto out = j.dump();
  std_msgs::msg::String msg;
  msg.data = out;
  ws_out_pub_->publish(msg);

  RCLCPP_WARN(this->get_logger(), "WS_OUT ROBOT_STUCK: %s", out.c_str());
}



void RobotApiNode::processNextGoal()
{
  if(sm_.getMode() != RobotMode::MOVING) {
    RCLCPP_INFO(this->get_logger(), 
    "processNextGoal ignored (mode != MOVING)");
    return;
  }

  GoalItem next;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if(goal_in_progress_ || goal_queue_.empty()) {
      return;
    }

    next = goal_queue_.front();
    goal_in_progress_ = true;
  }

  geometry_msgs::msg::PoseStamped current_goal_pose = next.pose;


  // ensure action client exists
  if (!nav_client_) {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  // wait for server (longer timeout)
  if (!nav_client_->wait_for_action_server(5s)) {
    RCLCPP_WARN(this->get_logger(), 
    "navigate_to_pose action server not available (cannot send queued goal) — will retry in %ld seconds", retry_interval_.count());

    if (sm_.getMode() != RobotMode::MOVING){
      std::lock_guard<std::mutex> lock(queue_mutex_);
      goal_in_progress_ = false;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      goal_in_progress_ = false;
    }

    if(!retry_timer_) {
      retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(retry_interval_),
        [this]() {
          if(sm_.getMode() != RobotMode::MOVING){
            retry_timer_->cancel();
            retry_timer_.reset();
            return;
          }

          retry_timer_->cancel();
          retry_timer_.reset();
          if(sm_.getMode() == RobotMode::MOVING){
            this->processNextGoal();
          }
        });
    }
    return;
  }

  // prepare goal
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = next.pose;

  RCLCPP_INFO(this->get_logger(), "Sending queued goal seq=%d", next.sequence);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // goal_response_callback (Humble signature: GoalHandle::SharedPtr)
  send_goal_options.goal_response_callback =
    [this, seq = next.sequence](GoalHandleNavigate::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_WARN(this->get_logger(), "Goal seq=%d was rejected by server", seq);
        // remove this goal and continue with next
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (!goal_queue_.empty()) goal_queue_.pop();
          goal_in_progress_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "Sequence %d Rejected", seq);
        // Tell the UI and leave MOVING; otherwise every later OP is refused
        // with "robot not IDLE".
        sendError("Goal seq=" + std::to_string(seq) +
                  " rejected by Nav2 (navigation not active?)", "NAV_REJECTED", "HIGH");
        bool more;
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          more = !goal_queue_.empty();
        }
        if (more) {
          processNextGoal();
        } else {
          sm_.setMode("IDLE");
        }
        return;
      }

      if(sm_.getMode() != RobotMode::MOVING){
        sm_.setMode("MOVING");
        RCLCPP_INFO(this->get_logger(),
          "STATE MACHINE: MODE = MOVING (goal accepted seq=%d)", seq);
      }

      RCLCPP_INFO(this->get_logger(), "GOAL_ACCEPTED");
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      if (feedback) {
        RCLCPP_DEBUG(this->get_logger(), "Nav2 feedback — dist remaining: %.3f",
                     feedback->distance_remaining);
      }
    };

  send_goal_options.result_callback =
  [this, finished_pose = current_goal_pose](const GoalHandleNavigate::WrappedResult & result) {

    if(result.code == rclcpp_action::ResultCode::CANCELED && 
        sm_.getMode() == RobotMode::INTERRUPTED) {

          RCLCPP_INFO(this->get_logger(), 
          "Result ignored due to INTERRUPT");

          std::lock_guard<std::mutex> lock(queue_mutex_);
          goal_in_progress_ = false;
          return;
    }



    int finished_seq = -1;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!goal_queue_.empty()) {
        finished_seq = goal_queue_.front().sequence;
      }
    }

    std::string result_str;
    bool success = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        result_str = "SUCCEEDED";
        success = true;
        sendLastPoseOnce();
        RCLCPP_INFO(this->get_logger(),
          "Nav2 result: SUCCEEDED (seq=%d)", finished_seq);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        sendError("Nav2 ABORTED (seq=" + std::to_string(finished_seq) + ") — auto-recovering",
          "NAV_ABORTED", "HIGH");
        RCLCPP_WARN(this->get_logger(),
          "Nav2 result: ABORTED (seq=%d) — triggering auto-recovery", finished_seq);
    
        // ── TAMBAHAN: jangan panggil resetToIdle jika sudah dalam proses reset
        if (!is_resetting_) {
          resetToIdle("OP ABORTED seq=" + std::to_string(finished_seq));
        } else {
          RCLCPP_WARN(this->get_logger(),
            "OP ABORTED result ignored — reset already in progress");
          std::lock_guard<std::mutex> lock(queue_mutex_);
          goal_in_progress_ = false;
        }
        return;
      case rclcpp_action::ResultCode::CANCELED:
        if (sm_.getMode() == RobotMode::INTERRUPTED) {
          RCLCPP_INFO(this->get_logger(),
          "Goal canceled due to INTERRUPT (expected)");
          return;
        }
        result_str = "CANCELED";
        break;
      default:
        result_str = "UNKNOWN";
        RCLCPP_WARN(this->get_logger(),
          "Nav2 result: UNKNOWN (seq=%d)", finished_seq);
        break;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      goal_in_progress_ = false;
    }

    // selalu kirim GOAL_RESULT (sukses/gagal)
    RCLCPP_INFO(this->get_logger(), "Sequence %d result : %s", finished_seq, result_str.c_str());

    if (success) {
      if (finished_seq == -1) {
        RCLCPP_INFO(this->get_logger(),"Home reached, switching to IDLE");
        sm_.setMode("IDLE");
        goal_in_progress_ = false;
        return;
      }


      sendGoalComplete(finished_seq);

      sm_.setMode("PAUSED");
      RCLCPP_INFO(this->get_logger(),
      "Waiting for PICKUP CONFIRM (seq=%d)", finished_seq);
      return;
    } 
  };


  // finally, send goal
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void RobotApiNode::sendExpression(const std::string & message)
{
  json j;
  j["code"] = "EXPRESSION";
  j["data"] = {
    {"robot_id", default_robot_id_},
    {"ui_id", default_ui_id_},
    {"expression_type", "EXPRESSION"},
    {"message", message}
  };

  std_msgs::msg::String msg;
  msg.data = j.dump();
  ws_out_pub_->publish(msg);
}


void RobotApiNode::handleRobotStuckInternal(
  int sequence,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // 1. Kirim event ke UI
  sendRobotStuck(sequence, pose, "OBSTACLE_DETECTED");
  sendLastPoseOnce();
 
  // 2. Kirim ekspresi
  sendExpression("confused");
 
  // 3. Reset ke IDLE — biarkan UI yang kirim goal berikutnya
  //    DIHAPUS: jangan auto-retry ke goal yang sama atau langsung pulang
  //    karena itulah yang menyebabkan race condition di resetToIdle
  //
  //    Sebelumnya ada logika:
  //      if (has_pending_goal) sendSingleNavGoal(retry)   ← DIHAPUS
  //      else sendSingleNavGoal(-1, home_base_pose_)      ← DIHAPUS
  //
  //    Pengganti: cukup reset ke IDLE, UI akan terima ROBOT_STUCK event
  //    dan bisa memutuskan apakah retry, ganti tujuan, atau panggil operator
 
  RCLCPP_WARN(this->get_logger(),
    "ROBOT_STUCK (seq=%d) — resetting to IDLE, waiting for operator decision",
    sequence);
 
  resetToIdle("ROBOT_STUCK seq=" + std::to_string(sequence), 1000);
}

// ============================================================
//              BASE64 DECODER (RFC 4648, no padding strict)
// ============================================================
 
static const std::string BASE64_CHARS =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
 
std::vector<uint8_t> RobotApiNode::base64Decode(const std::string & encoded)
{
    std::vector<uint8_t> out;
    out.reserve(encoded.size() * 3 / 4);
 
    std::array<uint8_t, 4> char4;
    std::array<uint8_t, 3> byte3;
    int i = 0;
 
    for (unsigned char c : encoded) {
        if (c == '=') break;
        if (c == '\n' || c == '\r' || c == ' ') continue; // toleransi whitespace
 
        size_t pos = BASE64_CHARS.find(static_cast<char>(c));
        if (pos == std::string::npos) continue; // karakter invalid dilewati
 
        char4[i++] = static_cast<uint8_t>(pos);
 
        if (i == 4) {
            byte3[0] = static_cast<uint8_t>((char4[0] << 2) | (char4[1] >> 4));
            byte3[1] = static_cast<uint8_t>((char4[1] << 4) | (char4[2] >> 2));
            byte3[2] = static_cast<uint8_t>((char4[2] << 6) | char4[3]);
 
            out.insert(out.end(), byte3.begin(), byte3.end());
            i = 0;
        }
    }
 
    // Handle sisa byte (padding tidak lengkap)
    if (i > 0) {
        for (int j = i; j < 4; ++j) char4[j] = 0;
 
        byte3[0] = static_cast<uint8_t>((char4[0] << 2) | (char4[1] >> 4));
        byte3[1] = static_cast<uint8_t>((char4[1] << 4) | (char4[2] >> 2));
        byte3[2] = static_cast<uint8_t>((char4[2] << 6) | char4[3]);
 
        for (int j = 0; j < i - 1; ++j) {
            out.push_back(byte3[j]);
        }
    }
 
    return out;
}
 
// ============================================================
//   UNZIP IN-MEMORY BUFFER → simpan ke map_base_path_/<map_id>/
//   Returns: map name (stem dari file .yaml atau .pgm yang ditemukan)
//   Throws: std::runtime_error jika gagal
// ============================================================
 
std::string RobotApiNode::unzipMapBuffer(
    const std::vector<uint8_t> & zip_bytes,
    int map_id)
{
    // ── 1. Tulis buffer ke file temp ─────────────────────────────────────────
    std::string tmp_zip = "/tmp/map_incoming_" + std::to_string(map_id) + ".zip";
    {
        std::ofstream ofs(tmp_zip, std::ios::binary);
        if (!ofs) {
            throw std::runtime_error("Cannot create temp zip file: " + tmp_zip);
        }
        ofs.write(reinterpret_cast<const char *>(zip_bytes.data()),
                  static_cast<std::streamsize>(zip_bytes.size()));
        ofs.close();  // ← explicit close, pastikan semua byte sudah ke disk
    }
 
    // ── 2. Buat direktori tujuan map_base_path_/map_<id>/ ────────────────────
    std::string out_dir = map_base_path_ + "/map_" + std::to_string(map_id);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::filesystem::remove(tmp_zip);
        throw std::runtime_error(
            "Cannot create output directory: " + out_dir + " — " + ec.message());
    }
 
    // ── 3. Buka zip ──────────────────────────────────────────────────────────
    int zip_err = 0;
    zip_t * archive = zip_open(tmp_zip.c_str(), ZIP_RDONLY, &zip_err);
    if (!archive) {
        zip_error_t zerr;
        zip_error_init_with_code(&zerr, zip_err);
        std::string msg = "zip_open failed: " + std::string(zip_error_strerror(&zerr));
        zip_error_fini(&zerr);
        std::filesystem::remove(tmp_zip);
        throw std::runtime_error(msg);
    }
 
    // ── 4. Extract .pgm dan .yaml, catat isi YAML ────────────────────────────
    std::string yaml_raw_content;   // isi YAML mentah untuk dibaca nama map-nya
    zip_int64_t num_entries = zip_get_num_entries(archive, 0);
 
    for (zip_int64_t idx = 0; idx < num_entries; ++idx) {
        const char * name = zip_get_name(archive, static_cast<zip_uint64_t>(idx), 0);
        if (!name) continue;
 
        std::string entry_name(name);
        std::filesystem::path entry_path(entry_name);
 
        // Skip direktori
        if (entry_name.back() == '/') continue;
 
        std::string ext = entry_path.extension().string();
        if (ext != ".pgm" && ext != ".yaml") {
            RCLCPP_WARN(this->get_logger(),
                "unzipMapBuffer: skip '%s' (bukan .pgm/.yaml)", entry_name.c_str());
            continue;
        }
 
        // Buka entry di dalam zip
        zip_file_t * zf = zip_fopen_index(archive,
                                          static_cast<zip_uint64_t>(idx), 0);
        if (!zf) {
            RCLCPP_ERROR(this->get_logger(),
                "zip_fopen_index failed for '%s'", entry_name.c_str());
            continue;
        }
 
        // Baca seluruh konten entry ke buffer memory dulu
        std::vector<char> file_buf;
        std::array<char, 65536> read_buf;
        zip_int64_t bytes_read = 0;
        while ((bytes_read = zip_fread(zf, read_buf.data(), read_buf.size())) > 0) {
            file_buf.insert(file_buf.end(),
                            read_buf.begin(),
                            read_buf.begin() + bytes_read);
        }
        zip_fclose(zf);
 
        if (file_buf.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "unzipMapBuffer: entry '%s' kosong setelah dibaca", entry_name.c_str());
            continue;
        }
 
        // Simpan isi YAML untuk parsing nama map
        if (ext == ".yaml") {
            yaml_raw_content = std::string(file_buf.begin(), file_buf.end());
        }
 
        // ── Tulis ke disk (nama file SAMA dengan yang ada di zip) ────────────
        // Gunakan hanya filename (tanpa path di dalam zip) untuk keamanan
        std::string out_file_path = out_dir + "/"
                                    + entry_path.filename().string();
 
        {
            std::ofstream ofs(out_file_path, std::ios::binary);
            if (!ofs) {
                RCLCPP_ERROR(this->get_logger(),
                    "Cannot open output file: %s", out_file_path.c_str());
                continue;
            }
            ofs.write(file_buf.data(),
                      static_cast<std::streamsize>(file_buf.size()));
            ofs.close();  // ← explicit close SEBELUM file_size()
        }
 
        // Sekarang file sudah benar-benar di disk, aman untuk file_size()
        auto fsize = std::filesystem::file_size(out_file_path);
        RCLCPP_INFO(this->get_logger(),
            "Extracted: '%s' → '%s' (%.1f KB)",
            entry_name.c_str(),
            out_file_path.c_str(),
            static_cast<double>(fsize) / 1024.0);
    }
 
    zip_close(archive);
    std::filesystem::remove(tmp_zip);  // bersihkan file temp
 
    // ── 5. Baca nama map yang benar dari field "image:" di YAML ──────────────
    // YAML berisi: "image: mapbaruhariini_selasa.pgm"
    // Kita ekstrak stem-nya → "mapbaruhariini_selasa"
    std::string detected_stem;
 
    if (yaml_raw_content.empty()) {
        throw std::runtime_error(
            "YAML tidak berhasil dibaca dari zip untuk map_id=" +
            std::to_string(map_id));
    }
 
    // Cari baris yang dimulai dengan "image:"
    std::istringstream ss(yaml_raw_content);
    std::string line;
    while (std::getline(ss, line)) {
        // Trim whitespace di awal
        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        line = line.substr(start);
 
        if (line.rfind("image:", 0) == 0) {
            // Ambil nilai setelah "image:"
            std::string img_val = line.substr(6);  // 6 = len("image:")
 
            // Trim whitespace
            size_t vs = img_val.find_first_not_of(" \t");
            if (vs != std::string::npos) img_val = img_val.substr(vs);
            size_t ve = img_val.find_last_not_of(" \t\r\n");
            if (ve != std::string::npos) img_val = img_val.substr(0, ve + 1);
 
            // Ambil hanya filename jika berupa path (misal: /some/path/map.pgm)
            std::filesystem::path img_path(img_val);
            detected_stem = img_path.stem().string();
 
            RCLCPP_INFO(this->get_logger(),
                "Map name from YAML image field: '%s' → stem='%s'",
                img_val.c_str(), detected_stem.c_str());
            break;
        }
    }
 
    // Fallback jika field "image:" tidak ditemukan
    if (detected_stem.empty()) {
        RCLCPP_WARN(this->get_logger(),
            "Field 'image:' tidak ditemukan di YAML, "
            "fallback ke nama file .pgm di folder");
 
        // Scan folder untuk cari .pgm
        for (const auto & entry :
             std::filesystem::directory_iterator(out_dir)) {
            if (entry.path().extension() == ".pgm") {
                detected_stem = entry.path().stem().string();
                break;
            }
        }
    }
 
    if (detected_stem.empty()) {
        throw std::runtime_error(
            "Tidak bisa menentukan nama map dari zip map_id=" +
            std::to_string(map_id));
    }
 
    // ── 6. Validasi file hasil extract ───────────────────────────────────────
    // File di zip selalu bernama map.pgm / map.yaml
    // tapi nama asli ada di field image: di YAML
    // Cukup validasi file ada dan tidak kosong
    std::string pgm_path  = out_dir + "/" + "map.pgm";
    std::string yaml_path = out_dir + "/" + "map.yaml";
 
    // Cek dengan nama asli juga (jika server sudah rename sebelum zip)
    if (!std::filesystem::exists(pgm_path)) {
        pgm_path = out_dir + "/" + detected_stem + ".pgm";
    }
    if (!std::filesystem::exists(yaml_path)) {
        yaml_path = out_dir + "/" + detected_stem + ".yaml";
    }
 
    if (!std::filesystem::exists(pgm_path) ||
        std::filesystem::file_size(pgm_path) == 0) {
        throw std::runtime_error(
            "PGM tidak ditemukan atau kosong: " + pgm_path);
    }
    if (!std::filesystem::exists(yaml_path) ||
        std::filesystem::file_size(yaml_path) == 0) {
        throw std::runtime_error(
            "YAML tidak ditemukan atau kosong: " + yaml_path);
    }
 
    // ── 7. Patch YAML: ganti field "image:" ke path absolut ──────────────────
    // map_server perlu path absolut agar bisa load dari direktori manapun
    {
        std::ifstream yaml_in(yaml_path);
        std::string yaml_content((std::istreambuf_iterator<char>(yaml_in)),
                                  std::istreambuf_iterator<char>());
        yaml_in.close();
 
        // Ganti apapun nilai "image:" ke path absolut pgm_path
        // Tangani: "image: map.pgm", "image: ./map.pgm",
        //          "image: mapbaruhariini_selasa.pgm", dll.
        std::string patched;
        std::istringstream pss(yaml_content);
        std::string pline;
        bool patched_image = false;
        while (std::getline(pss, pline)) {
            std::string trimmed = pline;
            size_t ts = trimmed.find_first_not_of(" \t");
            if (ts != std::string::npos && trimmed.substr(ts, 6) == "image:" && !patched_image) {
                patched += "image: " + pgm_path + "\n";
                patched_image = true;
            } else {
                patched += pline + "\n";
            }
        }
 
        std::ofstream yaml_out(yaml_path);
        yaml_out << patched;
        yaml_out.close();
 
        RCLCPP_INFO(this->get_logger(),
            "YAML patched: image → '%s'", pgm_path.c_str());
    }
 
    RCLCPP_INFO(this->get_logger(),
        "unzipMapBuffer OK → stem='%s', dir='%s'",
        detected_stem.c_str(), out_dir.c_str());
 
    return detected_stem;
}
 
// ============================================================
//   MAP_DATA HANDLER
//   Terima zip base64, extract, daftarkan ke registry
//   Payload: { "code": "MAP_DATA", "data": { "robot_id", "map_id",
//              "format": "zip", "encoding": "base64", "payload": "..." } }
// ============================================================
 
void RobotApiNode::handleMapData(const std::string & raw_json)
{
    try {
        json j    = json::parse(raw_json);
        json data = j.at("data");
 
        int map_id = data.at("map_id").get<int>();
 
        // ── Validasi format/encoding ─────────────────────────────────────────
        std::string format   = data.value("format",   "zip");
        std::string encoding = data.value("encoding", "base64");
 
        if (format != "zip") {
            RCLCPP_WARN(this->get_logger(),
                "MAP_DATA: format '%s' tidak didukung (hanya 'zip')",
                format.c_str());
            sendError("Unsupported map format: " + format, "MAP_FORMAT_ERROR", "MEDIUM");
            return;
        }
        if (encoding != "base64") {
            RCLCPP_WARN(this->get_logger(),
                "MAP_DATA: encoding '%s' tidak didukung (hanya 'base64')",
                encoding.c_str());
            sendError("Unsupported encoding: " + encoding, "MAP_FORMAT_ERROR", "MEDIUM");
            return;
        }
 
        std::string b64_payload = data.at("payload").get<std::string>();
 
        RCLCPP_INFO(this->get_logger(),
            "MAP_DATA received: map_id=%d, payload_len=%zu chars",
            map_id, b64_payload.size());
 
        // ── Decode base64 → binary ───────────────────────────────────────────
        std::vector<uint8_t> zip_bytes = base64Decode(b64_payload);
 
        if (zip_bytes.empty()) {
            sendError("base64 decode menghasilkan buffer kosong", "MAP_DATA_ERROR", "HIGH");
            return;
        }
 
        RCLCPP_INFO(this->get_logger(),
            "MAP_DATA: decoded %zu bytes", zip_bytes.size());
 
        // ── Extract zip ──────────────────────────────────────────────────────
        std::string stem = unzipMapBuffer(zip_bytes, map_id);
 
        // ── Daftarkan ke registry ────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(map_registry_mutex_);
            map_registry_[map_id] = stem;
        }
 
        RCLCPP_INFO(this->get_logger(),
            "MAP_DATA: map_id=%d registered as stem='%s'",
            map_id, stem.c_str());
 
        // MAP_DATA tidak perlu ACK wajib, tapi kita kirim INFO ke UI
        // (gunakan sendAck dengan code MAP_DATA)
        sendAck(default_robot_id_, "MAP_DATA", "RECEIVED");

        // Auto-trigger jika ada MAP_SELECTED yang menunggu map ini
        std::string pending_robot_id;
        bool has_pending = false;
        {
            std::lock_guard<std::mutex> plock(pending_map_mutex_);
            auto it = pending_map_selected_.find(map_id);
            if (it != pending_map_selected_.end()) {
                pending_robot_id = it->second;
                pending_map_selected_.erase(it);
                has_pending = true;
            }
        }
        if (has_pending) {
            RCLCPP_INFO(this->get_logger(),
                "MAP_DATA: auto-trigger pending MAP_SELECTED map_id=%d", map_id);
            json sel;
            sel["code"] = "MAP_SELECTED";
            sel["data"] = {{"robot_id", pending_robot_id}, {"map_id", map_id}};
            handleMapSelected(sel.dump());
        }
        
 
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "handleMapData error: %s", e.what());
        sendError(std::string("MAP_DATA error: ") + e.what(), "MAP_DATA_ERROR", "HIGH");
    }
}
 
// ============================================================
//   MAP_SELECTED HANDLER
//   Ganti map aktif Nav2 tanpa restart
//   Payload: { "code": "MAP_SELECTED", "data": { "robot_id",
//              "map_id": 3, "timestamp": "..." } }
// ============================================================
 
void RobotApiNode::handleMapSelected(const std::string & raw_json)
{
    try {
        json j    = json::parse(raw_json);
        json data = j.at("data");

        std::string incoming_robot_id = data.value("robot_id", "");
        int map_id = data.at("map_id").get<int>();

        RCLCPP_INFO(this->get_logger(),
            "MAP_SELECTED: map_id=%d", map_id);

        std::string robot_id_for_ack =
            incoming_robot_id.empty()
            ? default_robot_id_
            : incoming_robot_id;

        // ── Concurrency guard ───────────────────────────────────────────
        if (map_load_in_progress_.load()) {
            RCLCPP_WARN(this->get_logger(),
                "MAP_SELECTED: another LoadMap in progress, rejecting");
            sendError("Map sedang dalam proses loading, coba lagi nanti",
                      "MAP_BUSY", "MEDIUM");
            return;
        }

        // ── State machine guard ─────────────────────────────────────────
        // Cancel active navigation sebelum swap map
        if (sm_.getMode() == RobotMode::MOVING ||
            sm_.getMode() == RobotMode::INTERRUPTED) {
            RCLCPP_INFO(this->get_logger(),
                "MAP_SELECTED: cancelling active navigation before map swap");
            nav_client_->async_cancel_all_goals();
            goal_in_progress_ = false;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!goal_queue_.empty()) goal_queue_.pop();
            }
        }

        // ── Map files on disk? ───────────────────────────────────────────
        // Any previously downloaded map (~/nevo_maps/map_<id>/) loads
        // directly — no registry check, no MAP_DATA round-trip. This keeps
        // repeated switching reliable across restarts and re-selections.
        std::string map_dir = map_base_path_
                              + "/map_" + std::to_string(map_id);

        if (!std::filesystem::exists(map_dir) ||
            !std::filesystem::is_directory(map_dir)) {
            RCLCPP_WARN(this->get_logger(),
                "MAP_SELECTED map_id=%d belum ada di disk (%s), "
                "disimpan ke pending, tunggu MAP_DATA.",
                map_id, map_dir.c_str());
            {
                std::lock_guard<std::mutex> plock(pending_map_mutex_);
                pending_map_selected_[map_id] = robot_id_for_ack;
            }
            sendAck(robot_id_for_ack, "MAP_SELECTED", "PENDING_MAP_DATA");
            return;
        }

        std::string yaml_path;
        std::string pgm_path;

        for (const auto & entry : std::filesystem::directory_iterator(map_dir)) {
            if (entry.path().extension() == ".yaml" && yaml_path.empty()) {
                yaml_path = entry.path().string();
            }
            if (entry.path().extension() == ".pgm" && pgm_path.empty()) {
                pgm_path = entry.path().string();
            }
        }

        // ── Validasi file map ────────────────────────────────────────────
        if (yaml_path.empty() || std::filesystem::file_size(yaml_path) == 0) {
            RCLCPP_ERROR(this->get_logger(),
                "MAP_SELECTED: YAML tidak ditemukan/kosong di: %s",
                map_dir.c_str());
            sendError("YAML tidak ditemukan di: " + map_dir,
                      "MAP_FILE_MISSING", "HIGH");
            return;
        }

        if (pgm_path.empty() || std::filesystem::file_size(pgm_path) == 0) {
            RCLCPP_ERROR(this->get_logger(),
                "MAP_SELECTED: PGM tidak ditemukan/kosong di: %s",
                map_dir.c_str());
            sendError("PGM tidak ditemukan di: " + map_dir,
                      "MAP_FILE_MISSING", "HIGH");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "MAP_SELECTED: files valid, map_id=%d yaml='%s' pgm='%s'",
            map_id, yaml_path.c_str(), pgm_path.c_str());

        // ── Set concurrency guard & send ACK ─────────────────────────────
        map_load_in_progress_.store(true);
        sendAck(robot_id_for_ack, "MAP_SELECTED", "LOADING");

        // ── Panggil Nav2 LoadMap via restart (bypass all DDS issues) ──
        std::string yaml_escaped = yaml_path;
        size_t pos = 0;
        while ((pos = yaml_escaped.find("'", pos)) != std::string::npos) {
            yaml_escaped.replace(pos, 1, "'\\''");
            pos += 4;
        }

        std::string cmd =
            "/home/nevo/restart_map.sh '" + yaml_escaped + "'";

        RCLCPP_INFO(this->get_logger(),
            "MAP_SELECTED: loading map via restart: %s", cmd.c_str());

        int ret = std::system(cmd.c_str());
        RCLCPP_INFO(this->get_logger(),
            "MAP_SELECTED: restart result=%d", ret);

        map_load_in_progress_.store(false);

        if (WIFEXITED(ret) && WEXITSTATUS(ret) == 0) {
            RCLCPP_INFO(this->get_logger(),
                "MAP_SELECTED: map loaded successfully: %s",
                yaml_path.c_str());
            writeSelectedMapConfig(yaml_path);
            active_map_id_ = map_id;  // OP map_id check follows the newly loaded map
            RCLCPP_INFO(this->get_logger(), "Active map_id: %d", active_map_id_);
            sendAck(robot_id_for_ack, "MAP_SELECTED", "SUCCESS");
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "MAP_SELECTED: LoadMap failed ret=%d", ret);
            sendError("Gagal load map",
                      "MAP_LOAD_FAILED", "HIGH");
        }

    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(),
            "handleMapSelected error: %s", e.what());
        sendError(
            std::string("MAP_SELECTED error: ") + e.what(),
            "MAP_ERROR", "HIGH");
    }
}

// Di robot_api_node.cpp — tulis map path ke file temp
void RobotApiNode::writeSelectedMapConfig(const std::string & yaml_path)
{
    const char* home = std::getenv("HOME");
    if (!home) {
        RCLCPP_ERROR(this->get_logger(),
            "writeSelectedMapConfig: HOME env var not set, skipping");
        return;
    }
    std::string config_file = std::string(home) + "/nevo_selected_map.txt";
    std::ofstream ofs(config_file);
    if (!ofs) {
        RCLCPP_ERROR(this->get_logger(),
            "Cannot write map config: %s", config_file.c_str());
        return;
    }
    ofs << yaml_path;
    ofs.close();

    RCLCPP_INFO(this->get_logger(),
        "Selected map written to: %s -> %s",
        config_file.c_str(), yaml_path.c_str());
}

void RobotApiNode::resetToIdle(const std::string & reason, int delay_ms)
{
  // ── GUARD: Jika sudah dalam proses reset, abaikan panggilan kedua ──────────
  // Ini mencegah race condition ketika dua result_callback firing bersamaan
  // (contoh: HOMEBASE ABORTED + retry goal ABORTED dalam ~5ms)
  bool expected = false;
  if (!is_resetting_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(this->get_logger(),
      "resetToIdle called again while already resetting (reason: %s) — ignored",
      reason.c_str());
    return;
  }
 
  RCLCPP_WARN(this->get_logger(),
    "AUTO-RECOVERY: %s — resetting to IDLE in %dms", reason.c_str(), delay_ms);
 
  // 1. Cancel semua goal Nav2 yang mungkin masih jalan
  if (nav_client_) {
    nav_client_->async_cancel_all_goals();
  }
 
  // 2. Bersihkan queue dan state
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!goal_queue_.empty()) goal_queue_.pop();
    goal_in_progress_ = false;
    paused_goal_.reset();
  }
 
  current_move_sequence_ = -1;
  current_move_type_.clear();
 
  // 3. Cancel semua timer aktif
  if (delay_timer_)       { delay_timer_->cancel();       delay_timer_.reset(); }
  if (retry_timer_)       { retry_timer_->cancel();       retry_timer_.reset(); }
  if (home_timer_)        { home_timer_->cancel();        home_timer_.reset(); }
  if (idle_reset_timer_)  { idle_reset_timer_->cancel();  idle_reset_timer_.reset(); }
 
  // 4. Set mode ke IDLE setelah delay
  //    Delay memberi waktu Nav2 untuk settle setelah cancel
  idle_reset_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    [this]() {
      idle_reset_timer_->cancel();
      idle_reset_timer_.reset();
 
      sm_.setMode("IDLE");
      is_resetting_ = false;  // ← clear flag SETELAH IDLE berhasil di-set
 
      RCLCPP_INFO(this->get_logger(),
        "AUTO-RECOVERY complete: state = IDLE");
    }
  );
}

}  // namespace nevo_robot_api

// main.cpp (if you keep main here)
int main(int argc, char ** argv)
{
   rclcpp::init(argc,argv);
   auto node=std::make_shared<nevo_robot_api::RobotApiNode>();
   rclcpp::executors::MultiThreadedExecutor exec;
   exec.add_node(node);
   exec.spin();
   rclcpp::shutdown();
}
