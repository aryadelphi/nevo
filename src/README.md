# NEVO Robot — Runtime Support Components Report

**Date:** 2026-08-15
**Platform:** Ubuntu (ROS 2 Humble), Linux
**Workspace:** `/home/robot2/nevo_ws`
**Stack launcher:** `bash /home/robot2/nevo_launch_robot.sh <nav|mapping>`

---

## 1. Overview

The NEVO robot runtime is composed of five layers:

1. **First-party ROS 2 packages** — Nevo's own code in `nevo_ws/src`
2. **Vendored third-party ROS 2 packages** — bundled into the workspace
3. **ROS 2 ecosystem packages** — installed system-wide (apt), launched at runtime
4. **Non-ROS 2 libraries & tooling** — C++/Python libraries, systemd, tmux, udev, cloud backend
5. **Hardware** — sensors, motors, controllers

The active stack runs inside a **tmux session `nevo_robot`**, launched by a **transient systemd unit** (mode swap mechanism), and reports status to the **forgixrobotic.com cloud backend**.

---

## 2. First-party packages (in `nevo_ws/src`)

| Package | Type | Role in runtime |
|---|---|---|
| `nevo-dev/nevo_bot` | C++/Python/Launch | **Main launcher.** `launch_robot.launch.py` dispatches to `hw_mapping` / `hw_nav`. Contains robot URDF description, all runtime configs (EKF, controllers, docking, tags, exploration), maps, worlds, and autonomy scripts. |
| `nevo_robot_api` | C++ | Core logic node. `robot_api_node` — cloud REST/API, READY handshake, mode control, map upload. `sensor_manager_node` + `battery_sensor_node` — battery monitor over serial. `slam_supervisor_node` — mapping-state watchdog. `position_sensor_node`. |
| `nevo_ws_bridge` | C++ | Cloud WebSocket bridge. `ws_bridge_node` (ROS↔cloud routing, `network_mode` auto/online/offline, 3 s reconnect loop), `ws_client` (WSS client), `ws_server` (local WS server, port 8765), `cloud_upload` (60 s battery/heartbeat upload). |
| `nevo_gsheet_logger_v2` (+ v1) | Python | Telemetry logger to Google Sheets (gspread), 3 write retries then CSV fallback. |
| `nevo_diagnostic_node` | C++ | Health checks (mapping: odometry; nav: amcl) + automatic mode-restart logic. |
| `nevo_msgs` | msg | Custom ROS 2 message definitions. |
| `diffdrive_arduino` | C++ | ros2_control hardware interface to the Arduino motor controller over `/dev/ttyUSB0`. |
| `nav2_mppi_controller` | C++ | MPPI local-planner plugin for Nav2 (Xtensor/xsimd/libomp deps). |
| `serial` | C++ | Serial comms library used by `diffdrive_arduino`. |

**Autonomy/docking scripts inside `nevo_bot`:** `smart_dock.py`, `pure_dock.py`, `full_dock.py`, `tag_pose_bridge.py`, `ina226_bridge_node.py` (current/power), `safety_fail_node`, `map_watcher_node.py`, `exploration_save_node.py`.

---

## 3. Vendored third-party ROS 2 packages

| Package | Origin | Role |
|---|---|---|
| `rplidar_ros` | Slamtec (in `nevo-dev/`) | RPLidar A1 laser driver — `/dev/ttyLIDAR`. |
| `kinect_ros2` | In `nevo-dev/` | Kinect depth/visual camera driver. |
| `rf2o_laser_odometry` | In `nevo-dev/` | Laser-only odometry (falls back when wheel odometry unavailable). |
| `explore_lite` + `multirobot_map_merge` | `m-explore-ros2` (in `nevo-dev/`) | Frontier exploration in mapping mode; multi-map merging. |

---

## 4. ROS 2 ecosystem packages (system-installed, launched at runtime)

**Core/control:**
- `robot_state_publisher`, `tf2_ros` — TF + URDF state
- `controller_manager`, `hardware_interface` — ros2_control (diff drive, joint state broadcaster)
- `twist_mux` — cmd_vel multiplexing (teleop/docking/nav priorities)
- `laser_filters` — lidar scan filtering
- `image_proc`, `v4l2_camera` — camera pipeline

**Navigation (Nav2) — `hw_nav`:**
- `nav2_amcl`, `nav2_map_server` — localization + map serving
- `nav2_controller`, `nav2_smoother`, `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`, `nav2_velocity_smoother`, `nav2_waypoint_follower`
- `nav2_lifecycle_manager` — lifecycle coordination
- `robot_localization` — EKF fusion

**Mapping — `hw_mapping`:**
- `slam_toolbox` — online_async SLAM (the default mapper)
- `robot_localization` — EKF

**Docking suite (used across modes):**
- `opennav_docking` — docking controller
- `apriltag_ros` — AprilTag detection for docking/localization

---

## 5. Non-ROS 2 libraries & tooling

### C/C++ libraries
- **ixwebsocket** — vendored static library (`libixwebsocket.a`) inside `nevo_ws_bridge`; WebSocket client/server engine
- **OpenSSL** (`libssl`/`libcrypto`) — TLS for `wss://`
- **Xtensor, xsimd, libomp** — MPPI controller numerics
- **Eigen, OpenCV** — transforms/vision (AprilTags, camera)

### Python (pip, system Python)
- **gspread** (6.2.1) + **google-auth** (2.56.2) + google-auth-oauthlib — Google Sheets API for logger
- **requests** — HTTP
- **websockets** (16.1.1) — WS utilities/scripts

### System infrastructure
- **systemd services:**
  - `nevo.service` — Navigation stack (`nevo_launch_robot.sh nav`)
  - `nevo-mapping.service` — Mapping stack (`nevo_launch_robot.sh mapping`)
  - transient `nevo-restart-<mode>.service` — mode-swap mechanism (`KillMode=process`, escapes cgroup via `systemd-run`)
- **sudoers rule** `/etc/sudoers.d/nevo-robot` — NOPASSWD `/usr/bin/systemd-run` for robot2
- **tmux** — session `nevo_robot` hosts the running stack (survives unit exit)
- **udev rules** — `99-usb-serial.rules`, `99-serial-ports.rules` create stable symlink `/dev/ttyLIDAR` for the lidar
- **Shell scripts** — `/home/robot2/nevo_launch_robot.sh` (launcher), `/home/robot2/nevo_restart.sh <mode>` (swap), `/home/robot2/nevo_restart.log`

### Cloud backend (forgixrobotic.com)
- `tifa-ws.forgixrobotic.com` — WebSocket endpoint (`wss://`, port 443)
- `tifa-be.forgixrobotic.com` — REST API (map upload `/api/maps/upload/full`, robot API)

---

## 6. Hardware

| Component | Interface | Driven by |
|---|---|---|
| Differential drive motors | Arduino over serial `/dev/ttyUSB0` | `diffdrive_arduino` (ros2_control) |
| RP Lidar A1 | USB serial `/dev/ttyLIDAR` | `rplidar_ros` |
| Kinect / RGB-D camera | USB | `kinect_ros2`, `image_proc` |
| Battery/power monitor (INA226) | serial | `ina226_bridge_node.py`, `sensor_manager_node` |
| Position/slip sensor | serial | `position_sensor_node` |
| AprilTag markers | camera vision | `apriltag_ros`, `tag_pose_bridge.py` |

---

## 7. Runtime node topology (active stack)

**Navigation (`nav`):** rsp → rplidar_activate → ws_bridge → diagnostic_node → laser_filter → localization (amcl, map_server, ekf) → navigation (controller/planner/behavior/bt_navigator/smoother/velocity_smoother/waypoint_follower) → publish_initial_pose → safety_fail_node → robot_api → sensor_manager → gsheet logger

**Mapping (`mapping`):** rsp → rplidar_activate → ws_bridge → diagnostic_node → laser_filter → controller_manager + diff_cont/joint_state_broadcaster → ekf → twist_mux → slam_toolbox → explore_lite → safety_fail_node → robot_api → sensor_manager → slam_supervisor → position_sensor_node

---

## 8. Key configuration points

- `nevo_bot/config/` — `mapper_params_online_async.yaml` (SLAM), `ekf.yaml`, `twist_mux.yaml`, `my_controllers.yaml` (ros2_control), `docking.yaml`, `tags.yaml`, `controller_mppi_params.yaml`
- `nevo_ws_bridge/config/ws_bridge_params.yaml` — reconnect interval (3.0 s), robot_id, ws URIs
- `nevo_robot_api` launch/params — robot_id (`TFRB2`), ui_id, cloud API URL
- systemd unit files — `/etc/systemd/system/nevo.service`, `nevo-mapping.service`
