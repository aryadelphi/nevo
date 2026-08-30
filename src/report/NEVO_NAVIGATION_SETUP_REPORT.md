# NEVO Robot — ROS2 Jazzy Navigation Stack Setup Report

**Date:** 2026-08-30
**Platform:** Raspberry Pi 5, Ubuntu 24.04, ROS2 Jazzy
**Workspaces:** `/home/nevo/nevo/` (copy) and `/home/nevo/nevo_ws/` (active)
**Build ID:** `ament_2026-08-30-14-11-32`

---

## 1. Hardware Configuration

| Component | Device | Path |
|-----------|--------|------|
| Lidar | RPLidar A1 (CP2102) | `/dev/ttyLIDAR` → `/dev/ttyUSB0` |
| Motor | Arduino (CH340) | `/dev/ttyMOTOR` → `/dev/ttyUSB1` |
| Battery | INA226 | **Skipped** (broken module) |
| Camera | Rear camera | `/dev/video0` |
| SBC | Raspberry Pi 5 | aarch64, Ubuntu 24.04 |

**Network:** `192.168.12.195` via `wlan0`
**DDS:** `rmw_fastrtps_cpp` with FastDDS config at `~/fastdds.xml`
**Domain ID:** `ROS_DOMAIN_ID=7`

---

## 2. Issues Found and Fixed

### 2.1 Shell Environment (`.zshrc`)
- **Problem:** SSH sessions had no ROS2 environment variables.
- **Fix:** Added complete ROS2 Jazzy + workspace sourcing + env vars to `~/.zshrc`.

### 2.2 Udev Rules
- **Problem:** Serial device symlinks not persisting across reboots.
- **Fix:** Created `/etc/udev/rules.d/99-nevo.rules`:
  - `/dev/ttyLIDAR` → RPLidar A1 via serial number `0001`
  - `/dev/ttyMOTOR` → CH340 motor Arduino via physical port path

### 2.3 Sudoers for systemd-run
- **Problem:** `robot_api_node` couldn't restart services without password.
- **Fix:** Created `/etc/sudoers.d/nevo-robot`:
  ```
  nevo ALL=(root) NOPASSWD: /usr/bin/systemd-run
  ```

### 2.4 Hardcoded Paths (`rspx1` → `nevo`)
- **Problem:** ~15 source files had hardcoded paths to `/home/rspx1/nevo_ws/`.
- **Fix:** Replaced all with `/home/nevo/nevo_ws/` across the codebase.

### 2.5 Diagnostic Node `/map` QoS
- **Problem:** `nevo_diagnostic_node` couldn't receive `/map` due to QoS mismatch.
- **Fix:** Changed QoS to `reliable().transient_local()` — confirmed `map:OK`.

### 2.6 AMCL Stationary Updates
- **Problem:** AMCL didn't update pose when robot was stationary.
- **Fix:** Set `update_min_d: 0.0` and `update_min_a: 0.0` in `controller_mppi_params.yaml`.

### 2.7 Navigation Timeout Tuning
- **Changes:**
  - `bt_navigator.default_server_timeout`: `3000` ms (was default)
  - `planner_server.costmap_update_timeout`: `5.0` s
  - AMCL `transform_tolerance`: `1.0`

### 2.8 2D Goal Pose — Stale Goal TF Extrapolation (ROOT CAUSE FIX)
- **Problem:** Nav2 planner_server uses `input_pose.header.stamp` (from user's goal click in RViz) for TF lookups via `tf_buffer.transform()`. If the goal is created before AMCL starts broadcasting `map → odom`, the TF lookup permanently fails with "Extrapolation Error". The BT tree retries the same stale goal indefinitely.
- **Fix (localization_launch.py):**
  1. **Background initialpose publisher:** Changed from 5-time burst to continuous timer (every 5s) — keeps AMCL's `map → odom` TF fresh.
  2. **TF confirmation wait:** After lifecycle activation, the activator now waits (up to 120s) for `map → odom` to actually appear on the TF tree before declaring done — prevents race condition where goals arrive before AMCL's TF is available.

### 2.9 MPPI Controller `transform_tolerance` Too Low
- **Problem:** MPPI controller's `transform_tolerance: 0.1` caused "Extrapolation Error" when goal timestamp was 140ms ahead of latest TF.
- **Fix:** Changed `transform_tolerance` from `0.1` to `1.0` in `controller_mppi_params.yaml` (FollowPath section).

### 2.10 RPi5 Performance Overload
- **Problem:** EKF at 30Hz and controller at 10Hz overwhelmed the Pi5, causing constant "Failed to meet update rate" errors and control loop misses.
- **Fix:**
  - EKF `frequency`: `30.0` → `15.0` Hz
  - `controller_server.controller_frequency`: `10.0` → `5.0` Hz
  - MPPI `model_dt`: `0.1` → `0.2` (must match controller period ≤ model_dt, otherwise FATAL crash)

### 2.11 `joint_state_broadcaster` FATAL — Missing Wheel TFs
- **Problem:** `joint_state_broadcaster` and `diff_cont` spawned simultaneously on `OnProcessStart`. Both tried to claim the same state interfaces → `joint_state_broadcaster` timed out on `load_controller` (10s) → FATAL → no `/joint_states` → `robot_state_publisher` couldn't compute `left_wheel` and `right_wheel` TFs.
- **Fix (launch_robot.launch.py):**
  1. Spawn `joint_state_broadcaster` FIRST in `OnProcessStart` callback (claims state interfaces).
  2. Spawn `diff_cont` via `TimerAction` 10 seconds AFTER controller_manager starts (claims command interfaces after state interfaces are claimed).

### 2.12 Mapping Mode — SLAM Toolbox Not Building Map
- **Problem:** SLAM Toolbox dropped every scan with "Message Filter dropping message: queue is full". Root cause: `diff_cont` never got configured/activated because the `delayed_diff_drive` TimerAction (5s) fired at the same time as `delayed_controller_manager` (5s), racing with the controller_manager startup. Without diff_cont active, EKF had no odom input → no `odom → base_footprint` TF → SLAM couldn't synchronize scans with TF.
- **Fix (launch_robot.launch.py):** Changed `delayed_diff_drive` period from `5.0` to `10.0` seconds to avoid the race condition.

---

## 3. Default Map Change
- **Changed from:** `telkop_save_map.yaml`
- **Changed to:** `abie_lab_save.yaml` (current location)
- **Files modified:**
  - `launch_robot.launch.py` — `default_map`
  - `hw_nav.launch.py` — `default_map`

---

## 4. Configuration Files Modified

| File | Changes |
|------|---------|
| `~/.zshrc` | Added ROS2 Jazzy env + workspace sourcing |
| `/etc/udev/rules.d/99-nevo.rules` | Serial device symlinks |
| `/etc/sudoers.d/nevo-robot` | Passwordless `systemd-run` |
| `config/controller_mppi_params.yaml` | AMCL (update_min_d/a=0, transform_tolerance=1.0), bt_navigator (timeout=3000), controller_server (freq=5.0, transform_tolerance=2.0), MPPI (model_dt=0.2, transform_tolerance=1.0) |
| `config/ekf.yaml` | frequency: 15.0 |
| `launch/launch_robot.launch.py` | Map path, spawner ordering (JSB first, diff_cont 10s later), removed redundant variable |
| `launch/hw_nav.launch.py` | Map path |
| `launch/localization_launch.py` | Background initialpose publisher, TF confirmation wait |

---

## 5. Final TF Tree

```
map → odom (SLAM/AMCL)
  └─→ odom → base_footprint (EKF, 15Hz)
        └─→ base_footprint → base_link (RSP, static)
              └─→ base_link → chassis (RSP, static)
              │     ├─→ chassis → laser_frame (RSP, static)
              │     ├─→ chassis → camera_link (RSP, static)
              │     ├─→ chassis → c310_camera_link (RSP, static)
              │     ├─→ chassis → rear_caster_wheel (RSP, static)
              │     └─→ chassis → back_caster_wheel (RSP, static)
              ├─→ base_link → left_wheel (joint_states, ~15Hz)
              └─→ base_link → right_wheel (joint_states, ~15Hz)
```

---

## 6. Navigation Stack Status

| Component | Status |
|-----------|--------|
| map_server | OK |
| AMCL | OK |
| EKF | OK (15 Hz) |
| controller_server | OK (5 Hz, MPPI) |
| planner_server | OK (NavFn) |
| behavior_server | OK |
| bt_navigator | OK (custom BT with relocalization) |
| smoother_server | OK |
| waypoint_follower | OK |
| velocity_smoother | OK |
| diff_drive_controller | OK (5 Hz) |
| joint_state_broadcaster | OK |
| SLAM Toolbox | OK (mapping mode) |
| RPLidar A1 | OK (~8 Hz) |
| Robot API | OK (cloud WS bridge) |

---

## 7. Systemd Services

| Service | Status | Description |
|---------|--------|-------------|
| `nevo.service` | Enabled | Nav mode (default) |
| `nevo-mapping.service` | Disabled | Mapping mode |

**Mode switching:** Done via external UI → `robot_api_node` → `nevo_restart.sh` → tmux session.

---

## 8. Known Remaining Issues

1. **Power supply warning:** Pi5 reports "Reset due to low power" — use official 5V/5A PSU.
2. **EKF occasional rate errors:** EKF still occasionally fails to meet 15Hz target (~30-190ms spikes). Could reduce to 10Hz if needed.
3. **controller_manager overruns:** ros2_control loop (50Hz) occasionally overruns on Pi5 (20-40ms cycles). Non-critical but worth monitoring.
4. **FastDDS SHM port errors:** `Failed init_port fastrtps_port7002/7003` — cosmetic, doesn't affect functionality.
5. **`.setup.sh` sourcing warning:** ROS2 `setup.bash` tries to source nonexistent `/home/nevo/setup.sh` — cosmetic only.

---

## 9. Restart History

| Time | Event |
|------|-------|
| ~17:16 | Initial goal test after AMCL fix — TF extrapolation fixed but planner timeout |
| ~17:20 | Applied MPPI transform_tolerance + EKF/controller rate fixes — controller FATAL (model_dt conflict) |
| ~17:23 | Fixed model_dt (0.1→0.2) — controller_server FATAL from joint_state_broadcaster conflict |
| ~17:33 | Fixed spawner ordering (JSB first, diff_cont delayed) — all working |
| ~17:42 | UI swap to mapping mode — diff_cont race condition, SLAM dropping scans |
| ~17:55 | Manual diff_cont activation + fixed delayed_diff_drive timer (5→10s) — SLAM working |

---

## 10. Commands Reference

### Source environment for ros2 CLI:
```bash
export AMENT_PREFIX_PATH=/opt/ros/jazzy:/home/nevo/nevo_ws/install
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/nevo/nevo_ws/install/lib/python3.12/site-packages
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/home/nevo/nevo_ws/install/lib
export PATH=/opt/ros/jazzy/bin:$PATH
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Build:
```bash
cd /home/nevo/nevo_ws && colcon build --packages-select nevo_bot
```

### Check services:
```bash
sudo systemctl status nevo.service
tmux ls
tmux capture-pane -t nevo_robot -p | tail -40
```
