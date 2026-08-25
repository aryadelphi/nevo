# NEVO Session Report — 2026-08-24

## Summary

Major session covering: DDS migration to FastDDS, workspace rename (tifa→nevo, docking_ws→nevo_ws), SLAM localization fixes, mode swap reliability, rplidar self-healing, and diagnostic timeout tuning.

---

## Changes Made

### 1. DDS: CycloneDDS → FastDDS

**Problem:** CycloneDDS participant-index exhaustion caused total DDS failure after repeated mode swaps.

**Fix:**
- `nevo.service` env: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE=file:///home/rspx1/fastdds.xml`
- `nevo_restart.sh` and `launch_robot.launch.py` updated with FastDDS env vars
- New `fastdds.xml` config: DNS v2 disabled, 10MB socket buffers
- All launch files use `SetEnvironmentVariable` for FastDDS

**Impact:** DDS no longer crashes. Reliable topic discovery across reboots.

### 2. Cloud Domain Reverted to tifa-*

**Problem:** `nevo-ws.forgixrobotic.com` and `nevo-be.forgixrobotic.com` DNS records don't exist. Cloud backend still uses original tifa-* domains.

**Fix:** All cloud endpoint references reverted:
- `nevo-ws.forgixrobotic.com` → `tifa-ws.forgixrobotic.com`
- `nevo-be.forgixrobotic.com` → `tifa-be.forgixrobotic.com`
- Affected: 8 launch files, `robot_api_node.cpp`, `INSTALL.md`, `README.md`

### 3. SLAM Localization Fixes

**Problem 1:** SLAM dropping all scans ("queue full" / "Failed to compute odom pose")

**Root cause:** `enable_odom_tf: false` in diff_drive_controller meant no `odom → base_link` TF. EKF config used `base_link_frame: base_footprint` which created a split TF tree.

**Fix:**
- `enable_odom_tf: false` kept (EKF handles odom TF)
- EKF publishes `odom → base_footprint` at 30Hz
- RSP publishes `base_footprint → base_link → chassis → laser_frame`
- SLAM `base_frame: base_footprint` (matching EKF output)

**Problem 2:** `diff_cont` stuck in `unconfigured` state

**Root cause:** Timing race on cold boot — spawner starts before hardware is ready.

**Fix:** `diff_cont` now auto-configures and activates on launch (verified working).

### 4. Mode Swap Reliability

**Problem:** rplidar_node crashes with exit code 255 on every mode swap (mapping↔nav).

**Root cause:** `nevo_restart.sh` kills rplidar with SIGKILL, which doesn't release the serial port cleanly. New rplidar instance can't open the port.

**Fixes in `nevo_restart.sh`:**
- Added 2s grace period between hard-kill and serial reset
- Increased serial reset sleep from 1s to 3s
- Total teardown time increased for cleaner port release

**Fix in `rplidar_a1_launch.py`:**
- Added `respawn=True, respawn_delay=3.0` — rplidar auto-restarts if it crashes
- Added FastDDS env vars (`SetEnvironmentVariable`) so respawned node uses correct DDS
- Hardcoded `ROS_DOMAIN_ID=7` in launch to survive respawn

### 5. Diagnostic Node Timeout

**Problem:** On cold boot after reboot, the diagnostic node's 120s timeout was too tight. AMCL activates late (lifecycle activator waits 15s + map_server + AMCL setup), causing a restart loop.

**Fix in `launch_robot.launch.py`:**
- `grace_period_sec`: 10 → 30 (longer ignore window during boot)
- `global_timeout_sec`: 120 → 300 (enough time for cold boot initialization)

### 6. USB Serial Port Recovery

**Problem:** Loose CP2102 USB-to-UART adapter causes random USB disconnects, changing ttyUSB number.

**Current behavior:**
- udev symlink `/dev/ttyLIDAR` auto-updates to new ttyUSB
- rplidar `respawn=True` restarts the node
- Manual recovery if respawn fails: `sudo kill -9 <rplidar_pid>` triggers respawn

**Note:** This is a hardware issue — the CP2102 module needs physical re-seating or securing.

### 7. URDF Mesh Files (Cosmetic)

**Problem:** rviz2 on dev machine can't load robot STL meshes (body, wheels) because the `install` directory isn't copied to the dev machine.

**Note:** Purely cosmetic. Only affects robot model visualization in rviz. URDF primitive shapes (camera, lidar) still render correctly.

---

## Configuration Changes Summary

| File | Parameter | Old | New |
|------|-----------|-----|-----|
| `my_controllers.yaml` | `enable_odom_tf` | false | false (kept) |
| `mapper_params_online_async.yaml` | `base_frame` | base_footprint | base_footprint (kept) |
| `ekf.yaml` | `base_link_frame` | base_footprint | base_footprint (kept) |
| `rplidar_a1_launch.py` | `respawn` | — | true |
| `rplidar_a1_launch.py` | `respawn_delay` | — | 3.0 |
| `launch_robot.launch.py` | `grace_period_sec` | 10 | 30 |
| `launch_robot.launch.py` | `global_timeout_sec` | 120 | 300 |
| `nevo_restart.sh` | hard-kill grace | 0s | 2s |
| `nevo_restart.sh` | post-serial-reset sleep | 1s | 3s |

---

## Verified Working

- [x] FastDDS: DDS stable across mode swaps
- [x] Cloud connectivity: READY messages sent, MAPPING_START/STOP received
- [x] Navigation mode: all lifecycle nodes active, AMCL publishing
- [x] Mapping mode: SLAM building map, coverage reporting
- [x] Mode swap nav→mapping: works
- [x] Mode swap mapping→nav: works
- [x] rplidar respawn: tested on USB disconnect
- [x] TF chain: complete in both nav and mapping modes
- [x] Cold boot: survives without restart loop (300s timeout)

## Known Issues

- **Loose CP2102 adapter:** Hardware issue causing intermittent USB disconnects
- **Mesh files on dev machine:** Not deployed — cosmetic only
- **EKF "Failed to meet update rate":** Non-fatal warning, Pi CPU load during startup
- **Cloud endpoints:** Still using `tifa-*` domains — awaiting DNS record creation for `nevo-*`
