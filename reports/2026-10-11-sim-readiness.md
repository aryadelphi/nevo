# 2026-10-11 — Making the NEVO workspace simulation-ready

**Goal:** get the NEVO Gazebo simulation running at full speed inside the
`ubuntu24` distrobox on the MSI GF63, trim the workspace to the thesis scope
(STVL 3D perception + MPPI vs DWB on the NEVO serving robot), fix `sim_nav`, and
put the workspace under git.

**Result:** the sim runs at **RTF 0.99–1.00** (it was ~0.28), teleop is responsive, and
the workspace is a git checkout on branch `thesis-sim` with nothing committed yet.

---

## 1. Simulation performance

### Measurements (headless gz server, `turtlebot3_house`, NEVO robot)

| Step | RTF | gz server CPU | Note |
|---|---|---|---|
| Original (Intel iGPU renders gz) | ~0.28 | — | GUI run |
| + NVIDIA PRIME offload | ~0.60 | ~150% | GUI or headless, same |
| + static furniture | **~1.00** | **~36%** | physics no longer saturated |

Sensor bisect (PRIME on, original world). Each sensor costs only about 0.02 RTF:

| Loaded | RTF |
|---|---|
| world only (no robot) | 0.77 |
| + robot body | 0.75 |
| + lidar | 0.73 |
| + Kinect (current robot) | 0.71 |
| + C310 (old robot) | 0.68 |

World variants (current robot):

| Variant | RTF | CPU |
|---|---|---|
| base | 0.67 | ~102% |
| 4 ms step @ 250 Hz | 0.96 | ~96% |
| all furniture static | 1.00 | ~36% |
| static + 4 ms step | 1.00 | ~22% |

### Root causes

1. **Optimus laptop: gz rendered on the Intel UHD 630.** The distrobox was created
   with `--nvidia`, but nothing selected the GTX 1650. All three robot sensors are
   GPU-rendered in gz-sim (`gpu_lidar`, `depth_camera`, `camera`), and the physics
   step waits for each sensor render.
2. **Dynamic furniture in `turtlebot3_house`.** `Mailbox`, `first_2015_trash_can`
   and `first_2015_trash_can_0` had no `<static>` tag (and collide as triangle
   meshes), and `table` had `<static>0</static>`. DART solved their mesh–floor
   contacts every 2 ms, pinning the single-threaded physics loop at about one full core.

Side note: the world's `<physics type="ode">` / `<ode><solver>` block is ignored by
gz Harmonic, which always uses DART. Only `max_step_size` and
`real_time_update_rate` take effect.

### Fixes applied

| Fix | File |
|---|---|
| `nvidia_prime_env()`: sets `__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `__EGL_VENDOR_LIBRARY_FILENAMES=…/10_nvidia.json`. Skipped if the NVIDIA vendor file is absent; your own exports still win | `nevo_bot/launch/launch_sim.launch.py` |
| `<static>1</static>` on Mailbox, both trash cans, table | `nevo_bot/models/turtlebot3_house/model.sdf` |

Physics step unchanged (2 ms). Both fixes were reverted temporarily for the
"before" documentation and then restored. Copies and restore steps are in
`~/nevo_optimizations_2026-10-11/`.

**Lesson for the Tel-U café world:** make all furniture static; only people should
be dynamic. Otherwise RTF varies between trials and time-to-goal is not comparable.

---

## 2. Workspace trimmed to thesis scope

Moved (not deleted) to `~/nevo_archive_2026-10-11/`, keeping the same relative paths;
`MANIFEST.txt` lists all 91 items.

| Group | Items |
|---|---|
| amcl_health | `src/amcl_health`, its sim README, smoke-test and cleanup scripts, build/install dirs |
| table detection | `src/table_detection` |
| docs/clutter | `src/report`, `src/scripts`, the 43 MB source zip, `controller_mppi_params_backup/_test.yaml`, non-thesis maps |
| docking | dock + AprilTag scripts, `archive_tests/`, `docking_sim` / `docking_suite` / `apriltag_*` / `tag_pose_bridge` / `nav_test` launches, `docking.yaml`, `docking_sim.yaml`, `tags.yaml`, `docking_station.world`, tag textures, `c310_camera.xacro` |

Kept: `nevo_bot` nav/sim/mapping, the cloud layer (ws_bridge, robot_api, diagnostic,
msgs, gsheet, runtime), and the hardware stack (diffdrive_arduino, serial, rplidar,
kinect, rf2o). Maps kept: `telcoff_map` and `telkop_save_map` (the same ~15.8 m café
space), `turtlebot3_save_map` (sim default), `empty_test_map`, `abie_lab_save`
(robot default).

Edits so nothing references archived files:

| File | Change |
|---|---|
| `nevo_bot/CMakeLists.txt` | dock scripts removed from `install(PROGRAMS)` |
| `nevo_bot/description/robot.urdf.xacro` | C310 include removed (sim now has 2 GPU sensors) |
| `nevo_bot/launch/launch_sim.launch.py` | `/c310/*` bridge topics removed |
| `nevo_bot/launch/launch_robot.launch.py`, `hw_baseline.launch.py` | rear camera, rectify, AprilTag, `tag_pose_bridge`, `docking_server` nodes removed; Kinect kept |
| `nevo_bot/config/controller_mppi_params.yaml` | `docking_server` removed from navigation lifecycle `node_names` (would hang bringup) |
| `nevo_bot/config/controller_dwb_params.yaml` | unused `docking_server:` block removed |
| `nevo_bot/config/my_sim_controllers.yaml` | stale comment |

**Do not deploy `launch_robot.launch.py` or `controller_mppi_params.yaml` from this
branch to the robot**: the robot's `~/nevo_ws` still uses docking.

---

## 3. `sim_nav` bringup fixed

### Symptom
```
[bt_navigator]: Couldn't open input XML file: /home/nevo/nevo_ws/install/nevo_bot/share/nevo_bot/config/navigate_to_pose_w_relocalization.xml
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

### Causes and fixes

| Issue | Fix | Files |
|---|---|---|
| The BT XML path was hardcoded to the robot's `/home/nevo` install, so `bt_navigator` failed and aborted the navigation bringup (pre-existing; AMCL was unaffected) | All planners use Nav2's stock trees via `$(find-pkg-share nav2_bt_navigator)/behavior_trees/navigate_{to_pose,through_poses}_w_replanning_and_recovery.xml` | `stvl_sim_params.yaml`, `controller_mppi_params.yaml`, `controller_dwb_params.yaml`, `controller_graceful_params.yaml` |
| MPPI used a custom BT with `ReinitializeGlobalLocalization` while DWB/Graceful used the stock BT, so the comparison ran under unequal recovery behavior | Same as above: identical BT for every planner | same |
| Two initial-pose sources: `sim_nav`'s `publish_initial_pose` (lost, AMCL not up yet) and the lifecycle activator's (0, 0, 0) **re-published every 5 s forever**, snapping AMCL back to the origin | Activator now publishes the pose until `map→odom` appears (max 24 tries), then stops. New launch args `initial_pose_x/y/yaw` (default 0 = robot behavior unchanged). `sim_nav` drops its own node and passes (0.691, −0.788, −0.418) as overridable args | `localization_launch.py`, `sim_nav.launch.py` |

Backups of the pre-change files are in `~/nevo_backup_2026-10-11/`.

---

## 4. Git setup

- GitHub's `nevo` branch lives in the `diagonalrobotics/tifa` repo **by mistake**:
  NEVO and TIFA are different projects.
- GitHub `nevo` (last commit `ba518e1`, 2026-09-14) is an **older subset** of the
  local workspace. Missing on GitHub: the 2026-09-28 Gazebo Classic → Harmonic
  migration, `collision_monitor` params, diagnostic `enable_stack_lock`, the ESP8266
  IMU sketch, 9 report files, and all work since 2026-10-05.
- Fresh clone at `~/work/diagonal_robotics/nevo_git`, local branch `thesis-sim`
  (from `nevo`), with the trimmed `src/` mirrored in. **Nothing committed or pushed.**
- Sim build set (the sim launches look up the cloud packages unconditionally):
  ```bash
  colcon build --packages-select nevo_msgs nevo_ws_bridge nevo_diagnostic_node nevo_robot_api explore_lite_msgs explore_lite nevo_bot
  ```

---

## 5. How to verify

```bash
cd ~/work/diagonal_robotics/nevo_git && source /opt/ros/jazzy/setup.zsh && source install/setup.zsh
ros2 launch nevo_bot launch_sim.launch.py world:=turtlebot3_house    # T1
gz topic -e -t /stats | grep real_time_factor                         # T2: ~1.0
ros2 launch nevo_bot sim_nav.launch.py                                # T3
```
In T3 look for `map->odom TF confirmed after publish #1` and `Managed nodes are active`.

---

## 6. Open items

- [ ] **Run `sim_nav`** to confirm the BT and initial-pose fixes (not yet run).
- [ ] **Executable bits lost (10 files):** the scripts below show only
      `mode change 100755 => 100644` in git, most likely from the Google Drive zip.
      Committing them as-is would make them non-executable. Restore before committing:
      ```bash
      cd ~/work/diagonal_robotics/nevo_git
      git diff --summary | grep -oP 'mode change 100755 => 100644 \K.*' | xargs chmod +x
      ```
      Affected: `nevo_bot/scripts/{exploration_save_node,imu_publisher,ina226_bridge_node,map_watcher_node}.py`,
      `rplidar_ros/scripts/{create,delete}_udev_rules.sh`,
      `runtime/scripts/{nevo_launch_robot,nevo_restart,restart_map}.sh`,
      `serial/src/{serial.cc,impl/unix.cc}`.
- [ ] Check that the sim initial pose (0.691, −0.788, −0.418) matches the spawn
      point on `turtlebot3_save_map`.
- [ ] Optional: look up the cloud packages in `launch_sim` / `sim_nav` only when
      `nevo_extras:=true`, so a sim-only build needs just `nevo_bot`.
- [ ] Commit `thesis-sim` (include the untracked `models/` and the two turtlebot3
      worlds); decide where to push it (a dedicated NEVO repo is cleaner than the
      `tifa` repo).
- [ ] Build the Tel-U café world (static furniture, dynamic people) for MPPI vs DWB.
- [ ] Leftovers: `config_nevo.rviz` still references C310 frames and
      `/rear_camera/image_rect` (harmless); `hw_baseline` starts the Kinect a second
      time on top of `launch_robot` (pre-existing).
