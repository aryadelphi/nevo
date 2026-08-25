# NEVO Session Report — 2026-08-25 (Evening)

## Summary

MPPI controller tuning for smoother navigation, planner exploration (SMAC Hybrid attempted, reverted to NavFn), obstacle avoidance softening, velocity slew tuning, lifecycle reliability fixes, and auto-initial-pose on map swap.

---

## Changes Made

### 1. SMAC Planner Hybrid Attempt (Reverted)

**Goal:** Switch from NavFn to SmacPlannerHybrid with Reeds-Shepp curves for kinematically feasible paths.

**Config applied:**
```yaml
plugin: "nav2_smac_planner::SmacPlannerHybrid"
motion_model_for_search: "REEDS_SHEPP"
minimum_turning_radius: 0.25
angle_quantization_bins: 72
cache_obstacle_heuristic: true
smooth_path: true
```

**Result:** Planner server hung during activation — couldn't acknowledge `compute_path_to_pose` goals within timeout. Reeds-Shepp + 72 angle bins too computationally heavy for the hardware. **Reverted to NavFnPlanner.**

### 2. MPPI Controller Tuning (Swaying Reduction)

**Problem:** Robot swayed excessively during navigation.

**Changes applied iteratively:**

| Parameter | Original | Final | Effect |
|-----------|----------|-------|--------|
| `vx_std` | 0.4 | 0.2 | Less velocity noise |
| `wz_std` | 0.8 | 0.4 | Less angular noise |
| `temperature` | 0.3 | 0.5 | Less stochastic, more deterministic |
| `batch_size` | 500 | 800 | More trajectory samples |
| `iteration_count` | 3 | 4 | Better convergence |
| `PathAlignCritic` weight | 14.0 | 20.0 | Stronger path alignment |
| `PathFollowCritic` weight | 5.0 | 8.0 | Stronger path following |
| `PathAngleCritic` weight | 2.0 | 4.0 | Stronger heading control |

### 3. Obstacle Avoidance Softening

**Problem:** Robot swerved too aggressively around obstacles, contributing to sway.

**Changes in `ObstaclesCritic`:**

| Parameter | Old | New |
|-----------|-----|-----|
| `cost_weight` | 1.0 | 0.5 |
| `repulsion_weight` | 1.0 | 0.5 |
| `critical_weight` | 2.0 | 1.0 |

**Local costmap inflation tightened:**

| Parameter | Old | New |
|-----------|-----|-----|
| `inflation_radius` | 0.40m | 0.35m |
| `cost_scaling_factor` | 5.0 | 8.0 |

### 4. Speed & Acceleration Tuning (Multiple Iterations)

**Key discovery:** GitHub issue [#5694](https://github.com/ros-navigation/navigation2/issues/5694) — MPPI's trajectory horizon is short, so low acceleration limits prevent MPPI from sampling trajectories that reach max speed. The fix: set high MPPI accel limits, let velocity_smoother enforce real physical limits.

**Final MPPI config:**

| Parameter | Final Value |
|-----------|-------------|
| `vx_max` | 0.25 m/s |
| `vx_min` | -0.12 m/s |
| `wz_max` | 0.5 rad/s |
| `ax_max` | 5.0 m/s² |
| `ax_min` | -5.0 m/s² |
| `az_max` | 3.0 rad/s² |

**Final velocity_smoother config (smooth slew):**

| Parameter | Final Value | Effect |
|-----------|-------------|--------|
| `smoothing_frequency` | 20.0 Hz | Twice as many updates → smoother curve |
| `max_velocity` | [0.25, 0.0, 0.5] | Matches MPPI |
| `min_velocity` | [-0.12, 0.0, -0.5] | Matches MPPI |
| `max_accel` | [0.15, 0.0, 0.4] | Gentle ramp up (~1.7s to reach 0.25 m/s) |
| `max_decel` | [-0.25, 0.0, -0.5] | Gentle ramp down |

**diff_drive_controller angular limits synced:**
- `angular.z.max_velocity`: 1.5 → 0.5 rad/s

### 5. Twist Mux Configuration

Current priority layout:
| Source | Topic | Priority | Timeout |
|--------|-------|----------|---------|
| navigation | `/cmd_vel_nav` | 10 (low) | 0.3s |
| teleop | `/cmd_vel` | 20 (high) | 0.5s |

Navigation timeout reduced from 0.5s → 0.3s so it gives up faster when teleop takes over.

### 6. ReinitializeGlobalLocalization Duplicate Registration Fix

**Problem:** bt_navigator crashed on startup with `FATAL: ID [ReinitializeGlobalLocalization] already registered`.

**Root cause:** The plugin is loaded by default in Nav2. Adding `nav2_reinitialize_global_localization_service_bt_node` to `plugin_lib_names` caused duplicate registration.

**Fix:** Removed `plugin_lib_names` from both `controller_mppi_params.yaml` and `stvl_sim_params.yaml`.

### 7. Lifecycle Activator Reliability

**Problem:** Map server and AMCL failed to configure/activate intermittently due to timing races.

**Fixes in `localization_launch.py`:**
- Replaced blind `sleep 15` with node discovery loop (`ros2 node list` check, up to 30s)
- Added 3s settle after node discovery
- Increased retries from 5 → 8 with 5s delay
- Added 3s sleep between configure and activate steps
- Removed `ros2 lifecycle get` polling (was blocking map_server's service executor)
- Publishes initial pose 5 times with 2s gaps

### 8. Auto Initial Pose on Map Swap

**Problem:** When swapping maps via cloud UI, AMCL kept its old position estimate (wrong for the new map). No initial pose was published.

**Root cause:** `restart_map.sh` called `/map_server/load_map` and cleared costmaps but never re-published `/initialpose`.

**Fix in `/home/rspx1/restart_map.sh`:**
- After clearing costmaps, publishes `/initialpose` at (0,0,0) 5 times with 2s gaps
- AMCL relocalizes at map origin automatically

### 9. Launch Cleanup

- Removed redundant `publish_initial_pose` node from `launch_robot.launch.py` (lifecycle_activator handles it)
- Kept initial pose publishing in `localization_launch.py` lifecycle script only

---

## Files Modified

| File | Change |
|------|--------|
| `nevo_bot/config/controller_mppi_params.yaml` | MPPI tuning, velocity_smoother slew, obstacle softening, SMAC→NavFn revert |
| `nevo_bot/config/my_controllers.yaml` | Angular velocity limit synced to 0.5 rad/s |
| `nevo_bot/config/stvl_sim_params.yaml` | Removed `plugin_lib_names` |
| `nevo_bot/launch/localization_launch.py` | Robust lifecycle_activator with node discovery, 8 retries, 5x initial pose |
| `nevo_bot/launch/launch_robot.launch.py` | Removed redundant publish_initial_pose node |
| `/home/rspx1/restart_map.sh` | Added auto initial pose (0,0,0) after map swap |

---

## Known Issues

- **Loose CP2102 adapter:** Hardware issue causing intermittent rplidar disconnects → AMCL loses TF chain. Needs physical securing.
- **SMAC Hybrid too heavy for hardware:** Reeds-Shepp + high angle bins hangs planner_server. Would need reduced angle_quantization_bins (e.g. 36) or downsampled costmap to retry.
- **EKF "Failed to meet update rate":** Non-fatal warning, CPU occasionally overloaded during startup burst.
- **Cloud UI new battery fields:** Backend needs to parse/store/expose bus_voltage, soh, internal_resistance_mohm, cycle_count.
