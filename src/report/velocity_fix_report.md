# Navigation Velocity Fix Report

**Date:** 2026-08-22
**Robot:** TFRB2 (nevo_bot)
**Issue:** Robot frozen/moving at ~5% max speed during Nav2 navigation

---

## Problem Summary

The robot would navigate to goals successfully but barely move (~0.018-0.05 m/s vs 0.35 m/s max).
Recovery behaviors (spin) worked fine, path planning was correct, but velocity commands were tiny.

---

## Root Cause

**MPPI's `ax_max` parameter (0.02, later 0.3) was capping the optimizer's trajectory search space, NOT just the output acceleration.**

MPPI generates 500+ random trajectories and evaluates them using critic cost functions.
With `ax_max: 0.3`, the optimizer literally cannot generate trajectories that accelerate
to 0.35 m/s within the 2.8s planning horizon. It outputs the best trajectory it can find,
which is always low velocity.

This is a known Nav2 Jazzy issue (GitHub #5694).

---

## Changes Made

### 1. MPPI Controller (`controller_mppi_params.yaml`)

| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| `ax_max` | 0.02 -> 0.3 | **15.0** | ROOT FIX: optimizer needs high limits to explore full velocity space |
| `ax_min` | -0.05 -> -0.5 | **-15.0** | Symmetric deceleration |
| `az_max` | 0.8 | **8.0** | Same principle for angular |
| `open_loop` | false | **true** | Use last cmd_vel for trajectory init, not odom (avoids feedback loop) |
| `temperature` | 0.6 | **0.3** | More greedy trajectory selection |
| `vx_std` | 0.2 | **0.4** | More exploration variance |
| `iteration_count` | 1 | **3** | Better optimization convergence |
| `v_std` | [0.1, 0.15] | **removed** | Not a real Nav2 MPPI param (silently ignored) |
| `v_mag_scaler` | 0.25 | **removed** | Not a real Nav2 MPPI param (silently ignored) |

### 2. MPPI Critics (restored to Nav2 defaults)

| Critic | Before | After |
|--------|--------|-------|
| `ConstraintCritic cost_weight` | 4.0 -> 1.0 | **4.0** (default) |
| `PathAlignCritic cost_weight` | 14.0 -> 5.0 | **14.0** (default) |
| `ObstaclesCritic` | disabled | **enabled** (restored for safety) |
| `PreferForwardCritic` | not present | **added** (cost_weight: 5.0) |

**Lesson:** Low critic weights were never the problem. The optimizer couldn't find good
trajectories because ax_max limited the search space.

### 3. Velocity Pipeline Fix

| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| `velocity_smoother use_stamped_vel` | False -> True | **False** | MPPI publishes Twist (not TwistStamped) via nav2_util::TwistPublisher |

The controller_server uses `nav2_util::TwistPublisher` which defaults to `enable_stamped_cmd_vel: false`,
meaning it publishes `geometry_msgs::msg::Twist`. The velocity_smoother must subscribe to the same type.

### 4. diff_drive_controller (`my_controllers.yaml`)

| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| `linear.x.max_acceleration` | 0.4 | **1.5** | Real physical limit (was too restrictive) |
| `linear.x.min_acceleration` | -0.4 | **-1.5** | Symmetric |
| `angular.z.max_acceleration` | 1.5 | **3.0** | Matching angular capability |
| `angular.z.min_acceleration` | -1.5 | **-3.0** | Symmetric |

### 5. STVL / Costmap Changes (earlier in session)

| Parameter | Before | After | Rationale |
|-----------|--------|-------|-----------|
| `stvl_layer voxel_decay` | 0.3 | 1.5 | Reduced costmap flicker |
| `stvl_layer min_obstacle_height` | 0.10 | 0.20 | Filter floor detection |
| `depth_camera min_obstacle_height` | 0.15 | 0.30 | Kinect at 1.0m height detects floor |
| `depth_camera voxel_min_points` | 1 | 3 | Reduce noise from depth sensor |
| `local_costmap inflation_radius` | 0.55 | 0.40 | Matched robot footprint |
| `local_costmap cost_scaling_factor` | 10.0 | 5.0 | Gentler inflation gradient |

---

## Architecture Summary (final)

```
MPPI (ax_max: 15, open_loop: true)
  -> Twist on /cmd_vel_nav
  -> velocity_smoother (use_stamped_vel: false, max_accel: 0.3)
  -> Twist on /cmd_vel
  -> twist_mux (use_stamped: false)
  -> Twist on /diff_cont/cmd_vel_unstamped_raw
  -> safety_fail_node (Twist -> TwistStamped)
  -> TwistStamped on /diff_cont/cmd_vel
  -> diff_drive_controller (max_accel: 1.5)
```

**Key insight:** MPPI's ax_max is an optimizer search-space limit, not a robot physical limit.
Physical limits are enforced by velocity_smoother and diff_drive_controller downstream.

---

## Reference

- Nav2 GitHub Issue #5694: "Max Speed Capped at 0.17 m/s after ROS 2 Jazzy Migration"
- Nav2 PR #5617: Uses last cmd_vel instead of odom for MPPI initial state
