# NEVO Session Report — 2026-09-14

## Summary

Fixed IMU + wheel encoder fusion issues: yaw drift when turning in place, TF errors in RViz, and incorrect URDF frame hierarchy. Aligned all frames to standard ROS2/Nav2 convention with `base_link` as URDF root.

---

## Problems

### 1. Yaw Drift When Turning In Place
Robot slowly drifted in yaw over time while turning in place. No absolute heading reference to correct the drift.

### 2. TF Errors / Not Showing in RViz
After previous attempt to fix frames by setting `base_footprint` as URDF root, TF stopped showing in RViz. Multiple nodes expected `base_link` as the standard robot body frame.

### 3. Frame Hierarchy Was Backwards
URDF had `base_footprint` as root and `base_link` as child — opposite of ROS2/Nav2 convention. This caused confusion across the entire TF tree.

---

## Root Causes

### Yaw Drift
- **No absolute yaw reference**: MPU6050 has no magnetometer. Yaw relies on gyro integration + wheel differential, both of which accumulate bias over time.
- **`odom0_differential: false`**: EKF was using absolute wheel odom pose instead of relative velocity, causing drift to compound.
- **`odom0_relative: false`**: Odom wasn't anchored to initial pose.
- **Low yaw process noise**: EKF was overconfident in yaw estimates, unable to correct for gyro bias.

### TF Errors
- **URDF frame hierarchy inverted**: `base_footprint` as root broke nodes expecting `base_link` as root.
- **Frame mismatch between EKF and controllers**: EKF published `odom -> base_footprint` while Nav2, SLAM toolbox, and diff_drive_controller expected `base_link`.

---

## Changes Made

### 1. URDF Frame Hierarchy Fix (`robot_core.xacro`)
Swapped `base_link` and `base_footprint` so `base_link` is URDF root:

**Before:**
```xml
<link name="base_footprint"/>
<joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
</joint>
<link name="base_link"/>
```

**After:**
```xml
<link name="base_link"/>
<joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
</joint>
<link name="base_footprint"/>
```

### 2. EKF Configuration (`config/ekf.yaml`)
- `base_link_frame: base_footprint` → `base_link_frame: base_link`
- `odom0_differential: false` → `odom0_differential: true` (use only wheel velocity, not absolute pose)
- `odom0_relative: false` → `odom0_relative: true` (anchor to initial pose)
- Yaw process noise: `0.015` → `0.025` (allow more yaw uncertainty for MPU6050 noise)
- Updated comments to reflect new frame hierarchy

### 3. DiffDrive Controller (`config/my_controllers.yaml`)
- `base_frame_id: base_footprint` → `base_frame_id: base_link`

### 4. DiffDrive Controller Sim (`config/my_sim_controllers.yaml`)
- `base_frame_id: base_footprint` → `base_frame_id: base_link`

### 5. IMU Publisher (`scripts/imu_publisher.py`)
- `frame_id: base_footprint` → `frame_id: base_link`

### 6. SLAM Toolbox (`config/mapper_params_online_async.yaml`)
- `base_frame: base_footprint` → `base_frame: base_link`

### 7. Robot API Node (`robot_api_node.cpp`)
- TF lookup frame: `base_footprint` → `base_link`
- Log message updated to match

### 8. Encoder Count Default (`config.h`)
- `enc_counts_per_rev = 1920` → `enc_counts_per_rev = 2000` (matches URDF `ros2_control.xacro`)

---

## Files Modified

| File | Change |
|------|--------|
| `description/robot_core.xacro` | `base_link` is URDF root, `base_footprint` is child |
| `config/ekf.yaml` | `base_link_frame: base_link`, differential/relative odom, yaw noise |
| `config/my_controllers.yaml` | `base_frame_id: base_link` |
| `config/my_sim_controllers.yaml` | `base_frame_id: base_link` |
| `scripts/imu_publisher.py` | `frame_id: base_link` |
| `config/mapper_params_online_async.yaml` | `base_frame: base_link` |
| `nevo_robot_api/src/robot_api_node.cpp` | TF lookup to `base_link` |
| `diffdrive_arduino/include/diffdrive_arduino/config.h` | `enc_counts_per_rev = 2000` |

---

## Resulting TF Tree

```
map                    [AMCL / SLAM Toolbox]
 └── odom              [EKF, ~4Hz]
      └── base_link    [EKF publishes this]
           ├── base_footprint     [RSP, static, identity]
           ├── chassis            [RSP, static]
           │    ├── laser_frame
           │    ├── camera_link
           │    ├── c310_camera_link
           │    ├── rear_caster_wheel
           │    └── back_caster_wheel
           ├── left_wheel         [joint_states from diff_cont]
           └── right_wheel        [joint_states from diff_cont]
```

---

## Known Remaining Issues

### Controller Manager Overruns
Serial read/write to Arduino takes 10-20ms, exceeding the 20ms budget at 50Hz. Actual rates:
- Wheel odom: ~3-8 Hz (target: 50 Hz)
- Joint states: ~3-8 Hz (target: 50 Hz)
- EKF filtered odom: ~2-4 Hz (target: 5 Hz)

This is a Pi5 CPU bottleneck from blocking Arduino serial calls. Not a config issue.

### Yaw Drift (Inherent)
Without a magnetometer, yaw will always drift over long periods. The gyro bias correction from wheel odom differential helps but cannot fully eliminate drift. For long-duration autonomous operation, consider adding a magnetometer (e.g., BMM150) or visual heading correction.

### Position Sensor Uses Raw Odom
`position_sensor_node.cpp` subscribes to `/diff_cont/odom` instead of `/odometry/filtered`. Cloud position reports use unfiltered wheel odometry. Consider subscribing to `/odometry/filtered` for better accuracy.

### USB Power Supply (Hardware)
Pi5 was underpowered, causing:
- Repeated undervoltage events (`hwmon3: Undervoltage detected!`)
- USB over-current triggers resetting the entire USB bus
- All serial devices (lidar, motor, IMU) disconnecting simultaneously
- IMU publisher flooding logs with 5330 reconnect errors
- EKF timestamp jump of 1620982s from missing sensor data

**Root cause**: PSU could not deliver enough current for Pi5 + multiple USB peripherals.
**Fix**: Swapped to adequate power supply. `throttled=0x0` confirmed clean.
**Note**: `/dev/ttyIMU` is not in `nevo_launch_robot.sh`'s wait list — add it to prevent premature IMU publisher start on boot.
