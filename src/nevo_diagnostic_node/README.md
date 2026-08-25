# nevo_diagnostic_node

A **passive** ROS 2 (Humble) C++ node that monitors the robot boot process and
publishes real-time readiness status as JSON to `/nevo/ws_out`.

---

## Overview

`nevo_diagnostic_node` runs alongside your navigation stack as a silent observer.
It never calls lifecycle services, never publishes to control topics, and never
interferes with other nodes.  Its only job is to watch three topics and decide
whether the robot has successfully booted, then publish a machine-readable JSON
verdict that a UI or WebSocket bridge can consume.

---

## Features

- **Pure observer** — subscribes only, does not affect other nodes
- **State machine** — INIT → CHECKING → READY | FAILED
- **Grace period** — ignores all topics for a configurable warm-up window
- **Global timeout** — declares FAILED if READY is not reached in time
- **Debounce** — transient failures must persist for multiple evaluation cycles before triggering FAILED
- **Lightweight monitoring after READY** — continues watching odometry but does not trigger reset
- **Auto-recovery** — on FAILED, publishes an ERROR message and restarts `nevo.service` via systemctl
- **Fully configurable** — all timing and identity parameters exposed as ROS params

---

## Package Structure

```
nevo_diagnostic_node/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── diagnostic_params.yaml       # Default parameters
├── include/
│   └── nevo_diagnostic_node/
│       └── diagnostic_node.hpp      # Class declaration
├── launch/
│   └── diagnostic.launch.py         # Launch file
├── src/
│   └── diagnostic_node.cpp          # Full implementation
└── README.md
```

---

## Topics

### Subscribed

| Topic | Type | Purpose |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | Confirms map server is running |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Confirms localisation is active |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Confirms odometry pipeline is live |

### Published

| Topic | Type | Purpose |
|---|---|---|
| `/nevo/ws_out` | `std_msgs/msg/String` | JSON status messages for the UI |

---

## State Machine

```
              ┌─────────────────────────────────────────────┐
              │                   INIT                      │
              │  Node just started.                         │
              │  Publishes INITIALIZING.                    │
              │  Waits for grace_period_sec to elapse.      │
              └───────────────────┬─────────────────────────┘
                                  │ grace period elapsed
                                  ▼
              ┌─────────────────────────────────────────────┐
              │                 CHECKING                    │
              │  Evaluates topic health every 1 s.          │
              │  Publishes INITIALIZING while waiting.      │
              └──────────┬──────────────────────┬───────────┘
                         │ all conditions OK     │ global timeout OR
                         │                       │ topic hard-fail
                         ▼                       ▼
              ┌──────────────────┐   ┌───────────────────────┐
              │      READY       │   │        FAILED         │
              │                  │   │                       │
              │ Publishes READY. │   │ Publishes ERROR.      │
              │ Switches to      │   │ Waits 2 s.            │
              │ lightweight odom │   │ systemctl restart     │
              │ monitoring.      │   │ nevo.service          │
              └──────────────────┘   └───────────────────────┘
```

### READY conditions (all must be true)

1. `/map` has been received at least once
2. `/amcl_pose` was received and is not older than 5 seconds
3. `/odometry/filtered` is updating (last message age < `odom_timeout_sec`)

### FAILED conditions (any one triggers FAILED)

| Condition | Debounce |
|---|---|
| `/map` never received within `global_timeout_sec` | None (hard at timeout) |
| `/amcl_pose` never received within `global_timeout_sec` | None |
| `/odometry/filtered` stops updating for > `odom_timeout_sec` | 3 consecutive evaluation cycles (3 s) |
| System makes no progress within `global_timeout_sec` | None |

---

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `robot_id` | string | `"TFRB1"` | Robot identifier in JSON messages |
| `ui_id` | string | `"TFUI1"` | UI identifier in JSON messages |
| `grace_period_sec` | int | `10` | Seconds to ignore all topics after node start |
| `global_timeout_sec` | int | `40` | Boot window; FAILED if READY not reached in time |
| `odom_timeout_sec` | double | `3.0` | Max seconds between odometry messages before stale |

---

## How to Build

```bash
# From your ROS 2 workspace root:
cd ~/ros2_ws/src
# (copy or symlink nevo_diagnostic_node here)

cd ~/ros2_ws
colcon build --packages-select nevo_diagnostic_node
source install/setup.bash
```

---

## How to Run

### Using the launch file (recommended)

```bash
ros2 launch nevo_diagnostic_node diagnostic.launch.py
```

Override parameters on the command line:

```bash
ros2 launch nevo_diagnostic_node diagnostic.launch.py \
  robot_id:=RB002 \
  grace_period_sec:=5 \
  global_timeout_sec:=60
```

### Running the node directly

```bash
ros2 run nevo_diagnostic_node diagnostic_node \
  --ros-args \
  -p robot_id:=TFRB1 \
  -p ui_id:=TFUI1 \
  -p grace_period_sec:=10 \
  -p global_timeout_sec:=40 \
  -p odom_timeout_sec:=3.0
```

### Monitoring output

```bash
ros2 topic echo /nevo/ws_out
```

---

## Example Output JSON

### INITIALIZING (during grace period or checking)

```json
{
  "code": "INIT",
  "data": {
    "robot_id": "TFRB1",
    "ui_id": "TFUI1",
    "status": "INITIALIZING",
    "init_message": "Grace period elapsed. Checking system readiness."
  }
}
```

### READY

```json
{
  "code": "READY",
  "data": {
    "robot_id": "TFRB1",
    "ui_id": "TFUI1",
    "status": "READY",
    "init_message": "All systems are operational. Robot is ready."
  }
}
```

### ERROR (boot failure)

```json
{
  "code": "ERROR",
  "data": {
    "robot_id": "TFRB1",
    "ui_id": "TFUI1",
    "error_code": "SYSTEM_BOOT_FAIL",
    "error_message": "Global timeout (40 s). No /map received. No /amcl_pose received.",
    "severity": "CRITICAL"
  }
}
```

---

## Design Notes

- **No external JSON library required** — JSON is hand-built with `std::ostringstream` to keep dependencies minimal.
- **QoS** — Subscriptions use `best_effort` / `keep_last(1)` so the node does not block on slow or latching topics.
- **No log-based detection** — all decisions are made purely from topic timestamps, never from log output.
- **Thread safety** — all callbacks and the timer run in a single-threaded executor (default `rclcpp::spin`), so no mutex is needed.
- **Anti-false-trigger** — the 3-cycle debounce on odometry staleness prevents rf2o restarts or momentary gaps from causing a spurious FAILED.

---

## License

Apache-2.0
