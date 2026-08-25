# NEVO Robot — Runtime Installation Guide (ROS 2 Jazzy / Ubuntu 24.04)

This guide covers every step needed to restore the NEVO robot runtime on a fresh
**Ubuntu 24.04 (noble, arm64 — Raspberry Pi 5)** system. It assumes you have a copy
of this `nevo_ws/` directory on the target machine.

> **Migrated from:** ROS 2 Humble / Ubuntu 22.04 / Raspberry Pi 4 (`robot2` user).
> Key Jazzy changes baked into this workspace: `diff_drive_controller` now requires
> `TwistStamped` (handled by `safety_fail_node`), vendored `nav2_mppi_controller`
> replaced by the official Jazzy binary, and all paths use `/home/rspx1`.

---

## Prerequisites

```bash
sudo apt update && sudo apt install -y \
  curl git tmux \
  python3-pip python3-colcon-common-extensions \
  net-tools iputils-ping udev \
  nlohmann-json3-dev libcurl4-openssl-dev libzip-dev libssl-dev
```

### Install ROS 2 Jazzy (if not already installed)

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-jazzy-desktop ros-jazzy-ros-base
```

### Install Nav2, SLAM, control and sensor stacks (all binary on Jazzy)

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-opennav-docking \
  ros-jazzy-nav2-mppi-controller \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-tf2-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-controller-manager \
  ros-jazzy-hardware-interface \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-twist-mux \
  ros-jazzy-laser-filters \
  ros-jazzy-v4l2-camera \
  ros-jazzy-image-proc \
  ros-jazzy-apriltag-ros \
  ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-rosbridge-suite \
  ros-jazzy-rmw-fastrtps-cpp
```

> **`ros-jazzy-rmw-fastrtps-cpp` is REQUIRED** — it is not installed by
> `ros-jazzy-desktop`. The systemd units and `~/.bashrc` set
> `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`; without this package every ROS
> process fails at startup with "RMW implementation not installed".

Note: `rplidar_ros`, `kinect_ros2`, `rf2o_laser_odometry` and `m-explore-ros2`
are **vendored in `src/nevo-dev/`** and built from source.

### Install Python dependencies

Ubuntu 24.04 is PEP-668 "externally managed"; use `--break-system-packages`
(or a venv) for the robot's system-wide runtime deps:

```bash
pip3 install --break-system-packages gspread google-auth requests websockets
```

---

## File-by-file installation

All runtime support files live under `src/runtime/`. Each is described
below with its install command. Replace `rspx1` if your username differs.

### 1. Launch scripts → `~/`

```bash
cp src/runtime/scripts/nevo_launch_robot.sh ~/nevo_launch_robot.sh
cp src/runtime/scripts/nevo_restart.sh ~/nevo_restart.sh
cp src/runtime/scripts/restart_map.sh ~/restart_map.sh
chmod +x ~/nevo_launch_robot.sh ~/nevo_restart.sh ~/restart_map.sh
```

- **nevo_launch_robot.sh** — Main entry point. Sources ROS 2 Jazzy + workspace,
  waits up to 30 s for USB hardware to enumerate, then launches
  `nevo_bot launch_robot.launch.py` in the chosen `stack_mode` (`nav` or `mapping`).
- **nevo_restart.sh** — Mode-swap script. Called by `robot_api_node` via
  `systemd-run` when the cloud UI toggles between navigation and mapping.
  Kills the entire running stack tree (graceful → hard-kill), waits for old cloud
  sessions to clear, then relaunches in a detached `tmux` session.
- **restart_map.sh** — Map-swap script. Called by `robot_api_node`
  (`handleMapSelected`) when the cloud UI selects a different map. Calls
  `/map_server/load_map` via an rclpy client, then clears both costmaps.
  Must live at `$HOME/restart_map.sh` (hard-coded path in robot_api_node).
  Logs to `~/nevo_restart.log`.

### 2. Systemd services → `/etc/systemd/system/`

```bash
cp src/runtime/systemd/nevo.service /etc/systemd/system/nevo.service
cp src/runtime/systemd/nevo-mapping.service /etc/systemd/system/nevo-mapping.service
sudo systemctl daemon-reload
sudo systemctl enable nevo.service
# sudo systemctl enable nevo-mapping.service   # optional: boot into mapping
```

- **nevo.service** — Navigation stack service. Runs `nevo_launch_robot.sh nav`
  as user `rspx1`. Restarts on failure after 5 s. Uses CycloneDDS as RMW.
- **nevo-mapping.service** — Mapping stack service (launches
  `hw_mapping.launch.py` directly). Not enabled by default; mapping is normally
  entered via the cloud UI mode swap, which uses `launch_robot.launch.py
  stack_mode:=mapping` instead.

Both services use `KillMode=control-group` for a clean stop of the whole tree.

### 3. Sudoers rule → `/etc/sudoers.d/`

```bash
sudo cp src/runtime/sudoers/nevo-robot /etc/sudoers.d/nevo-robot
sudo chmod 440 /etc/sudoers.d/nevo-robot
sudo visudo -cf /etc/sudoers.d/nevo-robot   # validate syntax
```

- **nevo-robot** — Grants `rspx1` passwordless `sudo` for `/usr/bin/systemd-run`
  only. Required by mode swaps to launch transient units that escape the systemd
  service cgroup.

### 4. Udev rules → `/etc/udev/rules.d/` (Pi 5: regenerate first!)

The bundled rules were written for the Pi 4 USB layout. On the Pi 5 the USB
controller enumerates differently, so **regenerate port-based rules**:

```bash
# Plug in lidar + motor Arduino + battery Arduino, then:
bash src/runtime/scripts/gen_udev_rules.sh          # inspect
# Edit SYMLINK targets (ttyXXX -> ttyLIDAR/ttyMOTOR/ttyBATERAI), then:
bash src/runtime/scripts/gen_udev_rules.sh | sudo tee /etc/udev/rules.d/99-nevo.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Expected stable symlinks:

| Device | Symlink | Hardware |
|---|---|---|
| RP Lidar A1 | `/dev/ttyLIDAR` | CP2102 (10c4:ea60), usually has serial number |
| Motor Arduino | `/dev/ttyMOTOR` | CH340 (1a86:7523), no serial → match by port |
| Battery INA226 Arduino | `/dev/ttyBATERAI` | CH340 (1a86:7523), no serial → match by port |

`99-usb-serial.rules` / `99-serial-ports.rules` are kept as Pi 4 references.

### 5. DDS configs → `~/`

```bash
cp src/runtime/config/fastdds.xml ~/fastdds.xml
```

> FastDDS uses `FASTRTPS_DEFAULT_PROFILES_FILE` for configuration. The provided
> `fastdds.xml` sets larger socket buffers and DNS v2 disabled for stability.

### 5b. Kernel socket buffers

```bash
sudo cp src/runtime/sysctl/60-nevo-dds.conf /etc/sysctl.d/
sudo sysctl --system
```

Without `net.core.rmem_max >= 10MB`, nodes may fail under heavy load.

### 6. Set environment variables

Add to `~/.bashrc`:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=file://$HOME/fastdds.xml
```

(The systemd units set these directly; the bashrc entries cover manual runs.)

---

## Build the workspace

```bash
cd ~/nevo_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # best-effort; some pkgs are vendored
colcon build --symlink-install --parallel-workers 4
source install/setup.bash
```

---

## Start the robot

**Manual (navigation):**
```bash
bash ~/nevo_launch_robot.sh nav
```

**Manual (mapping):**
```bash
bash ~/nevo_launch_robot.sh mapping
```

**As a systemd service (auto-start on boot):**
```bash
sudo systemctl start nevo.service
sudo journalctl -u nevo.service -f   # watch logs
```

**Mode swap (from cloud UI or terminal):**
```bash
# Invoked automatically by robot_api_node when the cloud toggles mode.
# Manual equivalent:
sudo systemd-run --no-block --collect \
  --property=KillMode=process \
  --unit=nevo-restart-mapping \
  --uid=rspx1 --gid=rspx1 \
  --setenv=HOME=/home/rspx1 \
  bash /home/rspx1/nevo_restart.sh mapping
```

---

## Hardware setup

Verify before starting the stack:

```bash
ls -la /dev/ttyLIDAR /dev/ttyMOTOR /dev/ttyBATERAI /dev/video*
```

| Component | Interface | Driven by |
|---|---|---|
| Diff-drive motors | Arduino `/dev/ttyMOTOR` @57600 | `diffdrive_arduino` (ros2_control) |
| RP Lidar A1 | `/dev/ttyLIDAR` | `rplidar_ros` (vendored) |
| Kinect RGB-D | USB | `kinect_ros2` (vendored) |
| Rear dock camera | `/dev/video0` (Logitech C310) | `v4l2_camera` + `apriltag_ros` |
| Battery/power (INA226) | `/dev/ttyBATERAI` | `ina226_bridge_node.py` |

---

## Cloud configuration

Edit the ws_bridge parameters in:
```
nevo_ws/install/nevo_ws_bridge/share/nevo_ws_bridge/config/ws_bridge_params.yaml
```

Key parameters:
- `robot_id` — unique robot identifier (default: `TFRB2`)
- `ws_uri_primary` — cloud WebSocket endpoint (set in launch files)
- `network_mode` — `auto` (default), `online`, or `offline`

Cloud endpoints: `wss://tifa-ws.forgixrobotic.com` (WS),
`https://tifa-be.forgixrobotic.com` (REST map upload).

---

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot not visible in cloud UI | `ros2 topic echo /nevo/network_status` — should show `CONNECTED` |
| Mode swap hangs | Check `~/nevo_restart.log` — look for `robot_api_node is UP` |
| Lidar not reading | `ls -la /dev/ttyLIDAR`; regenerate udev rules (`gen_udev_rules.sh`) |
| Service fails on boot | `journalctl -u nevo.service -n 50` — check if USB devices appeared |
| WebSocket reconnects slowly | WsClient default interval is 3 s; check `ws_bridge_params.yaml` |
| `diff_cont` fails to configure | Ensure no `use_stamped_vel` key remains in `my_controllers.yaml` (removed in Jazzy) |
| Motors never move but controllers active | `ros2 topic echo /diff_cont/cmd_vel` must show **TwistStamped** (from safety_fail_node) |
| MPPI controller not found | `ros-jazzy-nav2-mppi-controller` must be installed (vendored copy was removed) |
