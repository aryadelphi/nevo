# Tel-U Cafe 3D survey (Kinect + lidar)

Goal: measure the cafe layout (table and chair positions and sizes, corridor
width, pillars, counter) for the thesis sim world. The output is a 3D point
cloud to scale, which you measure in CloudCompare.

Data flow: the RPLIDAR and wheel/EKF odometry give the geometry (RTAB-Map,
lidar only, with loop closure). The full-resolution Kinect depth adds the 3D
detail, such as tabletops and seats the 2D lidar cannot see.

## Files

| File | Runs on | What |
|---|---|---|
| `record_cafe_survey.sh` | robot | checks topics and disk space, records the survey bag |
| `survey_rtabmap.launch.py` | laptop (ubuntu24 distrobox) | bag -> RTAB-Map -> `cloud.ply` |
| `survey_cloud.py` | laptop (started by the launch) | depth -> voxel cloud, TF filter |

## Before going

- Robot charged. Laptop on the same network if you want RViz.
- On the robot, `/depth/image_raw/compressedDepth` must exist, or the record
  script falls back to raw depth (about 4x bigger). Fix:
  `sudo apt install ros-jazzy-compressed-depth-image-transport`, then
  restart the stack.
- At least 5 GB free on the robot (about 2 GB per 10 min).
- Tape measure, phone for photos, notepad.
- Pick an empty hour with no direct sun on the windows (sun blinds the Kinect).

## At the cafe

1. Start the stack without the cloud layer, in mapping mode (this also gives a fresh 2D map):
   ```
   ros2 launch nevo_bot launch_robot.launch.py stack_mode:=mapping cloud:=false
   ```
2. In a **second, foreground** terminal (zsh suspends a backgrounded record):
   ```
   bash ~/nevo_ws/src/scripts/cafe_survey/record_cafe_survey.sh
   ```
3. Do a 1-minute test first, then check it on the laptop (see below) before the full run.
4. Drive with teleop:
   - slowly, 0.1-0.2 m/s, and turn slowly (< 0.4 rad/s; fast turns smear the RPLIDAR scan)
   - pass every table at **1-2 m** (Kinect error is ~1 cm at 1 m, ~4 cm at 3 m, so it is cut at 3 m)
   - see every table from **both sides**, so chairs on the far side get covered
   - finish where you started (gives RTAB-Map a loop closure)
5. Ctrl+C the recorder. Take photos from 2-3 corners.
6. Tape cross-checks (these verify the scale of the cloud):
   - narrowest corridor width, leg to leg
   - one table to the nearest wall
   - one table top: length x width x height; one chair: seat size, seat height, backrest height

## On the laptop

Copy the bag over, then in the distrobox:
```
ros2 launch ~/work/diagonal_robotics/nevo_git_hw/src/scripts/cafe_survey/survey_rtabmap.launch.py \
    bag:=~/bags/cafe_survey_<date> out:=~/cafe_survey/<date>
```
- It plays the bag in real time. `cloud.ply` is written 15 s after the
  bag ends, then stop the launch with Ctrl+C (RTAB-Map saves `rtabmap.db`).
- `rviz:=true` shows progress (fixed frame `map`, add the `/rtabmap/map` grid).
- `max_range:=2.5` makes it stricter if the cloud looks noisy.
- Calibration (nevo, 2026-10-25, lab + wall bags): the launch defaults already apply
  the Kinect depth correction `depth_scale:=1.0436 depth_offset:=-0.026`
  (true = 1.0436 x Kinect - 0.041 m from the face, residuals <= 1 cm at 1-3 m).
  The URDF now has the measured Kinect mount (x 0.495, pitch 2.45 deg down, yaw 2.0 deg left).
- Bags recorded BEFORE that URDF fix (e.g. `lab_test_1025`) also need
  `cam_pitch:=2.45 cam_yaw:=2.0 cam_fwd:=0.035`.
- Sim bags: `depth_scale:=1 depth_offset:=0 clock:=false scan:=/scan
  depth_topic:=/camera/depth/image_raw info_topic:=/camera/depth/camera_info`.
- The body pitch varies 1-3 deg with how the front caster settles; level the caster
  for better heights. Expect about +-5 cm on small objects.

Measure in CloudCompare (`flatpak install flathub org.cloudcompare.CloudCompare`):
open `cloud.ply` (points are already coloured by height: blue floor -> red at 1.6 m+), use *Tools > Point picking* for distances. Slice at seat
height (0.40-0.50 m) or tabletop height (0.70-0.80 m) with *Edit > Segment*,
or *Tools > Projection > Rasterize* for a top view.

## Look at it

- Quick PNGs: `python3 render_views.py <out>/cloud.ply [--closeup X Y]`
- Watch the cloud being built while processing (fixed frame `map`): add `rviz:=true`
  to the survey launch, with nevo_bot sourced for the robot model:
  ```
  source ~/work/diagonal_robotics/nevo_git/install/setup.bash
  ros2 launch .../cafe_survey/survey_rtabmap.launch.py bag:=~/bags/<bag> out:=<out> rviz:=true
  ```
  The growing cloud (4 cm preview, every 1 s) is on `/survey_cloud_live`; `cloud.ply` is
  still written at full resolution at the end.
- Replay in RViz (robot model, lidar, live Kinect cloud, survey cloud as backdrop):
  ```
  source ~/work/diagonal_robotics/nevo_git/install/setup.bash   # nevo_bot, for the robot model
  ros2 launch .../cafe_survey/view_survey.launch.py bag:=~/bags/<bag> cloud:=<out>/cloud.ply
  ```
  `rate:=2.0` plays faster, Space pauses. Fixed frame `odom`.

## Tested

2026-10-25 on a sim bag from `nevo_git` `telu_cafe` world (70 s drive, 32FC1
depth): tabletops and chair seats in `cloud.ply` land on the modelled
furniture. 2026-10-25 on nevo (stationary, on the bench supply, 29 s): all 10 topics
recorded, compressed depth at 29 Hz / 3.2 MB/s (about 2 GB per 10 min),
depth decodes and projects 1-3.4 m in front of the robot. 2026-10-25 lab drives
(`lab_test_1025`, `lab_test2_1025` after the URDF calibration, with wheel slip): a
38x25x30 cm box comes out 37-38 x 30 cm, top 29 cm, same position from all
viewing directions within 5 cm; floor at 0-2 cm.
