#!/usr/bin/env python3
"""Offline helpers for the cafe survey (run on the laptop during bag playback).

Two nodes in one process:
  tf_filter   /tf_bag -> /tf, dropping every transform whose parent is `map`
              (AMCL / slam_toolbox map->odom recorded on the robot would fight
              RTAB-Map's map->odom).
  accumulator full-res Kinect depth -> points in `map` (via TF at the image
              stamp) -> 2 cm voxel cloud -> PLY. Frames wait in a queue until
              RTAB-Map's map->odom covers their stamp (it lags ~1-2 s).
              Writes the file when the bag has been quiet for --idle
              seconds, or on Ctrl+C.

Usage (normally started by survey_rtabmap.launch.py):
  python3 survey_cloud.py --out cafe.ply [--max-range 3.0] [--voxel 0.02]
"""
import argparse
import collections
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
import tf2_ros


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def cam_correction(pitch_deg, roll_deg, yaw_deg=0.0):
    """Rotation (optical frame) for a camera whose real mount differs from the URDF.

    Angles are about the camera_link axes (X fwd, Y left, Z up):
    pitch_deg > 0: tilted down; yaw_deg > 0: turned left; roll_deg > 0: right side down.
    """
    if not (pitch_deg or roll_deg or yaw_deg):
        return None
    p, r, y = (math.radians(v) for v in (pitch_deg, roll_deg, yaw_deg))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    C = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])   # optical -> camera_link
    return C.T @ (Rz @ Ry @ Rx) @ C


def height_colors(z, zmax):
    """Blue (floor) -> cyan -> green -> yellow -> red (zmax and above), RGB 0..1."""
    stops = np.array([[0.20, 0.30, 0.75], [0.20, 0.65, 0.75], [0.35, 0.75, 0.35],
                      [0.95, 0.80, 0.25], [0.90, 0.35, 0.20]])
    x = np.clip(z / zmax, 0, 1) * (len(stops) - 1)
    k = np.minimum(x.astype(int), len(stops) - 2)
    f = (x - k)[:, None]
    return stops[k] * (1 - f) + stops[k + 1] * f


def decode_compressed_depth(msg):
    """compressed_depth_image_transport: 12-byte ConfigHeader, then PNG (16UC1)."""
    if 'compressedDepth' not in msg.format or '16UC1' not in msg.format:
        return None
    img = cv2.imdecode(np.frombuffer(bytes(msg.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
    return img if img is not None and img.dtype == np.uint16 else None


class TfFilter(Node):
    def __init__(self):
        super().__init__('survey_tf_filter')
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(TFMessage, '/tf_bag', self.cb, 100)

    def cb(self, msg):
        keep = [t for t in msg.transforms if t.header.frame_id.lstrip('/') != 'map']
        if keep:
            self.pub.publish(TFMessage(transforms=keep))


class Accumulator(Node):
    def __init__(self, a):
        super().__init__('survey_cloud')
        self.a = a
        self.K = None
        self.frame = 'kinect_depth'
        self.R_cam = cam_correction(a.cam_pitch_deg, a.cam_roll_deg, a.cam_yaw_deg)
        if self.R_cam is not None:
            self.get_logger().info(f'camera mount correction: pitch {a.cam_pitch_deg:+.2f} '
                                   f'(+ = down), roll {a.cam_roll_deg:+.2f}, '
                                   f'yaw {a.cam_yaw_deg:+.2f} (+ = left) deg')
        # whole-run TF history: when integration lags playback (fast --rate, live
        # preview), frames are still placed with the pose of their own stamp
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=3600))
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        # voxel grid as parallel arrays: packed key, point sum, point count
        self.vkey = np.zeros(0, np.int64)
        self.vsum = np.zeros((0, 3))
        self.vcnt = np.zeros(0, np.int64)
        self.pending = []       # point batches not yet merged into the grid
        self.last_pose = None
        self.prev_yaw = None    # (stamp s, yaw) of the previous frame, used or not
        self.used = self.skipped_tf = self.skipped_still = self.skipped_turn = 0
        self.last_rx = None
        self.queue = collections.deque()   # (depth_m, header, wall time queued)
        self.done = False
        self.saved = False
        qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        # receiving and integrating in separate groups: a slow integration
        # must not make the subscriptions drop frames
        rx = MutuallyExclusiveCallbackGroup()
        work = MutuallyExclusiveCallbackGroup()
        self.create_subscription(CameraInfo, a.info_topic, self.on_info, qos,
                                 callback_group=rx)
        self.create_subscription(CompressedImage, a.depth_topic + '/compressedDepth',
                                 self.on_compressed, qos, callback_group=rx)
        self.create_subscription(Image, a.depth_topic, self.on_raw, qos, callback_group=rx)
        self.create_timer(0.1, self.drain, callback_group=work)
        self.create_timer(2.0, self.on_timer)
        if a.live_topic:  # growing cloud for RViz while the bag plays
            self.live_pub = self.create_publisher(
                PointCloud2, a.live_topic,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_timer(a.live_period, self.publish_live, callback_group=work)

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.frame = msg.header.frame_id or self.frame
            self.get_logger().info(f'camera_info: {msg.width}x{msg.height} frame {self.frame}')

    # Frames are queued undecoded: most get skipped (robot not moving), and
    # decoding all of them is what limits the playback rate.
    def on_compressed(self, msg):
        self.enqueue(msg, msg.header)

    def on_raw(self, msg):
        self.enqueue(msg, msg.header)

    @staticmethod
    def decode(msg):
        """Depth message -> float32 metres (0 = no reading), or None."""
        if isinstance(msg, CompressedImage):
            img = decode_compressed_depth(msg)
            return None if img is None else img.astype(np.float32) / 1000.0
        if msg.encoding in ('16UC1', 'mono16'):        # Kinect: mm
            img = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)
            return img[:, :msg.width].astype(np.float32) / 1000.0
        if msg.encoding == '32FC1':                    # Gazebo: m
            img = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)
            return np.nan_to_num(img[:, :msg.width], nan=0.0, posinf=0.0)
        return None

    def enqueue(self, msg, header):
        self.last_rx = time.monotonic()
        if self.K is not None and not self.done:
            self.queue.append((msg, header, self.last_rx))

    def drain(self, final=False):
        """Integrate queued frames in order once their TF exists."""
        while self.queue:
            msg, header, queued = self.queue[0]
            try:
                tf = self.tf_buf.lookup_transform(self.a.frame, header.frame_id or self.frame,
                                                  Time.from_msg(header.stamp))
            except tf2_ros.TransformException as e:
                if not final and time.monotonic() - queued < self.a.tf_wait:
                    return  # map->odom not there yet: keep the frame, retry later
                self.queue.popleft()
                self.skipped_tf += 1
                self.get_logger().warn(f'no TF {self.a.frame} <- {header.frame_id}: {e}',
                                       throttle_duration_sec=10.0)
                continue
            self.queue.popleft()
            self.integrate(msg, tf)

    def integrate(self, msg, tf):
        t = tf.transform.translation
        R = quat_to_mat(tf.transform.rotation)
        # skip near-duplicate views: robot standing still adds nothing but noise
        yaw = math.atan2(R[1, 2], R[0, 2])
        pose = (t.x, t.y, yaw)
        # skip frames taken while turning: any depth/TF timing offset smears
        # the cloud sideways by (yaw rate x offset x range)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        prev, self.prev_yaw = self.prev_yaw, (stamp, yaw)
        if self.a.max_yaw_rate > 0 and prev is not None and stamp > prev[0]:
            rate = abs(math.atan2(math.sin(yaw - prev[1]), math.cos(yaw - prev[1]))) / (stamp - prev[0])
            if rate > math.radians(self.a.max_yaw_rate):
                self.skipped_turn += 1
                return
        if self.a.yaw_range is not None:  # only frames looking into this sector
            lo, hi = (math.radians(v) for v in self.a.yaw_range)
            if (yaw - lo) % (2 * math.pi) > (hi - lo) % (2 * math.pi):
                return
        if self.last_pose is not None:
            dx = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            dyaw = abs(math.atan2(math.sin(pose[2] - self.last_pose[2]),
                                  math.cos(pose[2] - self.last_pose[2])))
            if dx < self.a.min_move and dyaw < math.radians(self.a.min_turn_deg):
                self.skipped_still += 1
                return
        self.last_pose = pose
        depth_m = self.decode(msg)
        if depth_m is None:
            return

        s = self.a.stride
        z = depth_m[::s, ::s]
        z = np.where(z > 0, z * self.a.depth_scale + self.a.depth_offset, 0.0)
        h, w = z.shape
        u = (np.arange(w) * s)[None, :]
        v = (np.arange(h) * s)[:, None]
        ok = (z > self.a.min_range) & (z < self.a.max_range)
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        x = ((u - cx) * z / fx)[ok]
        y = ((v - cy) * z / fy)[ok]
        pts = np.stack([x, y, z[ok]], axis=1)
        if self.R_cam is not None:  # mount correction, still in the optical frame
            pts = pts @ self.R_cam.T
        if self.a.cam_fwd:          # camera really sits further forward than the URDF says
            pts[:, 2] += self.a.cam_fwd
        pts = pts @ R.T + np.array([t.x, t.y, t.z])
        pts = pts[(pts[:, 2] > -0.05) & (pts[:, 2] < self.a.max_height)]
        self.pending.append(pts)
        self.used += 1
        if sum(len(p) for p in self.pending) > 1_000_000:
            self.merge()

    def merge(self):
        if not self.pending:
            return
        pts = np.concatenate(self.pending)
        self.pending = []
        # pack the 3 voxel indices into one int64 (21 bits each, +-20 km at 2 cm)
        ijk = np.floor(pts / self.a.voxel).astype(np.int64) + (1 << 20)
        key = (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]
        key = np.concatenate([self.vkey, key])
        val = np.concatenate([self.vsum, pts])
        cnt = np.concatenate([self.vcnt, np.ones(len(pts), np.int64)])
        self.vkey, inv = np.unique(key, return_inverse=True)
        self.vsum = np.zeros((len(self.vkey), 3))
        for c in range(3):
            self.vsum[:, c] = np.bincount(inv, weights=val[:, c], minlength=len(self.vkey))
        self.vcnt = np.bincount(inv, weights=cnt, minlength=len(self.vkey)).astype(np.int64)

    def publish_live(self):
        """Publish the cloud built so far, thinned to --live-voxel, height-coloured."""
        now = time.monotonic()  # wall-clock limit: at --rate 3 bag-time timers fire 3x as often
        if now - getattr(self, 'live_t', 0.0) < self.a.live_min_wall:
            return
        self.live_t = now
        self.merge()
        if not len(self.vkey):
            return
        keep = self.vcnt >= self.a.min_hits
        k = self.vkey[keep]
        f = max(1, int(round(self.a.live_voxel / self.a.voxel)))   # e.g. 4 cm / 2 cm = 2
        m = (1 << 21) - 1
        coarse = (((k >> 42) & m) // f << 42) | (((k >> 21) & m) // f << 21) | ((k & m) // f)
        _, first = np.unique(coarse, return_index=True)
        sel = np.flatnonzero(keep)[first]
        pts = (self.vsum[sel] / self.vcnt[sel, None]).astype(np.float32)
        rgb = (height_colors(pts[:, 2], self.a.color_max_z) * 255).astype(np.uint32)
        out = np.zeros(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        out['x'], out['y'], out['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
        out['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
        msg = PointCloud2()
        msg.header.frame_id = self.a.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(out)
        msg.fields = [PointField(name=n, offset=o, datatype=t, count=1) for n, o, t in
                      [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32),
                       ('z', 8, PointField.FLOAT32), ('rgb', 12, PointField.UINT32)]]
        msg.is_bigendian, msg.point_step = False, 16
        msg.row_step, msg.is_dense = 16 * len(out), True
        msg.data = out.tobytes()
        self.live_pub.publish(msg)

    def on_timer(self):
        if self.last_rx is None:
            return
        self.get_logger().info(
            f'frames used {self.used}, skipped (no TF) {self.skipped_tf}, '
            f'skipped (not moving) {self.skipped_still}, (turning) {self.skipped_turn}, '
            f'voxels {len(self.vkey)}',
            throttle_duration_sec=10.0)

    def save(self):
        if self.saved:
            return
        self.saved = True
        t0 = time.monotonic()
        self.get_logger().info(f'saving: {len(self.queue)} queued frames, '
                               f'{sum(len(p) for p in self.pending)} unmerged points')
        self.drain(final=True)
        self.merge()
        # drop voxels hit by fewer than min_hits points: removes flying pixels
        keep = self.vcnt >= self.a.min_hits
        pts = (self.vsum[keep] / self.vcnt[keep, None]).astype(np.float32)
        # colour = height, so the PLY opens readable in CloudCompare without a ramp
        rgb = (height_colors(pts[:, 2], self.a.color_max_z) * 255).astype(np.uint8)
        v = np.empty(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                      ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
        v['x'], v['y'], v['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
        v['red'], v['green'], v['blue'] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        with open(self.a.out, 'wb') as f:
            f.write((f'ply\nformat binary_little_endian 1.0\nelement vertex {len(pts)}\n'
                     'property float x\nproperty float y\nproperty float z\n'
                     'property uchar red\nproperty uchar green\nproperty uchar blue\n'
                     'end_header\n').encode())
            f.write(v.tobytes())
        self.get_logger().info(f'wrote {self.a.out}: {len(pts)} points '
                               f'({self.used} depth frames, {time.monotonic() - t0:.1f} s)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='cafe_cloud.ply')
    ap.add_argument('--depth-topic', default='/depth/image_raw',
                    help='raw Image topic; <topic>/compressedDepth is tried too')
    ap.add_argument('--info-topic', default='/depth/camera_info')
    ap.add_argument('--cam-pitch-deg', type=float, default=0.0,
                    help='real Kinect tilt not in the URDF, + = down (nevo 2026-10-25: 2.45)')
    ap.add_argument('--cam-roll-deg', type=float, default=0.0)
    ap.add_argument('--cam-yaw-deg', type=float, default=0.0,
                    help='real camera yaw vs the URDF, + = turned left')
    ap.add_argument('--cam-fwd', type=float, default=0.0,
                    help='m, real camera position ahead of the URDF one (along optical z)')
    ap.add_argument('--depth-scale', type=float, default=1.0,
                    help='depth calibration: z_true = scale * z + offset '
                         '(nevo wall calibration 2026-10-25: 1.0436)')
    ap.add_argument('--depth-offset', type=float, default=0.0,
                    help='m, see --depth-scale (nevo 2026-10-25: -0.026)')
    ap.add_argument('--yaw-range', type=float, nargs=2, metavar=('FROM', 'TO'),
                    help='deg, camera heading in the cloud frame; debug: build the '
                         'cloud from one viewing direction only')
    ap.add_argument('--max-yaw-rate', type=float, default=0.0,
                    help='deg/s; skip depth frames taken while turning faster (0 = off)')
    ap.add_argument('--frame', default='map',
                    help='fixed frame of the cloud (odom = raw odometry, no SLAM)')
    ap.add_argument('--voxel', type=float, default=0.02, help='m')
    ap.add_argument('--min-range', type=float, default=0.5, help='m (Kinect v1 min ~0.5)')
    ap.add_argument('--max-range', type=float, default=3.0,
                    help='m; Kinect v1 error is ~3-4 cm at 3 m and grows quadratically')
    ap.add_argument('--max-height', type=float, default=2.2, help='m, drops the ceiling')
    ap.add_argument('--stride', type=int, default=2, help='use every Nth pixel')
    ap.add_argument('--min-move', type=float, default=0.03, help='m between used frames')
    ap.add_argument('--min-turn-deg', type=float, default=2.0)
    ap.add_argument('--min-hits', type=int, default=3)
    ap.add_argument('--live-topic', default='',
                    help='publish the growing cloud here for RViz (e.g. /survey_cloud_live)')
    ap.add_argument('--live-period', type=float, default=2.0, help='s (ROS/bag time)')
    ap.add_argument('--live-voxel', type=float, default=0.04, help='m, thinning for RViz')
    ap.add_argument('--live-min-wall', type=float, default=3.0,
                    help='s of wall time between live updates (keeps integration ahead)')
    ap.add_argument('--color-max-z', type=float, default=1.6,
                    help='m, height that gets the top (red) colour in the PLY')
    ap.add_argument('--idle', type=float, default=15.0,
                    help='s without depth after the first frame -> save and exit')
    ap.add_argument('--tf-wait', type=float, default=5.0,
                    help='wall s a frame may wait for its map TF before it is dropped')
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    acc, filt = Accumulator(a), TfFilter()
    ex = rclpy.executors.MultiThreadedExecutor()
    ex.add_node(acc)
    ex.add_node(filt)

    # The idle check runs here on the wall clock: ROS timers follow /clock,
    # which stops when the bag ends.
    try:
        while rclpy.ok():
            ex.spin_once(timeout_sec=0.5)
            if acc.last_rx is not None and time.monotonic() - acc.last_rx > a.idle:
                acc.done = True
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        acc.save()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
