import sys, math, time, numpy as np, rosbag2_py
sys.path.insert(0, sys.argv[2]); import icp2d
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/scan_filtered', '/real_bench/event'])); scans, ev = [], []
while r.has_next():
    t, d, bt = r.read_next(); bt *= 1e-9
    if t == '/real_bench/event': ev.append((bt, deserialize_message(d, String).data))
    else: m = deserialize_message(d, LaserScan); scans.append((bt, icp2d.scan_points(m.ranges, m.angle_min, m.angle_increment)))
ends = [t for t, s in ev if s.startswith('run_end')]; print('runs', len(ends))
def stack(t0, t1): return icp2d.voxel(np.vstack([p for t, p in scans if t0 <= t <= t1]))
E = [stack(t + 1.0, t + 3.0) for t in ends]
tape = [tuple(map(float, x.split(','))) for x in sys.argv[3].split(';')]   # cm, cm, deg in the goal frame
def rel_from_tape(a, b):  # pose of run b in run a's frame
    ax, ay, at = a[0] / 100, a[1] / 100, math.radians(a[2]); bx, by, bt_ = b[0] / 100, b[1] / 100, math.radians(b[2])
    c, s = math.cos(-at), math.sin(-at); dx, dy = bx - ax, by - ay
    return 100 * (c * dx - s * dy), 100 * (s * dx + c * dy), math.degrees(bt_ - at)
for a, b in [(0, 1), (0, 2), (1, 2)]:
    t0 = time.time(); res = icp2d.icp(E[b], E[a]); ex = rel_from_tape(tape[a], tape[b])
    print(f'run{b+1} in run{a+1} frame: ICP ({100*res["x"]:6.1f}, {100*res["y"]:6.1f} cm, {math.degrees(res["yaw"]):5.1f} deg) '
          f'res {res["res_cm"]:.1f} cm inl {res["inliers"]:.2f} ok={res["ok"]} | tape diff ({ex[0]:6.1f}, {ex[1]:6.1f}, {ex[2]:5.1f}) | {time.time()-t0:.1f} s, pts {len(E[b])}/{len(E[a])}')
