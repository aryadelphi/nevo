import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/odometry/filtered'])); t, y = [], []
while r.has_next():
    _, d, bt = r.read_next(); q = deserialize_message(d, Odometry).pose.pose.orientation
    t.append(bt * 1e-9); y.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
t, y = np.array(t), np.unwrap(np.array(y)); z = np.load(sys.argv[2])
ph = [(l, a, b) for l, a, b in zip(z['ph_l'], z['ph_t'], list(z['ph_t'][1:]) + [t[-1]])]
st = [(a + b) / 2 for l, a, b in ph if l == 'still']
truth = [float(v) for v in sys.argv[3:5]]
for k, name in enumerate(('ccw', 'cw')):
    d = math.degrees(np.interp(st[k + 1], t, y) - np.interp(st[k], t, y))
    print(f'EKF /odometry/filtered {name}: {d:9.2f} deg vs lidar {truth[k]:9.2f} -> {100 * (d / truth[k] - 1):+.2f} % ({d - truth[k]:+.2f} deg over 3 turns)')
