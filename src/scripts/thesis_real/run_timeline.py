import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/cmd_vel_nav', '/diff_cont/odom', '/amcl_pose', '/real_bench/event', '/rosout']
r.set_filter(rosbag2_py.StorageFilter(topics=want)); cls = {t: get_message(types[t]) for t in want}
GX, GY, GTH = 5.091, -1.162, -3.112
cmd, od, am, ev, logs = [], [], [], [], []
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
while r.has_next():
    t, d, bt = r.read_next(); m = deserialize_message(d, cls[t]); bt *= 1e-9
    if t == '/cmd_vel_nav': cmd.append((bt, m.linear.x, m.angular.z))
    elif t == '/diff_cont/odom': od.append((bt, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == '/amcl_pose': p = m.pose.pose; am.append((bt, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == '/real_bench/event': ev.append((bt, m.data))
    elif t == '/rosout' and m.name in ('controller_server', 'behavior_server', 'bt_navigator') and m.level >= 20: logs.append((bt, m.name, m.msg[:70]))
cmd, od, am = np.array(cmd), np.array(od), np.array(am)
t0 = [t for t, s in ev if s.startswith('run_start')][0]; t1 = [t for t, s in ev if s.startswith('run_end')][0]
print('run', round(t1 - t0, 1), 's;', 'events', [s for _, s in ev])
print(' t[s]  cmd_vx  cmd_wz | odom_vx odom_wz | dist[cm] yaw_err[deg]')
for t in np.arange(t0, t1 + 1, 2.0):
    c = cmd[(cmd[:, 0] >= t) & (cmd[:, 0] < t + 2)]; o = od[(od[:, 0] >= t) & (od[:, 0] < t + 2)]
    a = am[am[:, 0] <= t + 2]
    d = 100 * math.hypot(a[-1, 1] - GX, a[-1, 2] - GY) if len(a) else float('nan')
    ye = math.degrees(math.atan2(math.sin(a[-1, 3] - GTH), math.cos(a[-1, 3] - GTH))) if len(a) else float('nan')
    fc = lambda x: f'{x:7.3f}' if len(c) else '      -'
    print(f'{t - t0:5.0f} {fc(np.abs(c[:, 1]).mean() if len(c) else 0)} {fc(np.abs(c[:, 2]).mean() if len(c) else 0)} | '
          f'{np.abs(o[:, 1]).mean():7.3f} {np.abs(o[:, 2]).mean():7.3f} | {d:7.1f} {ye:7.1f}')
print('--- controller/behavior events during the run')
for t, n, msg in logs:
    if t0 <= t <= t1 and not msg.startswith('Passing new path'): print(f'{t - t0:6.1f} {n}: {msg}')
