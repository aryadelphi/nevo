import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
GX, GY, GTH = 4.595, -0.67, 0.0285
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = ['/cmd_vel_nav', '/diff_cont/odom', '/amcl_pose', '/real_bench/event', '/rosout']; cls = {t: get_message(types[t]) for t in want}
r.set_filter(rosbag2_py.StorageFilter(topics=want)); cmd, od, am, ev, logs = [], [], [], [], []
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
while r.has_next():
    t, d, bt = r.read_next(); m = deserialize_message(d, cls[t]); bt *= 1e-9
    if t == '/cmd_vel_nav': cmd.append((bt, m.linear.x, m.angular.z))
    elif t == '/diff_cont/odom': od.append((bt, m.twist.twist.linear.x, m.twist.twist.angular.z))
    elif t == '/amcl_pose': p = m.pose.pose; am.append((bt, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == '/real_bench/event': ev.append((bt, m.data))
    elif t == '/rosout' and m.name in ('controller_server', 'behavior_server') and m.level >= 20: logs.append((bt, m.msg[:60]))
cmd, od, am = map(np.array, (cmd, od, am))
starts = [t for t, s in ev if s.startswith('run_start')]; ends = [t for t, s in ev if s.startswith('run_end')]
for i, (t0, t1) in enumerate(zip(starts, ends), 1):
    a = am[(am[:, 0] >= t0) & (am[:, 0] <= t1 + 1)]
    c, s_ = math.cos(GTH), math.sin(GTH)
    fx = (c * (a[:, 1] - GX) + s_ * (a[:, 2] - GY)) * 100; fy = (-s_ * (a[:, 1] - GX) + c * (a[:, 2] - GY)) * 100; dist = np.hypot(fx, fy)
    first15 = a[np.argmax(dist < 15), 0] - t0 if (dist < 15).any() else float('nan')
    cm = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t1)]; o = od[(od[:, 0] >= t0) & (od[:, 0] <= t1)]
    rev = (cm[:, 1] < -0.005).mean() * 100
    # stall: |cmd| small but non-zero and odom ~0, 1 s bins
    bins = np.arange(t0, t1, 1.0); stall = 0
    for b in bins:
        cc = cm[(cm[:, 0] >= b) & (cm[:, 0] < b + 1)]; oo = o[(o[:, 0] >= b) & (o[:, 0] < b + 1)]
        if len(cc) and len(oo) and (np.abs(cc[:, 1:]).max() > 0.005) and np.abs(oo[:, 1:]).max() < 0.01: stall += 1
    ev_r = [f'{tt - t0:.0f}s:{m.split(":")[0][:22]}' for tt, m in logs if t0 <= tt <= t1 and any(k in m for k in ('Failed to make', 'Running', 'Reached', 'oscill'))]
    print(f'run {i}: T {t1 - t0:5.1f}s | first <15cm (AMCL) at {first15:5.1f}s | min dist {dist.min():4.1f} cm | x range {fx.min():6.1f}..{fx.max():6.1f} cm (overshoot>0) | '
          f'cmd reverse {rev:4.1f}% | stall(cmd!=0, wheels 0) {stall}s | max|vx| last 5s {np.abs(cm[cm[:,0] > t1 - 5, 1]).max() if (cm[:,0] > t1-5).any() else 0:.2f}')
    if ev_r: print('        ', ', '.join(ev_r[:10]))
