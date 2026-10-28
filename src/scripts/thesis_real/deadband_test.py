#!/usr/bin/env python3
"""Motor deadband test for nevo: small speed steps from standstill, measured by the wheel encoders.

Run on nevo with the stack up (nav mode is fine), NO Nav2 goal active, teleop closed. Publishes
/cmd_vel (twist_mux teleop input, beats Nav2), so keep a clear area: ~1 m forward/back, in-place turns.

  ~/thesis_real/deadband_test.py              # full ladder, ~3.5 min
  ~/thesis_real/deadband_test.py --quick      # every other step, ~2 min

Sequence: linear 0.01..0.10 m/s (fwd then back), then angular 0.02..0.20 rad/s (CCW then CW).
Each step: 3 s at the command from a standstill, 2 s zero. Measured over the last 2 s of each step:
wheel speeds from /joint_states (encoders), body yaw rate from /imu/data_raw.
Output: table + thresholds, CSV ~/thesis_real/deadband_<time>.csv. Ctrl+C stops the robot.
"""
import argparse, csv, math, os, time
import numpy as np, rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu

R, B = 0.0660, 0.401                 # my_controllers.yaml wheel_radius / wheel_separation
COUNTS_PER_REV, PID_RATE = 2000, 30  # ros2_control.xacro enc_counts_per_rev / loop_rate
MOVE_FRAC = 0.5                      # "moved" = measured >= 50 % of the commanded wheel speed


def counts_per_frame(wheel_rad_s):
    return wheel_rad_s / (2 * math.pi) * COUNTS_PER_REV / PID_RATE


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--quick', action='store_true'); a = ap.parse_args()
    lin = [round(0.01 * i, 2) for i in range(1, 11)]; ang = [round(0.02 * i, 2) for i in range(1, 11)]
    if a.quick:
        lin, ang = lin[::2] + [0.10], ang[::2] + [0.20]
    steps = [('lin', s * v) for v in lin for s in (1, -1)] + [('ang', s * w) for w in ang for s in (1, -1)]
    rclpy.init(); n = rclpy.create_node('deadband_test')
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    st = {'wl': [], 'wr': [], 'gz': []}; names = {}

    def on_js(m):
        for i, nm in enumerate(m.name):
            if 'left' in nm and i < len(m.velocity): st['wl'].append((time.time(), m.velocity[i])); names['l'] = nm
            if 'right' in nm and i < len(m.velocity): st['wr'].append((time.time(), m.velocity[i])); names['r'] = nm
    n.create_subscription(JointState, '/joint_states', on_js, 50)
    n.create_subscription(Imu, '/imu/data_raw', lambda m: st['gz'].append((time.time(), m.angular_velocity.z)), 50)

    def hold(vx, wz, secs):
        end = time.time() + secs; msg = Twist(); msg.linear.x = float(vx); msg.angular.z = float(wz)
        while time.time() < end:
            pub.publish(msg); rclpy.spin_once(n, timeout_sec=0.05)

    def mean_since(key, t0):
        v = [x for t, x in st[key] if t >= t0]
        return float(np.mean(v)) if v else float('nan')

    hold(0, 0, 2.0)
    if not st['wl'] or not st['wr']:
        print('no wheel velocities on /joint_states: is the stack up?'); return
    print(f"wheels: {names.get('l')} / {names.get('r')}; {len(steps)} steps, ~{len(steps) * 5 / 60:.1f} min. Ctrl+C stops.")
    rows = []
    try:
        for kind, val in steps:
            vx, wz = (val, 0.0) if kind == 'lin' else (0.0, val)
            wl_cmd = (vx - wz * B / 2) / R; wr_cmd = (vx + wz * B / 2) / R          # rad/s per wheel
            hold(vx, wz, 1.0); t_meas = time.time(); hold(vx, wz, 2.0)
            wl, wr, gz = mean_since('wl', t_meas), mean_since('wr', t_meas), mean_since('gz', t_meas)
            hold(0, 0, 2.0)
            ml = abs(wl) >= MOVE_FRAC * abs(wl_cmd) and np.sign(wl) == np.sign(wl_cmd)
            mr = abs(wr) >= MOVE_FRAC * abs(wr_cmd) and np.sign(wr) == np.sign(wr_cmd)
            row = dict(kind=kind, cmd=val, wl_cmd=round(wl_cmd, 3), wr_cmd=round(wr_cmd, 3), wl=round(wl, 3), wr=round(wr, 3),
                       cpf_l=round(counts_per_frame(wl_cmd), 2), cpf_r=round(counts_per_frame(wr_cmd), 2),
                       moved_l=int(ml), moved_r=int(mr), imu_wz=round(gz, 3))
            rows.append(row)
            unit = 'm/s' if kind == 'lin' else 'rad/s'
            print(f"{kind} {val:+.2f} {unit:5s} | wheel cmd L {wl_cmd:+.3f} R {wr_cmd:+.3f} rad/s "
                  f"({row['cpf_l']:+.2f}/{row['cpf_r']:+.2f} counts/frame) | meas L {wl:+.3f} R {wr:+.3f} | "
                  f"moved L {'Y' if ml else '-'} R {'Y' if mr else '-'} | imu wz {gz:+.3f}", flush=True)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        for _ in range(5):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
    if not rows:
        return
    out = os.path.expanduser(f'~/thesis_real/deadband_{time.strftime("%m%d_%H%M%S")}.csv')
    with open(out, 'w', newline='') as fh:
        w = csv.DictWriter(fh, list(rows[0])); w.writeheader(); w.writerows(rows)
    print(f'\nsaved {out}\n-- smallest command that moved (both wheels):')
    for kind, unit in (('lin', 'm/s'), ('ang', 'rad/s')):
        for sgn, lab in ((1, '+'), (-1, '-')):
            ok = [r['cmd'] for r in rows if r['kind'] == kind and np.sign(r['cmd']) == sgn and r['moved_l'] and r['moved_r']]
            first = min(ok, key=abs) if ok else None
            print(f'  {kind} {lab}: {first if first is not None else "none in range"} {unit}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
