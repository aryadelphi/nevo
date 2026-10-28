#!/usr/bin/env python3
"""Thesis bench on the real robot (Bab 4 Tahap 2). Run on nevo with the stack up in nav mode.

No ground truth on the real robot. The robot's pose relative to a taught spot is measured by
lidar scan matching (ICP, icp2d.py) against a reference scan recorded when the spot was taught;
a tape measurement can be typed in as well (validation). Reference point = lidar centre
(= base_link origin on nevo). Convention: x forward along the taught heading, y = the robot's
left, yaw + = turned left (stand BEHIND the robot when reading a tape).

  teach:  put the robot exactly on the spot (AMCL settled: scan on the map walls), then
      real_bench.py teach start
      real_bench.py teach S1          # serving spot of the S1 table
    stores the AMCL pose in goals.yaml and a reference scan + lookup index in refs/<name>.npz.
  run:    N runs of one scenario; one bag per batch
      real_bench.py run --planner mppi --scenario S1 --n 5
  Per run: put the robot roughly on the start cross (within ~15 cm / 15 deg) -> Enter ->
  ICP vs the start reference gives the real start pose -> AMCL initial pose + clear local
  costmap -> goal -> wait -> ICP vs the goal reference = the measured final error ->
  Enter to accept (or type tape values too) -> one CSV row. The robot is not moved by the
  script between runs: carry/teleop it back (teleop keys OFF while a run is going).

Files (next to this script unless --dir): goals.yaml, refs/, results*.csv, bags/<planner>_<scenario>_<time>/
Bag topics follow Bab 4 (methodology paragraph) + markers on /real_bench/event for offline
metrics (deviation vs first plan, |a| max, jerk from /odometry/filtered).
"""
import argparse, collections, csv, math, os, signal, subprocess, sys, time
import numpy as np, yaml, rclpy, tf2_ros
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import icp2d  # noqa: E402

PASS_XY_CM, PASS_YAW_DEG, TIMEOUT_S = 20.0, 15.0, 120.0
REF_RANGE_M = 6.0   # reference / match points beyond this are dropped (bounded index size)
TOPICS = ['/tf', '/tf_static', '/odometry/filtered', '/diff_cont/odom', '/odom_rf2o', '/imu/data_raw',
          '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/diff_cont/cmd_vel', '/joint_states', '/plan', '/scan_filtered', '/amcl_pose',
          '/local_costmap/costmap', '/global_costmap/costmap', '/navigate_to_pose/_action/status',
          '/real_bench/event', '/rosout']
COLS = ['stamp', 'planner', 'scenario', 'run', 'success', 'status', 'T', 'meas_src', 'err_x_cm', 'err_y_cm', 'err_xy_cm',
        'err_yaw_deg', 'collision', 'recoveries', 'icp_x_cm', 'icp_y_cm', 'icp_yaw_deg', 'icp_res_cm', 'icp_inliers',
        'tape_x_cm', 'tape_y_cm', 'tape_yaw_deg', 'start_dx_cm', 'start_dy_cm', 'start_dyaw_deg', 'start_res_cm',
        'amcl_final_x', 'amcl_final_y', 'amcl_final_yaw', 'note', 'bag']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(p, d):
    """Pose d (in p's frame) expressed in the frame p lives in."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return (p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], ang(p[2] + d[2]))


def relative(p, q):
    """Pose q expressed in pose p's frame."""
    c, s = math.cos(p[2]), math.sin(p[2]); dx, dy = q[0] - p[0], q[1] - p[1]
    return (c * dx + s * dy, -s * dx + c * dy, ang(q[2] - p[2]))


class Real:
    def __init__(self):
        self.n = rclpy.create_node('real_bench')
        self.amcl = None
        self.rec = set()
        self.scans = collections.deque(maxlen=200)
        # AMCL publishes /amcl_pose latched and only after it moves (update_min_d): match its QoS,
        # and read the pose from TF map->base_link, which exists whenever AMCL runs
        self.n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.n.create_subscription(LaserScan, '/scan_filtered', lambda m: self.scans.append(
            (time.time(), icp2d.scan_points(m.ranges, m.angle_min, m.angle_increment, rmax=REF_RANGE_M))), 10)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self.n)
        for b in ('spin', 'backup', 'wait'):
            self.n.create_subscription(GoalStatusArray, f'/{b}/_action/status', lambda m, b=b: [self.rec.add((b, bytes(s.goal_info.goal_id.uuid))) for s in m.status_list], 10)
        self.init_pub = self.n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        # reliable + latched: a fresh one-off publisher lost markers under rmw_zenoh (pilot 2026-10-27)
        self.ev = self.n.create_publisher(String, '/real_bench/event', QoSProfile(
            depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        # local only: clearing the global costmap also empties its static layer for a moment and the
        # planner then failed at the start of every run ("GridBased plugin failed to plan", 2026-10-27)
        self.clear = [self.n.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')]
        self.nav = ActionClient(self.n, NavigateToPose, 'navigate_to_pose')

    def on_amcl(self, m):
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def pose(self):
        """Robot pose in map from TF (AMCL's estimate), or None."""
        try:
            tf = self.tfb.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def grab(self, secs):
        """Stacked, voxelised scan points over the next `secs` (robot must be still)."""
        t0 = time.time(); self.spin(secs)
        pts = [p for t, p in self.scans if t >= t0]
        return icp2d.voxel(np.vstack(pts), 0.03) if pts else np.zeros((0, 2))

    def event(self, s):
        self.ev.publish(String(data=s)); self.spin(0.1)

    def set_initial(self, x, y, th, sigma_xy, sigma_yaw):
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'; m.header.stamp = self.n.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = sigma_xy ** 2
        m.pose.covariance[35] = sigma_yaw ** 2
        self.init_pub.publish(m); self.spin(2.0)
        for c in self.clear:
            if c.wait_for_service(timeout_sec=2.0):
                f = c.call_async(ClearEntireCostmap.Request()); rclpy.spin_until_future_complete(self.n, f, timeout_sec=2.0)
        self.spin(2.5)   # STVL decay 2 s + a scan cycle

    def go(self, x, y, th):
        self.rec = set()
        g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        f = self.nav.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=10)
        gh = f.result()
        if gh is None or not gh.accepted:
            return 'rejected', 0.0
        t0 = self.n.get_clock().now(); rf = gh.get_result_async()
        while not rf.done():
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if (self.n.get_clock().now() - t0).nanoseconds * 1e-9 > TIMEOUT_S + 5:
                gh.cancel_goal_async(); self.spin(1.5); break
        T = (self.n.get_clock().now() - t0).nanoseconds * 1e-9
        return (rf.result().status if rf.done() else 'timeout'), T


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        return 'q'


def fmt(r):
    return f"({100 * r['x']:+.1f}, {100 * r['y']:+.1f} cm, {math.degrees(r['yaw']):+.1f} deg; match {r['res_cm']:.1f} cm, {100 * r['inliers']:.0f} % {'OK' if r['ok'] else 'POOR'})"


def open_csv(d):
    """results.csv, or results_v2.csv if an older results.csv has a different header."""
    for name in ('results.csv', 'results_v2.csv', 'results_v3.csv'):
        path = os.path.join(d, name)
        if not os.path.exists(path):
            fh = open(path, 'a', newline=''); w = csv.DictWriter(fh, COLS); w.writeheader(); return fh, w, path
        if open(path).readline().strip().split(',') == COLS:
            fh = open(path, 'a', newline=''); return fh, csv.DictWriter(fh, COLS), path
    sys.exit('no usable results CSV name left')


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest='cmd', required=True)
    t = sub.add_parser('teach'); t.add_argument('name')
    r = sub.add_parser('run'); r.add_argument('--planner', required=True); r.add_argument('--scenario', required=True)
    r.add_argument('--n', type=int, default=5); r.add_argument('--no-bag', action='store_true')
    for p in (t, r): p.add_argument('--dir', default=here)
    a = ap.parse_args()
    gpath = os.path.join(a.dir, 'goals.yaml'); rdir = os.path.join(a.dir, 'refs'); os.makedirs(rdir, exist_ok=True)
    goals = yaml.safe_load(open(gpath)) if os.path.exists(gpath) else {}
    rclpy.init(); b = Real()

    if a.cmd == 'teach':
        b.spin(3.0); ps = b.pose()
        if ps is None:
            sys.exit('no map->base_link TF: is the stack up in nav mode (AMCL active, initial pose set)?')
        ref = b.grab(2.0)
        if len(ref) < 100:
            sys.exit(f'only {len(ref)} scan points on /scan_filtered: lidar running?')
        print(f'building the reference index from {len(ref)} points (Pi: ~15 s) ...', flush=True)
        icp2d.RefIndex(ref).save(os.path.join(rdir, f'{a.name}.npz'))
        goals[a.name] = [round(ps[0], 3), round(ps[1], 3), round(ps[2], 4)]
        yaml.safe_dump(goals, open(gpath, 'w'), sort_keys=True)
        print(f'{a.name} = {goals[a.name]} + reference scan (saved). Mark the floor under the lidar centre + a heading line now.')
        return

    for k in ('start', a.scenario):
        if k not in goals:
            sys.exit(f"'{k}' not taught yet: real_bench.py teach {k}")
    refs = {}
    for k in ('start', a.scenario):
        f = os.path.join(rdir, f'{k}.npz')
        if os.path.exists(f):
            refs[k] = icp2d.RefIndex.load(f)
        else:
            print(f"  note: no reference scan for '{k}' (taught before ICP): re-teach it to get ICP "
                  f"{'start correction' if k == 'start' else 'end measurement'}; tape only for now")
    if not b.nav.wait_for_server(timeout_sec=30):
        sys.exit('navigate_to_pose not available: is the stack up in nav mode?')
    bag = None; bagdir = ''
    if not a.no_bag:
        bagdir = os.path.join(a.dir, 'bags', f'{a.planner}_{a.scenario}_{time.strftime("%m%d_%H%M%S")}')
        os.makedirs(os.path.dirname(bagdir), exist_ok=True)
        # no stdin + no keyboard controls: otherwise the recorder grabs the terminal and the
        # run prompts below read EOF (the script then quits at the first prompt)
        bag = subprocess.Popen(['ros2', 'bag', 'record', '--disable-keyboard-controls', '-o', bagdir, *TOPICS],
                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=open(bagdir + '.log', 'w'), start_new_session=True)
        b.spin(4.0)
    fh, w, csv_path = open_csv(a.dir)
    goal = tuple(goals[a.scenario]); start = tuple(goals['start'])
    print(f'{a.planner} {a.scenario}: start {list(start)} -> goal {list(goal)}, {a.n} runs -> {os.path.basename(csv_path)}. Teleop keys OFF during runs.')
    try:
        i = 0
        while i < a.n:
            i += 1
            if ask(f'\n[run {i}/{a.n}] robot roughly on the START cross, area clear? Enter = go, q = quit: ').strip().lower() == 'q':
                break
            # --- start: measured pose from ICP vs the start reference (falls back to the taught pose)
            sd = None
            if 'start' in refs:
                sd = icp2d.icp(b.grab(1.5), refs['start'])
                print(f'  start offset vs taught start: {fmt(sd)}')
                if not sd['ok'] or math.hypot(sd['x'], sd['y']) > 0.25 or abs(sd['yaw']) > math.radians(20):
                    c = ask('  start match poor or far off: Enter = re-place and retry this run, u = use the taught start anyway: ').strip().lower()
                    if c != 'u':
                        i -= 1; continue
                    sd = None
            if sd:
                init = compose(start, (sd['x'], sd['y'], sd['yaw'])); b.set_initial(*init, 0.03, math.radians(2))
            else:
                init = start; b.set_initial(*init, 0.10, math.radians(5))
            b.event(f'run_start {a.planner} {a.scenario} {i}')
            status, T = b.go(*goal)
            b.event(f'run_end {a.planner} {a.scenario} {i} status={status}')
            b.spin(1.0); am = b.pose() or (math.nan,) * 3
            print(f'  Nav2 status {status} (4=SUCCEEDED) in {T:.1f} s, recoveries {len(b.rec)}')
            # --- end: ICP vs the goal reference, initial guess from AMCL relative to the goal
            ic = None
            if a.scenario in refs:
                g0 = relative(goal, am) if am[0] == am[0] else (0.0, 0.0, 0.0)
                ic = icp2d.icp(b.grab(1.5), refs[a.scenario], guess=g0, starts_deg=(0, -8, 8))
                print(f'  final error by lidar (ICP): {fmt(ic)}   [AMCL said ({100 * g0[0]:+.1f}, {100 * g0[1]:+.1f} cm, {math.degrees(g0[2]):+.1f} deg)]')
            icp_ok = bool(ic and ic['ok'])
            if icp_ok:
                print("  Enter = accept ICP.  Or type tape 'x y [yaw]' to record it too (validation).  'c' = collision,")
            else:
                print("  ICP not available/reliable: tape needed. 'x y [yaw]' (cm, deg; yaw omitted = AMCL). 'c' = collision,")
            print("  'na' = never arrived, 's' = skip (no row), 'q' = skip + quit. Anything after the numbers = note.")
            tape = None; col = 0; note = ''; skip = quit_after = False
            while True:
                s = ask('  > ').split()
                if s and s[0] in ('s', 'q'):
                    skip = True; quit_after = s[0] == 'q'; break
                if s and s[0] == 'na':
                    col = int('c' in s[1:2]); note = ' '.join(['never arrived'] + s[1 + col:]); break
                if not s and icp_ok:
                    break
                if s and s[0] == 'c' and icp_ok:
                    col = 1; note = ' '.join(s[1:]); break
                try:
                    nums = []
                    for tok in s:
                        try:
                            nums.append(float(tok))
                        except ValueError:
                            break
                    if len(nums) < 2:
                        raise ValueError
                    nums = nums[:3]; rest = s[len(nums):]
                    col = int(bool(rest) and rest[0] == 'c'); note = ' '.join(rest[col:])
                    yaw_t = nums[2] if len(nums) == 3 else (math.degrees(relative(goal, am)[2]) if am[2] == am[2] else math.nan)
                    tape = (nums[0], nums[1], yaw_t)
                    if len(nums) == 2:
                        note = ' '.join(['tape yaw=amcl'] + ([note] if note else []))
                    break
                except (ValueError, IndexError):
                    print('  e.g. Enter (accept ICP), "4 -7 3", "12 2 -5 c bumped chair", "na stuck at table", s, q')
            if skip:
                print('  skipped (no row)')
                if quit_after:
                    break
                continue
            if icp_ok:
                ex, ey, eyaw, src = 100 * ic['x'], 100 * ic['y'], math.degrees(ic['yaw']), 'icp'
            elif tape:
                ex, ey, eyaw, src = (*tape, 'tape')
            else:
                ex = ey = eyaw = math.nan; src = 'none'
            exy = math.hypot(ex, ey)
            ok = int(status == 4 and exy <= PASS_XY_CM and abs(eyaw) <= PASS_YAW_DEG and not col and T <= TIMEOUT_S)
            r2 = lambda v: round(v, 1) if v == v else v
            w.writerow(dict(stamp=time.strftime('%F %T'), planner=a.planner, scenario=a.scenario, run=i, success=ok,
                            status=status, T=round(T, 2), meas_src=src, err_x_cm=r2(ex), err_y_cm=r2(ey), err_xy_cm=r2(exy),
                            err_yaw_deg=r2(eyaw), collision=col, recoveries=len(b.rec),
                            icp_x_cm=r2(100 * ic['x']) if ic else '', icp_y_cm=r2(100 * ic['y']) if ic else '',
                            icp_yaw_deg=r2(math.degrees(ic['yaw'])) if ic else '', icp_res_cm=r2(ic['res_cm']) if ic else '',
                            icp_inliers=round(ic['inliers'], 2) if ic else '',
                            tape_x_cm=tape[0] if tape else '', tape_y_cm=tape[1] if tape else '', tape_yaw_deg=r2(tape[2]) if tape else '',
                            start_dx_cm=r2(100 * sd['x']) if sd else '', start_dy_cm=r2(100 * sd['y']) if sd else '',
                            start_dyaw_deg=r2(math.degrees(sd['yaw'])) if sd else '', start_res_cm=r2(sd['res_cm']) if sd else '',
                            amcl_final_x=round(am[0], 3), amcl_final_y=round(am[1], 3), amcl_final_yaw=round(math.degrees(am[2]), 1),
                            note=note, bag=os.path.basename(bagdir)))
            fh.flush()
            extra = f'; tape ({tape[0]:+.1f}, {tape[1]:+.1f}, {tape[2]:+.1f})' if tape and icp_ok else ''
            print(f'  -> {"PASS" if ok else "FAIL"}  ({exy:.1f} cm, {eyaw:+.1f} deg, {src}{extra})')
    except KeyboardInterrupt:
        print('\ninterrupted (no row for an unfinished run)')
    finally:
        fh.close()
        if bag:
            os.killpg(bag.pid, signal.SIGINT); bag.wait(timeout=20)
            print(f'bag: {bagdir}')
        if rclpy.ok():  # Ctrl+C already shut the context down
            rclpy.shutdown()


if __name__ == '__main__':
    main()
