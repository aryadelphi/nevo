#!/usr/bin/env python3
import sys, math, numpy as np
from scipy.spatial import cKDTree
z = np.load(sys.argv[1], allow_pickle=True)
import os
OLD_UP = np.array(eval(os.environ.get("CUR_UP", "[0.0784, 0.0014, 0.9969]"))); OLD_S = float(os.environ.get("CUR_S", "1.0131"))  # params the node ran with
def rot_to_up(up):
    u = up / np.linalg.norm(up); vx, vy, vz = u[1], -u[0], 0.0; c = u[2]; k = 1 / (1 + c)
    return np.array([[1-k*(vy*vy+vz*vz), k*vx*vy-vz, k*vx*vz+vy], [k*vx*vy+vz, 1-k*(vx*vx+vz*vz), k*vy*vz-vx], [k*vx*vz-vy, k*vy*vz+vx, 1-k*(vx*vx+vy*vy)]])
imu = z['imu']; bt, st = imu[:, 0], imu[:, 1]
R0 = rot_to_up(OLD_UP)
raw_g = (R0.T @ imu[:, 2:5].T).T; raw_g[:, 2] /= OLD_S           # rad/s, sensor axes
raw_a = (R0.T @ imu[:, 5:8].T).T / 9.80665                         # g, sensor axes
ph_t, ph_l = z['ph_t'], list(z['ph_l']); ph_end = list(ph_t[1:]) + [bt[-1]]
P = [(l, a, b) for l, a, b in zip(ph_l, ph_t, ph_end)]
print('== phases'); [print(f'  {l:10s} {b-a:6.1f} s') for l, a, b in P]

# ---- health: gaps on header stamps (node clock) and bag arrival
print('== IMU health')
d = np.diff(st); print(f'  {len(st)} samples, {len(st)/(st[-1]-st[0]):.2f} Hz, stamp gaps >30 ms: {(d>0.03).sum()}, >100 ms: {(d>0.1).sum()}, max {1000*d.max():.0f} ms')
for l, a, b in P:
    m = (st >= a) & (st < b); dd = np.diff(st[m])
    if m.sum() > 2: print(f'  {l:10s} {m.sum()/(b-a):6.1f} Hz  max gap {1000*dd.max():5.0f} ms  max|w| {np.abs(raw_g[m]).max():.2f} rad/s  max|a| {np.linalg.norm(raw_a[m],axis=1).max():.2f} g')
logs = [(t, n, msg) for t, n, msg in zip(z['log_t'], z['log_n'], z['log_m']) if 'imu' in n]
rej = [msg for _, _, msg in logs if 'rejected' in msg]
print(f'  imu_publisher log lines: {len(logs)}; reject reports: {len(rej)}; others:', [m[:80] for _, _, m in logs if 'rejected' not in m][:6])
if rej: print('   rejects:', rej[:5])

# ---- bias from still phases (trim 2 s)
def sel(l_want, i, trim=2.0):
    segs = [(a, b) for l, a, b in P if l == l_want]; a, b = segs[i]; return (st >= a + trim) & (st < b - trim), a, b
stills = [sel('still', i)[0] for i in range(3)]
bias = np.array([raw_g[m].mean(0) for m in stills])
print('== gyro bias per still phase (deg/s, sensor axes):'); [print('  ', np.round(np.degrees(b), 4)) for b in bias]
b0 = bias.mean(0)
g_acc = np.array([raw_a[m].mean(0) for m in stills]).mean(0)
print(f'  accel at rest (g): {np.round(g_acc,3)} |g| {np.linalg.norm(g_acc):.3f} -> tilt from accel {math.degrees(math.acos(g_acc[2]/np.linalg.norm(g_acc))):.2f} deg (accel uncalibrated)')

# ---- spin axis
def steady(l):
    m, a, b = sel(l, 0, trim=0); w = raw_g - b0; nz = np.linalg.norm(w, axis=1)
    mm = m & (nz > 0.8 * np.median(nz[m]))                         # constant-rate part only
    return w[mm].mean(0), mm
mc, mcm = steady('spin_ccw'); mw, mwm = steady('spin_cw')
up = mc - mw; up /= np.linalg.norm(up)
if up[2] < 0: up = -up
tilt = math.degrees(math.acos(up[2]))
print('== spin axis (new mount_up):', np.round(up, 4), f'tilt {tilt:.2f} deg; old {np.round(OLD_UP/np.linalg.norm(OLD_UP),4)} tilt {math.degrees(math.acos(OLD_UP[2]/np.linalg.norm(OLD_UP))):.2f} deg')
for name, mm in (('ccw', mcm), ('cw', mwm)):
    w = raw_g[mm] - b0; u = w / np.linalg.norm(w, axis=1)[:, None]; ang = np.degrees(np.arccos(np.clip(np.abs(u @ up), 0, 1)))
    print(f'  {name}: axis scatter vs fitted up: median {np.median(ang):.2f} deg, 95% {np.percentile(ang,95):.2f} deg')

# ---- ICP between still scans
def pts(i):
    r = z['scan_r'][i]; a = z['scan_a0'][i] + np.arange(len(r)) * z['scan_da'][i]; ok = np.isfinite(r) & (r > 0.15) & (r < 8)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]
def icp(A, B, th0):  # find T so that T(B) ~ A; returns yaw
    tree = cKDTree(A); best = None
    for dth in np.radians(np.arange(-20, 21, 5)):
        th, tx, ty = th0 + dth, 0.0, 0.0
        for _ in range(60):
            c, s = math.cos(th), math.sin(th); Bt = B @ np.array([[c, s], [-s, c]]) + (tx, ty)
            dist, idx = tree.query(Bt); keep = dist < max(0.05, np.percentile(dist, 80))
            p, q = Bt[keep], A[idx[keep]]; pm, qm = p.mean(0), q.mean(0)
            H = (p - pm).T @ (q - qm); U, _, Vt = np.linalg.svd(H); Rr = Vt.T @ U.T
            if np.linalg.det(Rr) < 0: Vt[1] *= -1; Rr = Vt.T @ U.T
            dth2 = math.atan2(Rr[1, 0], Rr[0, 0]); t2 = qm - Rr @ pm
            th += dth2; tx, ty = (np.array([tx, ty]) @ Rr.T + t2)
            if abs(dth2) < 1e-6: break
        c, s = math.cos(th), math.sin(th); res = np.median(cKDTree(A).query(B @ np.array([[c, s], [-s, c]]) + (tx, ty))[0])
        if best is None or res < best[0]: best = (res, th, tx, ty)
    return best
sc_t = z['scan_t']
def avg_scan_idx(a, b):
    m = np.nonzero((sc_t >= a + 1) & (sc_t < b - 1))[0]; return m[len(m) // 2]
still_ab = [(a, b) for l, a, b in P if l == 'still']
print('== spins: IMU (new axis) vs lidar ICP vs wheel odom vs rf2o')
scales = []
for k, (name, sgn) in enumerate((('spin_ccw', 1), ('spin_cw', -1))):
    s0, s1 = still_ab[k], still_ab[k + 1]
    ta, tb = (s0[0] + s0[1]) / 2, (s1[0] + s1[1]) / 2               # middle of the stills around the spin
    m = (st >= ta) & (st < tb)
    yaw_rate = (raw_g[m] - b0) @ up
    ang_imu = np.trapz(yaw_rate, st[m])
    # node's published yaw with new mount_up and scale s: row3(Rn) . diag(1,1,s) . raw   (no bias removal in the node)
    Rn = rot_to_up(up); A_ = np.trapz(raw_g[m] @ np.array([Rn[2, 0], Rn[2, 1], 0.0]), st[m]); B_ = np.trapz(raw_g[m][:, 2] * Rn[2, 2], st[m])
    i0, i1 = avg_scan_idx(*s0), avg_scan_idx(*s1)
    res, th, tx, ty = icp(pts(i0), pts(i1), ang_imu % (2 * math.pi))
    resid = math.atan2(math.sin(th), math.cos(th))
    N = round((ang_imu - resid) / (2 * math.pi)); true = N * 2 * math.pi + resid
    od = z['odom']; mo = (od[:, 0] >= ta) & (od[:, 0] < tb); ang_wh = np.trapz(od[mo, 2], od[mo, 0])
    rf = z['rf2o']; mr = (rf[:, 0] >= ta) & (rf[:, 0] < tb); ang_rf = np.trapz(rf[mr, 2], rf[mr, 0])
    s_new = (true - A_) / B_; scales.append(s_new)
    print(f'  {name}: lidar truth {math.degrees(true):9.2f} deg (ICP res {100*res:.1f} cm, shift {100*math.hypot(tx,ty):.1f} cm) | '
          f'IMU raw-axis {math.degrees(ang_imu):9.2f} ({100*(ang_imu/true-1):+.2f} %) | wheels {math.degrees(ang_wh):9.2f} ({100*(ang_wh/true-1):+.2f} %) | '
          f'rf2o {math.degrees(ang_rf):9.2f} ({100*(ang_rf/true-1):+.2f} %) | gyro_scale_z fit {s_new:.4f}')
    pub_old = np.trapz(imu[m, 4], st[m])
    print(f'      as published today (old mount_up/scale): {math.degrees(pub_old):9.2f} deg ({100*(pub_old/true-1):+.2f} %)')
s = float(np.mean(scales))
print(f'== RESULT  mount_up: [{up[0]:.4f}, {up[1]:.4f}, {up[2]:.4f}]   gyro_scale_z: {s:.4f}  (ccw {scales[0]:.4f}, cw {scales[1]:.4f}, spread {100*abs(scales[0]-scales[1])/s:.2f} %)')
print(f'   residual bias along up after calibration: {math.degrees(b0 @ up):.4f} deg/s = {math.degrees(b0 @ up)*60:.2f} deg/min')
