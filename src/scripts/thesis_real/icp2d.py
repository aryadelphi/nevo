"""2D point-to-point ICP in plain numpy (the Pi has no scipy).

Used by real_bench.py to measure, from the lidar alone, where the robot is relative to a taught
pose: reference scan taken with the robot on the cross, current scan before/after a run.
On nevo the lidar sits at the base_link origin (chassis -0.315 + lidar +0.315, no rotation),
so scan points are robot-frame points and the result is directly the tape convention:
x forward along the taught heading, y left (the robot's left), yaw + = turned left.

Speed: RefIndex precomputes, on a 3 cm grid around the reference, the index of the nearest
reference point (once, at teach time). Each ICP iteration is then a table lookup instead of a
full N x M distance search (that took ~20 s per match on the laptop).
"""
import math
import numpy as np


def scan_points(ranges, angle_min, angle_inc, rmin=0.15, rmax=8.0):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r > rmin) & (r < rmax)
    return np.c_[r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]


def voxel(p, cell=0.02):
    """One point per cell: keeps the cost bounded when scans are stacked."""
    if len(p) == 0:
        return p
    k = np.floor(p / cell).astype(np.int64)
    _, idx = np.unique(k[:, 0] * 1000003 + k[:, 1], return_index=True)
    return p[np.sort(idx)]


class RefIndex:
    """Nearest-reference-point lookup grid. cell 3 cm, margin = how far outside the reference
    a query point may be and still get a correspondence."""

    def __init__(self, ref, cell=0.03, margin=0.5, chunk=4000):
        self.ref = np.asarray(ref, dtype=float)
        self.cell = cell
        self.o = self.ref.min(0) - margin
        hi = self.ref.max(0) + margin
        self.shape = tuple(np.ceil((hi - self.o) / cell).astype(int) + 1)
        gx, gy = np.meshgrid(np.arange(self.shape[0]), np.arange(self.shape[1]), indexing='ij')
        centres = np.c_[gx.ravel(), gy.ravel()] * cell + self.o + cell / 2
        idx = np.empty(len(centres), dtype=np.int32)
        for i in range(0, len(centres), chunk):
            c = centres[i:i + chunk]
            idx[i:i + chunk] = ((c[:, None, :] - self.ref[None, :, :]) ** 2).sum(-1).argmin(1)
        self.idx = idx.reshape(self.shape)

    def save(self, path):
        np.savez_compressed(path, ref=self.ref, idx=self.idx, o=self.o, cell=self.cell)

    @classmethod
    def load(cls, path):
        z = np.load(path); self = cls.__new__(cls)
        self.ref, self.idx, self.o, self.cell = z['ref'], z['idx'], z['o'], float(z['cell'])
        self.shape = self.idx.shape
        return self

    def nearest(self, p):
        """(distance, ref index) per point; distance inf where p is outside the grid."""
        k = np.floor((p - self.o) / self.cell).astype(int)
        inside = (k[:, 0] >= 0) & (k[:, 1] >= 0) & (k[:, 0] < self.shape[0]) & (k[:, 1] < self.shape[1])
        j = np.zeros(len(p), dtype=np.int64); d = np.full(len(p), np.inf)
        j[inside] = self.idx[k[inside, 0], k[inside, 1]]
        d[inside] = np.hypot(*(p[inside] - self.ref[j[inside]]).T)
        return d, j


def _apply(p, x, y, th):
    c, s = math.cos(th), math.sin(th)
    return p @ np.array([[c, s], [-s, c]]) + (x, y)


def icp(cur, ref_index, guess=(0.0, 0.0, 0.0), starts_deg=(0, -8, 8, -16, 16), iters=50):
    """Pose (x, y, yaw) of the current robot frame in the reference robot frame, i.e. the
    transform T with T(cur) ~ ref. Returns dict(x, y, yaw, res_cm, inliers, ok)."""
    best = None
    for sd in starts_deg:
        x, y, th = guess[0], guess[1], guess[2] + math.radians(sd)
        for _ in range(iters):
            p = _apply(cur, x, y, th)
            d, j = ref_index.nearest(p)
            fin = np.isfinite(d)
            if fin.sum() < 20:
                break
            keep = fin & (d < max(0.05, np.percentile(d[fin], 75)))
            a, b = p[keep], ref_index.ref[j[keep]]
            am, bm = a.mean(0), b.mean(0)
            U, _, Vt = np.linalg.svd((a - am).T @ (b - bm))
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1; R = Vt.T @ U.T
            dth = math.atan2(R[1, 0], R[0, 0]); t = bm - R @ am
            x, y = R @ np.array([x, y]) + t     # compose the increment with the estimate
            th += dth
            if abs(dth) < 1e-5 and math.hypot(*t) < 1e-5:
                break
        d, _ = ref_index.nearest(_apply(cur, x, y, th))
        fin = np.isfinite(d)
        res = float(np.median(d[fin])) if fin.any() else float('inf')
        inl = float(np.mean(d < 0.05))
        if best is None or res < best['res_cm'] / 100:
            best = dict(x=float(x), y=float(y), yaw=math.atan2(math.sin(th), math.cos(th)), res_cm=100 * res, inliers=inl)
    best['ok'] = best['res_cm'] < 2.5 and best['inliers'] > 0.7
    return best
