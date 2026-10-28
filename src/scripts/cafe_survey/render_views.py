#!/usr/bin/env python3
"""Render quick-look PNGs of a survey cloud.ply (no viewer needed).

  python3 render_views.py cloud.ply [--closeup X Y] [--title TEXT]

Writes next to the PLY:
  view_top.png       top view, 1 m grid, colour = height
  view_overview.png  oblique 3D view of the whole cloud
  view_closeup.png   oblique view of a 2 x 2 m area around (X, Y), floor removed
"""
import argparse
import math
import os

import cv2
import numpy as np

STOPS = np.array([[0.20, 0.30, 0.75], [0.20, 0.65, 0.75], [0.35, 0.75, 0.35],
                  [0.95, 0.80, 0.25], [0.90, 0.35, 0.20]])


def colors(z, zmax=1.6):
    x = np.clip(z / zmax, 0, 1) * (len(STOPS) - 1)
    k = np.minimum(x.astype(int), len(STOPS) - 2)
    f = (x - k)[:, None]
    return STOPS[k] * (1 - f) + STOPS[k + 1] * f


def load_ply(path):
    d = open(path, 'rb').read()
    i = d.index(b'end_header\n') + len(b'end_header\n')
    dt = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
    if b'property uchar red' in d[:i]:
        dt += [('r', 'u1'), ('g', 'u1'), ('b', 'u1')]
    v = np.frombuffer(d[i:], dtype=dt)
    return np.stack([v['x'], v['y'], v['z']], 1).astype(np.float64)


def legend(img):
    h, w = img.shape[:2]
    for j in range(200):
        c = (colors(np.array([j / 200 * 1.6]))[0][::-1] * 255).astype(int).tolist()
        cv2.line(img, (w - 40, h - 30 - j), (w - 20, h - 30 - j), c, 1)
    for v in (0, 0.5, 1.0, 1.5):
        cv2.putText(img, '%.1f m' % v, (w - 105, h - 26 - int(v / 1.6 * 200)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)


def perspective(P, eye, target, fov, W, H, splat, title, path):
    eye, tg = np.asarray(eye, float), np.asarray(target, float)
    f = tg - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1])
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    V = P - eye
    xc, yc, zc = V @ r, V @ u, V @ f
    ok = zc > 0.2
    xc, yc, zc, z = xc[ok], yc[ok], zc[ok], P[ok, 2]
    F = 0.5 * W / math.tan(math.radians(fov) / 2)
    px = (W / 2 + F * xc / zc).astype(int)
    py = (H / 2 - F * yc / zc).astype(int)
    img = np.full((H, W, 3), (0.125, 0.11, 0.11))
    col = colors(z) * np.clip(1.15 - 0.06 * zc, 0.45, 1.0)[:, None]
    o = np.argsort(-zc)  # far first, near points overwrite
    for dx in range(-splat, splat + 1):
        for dy in range(-splat, splat + 1):
            X, Y = px[o] + dx, py[o] + dy
            m = (X >= 0) & (X < W) & (Y >= 0) & (Y < H)
            img[Y[m], X[m]] = col[o][m]
    img = (img[:, :, ::-1] * 255).clip(0, 255).astype(np.uint8)
    cv2.putText(img, title, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (235, 235, 235), 2,
                cv2.LINE_AA)
    legend(img)
    cv2.imwrite(path, img)


def top_view(P, path, res=0.01):
    Q = P[(P[:, 2] > 0.03) & (P[:, 2] < 2.0)]
    x0, y0 = np.percentile(Q[:, 0], 0.5) - 0.3, np.percentile(Q[:, 1], 0.5) - 0.3
    x1, y1 = np.percentile(Q[:, 0], 99.5) + 0.3, np.percentile(Q[:, 1], 99.5) + 0.3
    W, H = int((x1 - x0) / res), int((y1 - y0) / res)
    img = np.full((H, W, 3), 255, np.uint8)
    for g in np.arange(math.ceil(x0), x1):
        img[:, int((g - x0) / res)] = (215, 215, 215)
    for g in np.arange(math.ceil(y0), y1):
        img[H - 1 - int((g - y0) / res), :] = (215, 215, 215)
    o = np.argsort(Q[:, 2])  # high points drawn last
    X = ((Q[o, 0] - x0) / res).astype(int)
    Y = (H - 1 - (Q[o, 1] - y0) / res).astype(int)
    m = (X >= 0) & (X < W) & (Y >= 0) & (Y < H)
    img[Y[m], X[m]] = (colors(Q[o, 2])[m][:, ::-1] * 255).astype(np.uint8)
    ox, oy = int(-x0 / res), H - 1 - int(-y0 / res)  # cross = map origin
    cv2.drawMarker(img, (ox, oy), (0, 0, 0), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(img, 'Top view, 1 m grid, colour = height, + = map origin (robot start)',
                (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
    cv2.imwrite(path, img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--closeup', type=float, nargs=2, metavar=('X', 'Y'))
    ap.add_argument('--title', default='3D overview (Kinect + lidar SLAM)')
    ap.add_argument('--top-res', type=float, default=0.01, help='m per pixel of the top view')
    a = ap.parse_args()
    P = load_ply(a.ply)
    out = os.path.dirname(os.path.abspath(a.ply))
    top_view(P, os.path.join(out, 'view_top.png'), a.top_res)
    # frame the whole cloud: look at the bounding-box centre from a distance that
    # scales with its size (a 10 m lab and a 100 m building both fit)
    lo, hi = np.percentile(P[:, :2], 1, axis=0), np.percentile(P[:, :2], 99, axis=0)
    c, size = (lo + hi) / 2, float(np.max(hi - lo))
    d = max(6.0, 0.75 * size)
    perspective(P, [c[0] - 0.7 * d, c[1] - 0.6 * d, 0.6 * d], [c[0], c[1], 0.3], 60, 1600, 1000,
                1, a.title, os.path.join(out, 'view_overview.png'))
    if a.closeup:
        cx, cy = a.closeup
        Q = P[(abs(P[:, 0] - cx) < 1.05) & (abs(P[:, 1] - cy) < 1.05) & (P[:, 2] > 0.04)]
        perspective(Q, [cx - 1.9, cy - 1.6, 1.9], [cx, cy, 0.3], 50, 1400, 1000, 2,
                    'Close-up around (%.1f, %.1f)' % (cx, cy),
                    os.path.join(out, 'view_closeup.png'))
    print('wrote view_*.png in', out)


if __name__ == '__main__':
    main()
