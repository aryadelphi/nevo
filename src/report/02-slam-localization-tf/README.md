# 02 — SLAM, Lokalisasi & Pohon TF

Konfigurasi EKF, tuning AMCL, pengaturan SLAM Toolbox, arsitektur pohon TF, dan manajemen initial pose.

**Laporan:** 2026-08-24, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [ekf-tf-tree.md](ekf-tf-tree.md) | Frekuensi EKF, odom TF, base_frame, pohon TF lengkap |
| [amcl.md](amcl.md) | Update stasioner AMCL, transform_tolerance, QoS costmap |
| [initial-pose.md](initial-pose.md) | Auto initial pose saat ganti peta, publisher berkelanjutan, konfirmasi TF |

---

## Pohon TF (Final)

```
map → odom (SLAM/AMCL)
  └─→ odom → base_footprint (EKF, 15Hz)
        └─→ base_footprint → base_link (RSP, statis)
              └─→ base_link → chassis (RSP, statis)
              │     ├─→ chassis → laser_frame (RSP, statis)
              │     ├─→ chassis → camera_link (RSP, statis)
              │     ├─→ chassis → c310_camera_link (RSP, statis)
              │     ├─→ chassis → rear_caster_wheel (RSP, statis)
              │     └─→ chassis → back_caster_wheel (RSP, statis)
              ├─→ base_link → left_wheel (joint_states, ~15Hz)
              └─→ base_link → right_wheel (joint_states, ~15Hz)
```

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | SLAM membuang scan — pohon TF terpisah dari ketidakcocokan EKF | 2026-08-24 |
| 2 | EKF mempublikasikan odom→base_footprint pada 30Hz (sekarang 15Hz) | 2026-08-24 |
| 3 | Ketidakcocokan base_frame SLAM Toolbox dengan output EKF | 2026-08-24 |
| 4 | AMCL tidak update saat robot stasioner | NEVO_NAVIGATION |
| 5 | TF extrapolation 2D Goal Pose (timestamp goal stale) | NEVO_NAVIGATION |
| 6 | Tidak ada initial pose saat ganti peta | 2026-08-25-evening |
| 7 | EKF pada 30Hz membanjiri RPi5 | NEVO_NAVIGATION |
| 8 | Error rate EKF sesekali (spike 30-190ms) | NEVO_NAVIGATION |
| 9 | Race condition SLAM Toolbox dengan startup diff_cont | NEVO_NAVIGATION |
