# EKF & Pohon TF

Konfigurasi Extended Kalman Filter, arsitektur pohon TF, dan manajemen odom frame.

**Laporan:** 2026-08-24, NEVO_NAVIGATION_SETUP_REPORT

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

## Konfigurasi EKF

| Parameter | Asli | Final | Alasan |
|-----------|------|-------|--------|
| `frequency` | 30.0 | **15.0** Hz | RPi5 tidak bisa mempertahankan 30Hz (terus-menerus "Failed to meet update rate") |
| `base_link_frame` | base_footprint | **base_footprint** (dipertahankan) | Sesuai output RSP |

**Keputusan kunci:** `enable_odom_tf: false` pada diff_drive_controller — EKF mempublikasikan `odom → base_footprint` sebagai gantinya. Ini menghindari pohon TF terpisah.

---

## Masalah: Pohon TF Terpisah (2026-08-24)

**Akar masalah:** `enable_odom_tf: false` pada diff_drive_controller berarti tidak ada TF `odom → base_link` dari controller. Config EKF menggunakan `base_link_frame: base_footprint` yang menciptakan pohon terpisah.

**Perbaikan:**
- EKF mempublikasikan `odom → base_footprint` pada 30Hz (sekarang 15Hz)
- RSP mempublikasikan `base_footprint → base_link → chassis → laser_frame`
- SLAM `base_frame: base_footprint` (sesuai output EKF)

---

## Konfigurasi SLAM Toolbox

| Parameter | Nilai | Catatan |
|-----------|-------|---------|
| `base_frame` | base_footprint | Harus sesuai output EKF |
| `enable_odom_tf` | false | EKF menangani odom TF |

---

## Masalah yang Diketahui

- Error rate EKF sesekali dengan spike 30-190ms — bisa dikurangi ke 10Hz jika perlu
- EKF "Failed to meet update rate" saat burst startup — non-fatal

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `ekf.yaml` | frequency: 15.0 |
| `controller_mppi_params.yaml` | update_min_d/a AMCL, transform_tolerance |
| `mapper_params_online_async.yaml` | base_frame terverifikasi |
