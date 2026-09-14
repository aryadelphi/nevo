# RPLidar

Keandalan RPLidar A1 — penanganan crash, pelepasan port serial, dan konfigurasi respawn.

**Laporan:** 2026-08-24, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Masalah: Crash Saat Mode Swap (exit 255)

**Akar masalah:** `nevo_restart.sh` membunuh rplidar dengan SIGKILL, yang tidak melepas port serial dengan bersih. Instance rplidar baru tidak bisa membuka port.

---

## Perbaikan: Grace Period di nevo_restart.sh

- Ditambahkan grace period 2s antara hard-kill dan serial reset
- Waktu tidur serial reset ditingkatkan dari 1s ke 3s
- Waktu teardown total ditingkatkan untuk pelepasan port yang lebih bersih

---

## Perbaikan: Auto-Respawn (rplidar_a1_launch.py)

```python
# Ditambahkan ke launch rplidar
respawn=True
respawn_delay=3.0
```

Juga ditambahkan env vars FastDDS (`SetEnvironmentVariable`) agar node respawn menggunakan DDS yang benar, dan `ROS_DOMAIN_ID=7` di-hardcode di launch agar survive respawn.

---

## Konfigurasi Hardware

| Komponen | Perangkat | Path |
|----------|-----------|------|
| Lidar | RPLidar A1 (CP2102) | `/dev/ttyLIDAR` → `/dev/ttyUSB0` |

---

## Masalah yang Diketahui

- **Adaptor CP2102 longgar:** Masalah hardware menyebabkan disconnect USB intermiten — perlu dipasang ulang atau diamankan secara fisik
- Symlink udev `/dev/ttyLIDAR` auto-update ke ttyUSB baru saat disconnect
- Pemulihan manual jika respawn gagal: `sudo kill -9 <rplidar_pid>` memicu respawn

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `rplidar_a1_launch.py` | respawn=True, respawn_delay=3.0, env vars FastDDS |
| `nevo_restart.sh` | Grace period, timing serial reset |
