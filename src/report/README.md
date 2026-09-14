# Laporan Pengembangan NEVO

Dokumentasi terorganisir berdasarkan topik dari semua sesi pengembangan, masalah, dan perbaikan untuk robot mobil otonom NEVO.

**Platform:** Raspberry Pi 5, Ubuntu 24.04, ROS2 Jazzy
**Rentang tanggal:** 2026-08-22 hingga 2026-08-30

---

## Topik

| # | Topik | Jumlah Masalah | Deskripsi |
|---|-------|----------------|-----------|
| 01 | [Navigasi & Kontroler](01-navigation-controller/) | 16 | Tuning MPPI, pipeline kecepatan, perencana, costmap, rintangan |
| 02 | [SLAM, Lokalisasi & TF](02-slam-localization-tf/) | 9 | EKF, AMCL, SLAM Toolbox, pohon TF, initial pose |
| 03 | [Lifecycle, Launch & Startup](03-lifecycle-launch-startup/) | 7 | Urutan spawner, lifecycle activator, cold boot |
| 04 | [Cloud & WebSocket](04-cloud-websocket/) | 6 | Connection stacking WS, BATTERY JSON, UI cloud |
| 05 | [DDS & Jaringan](05-dds-networking/) | 6 | Migrasi FastDDS, unicast peers, domain ID |
| 06 | [Hardware, Sensor & Daya](06-hardware-sensors-power/) | 10 | RPLidar, INA226, USB/udev, daya Pi5 |
| 07 | [Build, Lingkungan & Konfigurasi](07-build-environment-config/) | 11 | Path workspace, sudoers, pembaruan msg, konfig sistem |

**Total: 65 masalah dalam 7 topik**

---

## Arsip

Laporan sesi berbasis tanggal asli dipertahankan di [archive/](archive/):

- `velocity_fix_report.md` — 2026-08-22
- `2026-08-24-session-report.md` — Migrasi DDS, SLAM, mode swap
- `2026-08-25-session-report.md` — Perbaikan WebSocket, SoH Arduino
- `2026-08-25-session-report-evening.md` — Tuning MPPI, lifecycle
- `2026-08-26-session-report.md` — Penemuan unicast DDS
- `NEVO_NAVIGATION_SETUP_REPORT.md` — Pengaturan nav stack lengkap 2026-08-30

---

## Referensi Cepat — Nilai Konfigurasi Kunci

| Parameter | Nilai | File |
|-----------|-------|------|
| MPPI `ax_max` | 15.0 | `controller_mppi_params.yaml` |
| MPPI `vx_max` | 0.25 m/s | `controller_mppi_params.yaml` |
| MPPI `wz_max` | 0.5 rad/s | `controller_mppi_params.yaml` |
| MPPI `model_dt` | 0.2 | `controller_mppi_params.yaml` |
| MPPI `open_loop` | true | `controller_mppi_params.yaml` |
| velocity_smoother `max_accel` | [0.15, 0.0, 0.4] | `controller_mppi_params.yaml` |
| velocity_smoother `use_stamped_vel` | false | `controller_mppi_params.yaml` |
| controller_server frequency | 5.0 Hz | `controller_mppi_params.yaml` |
| EKF frequency | 15.0 Hz | `ekf.yaml` |
| diff_drive `angular.z.max_velocity` | 0.5 rad/s | `my_controllers.yaml` |
| diff_drive `linear.x.max_acceleration` | 1.5 | `my_controllers.yaml` |
| twist_mux nav timeout | 0.3s | config `twist_mux` |
| rplidar respawn | true, delay 3.0s | `rplidar_a1_launch.py` |
| diagnostic `grace_period` | 30s | `launch_robot.launch.py` |
| diagnostic `global_timeout` | 300s | `launch_robot.launch.py` |
| ROS_DOMAIN_ID | 7 | File launch |
| DDS | rmw_fastrtps_cpp | Env sistem |
