# 01 — Navigasi & Tuning Kontroler

Semua penyesuaian parameter kontroler MPPI, konfigurasi pipeline kecepatan, pemilihan perencana, pengaturan costmap, dan penyesuaian penghindaran rintangan.

**Laporan:** velocity_fix_report, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [mppi-tuning.md](mppi-tuning.md) | Bobot critic MPPI, batch size, temperature, pengurangan swaying |
| [planner-costmap.md](planner-costmap.md) | NavFn vs SMAC Hybrid, STVL, inflasi costmap, penghindaran rintangan |
| [velocity-pipeline.md](velocity-pipeline.md) | velocity_smoother, twist_mux, diff_drive_controller, akar masalah ax_max |

---

## Insight Kunci

**`ax_max` pada MPPI adalah batas ruang pencarian optimizer, bukan batas fisik robot.** Batas fisik diterapkan oleh velocity_smoother dan diff_drive_controller downstream. Mengatur `ax_max` terlalu rendah membatasi kecepatan yang bisa dipertimbangkan MPPI — robot secara fisik tidak bisa mencapai kecepatan lebih tinggi karena optimizer tidak pernah menghasilkan trajektori yang melakukannya.

**Referensi:** Nav2 GitHub Issue #5694 — "Max Speed Capped at 0.17 m/s after ROS 2 Jazzy Migration"

---

## Ringkasan Perubahan (semua sub-topik digabungkan)

### Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | Robot macet di ~5% kecepatan max (ax_max MPPI terlalu rendah) | velocity_fix |
| 2 | Ketidakcocokan use_stamped_vel antara controller_server dan velocity_smoother | velocity_fix |
| 3 | SMAC Planner Hybrid menggantung planner_server (Reeds-Shepp terlalu berat untuk Pi5) | 2026-08-25-evening |
| 4 | Robot berayun berlebihan selama navigasi | 2026-08-25-evening |
| 5 | Penghindaran rintangan terlalu agresif (berbelok) | 2026-08-25-evening |
| 6 | Tuning kecepatan/akselerasi — beberapa iterasi | 2026-08-25-evening |
| 7 | Timeout twist mux nav terlalu tinggi (teleop tidak bisa mengambil alih) | 2026-08-25-evening |
| 8 | Ketidakcocokan kecepatan angular diff_drive | 2026-08-25-evening |
| 9 | transform_tolerance MPPI terlalu rendah (error TF extrapolation) | NEVO_NAVIGATION |
| 10 | Performa RPi5 overload (EKF 30Hz + kontroler 10Hz) | NEVO_NAVIGATION |
| 11 | Ketidakcocokan model_dt dengan controller period (crash FATAL) | NEVO_NAVIGATION |
| 12 | bt_navigator default_server_timeout terlalu rendah | NEVO_NAVIGATION |
| 13 | Kedipan costmap STVL dan noise kamera kedalaman | velocity_fix |
| 14 | Inflasi radius local costmap tidak cocok dengan footprint robot | velocity_fix |
| 15 | costmap_update_timeout perencana | NEVO_NAVIGATION |
| 16 | PreferForwardCritic tidak ada | velocity_fix |
