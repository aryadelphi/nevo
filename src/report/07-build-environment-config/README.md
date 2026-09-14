# 07 — Build, Lingkungan & Konfigurasi Sistem

Pengaturan workspace, migrasi path, variabel lingkungan, definisi pesan, dan konfigurasi level sistem.

**Laporan:** 2026-08-24, 2026-08-25, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [workspace-paths.md](workspace-paths.md) | Rename workspace, migrasi path hardcoded |
| [system-config.md](system-config.md) | .zshrc, sudoers, QoS, pembaruan msg, peta default |

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | Rename workspace: tifa→nevo, docking_ws→nevo_ws | 2026-08-24 |
| 2 | ~15 file dengan path hardcoded /home/rspx1/nevo_ws/ | NEVO_NAVIGATION |
| 3 | .zshrc kehilangan env vars ROS2 (sesi SSH kosong) | NEVO_NAVIGATION |
| 4 | Sudoers untuk systemd-run (restart robot_api_node) | NEVO_NAVIGATION |
| 5 | NevoTelemetry.msg: menambahkan soh, internal_resistance_ohm, cycle_count | 2026-08-25 |
| 6 | Parser ina226_bridge: 10 → 13 field | 2026-08-25 |
| 7 | Ketidakcocokan QoS /map node diagnostik | NEVO_NAVIGATION |
| 8 | Peta default diubah ke abie_lab_save.yaml | NEVO_NAVIGATION |
| 9 | Peringatan sourcing .setup.sh (kosmetik) | NEVO_NAVIGATION |
| 10 | Overrun controller_manager di Pi5 (loop 50Hz) | NEVO_NAVIGATION |
| 11 | BATTERY JSON disesuaikan dengan format TIFA | 2026-08-25 |
