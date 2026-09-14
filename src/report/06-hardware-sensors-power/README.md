# 06 — Hardware, Sensor & Daya

Keandalan RPLidar, monitor baterai INA226, konfigurasi USB/udev, dan masalah catu daya.

**Laporan:** 2026-08-24, 2026-08-25, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [rplidar.md](rplidar.md) | Crash saat mode swap, pelepasan port serial, konfigurasi respawn |
| [battery-ina226.md](battery-ina226.md) | Komputasi SoH, bug kalibrasi, persistensi EEPROM |
| [usb-udev.md](usb-udev.md) | Disconnect CP2102, aturan udev, symlink |

---

## Konfigurasi Hardware

| Komponen | Perangkat | Path |
|----------|-----------|------|
| Lidar | RPLidar A1 (CP2102) | `/dev/ttyLIDAR` → `/dev/ttyUSB0` |
| Motor | Arduino (CH340) | `/dev/ttyMOTOR` → `/dev/ttyUSB1` |
| Baterai | INA226 | Dilewati (modul rusak pada unit saat ini) |
| Kamera | Kamera belakang | `/dev/video0` |
| SBC | Raspberry Pi 5 | aarch64, Ubuntu 24.04 |

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | RPLidar crash (exit 255) saat mode swap — masalah port SIGKILL | 2026-08-24 |
| 2 | CP2102 longgar menyebabkan disconnect USB acak | 2026-08-24, 2026-08-25, 2026-08-25-evening |
| 3 | Pemulihan port serial USB via symlink udev + respawn | 2026-08-24 |
| 4 | Komputasi SoH Arduino (kapasitas + resistansi) | 2026-08-25 |
| 5 | Bug kalibrasi SOH — v_oc_ref diinisialisasi ke VOLTAGE_MAX | 2026-08-25 |
| 6 | Aturan udev tidak bertahan setelah reboot | NEVO_NAVIGATION |
| 7 | Modul INA226 rusak pada robot saat ini | NEVO_NAVIGATION |
| 8 | joint_state_broadcaster kehilangan TF roda | NEVO_NAVIGATION |
| 9 | Peringatan "Reset due to low power" pada Pi5 | NEVO_NAVIGATION |
| 10 | File mesh URDF tidak di-deploy (hanya kosmetik) | 2026-08-24 |

## Masalah yang Diketahui

- **Adaptor CP2102 longgar:** Masalah hardware — perlu dipasang ulang atau diamankan secara fisik
- **INA226 rusak:** Modul tidak fungsional pada unit saat ini, dilewati di konfigurasi
