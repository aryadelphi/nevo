# 05 — DDS & Jaringan

Migrasi middleware DDS, konfigurasi FastDDS, penemuan unicast, dan pengaturan domain ROS2.

**Laporan:** 2026-08-24, 2026-08-26, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [fastdds-config.md](fastdds-config.md) | Migrasi CycloneDDS→FastDDS, unicast peers, domain ID, SHM |

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | Kekurangan participant-index CycloneDDS setelah mode swap | 2026-08-24 |
| 2 | Cloud domain dikembalikan ke tifa-* (record DNS nevo-* tidak ada) | 2026-08-24 |
| 3 | Mesin dev tidak bisa menemukan topik robot via WiFi hotspot | 2026-08-26 |
| 4 | ROS_DOMAIN_ID tidak survive respawn rplidar | 2026-08-24 |
| 5 | Error port SHM FastDDS (kosmetik) | NEVO_NAVIGATION |
| 6 | Multicast diblokir di WiFi hotspot (ap0) | 2026-08-26 |

## Masalah yang Diketahui

- Endpoint cloud masih menggunakan domain `tifa-*` — menunggu pembuatan record DNS untuk `nevo-*`
