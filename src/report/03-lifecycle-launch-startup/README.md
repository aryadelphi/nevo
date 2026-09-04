# 03 — Lifecycle, Launch & Keandalan Startup

Manajemen lifecycle node ROS2, urutan file launch, keandalan cold boot, dan timeout diagnostik.

**Laporan:** 2026-08-24, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Sub-topik

| File | Fokus |
|------|-------|
| [spawner-ordering.md](spawner-ordering.md) | Race joint_state_broadcaster vs diff_cont, penjadwalan TimerAction |
| [lifecycle-activator.md](lifecycle-activator.md) | Loop discovery node, retry, aktivasi map_server + AMCL |
| [cold-boot.md](cold-boot.md) | Grace period, timeout diagnostik, status diff_cont unconfigured |

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | diff_cont macet di unconfigured saat cold boot | 2026-08-24 |
| 2 | Timeout node diagnostik terlalu ketat (restart loop) | 2026-08-24 |
| 3 | joint_state_broadcaster FATAL — konflik spawn simultan | NEVO_NAVIGATION |
| 4 | Lifecycle activator sleep buta 15s terlalu tidak andal | 2026-08-25-evening |
| 5 | Duplikasi registrasi ReinitializeGlobalLocalization crash | 2026-08-25-evening |
| 6 | Node publish_initial_pose redundan di launch | 2026-08-25-evening |
| 7 | Timeout server navigasi belum dituning untuk Pi5 | NEVO_NAVIGATION |
