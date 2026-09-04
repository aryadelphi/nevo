# Cold Boot Reliability

Timeout node diagnostik, timing startup, dan pemulihan dari kondisi cold boot.

**Laporan:** 2026-08-24, NEVO_NAVIGATION_SETUP_REPORT

---

## Masalah: Timeout Loop Node Diagnostik

Pada cold boot setelah reboot, timeout node diagnostik 120s terlalu ketat. AMCL aktif terlambat (lifecycle activator menunggu 15s + map_server + setup AMCL), menyebabkan restart loop.

---

## Perbaikan: Timeout Ditingkatkan (launch_robot.launch.py)

| Parameter | Lama | Baru | Alasan |
|-----------|------|------|--------|
| `grace_period_sec` | 10 | **30** | Jendela ignore lebih panjang saat boot |
| `global_timeout_sec` | 120 | **300** | Waktu cukup untuk inisialisasi cold boot (5 menit) |

---

## Masalah: Race diff_cont pada Cold Boot

**Masalah:** `diff_cont` macet dalam status `unconfigured` pada cold boot — spawner dimulai sebelum hardware siap.

**Perbaikan:** `diff_cont` sekarang auto-configure dan aktif saat launch.

---

## Masalah yang Diketahui

- **"Reset due to low power" pada Pi5:** Perlu menggunakan PSU 5V/5A resmi
- **Overrun controller_manager:** Loop ros2_control (50Hz) sesekali overrun di Pi5 (20-40ms siklus) — non-kritis tapi perlu dipantau

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `launch_robot.launch.py` | grace_period_sec: 30, global_timeout_sec: 300 |
