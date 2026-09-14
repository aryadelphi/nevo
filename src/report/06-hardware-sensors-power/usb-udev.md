# USB & Udev

Manajemen port serial USB, aturan udev, keandalan CP2102, dan symlink perangkat.

**Laporan:** 2026-08-24, NEVO_NAVIGATION_SETUP_REPORT

---

## Konfigurasi Hardware

| Komponen | Perangkat | Symlink | Aktual |
|----------|-----------|---------|--------|
| Lidar | RPLidar A1 (CP2102) | `/dev/ttyLIDAR` | `/dev/ttyUSB0` |
| Motor | Arduino (CH340) | `/dev/ttyMOTOR` | `/dev/ttyUSB1` |

---

## Masalah: Aturan Udev Tidak Bertahan

Symlink perangkat serial tidak bertahan setelah reboot.

**Perbaikan:** Dibuat `/etc/udev/rules.d/99-nevo.rules`:
- `/dev/ttyLIDAR` → RPLidar A1 via serial number `0001`
- `/dev/ttyMOTOR` → Arduino motor CH340 via path port fisik

---

## Masalah: Adaptor CP2102 USB-to-UART Longgar

**Gejala:** Disconnect USB acak yang mengubah nomor ttyUSB, menyebabkan crash rplidar.

**Perilaku saat ini:**
- Symlink udev `/dev/ttyLIDAR` auto-update ke ttyUSB baru
- rplidar `respawn=True` me-restart node
- Pemulihan manual jika respawn gagal: `sudo kill -9 <rplidar_pid>` memicu respawn

**Status:** Masalah hardware — modul CP2102 perlu dipasang ulang atau diamankan secara fisik. Tidak bisa diperbaiki di software.

---

## Masalah yang Diketahui

- **Adaptor CP2102 longgar:** Masalah hardware menyebabkan disconnect intermiten
- **File mesh URDF:** Tidak di-deploy ke mesin dev — hanya kosmetik (tidak ada loading STL di rviz)

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `/etc/udev/rules.d/99-nevo.rules` | Membuat symlink perangkat serial |
| `rplidar_a1_launch.py` | Menambahkan respawn untuk pemulihan crash |
