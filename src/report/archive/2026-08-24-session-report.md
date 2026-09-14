# Laporan Sesi NEVO — 2026-08-24

## Ringkasan

Sesi besar mencakup: migrasi DDS ke FastDDS, rename workspace (tifa→nevo, docking_ws→nevo_ws), perbaikan lokalisasi SLAM, keandalan mode swap, self-healing rplidar, dan tuning timeout diagnostik.

---

## Perubahan yang Dilakukan

### 1. DDS: CycloneDDS → FastDDS

**Masalah:** Kekurangan participant-index CycloneDDS menyebabkan kegagalan total DDS setelah mode swap berulang.

**Perbaikan:**
- Env `nevo.service`: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE=file:///home/rspx1/fastdds.xml`
- `nevo_restart.sh` dan `launch_robot.launch.py` diperbarui dengan env vars FastDDS
- Config `fastdds.xml` baru: DNS v2 dinonaktifkan, buffer socket 10MB
- Semua file launch menggunakan `SetEnvironmentVariable` untuk FastDDS

**Dampak:** DDS tidak lagi crash. Temuan topik yang andal setelah reboot.

### 2. Cloud Domain Dikembalikan ke tifa-*

**Masalah:** Record DNS `nevo-ws.forgixrobotic.com` dan `nevo-be.forgixrobotic.com` tidak ada. Backend cloud masih menggunakan domain tifa-* asli.

**Perbaikan:** Semua referensi endpoint cloud dikembalikan:
- `nevo-ws.forgixrobotic.com` → `tifa-ws.forgixrobotic.com`
- `nevo-be.forgixrobotic.com` → `tifa-be.forgixrobotic.com`
- Terpengaruh: 8 file launch, `robot_api_node.cpp`, `INSTALL.md`, `README.md`

### 3. Perbaikan Lokalisasi SLAM

**Masalah 1:** SLAM membuang semua scan ("queue full" / "Failed to compute odom pose")

**Akar masalah:** `enable_odom_tf: false` pada diff_drive_controller berarti tidak ada TF `odom → base_link`. Config EKF menggunakan `base_link_frame: base_footprint` yang menciptakan pohon TF terpisah.

**Perbaikan:**
- `enable_odom_tf: false` dipertahankan (EKF menangani odom TF)
- EKF mempublikasikan `odom → base_footprint` pada 30Hz
- RSP mempublikasikan `base_footprint → base_link → chassis → laser_frame`
- SLAM `base_frame: base_footprint` (sesuai output EKF)

**Masalah 2:** `diff_cont` macet dalam status `unconfigured`

**Akar masalah:** Race condition pada cold boot — spawner dimulai sebelum hardware siap.

**Perbaikan:** `diff_cont` sekarang auto-configure dan aktif saat launch (verified working).

### 4. Keandalan Mode Swap

**Masalah:** rplidar_node crash dengan exit code 255 setiap kali mode swap (mapping↔nav).

**Akar masalah:** `nevo_restart.sh` membunuh rplidar dengan SIGKILL, yang tidak melepas port serial dengan bersih. Instance rplidar baru tidak bisa membuka port.

**Perbaikan di `nevo_restart.sh`:**
- Ditambahkan grace period 2s antara hard-kill dan serial reset
- Waktu tidur serial reset ditingkatkan dari 1s ke 3s
- Waktu teardown total ditingkatkan untuk pelepasan port yang lebih bersih

**Perbaikan di `rplidar_a1_launch.py`:**
- Ditambahkan `respawn=True, respawn_delay=3.0` — rplidar auto-restart jika crash
- Ditambahkan env vars FastDDS (`SetEnvironmentVariable`) agar node respawn menggunakan DDS yang benar
- `ROS_DOMAIN_ID=7` di-hardcode di launch agar survive respawn

### 5. Timeout Node Diagnostik

**Masalah:** Pada cold boot setelah reboot, timeout node diagnostik 120s terlalu ketat. AMCL aktif terlambat (lifecycle activator menunggu 15s + map_server + setup AMCL), menyebabkan restart loop.

**Perbaikan di `launch_robot.launch.py`:**
- `grace_period_sec`: 10 → 30 (jendela ignore lebih panjang saat boot)
- `global_timeout_sec`: 120 → 300 (waktu cukup untuk inisialisasi cold boot)

### 6. Pemulihan Port Serial USB

**Masalah:** Adaptor CP2102 USB-to-UART yang longgar menyebabkan disconnect USB acak, mengubah nomor ttyUSB.

**Perilaku saat ini:**
- symlink udev `/dev/ttyLIDAR` auto-update ke ttyUSB baru
- rplidar `respawn=True` me-restart node
- Pemulihan manual jika respawn gagal: `sudo kill -9 <rplidar_pid>` memicu respawn

**Catatan:** Ini masalah hardware — modul CP2102 perlu dipasang ulang atau diamankan secara fisik.

### 7. File Mesh URDF (Kosmetik)

**Masalah:** rviz2 di mesin dev tidak bisa memuat mesh STL robot (body, wheels) karena direktori `install` tidak disalin ke mesin dev.

**Catatan:** Murni kosmetik. Hanya mempengaruhi visualisasi model robot di rviz. Bentuk primitif URDF (camera, lidar) tetap render dengan benar.

---

## Ringkasan Perubahan Konfigurasi

| File | Parameter | Lama | Baru |
|------|-----------|------|------|
| `my_controllers.yaml` | `enable_odom_tf` | false | false (dipertahankan) |
| `mapper_params_online_async.yaml` | `base_frame` | base_footprint | base_footprint (dipertahankan) |
| `ekf.yaml` | `base_link_frame` | base_footprint | base_footprint (dipertahankan) |
| `rplidar_a1_launch.py` | `respawn` | — | true |
| `rplidar_a1_launch.py` | `respawn_delay` | — | 3.0 |
| `launch_robot.launch.py` | `grace_period_sec` | 10 | 30 |
| `launch_robot.launch.py` | `global_timeout_sec` | 120 | 300 |
| `nevo_restart.sh` | hard-kill grace | 0s | 2s |
| `nevo_restart.sh` | post-serial-reset sleep | 1s | 3s |

---

## Verified Working

- [x] FastDDS: DDS stabil melintasi mode swap
- [x] Konektivitas cloud: Pesan READY terkirim, MAPPING_START/STOP diterima
- [x] Mode navigasi: semua lifecycle node aktif, AMCL mempublikasikan
- [x] Mode mapping: SLAM membangun peta, pelaporan cakupan
- [x] Mode swap nav→mapping: berfungsi
- [x] Mode swap mapping→nav: berfungsi
- [x] respawn rplidar: tested pada disconnect USB
- [x] Rantai TF: lengkap di kedua mode nav dan mapping
- [x] Cold boot: survive tanpa restart loop (timeout 300s)

## Masalah yang Diketahui

- **Adaptor CP2102 longgar:** Masalah hardware menyebabkan disconnect USB intermiten
- **File mesh di mesin dev:** Tidak di-deploy — hanya kosmetik
- **EKF "Failed to meet update rate":** Warning non-fatal, beban CPU Pi saat startup
- **Endpoint cloud:** Masih menggunakan domain `tifa-*` — menunggu pembuatan record DNS untuk `nevo-*`
