# Robot NEVO — Laporan Pengaturan Navigation Stack ROS2 Jazzy

**Tanggal:** 2026-08-30
**Platform:** Raspberry Pi 5, Ubuntu 24.04, ROS2 Jazzy
**Workspace:** `/home/nevo/nevo/` (copy) dan `/home/nevo/nevo_ws/` (aktif)
**Build ID:** `ament_2026-08-30-14-11-32`

---

## 1. Konfigurasi Hardware

| Komponen | Perangkat | Path |
|----------|-----------|------|
| Lidar | RPLidar A1 (CP2102) | `/dev/ttyLIDAR` → `/dev/ttyUSB0` |
| Motor | Arduino (CH340) | `/dev/ttyMOTOR` → `/dev/ttyUSB1` |
| Baterai | INA226 | **Dilewati** (modul rusak) |
| Kamera | Kamera belakang | `/dev/video0` |
| SBC | Raspberry Pi 5 | aarch64, Ubuntu 24.04 |

**Jaringan:** `192.168.12.195` via `wlan0`
**DDS:** `rmw_fastrtps_cpp` dengan config FastDDS di `~/fastdds.xml`
**Domain ID:** `ROS_DOMAIN_ID=7`

---

## 2. Masalah yang Ditemukan dan Diperbaiki

### 2.1 Lingkungan Shell (`.zshrc`)
- **Masalah:** Sesi SSH tidak memiliki variabel lingkungan ROS2.
- **Perbaikan:** Ditambahkan ROS2 Jazzy lengkap + sourcing workspace + env vars ke `~/.zshrc`.

### 2.2 Aturan Udev
- **Masalah:** Symlink perangkat serial tidak bertahan setelah reboot.
- **Perbaikan:** Dibuat `/etc/udev/rules.d/99-nevo.rules`:
  - `/dev/ttyLIDAR` → RPLidar A1 via serial number `0001`
  - `/dev/ttyMOTOR` → Arduino motor CH340 via path port fisik

### 2.3 Sudoers untuk systemd-run
- **Masalah:** `robot_api_node` tidak bisa me-restart service tanpa password.
- **Perbaikan:** Dibuat `/etc/sudoers.d/nevo-robot`:
  ```
  nevo ALL=(root) NOPASSWD: /usr/bin/systemd-run
  ```

### 2.4 Path Hardcoded (`rspx1` → `nevo`)
- **Masalah:** ~15 file sumber memiliki path hardcoded ke `/home/rspx1/nevo_ws/`.
- **Perbaikan:** Semua diganti dengan `/home/nevo/nevo_ws/` di seluruh codebase.

### 2.5 QoS Node Diagnostik `/map`
- **Masalah:** `nevo_diagnostic_node` tidak bisa menerima `/map` karena ketidakcocokan QoS.
- **Perbaikan:** QoS diubah ke `reliable().transient_local()` — terkonfirmasi `map:OK`.

### 2.6 Update Stasioner AMCL
- **Masalah:** AMCL tidak memperbarui pose saat robot stasioner.
- **Perbaikan:** Atur `update_min_d: 0.0` dan `update_min_a: 0.0` di `controller_mppi_params.yaml`.

### 2.7 Tuning Timeout Navigasi
- **Perubahan:**
  - `bt_navigator.default_server_timeout`: `3000` ms (sebelumnya default)
  - `planner_server.costmap_update_timeout`: `5.0` s
  - AMCL `transform_tolerance`: `1.0`

### 2.8 2D Goal Pose — TF Extrapolation Goal Stale (FIX AKAR MASALAH)
- **Masalah:** Nav2 planner_server menggunakan `input_pose.header.stamp` (dari klik goal pengguna di RViz) untuk lookup TF via `tf_buffer.transform()`. Jika goal dibuat sebelum AMCL mulai menyiarkan `map → odom`, lookup TF gagal permanen dengan "Extrapolation Error". Pohon BT mencoba ulang goal stale yang sama secara tak terbatas.
- **Perbaikan (localization_launch.py):**
  1. **Publisher initialpose latar belakang:** Diubah dari burst 5 kali ke timer berkelanjutan (setiap 5s) — menjaga TF `map → odom` AMCL tetap segar.
  2. **Menunggu konfirmasi TF:** Setelah aktivasi lifecycle, activator sekarang menunggu (hingga 120s) untuk `map → odom` benar-benar muncul di pohon TF sebelum menyatakan selesai — mencegah race condition di mana goal tiba sebelum TF AMCL tersedia.

### 2.9 `transform_tolerance` Kontroler MPPI Terlalu Rendah
- **Masalah:** `transform_tolerance: 0.1` pada kontroler MPPI menyebabkan "Extrapolation Error" saat timestamp goal 140ms di depan TF terbaru.
- **Perbaikan:** `transform_tolerance` diubah dari `0.1` ke `1.0` di `controller_mppi_params.yaml` (bagian FollowPath).

### 2.10 Performa RPi5 Overload
- **Masalah:** EKF pada 30Hz dan kontroler pada 10Hz membanjiri Pi5, menyebabkan error "Failed to meet update rate" terus-menerus dan kontrol loop miss.
- **Perbaikan:**
  - EKF `frequency`: `30.0` → `15.0` Hz
  - `controller_server.controller_frequency`: `10.0` → `5.0` Hz
  - MPPI `model_dt`: `0.1` → `0.2` (harus mencocokkan controller period ≤ model_dt, jika tidak FATAL crash)

### 2.11 `joint_state_broadcaster` FATAL — Missing Wheel TFs
- **Masalah:** `joint_state_broadcaster` dan `diff_cont` di-spawn secara bersamaan di `OnProcessStart`. Keduanya mencoba mengklaim state interfaces yang sama → `joint_state_broadcaster` timeout pada `load_controller` (10s) → FATAL → tidak ada `/joint_states` → `robot_state_publisher` tidak bisa menghitung TF `left_wheel` dan `right_wheel`.
- **Perbaikan (launch_robot.launch.py):**
  1. Spawn `joint_state_broadcaster` TERLEBIH DAHULU di callback `OnProcessStart` (mengklaim state interfaces).
  2. Spawn `diff_cont` via `TimerAction` 10 detik SETELAH controller_manager mulai (mengklaim command interfaces setelah state interfaces diklaim).

### 2.12 Mode Mapping — SLAM Toolbox Tidak Membangun Peta
- **Masalah:** SLAM Toolbox membuang setiap scan dengan "Message Filter dropping message: queue is full". Akar masalah: `diff_cont` tidak pernah ter-configure/aktif karena `delayed_diff_drive` TimerAction (5s) fire pada waktu yang sama dengan `delayed_controller_manager` (5s), berlomba dengan startup controller_manager. Tanpa diff_cont aktif, EKF tidak memiliki input odom → tidak ada TF `odom → base_footprint` → SLAM tidak bisa menyinkronkan scan dengan TF.
- **Perbaikan (launch_robot.launch.py):** Periode `delayed_diff_drive` diubah dari `5.0` ke `10.0` detik untuk menghindari race condition.

---

## 3. Perubahan Peta Default
- **Diubah dari:** `telkop_save_map.yaml`
- **Diubah ke:** `abie_lab_save.yaml` (lokasi saat ini)
- **File yang diubah:**
  - `launch_robot.launch.py` — `default_map`
  - `hw_nav.launch.py` — `default_map`

---

## 4. File Konfigurasi yang Diubah

| File | Perubahan |
|------|-----------|
| `~/.zshrc` | Menambahkan env ROS2 Jazzy + sourcing workspace |
| `/etc/udev/rules.d/99-nevo.rules` | Symlink perangkat serial |
| `/etc/sudoers.d/nevo-robot` | `systemd-run` tanpa password |
| `config/controller_mppi_params.yaml` | AMCL (update_min_d/a=0, transform_tolerance=1.0), bt_navigator (timeout=3000), controller_server (freq=5.0, transform_tolerance=2.0), MPPI (model_dt=0.2, transform_tolerance=1.0) |
| `config/ekf.yaml` | frequency: 15.0 |
| `launch/launch_robot.launch.py` | Path peta, urutan spawner (JSB duluan, diff_cont 10s kemudian), menghapus variabel redundan |
| `launch/hw_nav.launch.py` | Path peta |
| `launch/localization_launch.py` | Publisher initialpose latar belakang, menunggu konfirmasi TF |

---

## 5. Pohon TF Final

```
map → odom (SLAM/AMCL)
  └─→ odom → base_footprint (EKF, 15Hz)
        └─→ base_footprint → base_link (RSP, statis)
              └─→ base_link → chassis (RSP, statis)
              │     ├─→ chassis → laser_frame (RSP, statis)
              │     ├─→ chassis → camera_link (RSP, statis)
              │     ├─→ chassis → c310_camera_link (RSP, statis)
              │     ├─→ chassis → rear_caster_wheel (RSP, statis)
              │     └─→ chassis → back_caster_wheel (RSP, statis)
              ├─→ base_link → left_wheel (joint_states, ~15Hz)
              └─→ base_link → right_wheel (joint_states, ~15Hz)
```

---

## 6. Status Navigation Stack

| Komponen | Status |
|----------|--------|
| map_server | OK |
| AMCL | OK |
| EKF | OK (15 Hz) |
| controller_server | OK (5 Hz, MPPI) |
| planner_server | OK (NavFn) |
| behavior_server | OK |
| bt_navigator | OK (BT kustom dengan relokalisasi) |
| smoother_server | OK |
| waypoint_follower | OK |
| velocity_smoother | OK |
| diff_drive_controller | OK (5 Hz) |
| joint_state_broadcaster | OK |
| SLAM Toolbox | OK (mode mapping) |
| RPLidar A1 | OK (~8 Hz) |
| Robot API | OK (cloud WS bridge) |

---

## 7. Layanan Systemd

| Layanan | Status | Deskripsi |
|---------|--------|-----------|
| `nevo.service` | Enabled | Mode Nav (default) |
| `nevo-mapping.service` | Disabled | Mode Mapping |

**Peralihan mode:** Dilakukan via UI eksternal → `robot_api_node` → `nevo_restart.sh` → sesi tmux.

---

## 8. Masalah yang Tersisa

1. **Peringatan catu daya:** Pi5 melaporkan "Reset due to low power" — gunakan PSU 5V/5A resmi.
2. **Error rate EKF sesekali:** EKF sesekali gagal mencapai target 15Hz (~30-190ms spike). Bisa dikurangi ke 10Hz jika perlu.
3. **Overrun controller_manager:** Loop ros2_control (50Hz) sesekali overrun di Pi5 (20-40ms siklus). Non-kritis tapi perlu dipantau.
4. **Error port SHM FastDDS:** `Failed init_port fastrtps_port7002/7003` — kosmetik, tidak mempengaruhi fungsionalitas.
5. **Peringatan sourcing `.setup.sh`:** ROS2 `setup.bash` mencoba source yang tidak ada `/home/nevo/setup.sh` — hanya kosmetik.

---

## 9. Riwayat Restart

| Waktu | Kejadian |
|-------|----------|
| ~17:16 | Uji goal awal setelah fix AMCL — TF extrapolation diperbaiki tapi timeout planner |
| ~17:20 | Menerapkan transform_tolerance MPPI + fix rate EKF/controller — controller FATAL (konflik model_dt) |
| ~17:23 | Memperbaiki model_dt (0.1→0.2) — controller_server FATAL dari konflik joint_state_broadcaster |
| ~17:33 | Memperbaiki urutan spawner (JSB duluan, diff_cont ditunda) — semua berfungsi |
| ~17:42 | UI swap ke mode mapping — race condition diff_cont, SLAM membuang scan |
| ~17:55 | Aktivasi diff_cont manual + memperbaiki timer delayed_diff_drive (5→10s) — SLAM berfungsi |

---

## 10. Referensi Perintah

### Source lingkungan untuk ros2 CLI:
```bash
export AMENT_PREFIX_PATH=/opt/ros/jazzy:/home/nevo/nevo_ws/install
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/nevo/nevo_ws/install/lib/python3.12/site-packages
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/home/nevo/nevo_ws/install/lib
export PATH=/opt/ros/jazzy/bin:$PATH
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### Build:
```bash
cd /home/nevo/nevo_ws && colcon build --packages-select nevo_bot
```

### Cek layanan:
```bash
sudo systemctl status nevo.service
tmux ls
tmux capture-pane -t nevo_robot -p | tail -40
```
