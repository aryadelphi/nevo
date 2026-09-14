# Laporan Perbaikan Kecepatan Navigasi

**Tanggal:** 2026-08-22
**Robot:** TFRB2 (nevo_bot)
**Masalah:** Robot macet/moving dengan kecepatan ~5% dari max saat navigasi Nav2

---

## Ringkasan Masalah

Robot berhasil navigasi ke tujuan namun hampir tidak bergerak (~0.018-0.05 m/s vs 0.35 m/s max).
Recovery behaviors (spin) berfungsi baik, perencanaan jalur benar, namun perintah kecepatan sangat kecil.

---

## Akar Masalah

**Parameter `ax_max` pada MPPI (0.02, kemudian 0.3) membatasi ruang pencarian trajektori optimizer, BUKAN hanya akselerasi output.**

MPPI menghasilkan 500+ trajektori acak dan mengevaluasinya menggunakan fungsi biaya critic.
Dengan `ax_max: 0.3`, optimizer secara literal tidak dapat menghasilkan trajektori yang mencapai
kecepatan 0.35 m/s dalam horizon perencanaan 2.8s. Outputnya adalah trajektori terbaik yang bisa ditemukan,
yang selalu bernilai kecepatan rendah.

Ini adalah masalah yang diketahui pada Nav2 Jazzy (GitHub #5694).

---

## Perubahan yang Dilakukan

### 1. Kontroler MPPI (`controller_mppi_params.yaml`)

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `ax_max` | 0.02 -> 0.3 | **15.0** | FIX UTAMA: optimizer perlu batas tinggi untuk eksplorasi ruang kecepatan penuh |
| `ax_min` | -0.05 -> -0.5 | **-15.0** | Deselerasi simetris |
| `az_max` | 0.8 | **8.0** | Prinsip sama untuk angular |
| `open_loop` | false | **true** | Gunakan cmd_vel terakhir untuk inisialisasi trajektori, bukan odom (hindari feedback loop) |
| `temperature` | 0.6 | **0.3** | Pemilihan trajektori lebih greedy |
| `vx_std` | 0.2 | **0.4** | Variansi eksplorasi lebih besar |
| `iteration_count` | 1 | **3** | Konvergensi optimasi lebih baik |
| `v_std` | [0.1, 0.15] | **dihapus** | Bukan parameter Nav2 MPPI asli (diabaikan) |
| `v_mag_scaler` | 0.25 | **dihapus** | Bukan parameter Nav2 MPPI asli (diabaikan) |

### 2. Critic MPPI (dikembalikan ke default Nav2)

| Critic | Sebelum | Sesudah |
|--------|---------|---------|
| `ConstraintCritic cost_weight` | 4.0 -> 1.0 | **4.0** (default) |
| `PathAlignCritic cost_weight` | 14.0 -> 5.0 | **14.0** (default) |
| `ObstaclesCritic` | dinonaktifkan | **diaktifkan** (dikembalikan untuk keamanan) |
| `PreferForwardCritic` | tidak ada | **ditambahkan** (cost_weight: 5.0) |

**Pelajaran:** Bobot critic rendah bukanlah masalah. Optimizer tidak dapat menemukan
trajektori yang baik karena ax_max membatasi ruang pencarian.

### 3. Perbaikan Pipeline Kecepatan

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `velocity_smoother use_stamped_vel` | False -> True | **False** | MPPI mempublikasikan Twist (bukan TwistStamped) via nav2_util::TwistPublisher |

controller_server menggunakan `nav2_util::TwistPublisher` yang defaultnya `enable_stamped_cmd_vel: false`,
artinya mempublikasikan `geometry_msgs::msg::Twist`. velocity_smoother harus berlangganan tipe yang sama.

### 4. diff_drive_controller (`my_controllers.yaml`)

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `linear.x.max_acceleration` | 0.4 | **1.5** | Batas fisik aktual (terlalu ketat) |
| `linear.x.min_acceleration` | -0.4 | **-1.5** | Simetris |
| `angular.z.max_acceleration` | 1.5 | **3.0** | Menyesuaikan kemampuan angular |
| `angular.z.min_acceleration` | -1.5 | **-3.0** | Simetris |

### 5. Perubahan STVL / Costmap (lebih awal dalam sesi)

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `stvl_layer voxel_decay` | 0.3 | 1.5 | Mengurangi kedipan costmap |
| `stvl_layer min_obstacle_height` | 0.10 | 0.20 | Filter deteksi lantai |
| `depth_camera min_obstacle_height` | 0.15 | 0.30 | Kinect pada ketinggian 1.0m mendeteksi lantai |
| `depth_camera voxel_min_points` | 1 | 3 | Mengurangi noise dari sensor kedalaman |
| `local_costmap inflation_radius` | 0.55 | 0.40 | Menyesuaikan footprint robot |
| `local_costmap cost_scaling_factor` | 10.0 | 5.0 | Gradient inflasi lebih halus |

---

## Ringkasan Arsitektur (final)

```
MPPI (ax_max: 15, open_loop: true)
  -> Twist on /cmd_vel_nav
  -> velocity_smoother (use_stamped_vel: false, max_accel: 0.3)
  -> Twist on /cmd_vel
  -> twist_mux (use_stamped: false)
  -> Twist on /diff_cont/cmd_vel_unstamped_raw
  -> safety_fail_node (Twist -> TwistStamped)
  -> TwistStamped on /diff_cont/cmd_vel
  -> diff_drive_controller (max_accel: 1.5)
```

**Insight kunci:** ax_max pada MPPI adalah batas ruang pencarian optimizer, bukan batas fisik robot.
Batas fisik diterapkan oleh velocity_smoother dan diff_drive_controller downstream.

---

## Referensi

- Nav2 GitHub Issue #5694: "Max Speed Capped at 0.17 m/s after ROS 2 Jazzy Migration"
- Nav2 PR #5617: Uses last cmd_vel instead of odom for MPPI initial state
