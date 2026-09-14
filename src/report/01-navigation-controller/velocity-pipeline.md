# Pipeline Kecepatan

Pipeline perintah kecepatan lengkap dari output MPPI hingga perintah motor, termasuk velocity_smoother, twist_mux, dan diff_drive_controller.

**Laporan:** velocity_fix_report, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Arsitektur Pipeline

```
MPPI (ax_max: 15, open_loop: true)
  → Twist on /cmd_vel_nav
  → velocity_smoother (use_stamped_vel: false, max_accel: [0.15, 0.0, 0.4])
  → Twist on /cmd_vel
  → twist_mux (use_stamped: false)
  → Twist on /diff_cont/cmd_vel_unstamped_raw
  → safety_fail_node (Twist → TwistStamped)
  → TwistStamped on /diff_cont/cmd_vel
  → diff_drive_controller (max_accel: 1.5)
```

---

## velocity_smoother

**Penemuan kunci:** controller_server menggunakan `nav2_util::TwistPublisher` yang defaultnya `enable_stamped_cmd_vel: false`, artinya mempublikasikan `geometry_msgs::msg::Twist`. velocity_smoother harus berlangganan tipe yang sama.

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `use_stamped_vel` | True | **False** | MPPI mempublikasikan Twist (bukan TwistStamped) |
| `smoothing_frequency` | 10.0 | **20.0** Hz | Update dua kali lipat → kurva lebih halus |
| `max_velocity` | — | **[0.25, 0.0, 0.5]** | Sesuai vx_max dan wz_max MPPI |
| `min_velocity` | — | **[-0.12, 0.0, -0.5]** | Sesuai MPPI |
| `max_accel` | — | **[0.15, 0.0, 0.4]** | Ramp up halus (~1.7s untuk mencapai 0.25 m/s) |
| `max_decel` | — | **[-0.25, 0.0, -0.5]** | Ramp down halus |

**Insight kunci:** Atur batas akselerasi MPPI tinggi (15.0), biarkan velocity_smoother menerapkan batas fisik aktual (0.15). Ini memberi MPPI ruang pencarian trajektori penuh sambil menjaga perintah motor halus.

---

## twist_mux

Tata letak prioritas saat ini:

| Sumber | Topic | Prioritas | Timeout |
|--------|-------|-----------|---------|
| navigasi | `/cmd_vel_nav` | 10 (rendah) | 0.3s |
| teleop | `/cmd_vel` | 20 (tinggi) | 0.5s |

Timeout navigasi dikurangi dari 0.5s → 0.3s agar lebih cepat menyerah saat teleop mengambil alih.

| Parameter | Nilai |
|-----------|-------|
| `use_stamped` | false |

---

## diff_drive_controller

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `linear.x.max_acceleration` | 0.4 | **1.5** | Batas fisik aktual (terlalu ketat) |
| `linear.x.min_acceleration` | -0.4 | **-1.5** | Simetris |
| `angular.z.max_acceleration` | 1.5 | **3.0** | Menyesuaikan kemampuan angular |
| `angular.z.min_acceleration` | -1.5 | **-3.0** | Simetris |
| `angular.z.max_velocity` | 1.5 | **0.5** rad/s | Disinkronkan dengan wz_max MPPI |
| `enable_odom_tf` | false | **false** (dipertahankan) | EKF menangani odom TF |

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `controller_mppi_params.yaml` | Konfigurasi velocity_smoother, parameter twist_mux |
| `my_controllers.yaml` | Batas velocity/accel diff_drive_controller |
| Config `twist_mux` | Tata letak prioritas, timeout |
