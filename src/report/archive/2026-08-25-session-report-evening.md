# Laporan Sesi NEVO — 2026-08-25 (Malam)

## Ringkasan

Tuning kontroler MPPI untuk navigasi lebih halus, eksplorasi perencanaan (SMAC Hybrid dicoba, dikembalikan ke NavFn), pelembutan penghindaran rintangan, tuning velocity slew, perbaikan keandalan lifecycle, dan auto-initial-pose saat ganti peta.

---

## Perubahan yang Dilakukan

### 1. Percobaan SMAC Planner Hybrid (Dikembalikan)

**Tujuan:** Beralih dari NavFn ke SmacPlannerHybrid dengan kurva Reeds-Shepp untuk jalur yang feasible secara kinematik.

**Konfigurasi yang diterapkan:**
```yaml
plugin: "nav2_smac_planner::SmacPlannerHybrid"
motion_model_for_search: "REEDS_SHEPP"
minimum_turning_radius: 0.25
angle_quantization_bins: 72
cache_obstacle_heuristic: true
smooth_path: true
```

**Hasil:** Planner server hang selama aktivasi — tidak bisa mengakui goal `compute_path_to_pose` dalam timeout. Reeds-Shepp + 72 angle bins terlalu berat secara komputasi untuk hardware. **Dikembalikan ke NavFnPlanner.**

### 2. Tuning Kontroler MPPI (Pengurangan Swaying)

**Masalah:** Robot berayun berlebihan selama navigasi.

**Perubahan yang diterapkan secara iteratif:**

| Parameter | Asli | Akhir | Efek |
|-----------|------|-------|------|
| `vx_std` | 0.4 | 0.2 | Noise kecepatan lebih rendah |
| `wz_std` | 0.8 | 0.4 | Noise angular lebih rendah |
| `temperature` | 0.3 | 0.5 | Lebih deterministik, kurang stokastik |
| `batch_size` | 500 | 800 | Sampel trajektori lebih banyak |
| `iteration_count` | 3 | 4 | Konvergensi lebih baik |
| Bobot `PathAlignCritic` | 14.0 | 20.0 | Alignment jalur lebih kuat |
| Bobot `PathFollowCritic` | 5.0 | 8.0 | Pengikutan jalur lebih kuat |
| Bobot `PathAngleCritic` | 2.0 | 4.0 | Kontrol heading lebih kuat |

### 3. Pelembutan Penghindaran Rintangan

**Masalah:** Robot berbelok terlalu agresif menghindari rintangan, berkontribusi pada ayunan.

**Perubahan di `ObstaclesCritic`:**

| Parameter | Lama | Baru |
|-----------|------|------|
| `cost_weight` | 1.0 | 0.5 |
| `repulsion_weight` | 1.0 | 0.5 |
| `critical_weight` | 2.0 | 1.0 |

**Inflasi local costmap dikencangkan:**

| Parameter | Lama | Baru |
|-----------|------|------|
| `inflation_radius` | 0.40m | 0.35m |
| `cost_scaling_factor` | 5.0 | 8.0 |

### 4. Tuning Kecepatan & Akselerasi (Beberapa Iterasi)

**Penemuan kunci:** Issue GitHub [#5694](https://github.com/ros-navigation/navigation2/issues/5694) — Horizon trajektori MPPI pendek, sehingga batas akselerasi rendah mencegah MPPI mengambil sampel trajektori yang mencapai kecepatan maks. Perbaikannya: atur batas akselerasi MPPI tinggi, biarkan velocity_smoother menerapkan batas fisik aktual.

**Konfigurasi MPPI final:**

| Parameter | Nilai Akhir |
|-----------|-------------|
| `vx_max` | 0.25 m/s |
| `vx_min` | -0.12 m/s |
| `wz_max` | 0.5 rad/s |
| `ax_max` | 5.0 m/s² |
| `ax_min` | -5.0 m/s² |
| `az_max` | 3.0 rad/s² |

**Konfigurasi velocity_smoother final (slew halus):**

| Parameter | Nilai Akhir | Efek |
|-----------|-------------|------|
| `smoothing_frequency` | 20.0 Hz | Update dua kali lipat → kurva lebih halus |
| `max_velocity` | [0.25, 0.0, 0.5] | Sesuai MPPI |
| `min_velocity` | [-0.12, 0.0, -0.5] | Sesuai MPPI |
| `max_accel` | [0.15, 0.0, 0.4] | Ramp up halus (~1.7s untuk mencapai 0.25 m/s) |
| `max_decel` | [-0.25, 0.0, -0.5] | Ramp down halus |

**Batas angular diff_controller disinkronkan:**
- `angular.z.max_velocity`: 1.5 → 0.5 rad/s

### 5. Konfigurasi Twist Mux

Tata letak prioritas saat ini:
| Sumber | Topic | Prioritas | Timeout |
|--------|-------|-----------|---------|
| navigasi | `/cmd_vel_nav` | 10 (rendah) | 0.3s |
| teleop | `/cmd_vel` | 20 (tinggi) | 0.5s |

Timeout navigasi dikurangi dari 0.5s → 0.3s agar lebih cepat menyerah saat teleop mengambil alih.

### 6. Perbaikan Duplikasi Registrasi ReinitializeGlobalLocalization

**Masalah:** bt_navigator crash saat startup dengan `FATAL: ID [ReinitializeGlobalLocalization] already registered`.

**Akar masalah:** Plugin dimuat secara default di Nav2. Menambahkan `nav2_reinitialize_global_localization_service_bt_node` ke `plugin_lib_names` menyebabkan duplikasi registrasi.

**Perbaikan:** Menghapus `plugin_lib_names` dari `controller_mppi_params.yaml` dan `stvl_sim_params.yaml`.

### 7. Keandalan Lifecycle Activator

**Masalah:** Map server dan AMCL gagal configure/activate secara intermiten karena race condition timing.

**Perbaikan di `localization_launch.py`:**
- Mengganti `sleep 15` buta dengan loop discovery node (pengecekan `ros2 node list`, hingga 30s)
- Ditambahkan settle 3s setelah discovery node
- Retry ditingkatkan dari 5 → 8 dengan delay 5s
- Ditambahkan tidur 3s antara langkah configure dan activate
- Menghapus polling `ros2 lifecycle get` (memblokir service executor map_server)
- Mempublikasikan initial pose 5 kali dengan jarak 2s

### 8. Auto Initial Pose Saat Ganti Peta

**Masalah:** Saat ganti peta via UI cloud, AMCL mempertahankan estimasi posisi lama (salah untuk peta baru). Tidak ada initial pose yang dipublikasikan.

**Akar masalah:** `restart_map.sh` memanggil `/map_server/load_map` dan membersihkan costmap tapi tidak pernah mempublikasikan ulang `/initialpose`.

**Perbaikan di `/home/rspx1/restart_map.sh`:**
- Setelah membersihkan costmap, mempublikasikan `/initialpose` di (0,0,0) 5 kali dengan jarak 2s
- AMCL merelokasi di origin peta secara otomatis

### 9. Pembersihan Launch

- Menghapus node `publish_initial_pose` redundan dari `launch_robot.launch.py` (lifecycle_activator menanganinya)
- Mempertahankan publikasi initial pose di script lifecycle `localization_launch.py` saja

---

## File yang Diubah

| File | Perubahan |
|------|-----------|
| `nevo_bot/config/controller_mppi_params.yaml` | Tuning MPPI, slew velocity_smoother, pelembutan obstacle, SMAC→NavFn revert |
| `nevo_bot/config/my_controllers.yaml` | Batas kecepatan angular disinkronkan ke 0.5 rad/s |
| `nevo_bot/config/stvl_sim_params.yaml` | Menghapus `plugin_lib_names` |
| `nevo_bot/launch/localization_launch.py` | Lifecycle_activator robust dengan node discovery, 8 retry, 5x initial pose |
| `nevo_bot/launch/launch_robot.launch.py` | Menghapus node publish_initial_pose redundan |
| `/home/rspx1/restart_map.sh` | Menambahkan auto initial pose (0,0,0) setelah ganti peta |

---

## Masalah yang Diketahui

- **Adaptor CP2102 longgar:** Masalah hardware menyebabkan disconnect rplidar intermiten → AMCL kehilangan rantai TF. Perlu pengamanan fisik.
- **SMAC Hybrid terlalu berat untuk hardware:** Reeds-Shepp + angle bins tinggi menggantung planner_server. Perlu mengurangi angle_quantization_bins (mis. 36) atau costmap yang di-downsample untuk mencoba ulang.
- **EKF "Failed to meet update rate":** Warning non-fatal, CPU sesekali overload saat burst startup.
- **Field baterai baru di UI Cloud:** Backend perlu mengurai/menyimpan/mengekspos bus_voltage, soh, internal_resistance_mohm, cycle_count.
