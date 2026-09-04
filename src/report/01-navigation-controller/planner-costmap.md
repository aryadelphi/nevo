# Konfigurasi Perencana & Costmap

Pemilihan perencana (NavFn vs SMAC Hybrid), STVL (spatio-temporal voxel layer), inflasi costmap, dan penghindaran rintangan.

**Laporan:** 2026-08-25-evening, velocity_fix_report, NEVO_NAVIGATION_SETUP_REPORT

---

## Percobaan SMAC Planner Hybrid (Dikembalikan)

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

**Hasil:** Planner server hang selama aktivasi — tidak bisa mengakui goal `compute_path_to_pose` dalam timeout. Reeds-Shepp + 72 angle bins terlalu berat secara komputasi untuk hardware Raspberry Pi 5.

**Dikembalikan ke:** NavFnPlanner

**Catatan:** Bisa mencoba ulang dengan mengurangi `angle_quantization_bins` (mis. 36) atau costmap yang di-downsample.

---

## Pelembutan Penghindaran Rintangan

Masalah: Robot berbelok terlalu agresif menghindari rintangan, berkontribusi pada ayunan.

### Perubahan ObstaclesCritic

| Parameter | Lama | Baru |
|-----------|------|------|
| `cost_weight` | 1.0 | **0.5** |
| `repulsion_weight` | 1.0 | **0.5** |
| `critical_weight` | 2.0 | **1.0** |

### Inflasi Local Costmap Dikencangkan

| Parameter | Lama | Baru |
|-----------|------|------|
| `inflation_radius` | 0.40m | **0.35m** |
| `cost_scaling_factor` | 5.0 | **8.0** |

---

## Tuning STVL (Spatio-Temporal Voxel Layer)

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `voxel_decay` | 0.3 | **1.5** | Mengurangi kedipan costmap |
| `min_obstacle_height` | 0.10 | **0.20** | Filter deteksi lantai |
| `depth_camera min_obstacle_height` | 0.15 | **0.30** | Kinect pada ketinggian 1.0m mendeteksi lantai |
| `depth_camera voxel_min_points` | 1 | **3** | Mengurangi noise dari sensor kedalaman |
| `local_costmap inflation_radius` | 0.55 | **0.40** | Menyesuaikan footprint robot |
| `local_costmap cost_scaling_factor` | 10.0 | **5.0** | Gradient inflasi lebih halus |

---

## Local Costmap (Final)

| Parameter | Nilai |
|-----------|-------|
| `inflation_radius` | 0.35m |
| `cost_scaling_factor` | 8.0 |

---

## Timeout Server Navigasi

| Parameter | Nilai | File |
|-----------|-------|------|
| `bt_navigator.default_server_timeout` | 3000 ms | `controller_mppi_params.yaml` |
| `planner_server.costmap_update_timeout` | 5.0 s | `controller_mppi_params.yaml` |

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `controller_mppi_params.yaml` | Bobot ObstaclesCritic, inflasi costmap, parameter STVL |
| `stvl_sim_params.yaml` | voxel_decay STVL, ketinggian min, menghapus plugin_lib_names |
