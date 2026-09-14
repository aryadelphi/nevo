# Konfigurasi AMCL

Tuning Adaptive Monte Carlo Localization untuk estimasi pose yang andal.

**Laporan:** NEVO_NAVIGATION_SETUP_REPORT

---

## Update Stasioner

**Masalah:** AMCL tidak memperbarui estimasi pose saat robot stasioner (menunggu di tujuan, misalnya).

**Perbaikan:**

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `update_min_d` | default (0.2m) | **0.0** | Update meskipun robot belum bergerak |
| `update_min_a` | default (0.2rad) | **0.0** | Update meskipun robot belum berputar |

Ini memungkinkan AMCL terus memperbaiki estimasi pose menggunakan scan LIDAR meskipun robot stasioner, meningkatkan akurasi lokalisasi.

---

## Transform Tolerance

| Parameter | Nilai | Alasan |
|-----------|-------|--------|
| `transform_tolerance` | **1.0** | Mencegah error TF extrapolation saat timestamp goal lebih maju dari TF terbaru |

**Masalah yang diselesaikan:** Nav2 planner_server menggunakan `input_pose.header.stamp` untuk lookup TF. Jika goal dibuat sebelum AMCL mulai menyiarkan `map → odom`, lookup TF gagal permanen dengan "Extrapolation Error".

---

## Perbaikan QoS Costmap

**Masalah:** `nevo_diagnostic_node` tidak bisa menerima `/map` karena ketidakcocokan QoS.

**Perbaikan:** QoS diubah ke `reliable().transient_local()` — terkonfirmasi `map:OK`.

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `controller_mppi_params.yaml` | update_min_d: 0.0, update_min_a: 0.0, transform_tolerance: 1.0 |
| Node diagnostik | QoS diubah ke reliable().transient_local() |
