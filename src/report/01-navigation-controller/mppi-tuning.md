# Tuning Kontroler MPPI

Semua penyesuaian parameter MPPI, tuning bobot critic, dan upaya pengurangan swaying.

**Laporan:** velocity_fix_report, 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Akar Masalah: ax_max Membatasi Ruang Pencarian Optimizer

MPPI menghasilkan 500+ trajektori acak dan mengevaluasinya menggunakan fungsi biaya critic. Dengan `ax_max` rendah, optimizer secara literal tidak dapat menghasilkan trajektori yang mencapai kecepatan lebih tinggi dalam horizon perencanaannya. Outputnya adalah trajektori terbaik yang bisa ditemukan — yang selalu bernilai kecepatan rendah.

**Ini BUKAN batas akselerasi fisik.** Batas fisik diterapkan downstream oleh velocity_smoother dan diff_drive_controller.

**Referensi:** Nav2 GitHub Issue #5694 — "Max Speed Capped at 0.17 m/s after ROS 2 Jazzy Migration"
**Referensi:** Nav2 PR #5617 — Uses last cmd_vel instead of odom for MPPI initial state

---

## Parameter MPPI Final

| Parameter | Asli | Final | Alasan |
|-----------|------|-------|--------|
| `vx_max` | 0.35 | **0.25** m/s | Sesuai kemampuan robot |
| `vx_min` | — | **-0.12** m/s | Sedikit reverse |
| `wz_max` | — | **0.5** rad/s | Sesuai batas diff_drive |
| `ax_max` | 0.02 → 0.3 | **15.0** | FIX UTAMA: optimizer perlu batas tinggi |
| `ax_min` | -0.05 → -0.5 | **-15.0** | Deselerasi simetris |
| `az_max` | 0.8 | **8.0** | Prinsip sama untuk angular |
| `open_loop` | false | **true** | Gunakan cmd_vel terakhir untuk inisialisasi trajektori (hindari feedback loop) |
| `temperature` | 0.6 | **0.3** | Pemilihan trajektori lebih greedy |
| `vx_std` | 0.2 → 0.4 | **0.2** | Noise kecepatan lebih rendah (final: dikurangi untuk fix swaying) |
| `wz_std` | — | **0.4** | Noise angular lebih rendah |
| `batch_size` | 500 | **800** | Sampel trajektori lebih banyak |
| `iteration_count` | 1 → 3 | **4** | Konvergensi lebih baik |
| `model_dt` | 0.1 | **0.2** | Harus mencocokkan controller period ≤ model_dt, jika tidak FATAL |
| `transform_tolerance` | 0.1 | **1.0** | Mencegah error TF extrapolation |

**Parameter yang dihapus** (bukan parameter Nav2 MPPI asli, diabaikan secara diam-diam):
- `v_std` — bukan parameter valid
- `v_mag_scaler` — bukan parameter valid

---

## Bobot Critic (Final)

| Critic | Asli | Final | Efek |
|--------|------|-------|------|
| `ConstraintCritic cost_weight` | 4.0 → 1.0 | **4.0** (default) | Dipulihkan — bobot rendah bukanlah masalah |
| `PathAlignCritic cost_weight` | 14.0 → 5.0 | **20.0** | Alignment jalur lebih kuat (fix swaying) |
| `PathFollowCritic cost_weight` | 5.0 | **8.0** | Pengikutan jalur lebih kuat |
| `PathAngleCritic cost_weight` | 2.0 | **4.0** | Kontrol heading lebih kuat |
| `ObstaclesCritic` | dinonaktifkan | **diaktifkan** | Dipulihkan untuk keamanan |
| `PreferForwardCritic` | tidak ada | **5.0** | Ditambahkan — lebih memilih gerakan maju |

**Pelajaran:** Bobot critic rendah bukanlah masalah. Optimizer tidak dapat menemukan trajektori yang baik karena ax_max membatasi ruang pencarian.

---

## Pengurangan Swaying (Tuning Iteratif)

Masalah: Robot berayun berlebihan selama navigasi.

Perubahan yang diterapkan secara iteratif:

1. **Mengurangi noise** — `vx_std` 0.4→0.2, `wz_std` 0.8→0.4
2. **Meningkatkan determinisme** — `temperature` 0.3→0.5 (kurang stokastik)
3. **Lebih banyak sampel** — `batch_size` 500→800, `iteration_count` 3→4
4. **Critics lebih kuat** — PathAlign 14→20, PathFollow 5→8, PathAngle 2→4
5. **Melembutkan rintangan** — cost_weight 1.0→0.5, repulsion_weight 1.0→0.5, critical_weight 2.0→1.0

---

## Perbaikan Performa (RPi5)

| Parameter | Sebelum | Sesudah | Alasan |
|-----------|---------|---------|--------|
| `controller_server.controller_frequency` | 10.0 | **5.0** Hz | RPi5 tidak bisa mempertahankan 10Hz |
| `model_dt` | 0.1 | **0.2** | Harus mencocokkan controller period, FATAL jika tidak |

**Penemuan:** Jika `model_dt` < `1/controller_frequency`, MPPI crash dengan FATAL. Timestep model harus ≥ controller period.

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `controller_mppi_params.yaml` | Semua parameter MPPI, bobot critic, konfigurasi velocity_smoother |
| `my_controllers.yaml` | Batas diff_drive_controller disinkronkan |
