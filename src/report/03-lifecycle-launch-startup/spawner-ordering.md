# Urutan Spawner

Penjadwalan spawner ROS2 control — race condition joint_state_broadcaster vs diff_drive_controller.

**Laporan:** NEVO_NAVIGATION_SETUP_REPORT

---

## Masalah: joint_state_broadcaster FATAL

`joint_state_broadcaster` dan `diff_cont` di-spawn secara bersamaan di `OnProcessStart`. Keduanya mencoba mengklaim state interfaces yang sama → `joint_state_broadcaster` timeout pada `load_controller` (10s) → FATAL → tidak ada `/joint_states` → `robot_state_publisher` tidak bisa menghitung TF `left_wheel` dan `right_wheel`.

---

## Perbaikan (launch_robot.launch.py)

1. **Spawn `joint_state_broadcaster` TERLEBIH DAHULU** di callback `OnProcessStart` — mengklaim state interfaces.
2. **Spawn `diff_cont` via `TimerAction` 10 detik SETELAH** controller_manager mulai — mengklaim command interfaces setelah state interfaces diklaim.

```python
# Callback OnProcessStart
# 1. Spawn joint_state_broadcaster segera
# 2. Spawn diff_cont setelah delay 10s via TimerAction
```

**Hasil:** joint_state_broadcaster mendapatkan state interfaces terlebih dahulu, kemudian diff_cont mendapatkan command interfaces. Tidak ada konflik.

---

## Terkait: diff_cont Macet di Unconfigured (Cold Boot)

**Masalah:** Pada cold boot, `diff_cont` dimulai dalam status `unconfigured` karena spawner berlomba dengan inisialisasi hardware.

**Perbaikan:** `diff_cont` sekarang auto-configure dan aktif saat launch (verified working).

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `launch_robot.launch.py` | Urutan spawner: JSB duluan, diff_cont 10s kemudian |
