# Lifecycle Activator

Keandalan aktivasi lifecycle node ROS2 — discovery node, retry, dan aktivasi map_server/AMCL.

**Laporan:** 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Masalah: Sleep Buta Terlalu Tidak Andal

Lifecycle activator menggunakan `sleep 15` sebelum mencoba mengaktifkan node. Ini terlalu pendek pada cold boot (AMCL aktif terlambat) dan membuang waktu jika node sudah siap lebih cepat.

---

## Perbaikan: Loop Discovery Node

Mengganti `sleep 15` buta dengan discovery node aktif:

1. **Discovery node** — mem-poll `ros2 node list` hingga 30s menunggu node target
2. **Settle 3s** — setelah discovery, menunggu 3s agar node stabil
3. **8 retry** dengan delay 5s (sebelumnya 5 retry)
4. **Tidur 3s** antara langkah configure dan activate
5. **Menghapus** polling `ros2 lifecycle get` (memblokir service executor map_server)
6. Mempublikasikan initial pose 5 kali dengan jarak 2s (nanti diubah ke timer berkelanjutan)

---

## Perbaikan: Duplikasi ReinitializeGlobalLocalization

**Masalah:** `bt_navigator` crash saat startup dengan `FATAL: ID [ReinitializeGlobalLocalization] already registered`.

**Akar masalah:** Plugin dimuat secara default di Nav2. Menambahkan `nav2_reinitialize_global_localization_service_bt_node` ke `plugin_lib_names` menyebabkan duplikasi registrasi.

**Perbaikan:** Menghapus `plugin_lib_names` dari `controller_mppi_params.yaml` dan `stvl_sim_params.yaml`.

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `localization_launch.py` | Lifecycle activator robust dengan discovery node, 8 retry, 5x initial pose, menghapus lifecycle get polling |
| `launch_robot.launch.py` | Menghapus node publish_initial_pose redundan |
| `controller_mppi_params.yaml` | Menghapus plugin_lib_names |
| `stvl_sim_params.yaml` | Menghapus plugin_lib_names |
