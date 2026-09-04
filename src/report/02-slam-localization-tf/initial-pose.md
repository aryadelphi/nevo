# Manajemen Initial Pose

Publikasi initial pose otomatis, penanganan ganti peta, dan konfirmasi TF.

**Laporan:** 2026-08-25-evening, NEVO_NAVIGATION_SETUP_REPORT

---

## Masalah: Tidak Ada Initial Pose Saat Ganti Peta

Saat ganti peta via UI cloud, AMCL mempertahankan estimasi posisi lama (salah untuk peta baru). Tidak ada initial pose yang dipublikasikan.

**Akar masalah:** `restart_map.sh` memanggil `/map_server/load_map` dan membersihkan costmap tapi tidak pernah mempublikasikan ulang `/initialpose`.

---

## Perbaikan: Auto Initial Pose (restart_map.sh)

Setelah membersihkan costmap, mempublikasikan `/initialpose` di (0,0,0) 5 kali dengan jarak 2s. AMCL merelokasi di origin peta secara otomatis.

```bash
# Setelah membersihkan costmap
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}
}'
```

---

## Perbaikan: Publisher Initial Pose Berkelanjutan (localization_launch.py)

**Masalah:** Publisher initialpose latar belakang adalah burst 5 kali — jika AMCL mulai terlambat, semua 5 publikasi sia-sia.

**Perbaikan:** Diubah dari burst 5 kali ke timer berkelanjutan (setiap 5s) — menjaga TF `map → odom` AMCL tetap segar.

---

## Perbaikan: Menunggu Konfirmasi TF

Setelah aktivasi lifecycle, activator sekarang menunggu (hingga 120s) untuk `map → odom` benar-benar muncul di pohon TF sebelum menyatakan selesai.

**Ini mencegah:** Race condition di mana goal tiba sebelum TF AMCL tersedia.

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `restart_map.sh` | Menambahkan auto initial pose (0,0,0) setelah ganti peta |
| `localization_launch.py` | Publisher initialpose berkelanjutan, menunggu konfirmasi TF |
