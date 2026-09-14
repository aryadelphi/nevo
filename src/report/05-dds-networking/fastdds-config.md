# Konfigurasi FastDDS

Migrasi middleware DDS, konfigurasi XML FastDDS, penemuan unicast, dan pengaturan domain ID.

**Laporan:** 2026-08-24, 2026-08-26, NEVO_NAVIGATION_SETUP_REPORT

---

## Migrasi: CycloneDDS → FastDDS

**Masalah:** Kekurangan participant-index CycloneDDS menyebabkan kegagalan total DDS setelah mode swap berulang.

**Perbaikan:**
- Env `nevo.service`: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `FASTRTPS_DEFAULT_PROFILES_FILE=file:///home/rspx1/fastdds.xml`
- `nevo_restart.sh` dan `launch_robot.launch.py` diperbarui dengan env vars FastDDS
- Config `fastdds.xml` baru: DNS v2 dinonaktifkan, buffer socket 10MB
- Semua file launch menggunakan `SetEnvironmentVariable` untuk FastDDS

**Dampak:** DDS tidak lagi crash. Temuan topik yang andal setelah reboot.

---

## Penemuan Unicast (WiFi Hotspot)

**Masalah:** Mesin dev (host Arch Linux, distrobox Ubuntu 24) tidak bisa menemukan topik ROS 2 dari robot melalui WiFi hotspot (`ap0`). WiFi hotspot memblokir atau tidak mendukung multicast.

**Temuan kunci:**
- `ros2 topic list --spin-time 10` berfungsi — DDS hanya perlu lebih banyak waktu untuk penemuan unicast
- Distrobox menggunakan namespace jaringan host, sehingga jangkauan jaringan baik (ping berfungsi)

**Solusi:** Ditambahkan `initial_peers` ke `~/fastdds.xml` mesin dev:

```xml
<initial_peers>
  <locator>
    <udpv4>
      <address>192.168.12.195</address>
    </udpv4>
  </locator>
</initial_peers>
```

**Penting:** `fastdds.xml` sisi robot TIDAK memerlukan `initial_peers` — menambahkannya merusak penemuan inbound. Hanya mesin dev yang memerlukan konfigurasi peer eksplisit.

---

## Variabel Lingkungan (Mesin Dev)

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="file:///home/$(whoami)/fastdds.xml"
```

Penggunaan: `ros2 topic list --spin-time 10` (waktu spin default terlalu pendek untuk unicast WiFi)

---

## Domain ID

`ROS_DOMAIN_ID=7` di-hardcode di file launch agar survive respawn rplidar.

---

## Masalah yang Diketahui

- Error port SHM FastDDS (`fastrtps_port7002/7003`) — kosmetik, tidak mempengaruhi fungsionalitas
- Endpoint cloud masih menggunakan domain `tifa-*` — menunggu pembuatan record DNS untuk `nevo-*`

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `~/fastdds.xml` (mesin dev) | Menambahkan `initial_peers` yang menunjuk ke robot |
| `/home/rspx1/fastdds.xml` (robot) | Dikembalikan ke asli (tanpa `initial_peers`) |
| `nevo.service` | Env vars FastDDS |
| `nevo_restart.sh` | Env vars FastDDS |
| `launch_robot.launch.py` | Env vars FastDDS, SetEnvironmentVariable |
