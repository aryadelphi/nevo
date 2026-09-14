# Laporan Sesi NEVO — 2026-08-26

## Ringkasan

Perbaikan penemuan jaringan DDS antara mesin dev (Arch Linux distrobox) dan robot (Raspberry Pi 5). Konfigurasi `initial_peers` FastDDS untuk penemuan unicast WiFi hotspot (`ap0`).

---

## Masalah

Mesin dev (host Arch Linux, distrobox Ubuntu 24) tidak bisa menemukan topik ROS 2 dari robot (Raspberry Pi 5 di `192.168.12.195`) melalui WiFi hotspot (`ap0` di `192.168.12.1`). `ros2 topic list` hanya mengembalikan `/parameter_events` dan `/rosout`.

**Temuan kunci:**
- WiFi hotspot (`ap0`) memblokir atau tidak mendukung multicast — mekanisme penemuan default FastDDS
- Distrobox menggunakan namespace jaringan host, sehingga jangkauan jaringan baik (ping berfungsi)
- `ros2 topic list --spin-time 10` berfungsi — DDS hanya perlu lebih banyak waktu untuk penemuan unicast

## Solusi

Ditambahkan `initial_peers` ke `~/fastdds.xml` mesin dev untuk memaksa penemuan unicast ke robot:

```xml
<initial_peers>
  <locator>
    <udpv4>
      <address>192.168.12.195</address>
    </udpv4>
  </locator>
</initial_peers>
```

**Penting:** `fastdds.xml` sisi robot TIDAK memerlukan `initial_peers` — menambahkannya merusak penemuan inbound. Robot menyiarkan secara normal via multicast di `ap0`; hanya mesin dev yang memerlukan konfigurasi peer eksplisit.

## Pengaturan Lingkungan (Mesin Dev)

Env vars yang diperlukan untuk akses topik ROS 2 dari distrobox:

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="file:///home/$(whoami)/fastdds.xml"
```

Penggunaan: `ros2 topic list --spin-time 10` (waktu spin default terlalu pendek untuk unicast WiFi)

## File yang Diubah

| File | Perubahan |
|------|-----------|
| `~/fastdds.xml` (mesin dev) | Menambahkan `initial_peers` yang menunjuk ke robot `192.168.12.195` |
| `/home/rspx1/fastdds.xml` (robot) | Dikembalikan ke asli (tanpa `initial_peers`) |
