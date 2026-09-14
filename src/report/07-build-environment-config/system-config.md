# Konfigurasi Sistem

Lingkungan shell, sudoers, definisi pesan, perbaikan QoS, dan konfigurasi sistem miscellaneous.

**Laporan:** 2026-08-25, NEVO_NAVIGATION_SETUP_REPORT

---

## Lingkungan Shell (.zshrc)

**Masalah:** Sesi SSH tidak memiliki variabel lingkungan ROS2.

**Perbaikan:** Ditambahkan ROS2 Jazzy lengkap + sourcing workspace + env vars ke `~/.zshrc`:

```bash
export AMENT_PREFIX_PATH=/opt/ros/jazzy:/home/nevo/nevo_ws/install
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/nevo/nevo_ws/install/lib/python3.12/site-packages
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:/home/nevo/nevo_ws/install/lib
export PATH=/opt/ros/jazzy/bin:$PATH
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## Sudoers untuk systemd-run

**Masalah:** `robot_api_node` tidak bisa me-restart service tanpa password.

**Perbaikan:** Dibuat `/etc/sudoers.d/nevo-robot`:
```
nevo ALL=(root) NOPASSWD: /usr/bin/systemd-run
```

---

## Pembaruan NevoTelemetry.msg

Ditambahkan 3 field baru untuk mencocokkan data SoH dari Arduino:
```
float32 soh
float32 internal_resistance_ohm
int32 cycle_count
```

---

## Pembaruan Parser ina226_bridge

Diperbarui untuk mengurai 13 field (sebelumnya 10):
- Ditambahkan: `msg.soh`, `msg.internal_resistance_ohm`, `msg.cycle_count`
- R_int dikonversi dari mΩ (serial) ke Ω (pesan ROS)

---

## Perbaikan QoS /map Node Diagnostik

**Masalah:** `nevo_diagnostic_node` tidak bisa menerima `/map` karena ketidakcocokan QoS.

**Perbaikan:** QoS diubah ke `reliable().transient_local()` — terkonfirmasi `map:OK`.

---

## Peringatan Sourcing .setup.sh

**Masalah:** ROS2 `setup.bash` mencoba source yang tidak ada `/home/nevo/setup.sh`.

**Status:** Hanya kosmetik — tidak ada dampak fungsional.

---

## Overrun controller_manager

**Masalah:** Loop ros2_control (50Hz) sesekali overrun di Pi5 (20-40ms siklus).

**Status:** Non-kritis tapi perlu dipantau.

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `~/.zshrc` | Menambahkan env ROS2 Jazzy + sourcing workspace |
| `/etc/sudoers.d/nevo-robot` | systemd-run tanpa password |
| `nevo_msgs/msg/NevoTelemetry.msg` | Menambahkan soh, internal_resistance_ohm, cycle_count |
| `nevo_bot/scripts/ina226_bridge_node.py` | Mengurai 13 field |
| Node diagnostik | QoS diubah ke reliable().transient_local() |
