# Laporan Sesi NEVO — 2026-08-25

## Ringkasan

Sesi mencakup: perbaikan connection stacking WebSocket, komputasi SoH Arduino, ekspansi telemetry baterai agar sesuai format TIFA, perbaikan kalibrasi SOH, dan verifikasi upload cloud.

---

## Perubahan yang Dilakukan

### 1. Perbaikan Connection Stacking WebSocket

**Masalah:** UI Cloud menampilkan 5+ instance koneksi duplikat yang menumpuk setelah mode swap. Setiap reconnect membuka koneksi TCP baru tanpa menutup yang lama.

**Akar masalah yang diidentifikasi (6 masalah):**

| # | Masalah | File |
|---|---------|------|
| 1 | Flag `reconnecting_` direset sebelum koneksi TCP selesai | `ws_client.cpp:256` |
| 2 | `forceReconnect()` tidak mengambil reconnecting guard | `ws_client.cpp:209` |
| 3 | `scheduleReconnect()` tidak memanggil `ws_.stop()` sebelum reconnect | `ws_client.cpp:254` |
| 4 | `scheduleReconnect()` memiliki panggilan `setUrl()` duplikat | `ws_client.cpp:242-248` |
| 5 | Jalur pesan SI ganda (WsClient + robot_api_node) | `robot_api_node.cpp:269-277` |
| 6 | Sumber pesan READY ganda (timer + networkStatusCallback) | `robot_api_node.cpp:88-96, 1487-1493` |

**Perbaikan yang diterapkan:**

| File | Perbaikan |
|------|-----------|
| `ws_client.cpp:77` | `reconnecting_.store(false)` dipindah ke Open callback (hanya setelah TCP sukses) |
| `ws_client.cpp:255` | Ditambahkan `ws_.stop()` sebelum `ws_.setUrl()` + `ws_.start()` |
| `ws_client.cpp:209-218` | `forceReconnect()` mengambil reconnecting guard |
| `ws_client.cpp:239-251` | Menghapus `setUrl()` duplikat dari cabang if/else |
| `robot_api_node.cpp:269-277` | Menghapus jalur SI-from-ROS-path redundan pada DUPLICATE_ROBOT_ID |
| `robot_api_node.cpp:1491` | Membatalkan ready timer saat `networkStatusCallback` mengirim READY |

### 2. Komputasi SoH Arduino

**Masalah:** Telemetri baterai hanya mengirim SoC. Tidak ada data State of Health atau resistansi internal.

**Hardware:** INA226 #1 (0x40) monitor baterai, INA226 #2 (0x44) monitor dock, 6S LiPo (22.8-25.0V, 6000mAh).

**Perbaikan di `INA226_Monitor.ino`:**

Ditambahkan dua metode estimasi SoH yang diblend via EMA:

1. **Pelacakan kapasitas:** mAh yang dihabiskan per siklus penuh vs rating 6000mAh
   - `saved_cycle_capacity = saved_cycle_capacity * 0.7 + cycle_mAh_discharged * 0.3`
   - `SoH = saved_cycle_capacity / 6000.0 * 100`

2. **Resistansi internal:** voltage sag di bawah beban vs baseline 80mΩ
   - `R_int = (V_oc - V_load) / I_load`
   - `SoH = baseline_R / measured_R * 100`

**Format serial diperluas dari 10 ke 13 field:**
```
<BatV,BatmA,SoC,DockmA,State,BatShunt,BatPwr,DockV,DockShunt,DockPwr,SoH,Rint_mOhm,Cycles>
```

**Persistensi EEPROM:** SoH, kapasitas siklus, jumlah siklus, v_oc_ref survive reboot (magic byte 0xB7).

### 3. Perbaikan Bug Kalibrasi SOH

**Masalah:** SOH menampilkan 24% pada baterai yang sehat. `v_oc_ref` (referensi voltage open-circuit) diinisialisasi ke `VOLTAGE_MAX` (25.0V) bukan voltage baterai aktual, menyebabkan perhitungan resistansi berpikir ada voltage sag yang besar.

**Contoh:** Pada SoC 72%, voltage aktual = 24.4V, tapi `v_oc_ref` = 25.0V → R_int = (25.0-24.4)/2.0 = 0.3Ω → SoH = 80/300 = 26% (sampah).

**Perbaikan:**
- `v_oc_ref` diinisialisasi dari voltage baterai aktual saat boot (bukan `VOLTAGE_MAX`)
- `v_oc_ref` diperbarui selama navigasi arus rendah (<500mA) untuk melacak voltage istirahat
- `v_oc_ref` hanya diperbarui selama pengisian pada SoC ≥ 95% DAN arus < 500mA (istirahat)

**Hasil:** SOH sekarang menampilkan 100.0% pada baterai baru, menurun secara alami seiring siklus pengisian.

### 4. Pembaruan NevoTelemetry.msg

Ditambahkan 3 field baru untuk mencocokkan data SoH dari Arduino:
```
float32 soh
float32 internal_resistance_ohm
int32 cycle_count
```

### 5. Pembaruan Parser ina226_bridge_node.py

Diperbarui untuk mengurai 13 field (sebelumnya 10):
- Ditambahkan: `msg.soh`, `msg.internal_resistance_ohm`, `msg.cycle_count`
- R_int dikonversi dari mΩ (serial) ke Ω (pesan ROS)

### 6. BATTERY JSON Disesuaikan dengan Format TIFA

**Sebelum (NEVO):**
```json
{
  "code": "BATTERY",
  "data": {
    "robot_id": "TFRB2", "ui_id": "TFUI2",
    "bus_voltage": 24.12, "battery_current": 1.23, "battery_percent": 85.5,
    "soh": 98.5, "internal_resistance_ohm": 0.0800, "cycle_count": 42
  }
}
```

**Sesudah (persis sama dengan TIFA):**
```json
{
  "code": "BATTERY",
  "data": {
    "robot_id": "TFRB2", "ui_id": "TFUI2",
    "bus_voltage": 24.12, "battery_current": 1.23, "battery_percent": 85.5,
    "dock_current": 0.0, "state_code": 0,
    "bat_shunt_voltage": 1.23, "bat_power": 29.67,
    "dock_voltage": 0.0, "dock_shunt_voltage": 0.0, "dock_power": 0.0,
    "soh": 98.5, "internal_resistance_mohm": 80.0, "cycle_count": 42
  }
}
```

**Perubahan kunci:** Ditambahkan `dock_current`, `state_code`, `bat_shunt_voltage`, `bat_power`, `dock_voltage`, `dock_shunt_voltage`, `dock_power`. Rename `internal_resistance_ohm` → `internal_resistance_mohm` (satuan: mΩ) agar sesuai TIFA.

---

## Verifikasi Upload Cloud

**Alur data terbukti berfungsi:**
```
Arduino CSV → ina226_bridge.py → nevo/sensor/battery
  → cloud_upload.cpp → nevo/ws_out (setiap 60s)
    → ws_bridge_node.cpp → ws_client.cpp → WSS ke tifa-ws.forgixrobotic.com
```

**Terverifikasi di log:**
- `ina226_bridge`: "BATTERY published: battery_percent=71.3 soh=100.0" ✓
- `cloud_upload`: "Uploaded Battery to Cloud: {"code":"BATTERY","data":{...}}" ✓

**Masalah tersisa:** Backend/frontend cloud di `tifa-ws.forgixrobotic.com` dan `tifa-web.forgixrobotic.com` hanya menampilkan `battery_percent` (SoC). Field baru (`bus_voltage`, `soh`, `internal_resistance_mohm`, `cycle_count`, dll.) dikirimkan tapi tidak di-render di UI. Memerlukan pembaruan backend cloud untuk menyimpan dan mengekspos field ini via `/api/battery` dan `/api/device-status`.

---

## File yang Diubah

| File | Perubahan |
|------|-----------|
| `nevo_ws_bridge/src/ws_client.cpp` | Memperbaiki 4 bug reconnect/stacking |
| `nevo_robot_api/src/robot_api_node.cpp` | Menghapus jalur SI duplikat, membatalkan ready timer |
| `nevo_bot/scripts/ina226_bridge_node.py` | Mengurai 13 field, memperluas BATTERY JSON |
| `nevo_msgs/msg/NevoTelemetry.msg` | Menambahkan soh, internal_resistance_ohm, cycle_count |
| `nevo_ws_bridge/src/cloud_upload.cpp` | Menambahkan debug logging sementara (dihapus) |
| `INA226_Monitor/INA226_Monitor.ino` | Menambahkan komputasi SoH, persistensi EEPROM |

## Masalah yang Diketahui

- **UI Cloud tidak menampilkan field baru:** Backend perlu mengurai/menyimpan/mengekspos bus_voltage, soh, dll.
- **Adaptor CP2102 longgar:** Masalah hardware menyebabkan disconnect rplidar intermiten
- **EKF "Failed to meet update rate":** Warning non-fatal saat startup
