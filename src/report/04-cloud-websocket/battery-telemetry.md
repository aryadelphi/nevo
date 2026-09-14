# Telemetri Baterai

Integrasi monitor baterai INA226, format BATTERY JSON, komputasi SoH, dan upload cloud.

**Laporan:** 2026-08-25

---

## Alur Data

```
Arduino CSV → ina226_bridge.py → nevo/sensor/battery
  → cloud_upload.cpp → nevo/ws_out (setiap 60s)
    → ws_bridge_node.cpp → ws_client.cpp → WSS ke tifa-ws.forgixrobotic.com
```

---

## Format BATTERY JSON (Disesuaikan dengan TIFA)

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

**Perubahan kunci:** Ditambahkan `dock_current`, `state_code`, `bat_shunt_voltage`, `bat_power`, `dock_voltage`, `dock_shunt_voltage`, `dock_power`. Rename `internal_resistance_ohm` → `internal_resistance_mohm` (satuan: mΩ).

---

## Komputasi SoH Arduino

**Hardware:** INA226 #1 (0x40) monitor baterai, INA226 #2 (0x44) monitor dock, 6S LiPo (22.8-25.0V, 6000mAh).

Dua metode estimasi SoH yang diblend via EMA:

### 1. Pelacakan Kapasitas
mAh yang dihabiskan per siklus penuh vs rating 6000mAh:
```
saved_cycle_capacity = saved_cycle_capacity * 0.7 + cycle_mAh_discharged * 0.3
SoH = saved_cycle_capacity / 6000.0 * 100
```

### 2. Resistansi Internal
Voltage sag di bawah beban vs baseline 80mΩ:
```
R_int = (V_oc - V_load) / I_load
SoH = baseline_R / measured_R * 100
```

**Format serial (13 field):**
```
<BatV,BatmA,SoC,DockmA,State,BatShunt,BatPwr,DockV,DockShunt,DockPwr,SoH,Rint_mOhm,Cycles>
```

**Persistensi EEPROM:** SoH, kapasitas siklus, jumlah siklus, v_oc_ref survive reboot (magic byte 0xB7).

---

## Bug Kalibrasi SOH

**Masalah:** SOH menampilkan 24% pada baterai yang sehat. `v_oc_ref` diinisialisasi ke `VOLTAGE_MAX` (25.0V) bukan voltage baterai aktual.

**Contoh:** Pada SoC 72%, voltage aktual = 24.4V, tapi `v_oc_ref` = 25.0V → R_int = (25.0-24.4)/2.0 = 0.3Ω → SoH = 80/300 = 26%.

**Perbaikan:**
- `v_oc_ref` diinisialisasi dari voltage baterai aktual saat boot
- `v_oc_ref` diperbarui selama navigasi arus rendah (<500mA)
- `v_oc_ref` hanya diperbarui selama pengisian pada SoC ≥ 95% DAN arus < 500mA

**Hasil:** SOH sekarang menampilkan 100.0% pada baterai baru, menurun secara alami.

---

## Pembaruan NevoTelemetry.msg

Ditambahkan 3 field baru:
```
float32 soh
float32 internal_resistance_ohm
int32 cycle_count
```

---

## Pembaruan Parser ina226_bridge_node.py

Diperbarui untuk mengurai 13 field (sebelumnya 10):
- Ditambahkan: `msg.soh`, `msg.internal_resistance_ohm`, `msg.cycle_count`
- R_int dikonversi dari mΩ (serial) ke Ω (pesan ROS)

---

## Verified Working

```
ina226_bridge: "BATTERY published: battery_percent=71.3 soh=100.0" ✓
cloud_upload: "Uploaded Battery to Cloud: {"code":"BATTERY","data":{...}}" ✓
```

---

## Masalah yang Diketahui

- UI cloud tidak menampilkan field baru (SoH, resistansi internal, jumlah siklus) — backend perlu pembaruan

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `INA226_Monitor/INA226_Monitor.ino` | Menambahkan komputasi SoH, persistensi EEPROM |
| `nevo_bot/scripts/ina226_bridge_node.py` | Mengurai 13 field, memperluas BATTERY JSON |
| `nevo_msgs/msg/NevoTelemetry.msg` | Menambahkan soh, internal_resistance_ohm, cycle_count |
| `nevo_ws_bridge/src/cloud_upload.cpp` | Menambahkan debug logging sementara (dihapus) |
