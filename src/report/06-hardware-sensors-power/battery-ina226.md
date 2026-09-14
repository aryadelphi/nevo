# Baterai — INA226

Hardware monitor baterai INA226, komputasi SoH, kalibrasi, dan persistensi EEPROM.

**Laporan:** 2026-08-25, NEVO_NAVIGATION_SETUP_REPORT

---

## Hardware

| Komponen | Perangkat | Alamat I2C | Path |
|----------|-----------|------------|------|
| Monitor baterai | INA226 #1 | 0x40 | — |
| Monitor dock | INA226 #2 | 0x44 | — |
| Baterai | 6S LiPo | — | 22.8-25.0V, 6000mAh |

**Catatan:** Modul INA226 rusak pada unit robot saat ini — dilewati di konfigurasi hardware.

---

## Komputasi SoH (Arduino)

Dua metode estimasi yang diblend via EMA:

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

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `INA226_Monitor/INA226_Monitor.ino` | Menambahkan komputasi SoH, persistensi EEPROM |
| `nevo_bot/scripts/ina226_bridge_node.py` | Mengurai 13 field |
| `nevo_msgs/msg/NevoTelemetry.msg` | Menambahkan soh, internal_resistance_ohm, cycle_count |
