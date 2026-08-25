# nevo_gsheet_logger_v2 v2

ROS2 Python package untuk logging data pengujian NEVO ke Google Sheets real-time.

## Changelog v2
- Fix: run counter tidak lagi baca baris legenda sheet
- Fix: kolom formula (R/S/T) tidak di-overwrite oleh node
- Fix: Success dan Status otomatis dari Nav2 (tidak perlu input manual)
- NEW: Subscribe `/nevo/sensor/battery` → battery_start & battery_end otomatis
- NEW: Parameter `battery_start` dihapus (tidak perlu di-set manual lagi)

---

## Instalasi

```bash
# 1. Install dependencies Python
pip install gspread google-auth

# 2. Taruh credentials JSON
cp nevo-logging-xxxxx.json ~/nevo_ws/src/nevo_gsheet_logger_v2/credentials.json

# 3. Copy ke workspace dan build
cp -r nevo_gsheet_logger_v2 ~/nevo_ws/src/
cd ~/nevo_ws
colcon build --packages-select nevo_gsheet_logger_v2
source install/setup.bash
```

---

## Cara Jalankan

```bash
ros2 run nevo_gsheet_logger_v2 nevo_gsheet_node \
  --ros-args \
  -p uji_kode:=V1 \
  -p planner:=DWB \
  -p load_kg:=0.0 \
  -p obstacle_count:=0 \
  -p koneksi:=Online \
  -p goal_sequence:="G1-G2-G3-G4-G5"
```

### Parameter (v2 — battery_start sudah tidak ada)

| Parameter        | Default           | Pilihan                            |
|------------------|-------------------|------------------------------------|
| `uji_kode`       | `V1`              | V1/V2/V3/V4/V5/V6/V7              |
| `planner`        | `DWB`             | DWB / MPPI                         |
| `load_kg`        | `0.0`             | 0.0 / 1.0 / 3.0 / 5.0             |
| `obstacle_count` | `0`               | 0 / 1 / 3 / 5                      |
| `koneksi`        | `Online`          | Online / Offline / Degraded        |
| `goal_sequence`  | `G1-G2-G3-G4-G5`  | Sesuaikan urutan goal aktual       |

---

## Struktur Kolom Spreadsheet (v2)

| Kolom | Field            | Sumber  | Keterangan                        |
|-------|------------------|---------|-----------------------------------|
| A     | No Run           | Node    | Format R001-L1                    |
| B     | Tanggal          | Node    | dd/mm/yyyy HH:MM                  |
| C     | Kode Uji         | Node    | Dari parameter                    |
| D     | Planner          | Node    | Dari parameter                    |
| E     | Load (kg)        | Node    | Dari parameter                    |
| F     | Obstacle Count   | Node    | Dari parameter                    |
| G     | Battery Start(%) | Node    | Otomatis dari /nevo/sensor/battery|
| H     | Battery End (%)  | Node    | Otomatis dari /nevo/sensor/battery|
| I     | Battery Drop (%) | Node    | G - H                             |
| J     | Koneksi          | Node    | Dari parameter                    |
| K     | Goal Sequence    | Node    | Dari parameter                    |
| L     | EKF Distance (m) | Node    | Dari /odometry/filtered           |
| M     | Waktu Sistem (s) | Node    | Duration Nav2                     |
| N     | Replanning Count | Node    | Dari /plan                        |
| O     | Avg Velocity     | Node    | Dari /odometry/filtered           |
| P     | Max Velocity     | Node    | Dari /odometry/filtered           |
| Q     | Waktu Stopwatch  | MANUAL  | Ground truth operator             |
| R     | Error Waktu      | FORMULA | =M-Q                              |
| S     | V Aktual (m/s)   | FORMULA | =L/Q                              |
| T     | Error Velocity   | FORMULA | =O-S                              |
| U     | Success          | Node    | TRUE/FALSE otomatis Nav2          |
| V     | Status           | Node    | VALID/INVALID otomatis Nav2       |
| W     | Notes            | MANUAL  | Observasi lapangan                |

---

## Logika Battery

- Node subscribe `/nevo/sensor/battery` (JSON field: `battery_percent`)
- `battery_start` diambil saat robot mulai bergerak (status Nav2 = 2)
- `battery_end` diambil saat robot selesai/gagal (status Nav2 = 4 atau 6)
- Kalau topic battery belum ada data, nilai tercatat sebagai `N/A`

## Logika Success & Status

| Nav2 Status | Success | Status  |
|-------------|---------|---------|
| 4 SUCCEEDED | TRUE    | VALID   |
| 6 ABORTED   | FALSE   | INVALID |

Operator tetap bisa override kolom V (Status) secara manual di Sheets
jika ada kondisi khusus yang tidak ter-capture Nav2.

---

## Fallback CSV

Jika Sheets tidak tersambung, data otomatis disimpan ke:
```
~/nevo_ws/logs/nevo_V1_fallback.csv
```
