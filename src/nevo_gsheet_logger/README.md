# nevo_gsheet_logger

ROS2 Python package untuk logging data pengujian navigasi NEVO
langsung ke Google Sheets secara real-time.

---

## Struktur Package

```
nevo_gsheet_logger/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── nevo_gsheet_logger
└── nevo_gsheet_logger/
    ├── __init__.py
    └── nevo_gsheet_node.py
```

---

## Instalasi

### 1. Install dependencies Python

```bash
pip install gspread google-auth --break-system-packages
```

### 2. Taruh credentials JSON

```bash
# Buat folder credentials
mkdir -p ~/nevo_ws/src/nevo_gsheet_logger/

# Copy file JSON ke sana (ganti nama sesuai file kamu)
cp /path/to/nevo-logging-xxxxx.json \
   ~/nevo_ws/src/nevo_gsheet_logger/credentials.json
```

> ⚠️ PENTING: Jangan commit credentials.json ke git.
> Tambahkan ke .gitignore:
> ```
> credentials.json
> ```

### 3. Copy package ke workspace ROS2

```bash
cp -r nevo_gsheet_logger ~/nevo_ws/src/
```

### 4. Build

```bash
cd ~/nevo_ws
colcon build --packages-select nevo_gsheet_logger
source install/setup.bash
```

---

## Cara Jalankan

Jalankan node dengan parameter sesuai sesi pengujian:

```bash
ros2 run nevo_gsheet_logger nevo_gsheet_node \
  --ros-args \
  -p uji_kode:=V1 \
  -p planner:=DWB \
  -p load_kg:=0.0 \
  -p obstacle_count:=0 \
  -p battery_start:=100 \
  -p koneksi:=Online \
  -p goal_sequence:="G1-G2-G3-G4-G5"
```

### Parameter lengkap

| Parameter         | Default                | Pilihan / Keterangan              |
|-------------------|------------------------|-----------------------------------|
| `uji_kode`        | `V1`                   | V1 / V2 / V3 / V4 / V5 / V6 / V7 |
| `planner`         | `DWB`                  | DWB / MPPI                        |
| `load_kg`         | `0.0`                  | 0.0 / 1.0 / 3.0 / 5.0            |
| `obstacle_count`  | `0`                    | 0 / 1 / 3 / 5                     |
| `battery_start`   | `100`                  | 100 / 50 / 20                     |
| `koneksi`         | `Online`               | Online / Offline / Degraded       |
| `goal_sequence`   | `G1-G2-G3-G4-G5`       | Sesuaikan urutan goal aktual      |
| `spreadsheet_id`  | (sudah diset di node)  | ID Google Sheets kamu             |
| `credentials_path`| `~/nevo_ws/src/.../credentials.json` | Path ke file JSON |

---

## Contoh per Variabel Uji

### V1 — Kepadatan 3 orang
```bash
ros2 run nevo_gsheet_logger nevo_gsheet_node \
  --ros-args -p uji_kode:=V1 -p obstacle_count:=3
```

### V2 — Load 3 kg
```bash
ros2 run nevo_gsheet_logger nevo_gsheet_node \
  --ros-args -p uji_kode:=V2 -p load_kg:=3.0
```

### V3 — Planner MPPI
```bash
ros2 run nevo_gsheet_logger nevo_gsheet_node \
  --ros-args -p uji_kode:=V3 -p planner:=MPPI -p obstacle_count:=1
```

### V6 — Battery 50%
```bash
ros2 run nevo_gsheet_logger nevo_gsheet_node \
  --ros-args -p uji_kode:=V6 -p battery_start:=50
```

---

## Alur Data

```
ROS2 Topics                Node                  Google Sheets
──────────────             ──────────────────    ─────────────────────
/plan              ──────→ replan_count     ──→  Kolom L (Replanning)
/odometry/filtered ──────→ ekf_distance    ──→  Kolom J (EKF Distance)
                           avg/max velocity ──→  Kolom M, N
/cmd_vel           ──────→ cmd_vel tracking
/navigate_to_pose  ──────→ start/end goal  ──→  Baris baru per goal selesai
  /_action/status          duration        ──→  Kolom K (Waktu Sistem)
```

### Kolom yang diisi node otomatis (warna hijau di Sheets)
- A: No Run (format R001-L1 dst)
- B: Tanggal & waktu
- C–I: Parameter sesi (dari ros2 run arguments)
- J: EKF Distance (m)
- K: Waktu Sistem (s)
- L: Replanning Count
- M: Avg Velocity Sistem (m/s)
- N: Max Velocity (m/s)
- T: Status (default VALID)

### Kolom yang diisi manual operator setelah tiap run
- O: Waktu Stopwatch (s) ← ground truth
- S: Success (TRUE/FALSE)
- T: Ubah ke INVALID jika run bermasalah
- U: Notes / observasi lapangan

### Kolom formula otomatis di Sheets (jangan diubah manual)
- P: Error Waktu = K - O
- Q: Kecepatan Aktual = J / O
- R: Error Kecepatan = M - Q

---

## Numbering Run

Format: `R{run_number:03d}-L{leg_number}`

Contoh dalam satu sesi 2 run (10 goal):
```
R001-L1  → Run 1, Goal ke-1
R001-L2  → Run 1, Goal ke-2
R001-L3  → Run 1, Goal ke-3
R001-L4  → Run 1, Goal ke-4
R001-L5  → Run 1, Goal ke-5 (run 1 selesai)
R002-L1  → Run 2, Goal ke-1
...
```

Node otomatis lanjutkan nomor run dari data terakhir di sheet,
jadi aman kalau node direstart di tengah sesi.

---

## Fallback Mode

Jika koneksi ke Google Sheets gagal (internet putus), node otomatis
simpan data ke file CSV lokal:

```
~/nevo_ws/logs/nevo_V1_fallback.csv
```

Setelah koneksi pulih, import CSV ini ke sheet yang sesuai.

---

## Topics yang Di-subscribe

| Topic | Message Type | Keterangan |
|-------|-------------|------------|
| `/plan` | `nav_msgs/Path` | Trigger replan count++ |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF distance + real velocity |
| `/cmd_vel` | `geometry_msgs/Twist` | Command velocity |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | Status 2=start, 4=success, 6=abort |
