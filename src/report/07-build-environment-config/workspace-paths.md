# Workspace & Path

Rename workspace, migrasi path hardcoded, dan perubahan struktur direktori.

**Laporan:** 2026-08-24, NEVO_NAVIGATION_SETUP_REPORT

---

## Rename Workspace

| Sebelum | Sesudah |
|---------|---------|
| `tifa` | `nevo` |
| `docking_ws` | `nevo_ws` |

**Path workspace:**
- `/home/nevo/nevo/` — copy
- `/home/nevo/nevo_ws/` — workspace aktif

---

## Migrasi Path Hardcoded

**Masalah:** ~15 file sumber memiliki path hardcoded ke `/home/rspx1/nevo_ws/`.

**Perbaikan:** Semua diganti dengan `/home/nevo/nevo_ws/` di seluruh codebase.

---

## Perubahan Peta Default

| Sebelum | Sesudah |
|---------|---------|
| `telkop_save_map.yaml` | `abie_lab_save.yaml` (lokasi saat ini) |

**File yang diubah:**
- `launch_robot.launch.py` — path `default_map`
- `hw_nav.launch.py` — path `default_map`

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| ~15 file sumber | `/home/rspx1/nevo_ws/` → `/home/nevo/nevo_ws/` |
| `launch_robot.launch.py` | Path peta default |
| `hw_nav.launch.py` | Path peta default |
