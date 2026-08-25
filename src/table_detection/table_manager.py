#!/usr/bin/env python3

import json, os, math

TABLES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tables.json')


def load_tables():
    if not os.path.exists(TABLES_FILE):
        return []
    try:
        with open(TABLES_FILE, 'r') as f:
            return json.load(f)
    except Exception:
        return []


def save_tables(tables):
    with open(TABLES_FILE, 'w') as f:
        json.dump(tables, f, indent=2)


def list_tables():
    return load_tables()


def find_matching_table(cx, cy, tables, radius=80):
    """Cocokkan deteksi YOLO (cx, cy) ke meja terdaftar berdasarkan radius piksel."""
    best      = None
    best_dist = float('inf')
    for t in tables:
        dist = math.sqrt((cx - t['cx'])**2 + (cy - t['cy'])**2)
        if dist < radius and dist < best_dist:
            best_dist = dist
            best = t
    return best


def register_table(cx, cy, confidence, label, table_id,
                   world_x=None, world_y=None,
                   nav_x=None,   nav_y=None, nav_yaw=None):
    """
    Daftarkan meja baru atau update meja yang sudah ada.

    Args:
        cx, cy      : posisi piksel center bbox YOLO
        confidence  : confidence score YOLO
        label       : 'meja_bersih' | 'meja_terisi' | 'meja_kotor'
        table_id    : nomor meja (int)
        world_x/y   : koordinat pusat meja di peta SLAM (meter)
        nav_x/y     : koordinat goal robot (sudah di-offset, meter)
        nav_yaw     : orientasi robot saat tiba (radian)
    """
    tables = load_tables()
    tables = [t for t in tables if t['id'] != table_id]

    entry = {
        'id':         table_id,
        'cx':         cx,
        'cy':         cy,
        'label':      label,
        'confidence': round(confidence, 4),
        'world_x':    world_x,
        'world_y':    world_y,
        'nav_x':      nav_x,
        'nav_y':      nav_y,
        'nav_yaw':    nav_yaw,
    }

    tables.append(entry)
    tables.sort(key=lambda t: t['id'])
    save_tables(tables)
    return entry


def update_table_label(table_id, new_label):
    """Update label/status meja yang sudah terdaftar."""
    tables = load_tables()
    for t in tables:
        if t['id'] == table_id:
            t['label'] = new_label
            save_tables(tables)
            return True
    return False


def update_table_coords(table_id, world_x, world_y,
                        nav_x=None, nav_y=None, nav_yaw=None):
    """Update koordinat meja yang sudah terdaftar."""
    tables = load_tables()
    for t in tables:
        if t['id'] == table_id:
            t['world_x'] = round(float(world_x), 3)
            t['world_y'] = round(float(world_y), 3)
            if nav_x   is not None: t['nav_x']   = round(float(nav_x),   3)
            if nav_y   is not None: t['nav_y']   = round(float(nav_y),   3)
            if nav_yaw is not None: t['nav_yaw'] = round(float(nav_yaw), 4)
            save_tables(tables)
            return True
    return False


def get_tables_missing_coords():
    """Return list meja yang belum punya nav_x/nav_y."""
    return [t for t in load_tables()
            if t.get('nav_x') is None]


def get_tables_nav2_ready():
    """Return list meja yang sudah siap dipakai Nav2."""
    return [t for t in load_tables()
            if t.get('nav_x') is not None]
