# ini yang paling bagus

#!/usr/bin/env python3

import requests
import rclpy, threading, time, cv2, os, json, asyncio
import websockets
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from flask import Flask, jsonify, request
from detector import Detector
from table_manager import load_tables, list_tables, save_tables



# ── Model / file paths ── (bisa override via env)
MODEL_FILENAME = os.environ.get('MODEL_FILENAME', 'best_yolov26n.onnx')
MODEL_PATH     = os.path.join(os.path.dirname(os.path.abspath(__file__)), MODEL_FILENAME)
MODEL_NAME     = os.path.basename(MODEL_PATH).replace('.onnx', '')
PATROL_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'patrol_log.json')

# ── Timing (bisa override via env) ──
YOLO_INTERVAL    = float(os.environ.get('YOLO_INTERVAL',    '1.0'))
MOVE_READY_DELAY = float(os.environ.get('MOVE_READY_DELAY', '1.5'))
WS_RECONNECT_DELAY = float(os.environ.get('WS_RECONNECT_DELAY', '3.0'))
IDLE_WAIT_TIMEOUT  = float(os.environ.get('IDLE_WAIT_TIMEOUT',  '5.0'))

# ── WebSocket config ── (bisa override via env)
WSL2_IP      = os.environ.get('WSL2_IP',      '127.0.0.1')
WSL2_WS_PORT = int(os.environ.get('WSL2_WS_PORT', '8765'))
WSL2_WS_URI  = f'ws://{WSL2_IP}:{WSL2_WS_PORT}'
ROBOT_ID     = os.environ.get('ROBOT_ID', 'TFRB2')
UI_ID        = os.environ.get('UI_ID',    'TFUI2')

# ── Debug flags ──
SKIP_YOLO_SCAN = os.environ.get('SKIP_YOLO_SCAN', '0') == '1'

# ── API & config ──
API_BASE    = os.environ.get('API_BASE',  'https://tifa-be.forgixrobotic.com')
WEB_PORT    = int(os.environ.get('WEB_PORT', '5000'))
CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'patrol_config.json')

# DEFAULT_MAP_ID: fallback jika belum pernah pilih map dari UI.
# Kosongkan ("") jika ingin robot menunggu sampai operator memilih map dulu.
DEFAULT_MAP_ID      = os.environ.get('DEFAULT_MAP_ID', '91')
DEFAULT_PATROL_ORDER = [f'table_{i}' for i in range(1, 13)]



def load_config():
    """Baca map_id & urutan patroli yang tersimpan (kalau ada), supaya
    pilihan operator tetap dipakai walau main.py di-restart.

    map_id yang terakhir dipilih user dari UI akan selalu dipakai kembali
    saat restart. Kalau belum ada config tersimpan, fallback ke DEFAULT_MAP_ID.
    """
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                cfg = json.load(f)
            saved_map   = cfg.get('map_id',       DEFAULT_MAP_ID)
            saved_order = cfg.get('patrol_order', DEFAULT_PATROL_ORDER)
            print(f"[CONFIG] Map tersimpan: {saved_map}, urutan: {saved_order}")
            return saved_map, saved_order
        except (OSError, json.JSONDecodeError) as e:
            print(f"[CONFIG] Gagal baca {CONFIG_FILE}: {e}, pakai default")
    return DEFAULT_MAP_ID, list(DEFAULT_PATROL_ORDER)


def save_config():
    tmp_path = CONFIG_FILE + '.tmp'
    with open(tmp_path, 'w') as f:
        json.dump({'map_id': MAP_ID, 'patrol_order': PATROL_ORDER}, f, indent=2)
    os.replace(tmp_path, CONFIG_FILE)


MAP_ID, PATROL_ORDER = load_config()
MAP_NAME = "?"   # di-resolve dari API waktu startup (lihat resolve_map_name())


def resolve_map_name(map_id: str) -> str:
    """Cari map_name dari daftar /api/maps berdasarkan map_id. Selalu
    tanya API (bukan cache statis) supaya nama yang ditampilkan konsisten
    sama yang beneran ada di server."""
    for m in fetch_map_list():
        if str(m.get('map_id')) == str(map_id):
            return m.get('map_name') or map_id
    return map_id  # fallback: tampilkan ID kalau nama tidak ketemu
# Waypoints koordinat TEL-U Coffee
WAYPOINTS = {}
WAYPOINTS_CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints_cache.json')

def fetch_waypoints():
    """Mengambil koordinat tujuan patroli dari server Forgix secara dinamis.
    Kalau gagal (mis. robot tidak ada akses internet), pakai cache lokal
    hasil fetch sukses terakhir, supaya patroli tetap bisa jalan."""
    global WAYPOINTS

    url = f"{API_BASE}/api/robot-control?action=all-goals&mapId={MAP_ID}"

    try:
        print(f"[API] Mengambil koordinat meja dari map {MAP_ID}...")
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        goals_data = response.json().get('data', [])
        new_wp = {}

        for item in goals_data:
            name_api = str(item.get('goal_name', '')).upper()

            # Mapping nama dari API ke format internal main.py
            if name_api == 'HOME':
                key = 'home'
            elif name_api.startswith('MEJA_') or name_api.startswith('TABLE_'):
                num = name_api.split('_')[1]
                key = f'table_{num}'
            else:
                continue

            new_wp[key] = {
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'yaw': float(item.get('yaw', 0.0))
            }

        if new_wp:
            WAYPOINTS = new_wp
            try:
                with open(WAYPOINTS_CACHE_FILE, 'w') as f:
                    json.dump(WAYPOINTS, f, indent=2)
            except OSError as e:
                print(f"[API] Gagal simpan cache lokal: {e}")
            print(f"[API] Sukses! Memuat {len(WAYPOINTS)} titik koordinat (termasuk Home).")
        else:
            print("[API] ⚠️ Response API kosong / format goal_name tidak dikenali.")

    except Exception as e:
        print(f"[API] Gagal mengambil data koordinat: {e}")

    # Fallback ke cache lokal kalau fetch barusan gagal/kosong
    if not WAYPOINTS and os.path.exists(WAYPOINTS_CACHE_FILE):
        try:
            with open(WAYPOINTS_CACHE_FILE) as f:
                WAYPOINTS = json.load(f)
            print(f"[API] Pakai cache lokal: {len(WAYPOINTS)} titik koordinat.")
        except (OSError, json.JSONDecodeError) as e:
            print(f"[API] Gagal baca cache lokal: {e}")

    if not WAYPOINTS:
        print("[API] ⚠️⚠️⚠️ WAYPOINTS KOSONG TOTAL — patroli TIDAK akan bisa "
              "jalan sampai ini diperbaiki (cek koneksi internet Raspi ke "
              f"{API_BASE}, atau isi manual WAYPOINTS di kode). ⚠️⚠️⚠️")

    # Kalau map yang dipilih ternyata tidak punya semua meja yang ada di
    # PATROL_ORDER tersimpan (mis. ganti ke map dengan jumlah meja beda),
    # buang otomatis meja yang tidak ada, supaya tidak error waktu patroli.
    global PATROL_ORDER
    valid_order = [t for t in PATROL_ORDER if t in WAYPOINTS]
    if valid_order != PATROL_ORDER:
        removed = [t for t in PATROL_ORDER if t not in WAYPOINTS]
        print(f"[PATROL_ORDER] Meja {removed} tidak ada di map ini, dibuang dari urutan patroli")
        PATROL_ORDER = valid_order
    if not PATROL_ORDER:
        # Map baru, belum ada urutan tersimpan yang cocok -> pakai semua
        # meja yang tersedia di map ini, urut angka menaik.
        table_keys = sorted(
            (k for k in WAYPOINTS if k.startswith('table_')),
            key=lambda k: int(k.split('_')[1])
        )
        PATROL_ORDER = table_keys
        print(f"[PATROL_ORDER] Urutan default dipakai: {PATROL_ORDER}")


def fetch_map_list():
    """Ambil daftar semua map yang tersedia di server Forgix, untuk
    ditampilkan sebagai pilihan di Web UI."""
    url = f"{API_BASE}/api/maps"
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()
        maps = response.json().get('data', [])
        return [{'map_id': m.get('map_id'), 'map_name': m.get('map_name')} for m in maps]
    except Exception as e:
        print(f"[API] Gagal mengambil daftar map: {e}")
        return []


app = Flask(__name__)

# ── Global Variables & Locks ──
frame_lock = threading.Lock()
latest_frame = None
latest_frame_time = 0.0

annotated_lock = threading.Lock()
annotated_frame = None
annotated_frame_time = 0.0

status_lock    = threading.Lock()
fps_lock       = threading.Lock()
patrol_lock    = threading.Lock()
ws_lock        = threading.Lock()

# ── Shared state ──
latest_frame       = None
annotated_frame    = None
detector           = None
last_detections    = []
robot_status       = 'idle'   # idle | moving | interrupted
fps_value          = 0.0
node_ref           = None
force_scan         = False
last_error         = None    # pesan error terakhir dari robot_api (ditampilkan di Web UI)

# ── Snapshot state ──
snapshot_jpeg_lock = threading.Lock()
snapshot_jpeg      = None

# ── Patrol state ──
patrol_active   = False
patrol_index    = 0
patrol_retry_count = 0
patrol_summary  = {}
patrol_done     = False
current_target  = None
current_seq     = -1
latest_frame_time = 0.0

# ── WebSocket state ──
ws_connection   = None
ws_connected    = False
ws_loop         = None


# ══════════════════════════════════════════════════════════════
# WEBSOCKET CLIENT
# ══════════════════════════════════════════════════════════════

async def ws_send(payload: dict):
    """Kirim pesan ke WSL2 via WebSocket."""
    global ws_connection, ws_connected
    with ws_lock:
        ws = ws_connection
    if ws and ws_connected:
        try:
            await ws.send(json.dumps(payload))
            print(f"[WS] Sent: {payload.get('code', '?')}")
        except Exception as e:
            print(f"[WS] Send error: {e}")
    else:
        print(f"[WS] Not connected, skip send: {payload.get('code', '?')}")


def ws_send_sync(payload: dict):
    """Thread-safe wrapper untuk kirim WebSocket dari thread non-async."""
    global ws_loop
    if ws_loop and ws_loop.is_running():
        asyncio.run_coroutine_threadsafe(ws_send(payload), ws_loop)


async def on_ws_message(message: str):
    """
    Handle pesan dari WSL2.
    Format: {"code": "GOAL_COMPLETE", "data": {...}}
    """
    global robot_status, force_scan, patrol_index, current_target
    global patrol_active, patrol_summary, patrol_done, last_error, patrol_retry_count

    try:
        msg = json.loads(message)
        code = msg.get('code', '')
        data = msg.get('data', {})

        print(f"[WS] Received: {code}")

        if code == 'CONNECTED':
            print(f"[WS] Server: {data.get('message', '')}")

        elif code == 'GOAL_COMPLETE':
            # Robot tiba di tujuan
            sequence = data.get('sequence', -1)
            print(f"[WS] GOAL_COMPLETE seq={sequence} target={current_target}")

            with status_lock:
                robot_status = 'idle'
                last_error   = None   # goal sukses → bersihkan error lama

            if patrol_active and current_target and current_target != 'home':
                if SKIP_YOLO_SCAN:
                    # DEBUG: lewati scan YOLO sepenuhnya, langsung lanjut ke
                    # meja berikutnya. Dipakai untuk isolasi masalah navigasi
                    # tanpa terganggu urusan kamera/deteksi.
                    print(f"[PATROL] SKIP_YOLO_SCAN aktif — lewati scan di {current_target}")
                    with patrol_lock:
                        patrol_summary[current_target] = {
                            'label': 'unknown', 'confidence': 0.0,
                            'timestamp': time.time(), 'note': 'scan dilewati (debug)',
                        }
                        snapshot = dict(patrol_summary)
                    write_patrol_log(snapshot)
                    print(f"[WS] Mengirim PICKUP_CONFIRM otomatis (SKIP_YOLO_SCAN)...")
                    ws_send_sync({'code': 'PICKUP_CONFIRM', 'data': {'robot_id': ROBOT_ID, 'sequence': current_seq}})
                    patrol_next()
                else:
                    # Tiba di meja → trigger YOLO scan (YOLO loop yang akan kirim PICKUP_CONFIRM)
                    print(f"[WS] Tiba di {current_target} → Kamera standby mencari meja...")
                    force_scan = True

            elif patrol_active and current_target == 'home':
                # Tiba di home setelah patroli → kirim PATROL_DONE
                print('[WS] Tiba di home → patroli selesai')
                patrol_active = False
                _finish_patrol()

        elif code == 'ACK':
            command_id = data.get('command_id', '')
            status_val = data.get('status', '')
            print(f"[WS] ACK: command_id={command_id} status={status_val}")

            if command_id == 'MOVE' and status_val == 'ACCEPTED':
                # Robot benar-benar mulai gerak — INI satu-satunya tempat
                # status berubah jadi 'moving', bukan pas kita kirim MOVE.
                with status_lock:
                    robot_status = 'moving'
                    last_error   = None
                print('[WS] robot_api konfirmasi MOVE diterima → status = moving')

            elif command_id == 'INTERRUPT' and status_val == 'ACCEPTED':
                # Stop beneran berhasil — robot berhenti persis di tempat
                with status_lock:
                    robot_status = 'interrupted'
                    last_error   = None
                print('[WS] Robot benar-benar berhenti (INTERRUPTED)')

            elif command_id == 'INTERRUPT' and status_val == 'REJECTED':
                with status_lock:
                    last_error = 'Stop ditolak robot_api (robot mungkin sudah IDLE, tidak sedang bergerak)'

            elif command_id == 'INTERRUPT_CONFIRM' and status_val == 'RETURNING_HOME':
                with status_lock:
                    robot_status = 'moving'
                    last_error   = None
                print('[WS] Robot mulai jalan pulang ke Home')

        elif code == 'ERROR':
            print(f"[WS] ERROR dari WSL2: {data}")
            with status_lock:
                last_error = data.get('error_message', str(data))
                # Set ke idle agar sistem siap menerima perintah ulang
                robot_status = 'idle'

            if patrol_active:
                with patrol_lock:
                    patrol_retry_count += 1
                    retry_limit = 2
                    is_exceeded = patrol_retry_count > retry_limit
                    target_to_act = current_target

                if is_exceeded:
                    print(f'[PATROL] Gagal mencapai {target_to_act} setelah {retry_limit} kali coba. Melewati meja ini.')
                    def skip_continue():
                        time.sleep(2.0)
                        with patrol_lock:
                            if patrol_active and current_target:
                                patrol_summary[current_target] = {
                                    'label': 'unknown',
                                    'confidence': 0.0,
                                    'timestamp': time.time(),
                                    'note': f"Dilewati karena macet permanen (retry {retry_limit}x): {last_error}"
                                }
                                snapshot = dict(patrol_summary)
                                
                        if patrol_active and current_target:
                            write_patrol_log(snapshot)
                            patrol_next()
                            
                    threading.Thread(target=skip_continue, daemon=True).start()
                else:
                    print(f'[PATROL] Terjadi macet. Percobaan ulang ke-{patrol_retry_count}/{retry_limit} menuju {target_to_act} dalam 3 detik...')
                    def auto_retry():
                        time.sleep(3.0)
                        with patrol_lock:
                            if not (patrol_active and current_target):
                                return
                            seq = patrol_index + 1 if target_to_act != 'home' else -1
                            m_type = 'PATROL' if target_to_act != 'home' else 'HOMEBASE'
                            
                        print(f"[PATROL] Retry otomatis: Mengirim ulang perintah menuju {target_to_act}...")
                        send_move(seq, target_to_act, m_type)

                threading.Thread(target=auto_retry if not is_exceeded else lambda: None, daemon=True).start()

    except (json.JSONDecodeError, ValueError) as e:
        print(f"[WS] Parse error: {e}")


async def ws_client_loop():
    """Loop utama WebSocket client — auto reconnect."""
    global ws_connection, ws_connected, ws_loop, robot_status
    ws_loop = asyncio.get_event_loop()

    while True:
        try:
            print(f"[WS] Connecting to {WSL2_WS_URI}...")
            async with websockets.connect(WSL2_WS_URI) as ws:
                with ws_lock:
                    ws_connection = ws
                    ws_connected  = True
                print(f"[WS] Connected to {WSL2_WS_URI}")

                # [ANTI ZOMBIE STATE] Sinkronisasi saat Reconnect
                with status_lock:
                    if robot_status == 'moving':
                        print("[WS] Reconnect: Memaksa status kembali 'idle' agar tidak stuck (zombie state).")
                        robot_status = 'idle'
                
                if patrol_active and current_target:
                    print(f"[WS] Reconnect: Menyinkronkan ulang patroli yang terputus, retry {current_target}...")
                    threading.Thread(target=lambda: send_move(
                        patrol_index + 1 if current_target != 'home' else -1,
                        current_target,
                        'PATROL' if current_target != 'home' else 'HOMEBASE'
                    ), daemon=True).start()

                async for message in ws:
                    await on_ws_message(message)

        except Exception as e:
            print(f"[WS] Disconnected: {e}")
        finally:
            with ws_lock:
                ws_connection = None
                ws_connected  = False

        print("[WS] Reconnect dalam {:.0f} detik...".format(WS_RECONNECT_DELAY))
        await asyncio.sleep(WS_RECONNECT_DELAY)


def start_ws_thread():
    """Jalankan WebSocket client di thread terpisah."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(ws_client_loop())


# ══════════════════════════════════════════════════════════════
# PATROL LOGIC
# ══════════════════════════════════════════════════════════════

def send_move(sequence: int, target: str, move_type: str = 'GENERIC'):
    """Kirim perintah MOVE ke WSL2 via WebSocket.
    PENTING: status 'moving' TIDAK di-set di sini. Robot baru dianggap
    benar-benar bergerak setelah robot_api membalas ACK MOVE/ACCEPTED
    (lihat on_ws_message). Kalau robot_api menolak (ERROR), status tetap
    apa adanya dan last_error akan menunjukkan alasannya."""
    global current_target, last_error, current_seq
    current_seq = sequence
    wp = WAYPOINTS.get(target)
    if not wp:
        msg = (f"Waypoint '{target}' tidak ditemukan (WAYPOINTS ada "
               f"{len(WAYPOINTS)} entri) — cek koneksi internet Raspi "
               f"ke {API_BASE}, MOVE dibatalkan.")
        print(f"[PATROL] {msg}")
        with status_lock:
            last_error = msg
        return

    current_target = target

    # KITA TIDAK LAGI MENGIRIM 'home_base' (Sesuai dokumentasi/API).
    # Jika 'home_base' tidak dikirim, robot_api tidak akan memaksa pulang ke home
    # setiap kali PICKUP_CONFIRM (tablet) dipencet, melainkan akan kembali ke status IDLE.

    payload = {
        'code': 'MOVE',
        'data': {
            'robot_id': ROBOT_ID,
            'ui_id':    UI_ID,
            'sequence': sequence,
            'type':     move_type,
            'target':   target,
            'dest': {
                'x':   wp['x'],
                'y':   wp['y'],
                'yaw': wp['yaw'],
            },
            'timestamp': time.time(),
        }
    }
    ws_send_sync(payload)
    print(f"[PATROL] MOVE seq={sequence} → {target} ({wp['x']:.2f}, {wp['y']:.2f})")


def start_patrol():
    """Mulai patroli dari meja pertama."""
    global patrol_active, patrol_index, patrol_summary, patrol_done, current_target

    # Guard: waypoints harus sudah termuat sebelum patroli bisa dimulai
    if not WAYPOINTS:
        msg = f"Waypoints belum termuat — pilih map dulu dari UI. Cek koneksi ke {API_BASE}."
        print(f"[PATROL] ⚠ {msg}")
        with status_lock:
            last_error = msg
        return

    # Guard: urutan patroli tidak boleh kosong
    if not PATROL_ORDER:
        msg = "PATROL_ORDER kosong — tambahkan meja dari UI terlebih dahulu."
        print(f"[PATROL] ⚠ {msg}")
        with status_lock:
            last_error = msg
        return

    with patrol_lock:
        patrol_active  = True
        patrol_index   = 0
        patrol_retry_count = 0
        patrol_summary = {}
        patrol_done    = False
        current_target = None

    print(f"[PATROL] Mulai patroli {len(PATROL_ORDER)} meja: {PATROL_ORDER}")
    send_move(1, PATROL_ORDER[0], 'PATROL')


def patrol_next():
    """Lanjut ke meja berikutnya setelah scan selesai.
    
    PENTING: Pastikan robot benar-benar IDLE sebelum kirim MOVE berikutnya.
    Kalau robot_api belum siap, MOVE akan ditolak dengan 'robot not IDLE'.
    """
    global patrol_index

    with patrol_lock:
        patrol_index += 1
        patrol_retry_count = 0
        idx = patrol_index

    # Guard: tunggu robot benar-benar idle sebelum kirim MOVE berikutnya.
    # Ini mencegah race condition di mana robot_api belum selesai cleanup
    # Nav2 goal-nya meski GOAL_COMPLETE sudah kita terima.
    # Tunggu sampai status robot berubah jadi IDLE (artinya tombol tablet sudah ditekan)
    # Kita buat timeout-nya sangat lama agar robot tidak kabur ke Home saat ditinggal
    deadline = time.time() + 3600.0   # tunggu maksimal 1 jam
    while time.time() < deadline:
        with status_lock:
            is_idle = robot_status == 'idle'
        if is_idle:
            break
        time.sleep(0.5)
    else:
        print('[PATROL] ⚠ Timeout menunggu tablet ditekan, skip meja berikutnya')
        return

    # Tambahan jeda kecil setelah robot idle — beri waktu robot_api
    # menyelesaikan proses internal sebelum menerima MOVE baru.
    time.sleep(MOVE_READY_DELAY)

    # Cek lagi apakah patroli masih aktif (mungkin di-stop saat kita tunggu)
    with patrol_lock:
        still_active = patrol_active
    if not still_active:
        print('[PATROL] Patroli dibatalkan saat menunggu IDLE, tidak kirim MOVE')
        return

    if idx >= len(PATROL_ORDER):
        # Semua meja selesai → pulang ke home (sequence -1 = konvensi
        # HOMEBASE resmi di robot_api, bukan angka sembarang)
        print('[PATROL] Semua meja selesai → pulang ke home')
        send_move(-1, 'home', 'HOMEBASE')
    else:
        target = PATROL_ORDER[idx]
        print(f"[PATROL] Lanjut ke {target} ({idx+1}/{len(PATROL_ORDER)})")
        send_move(idx + 1, target, 'PATROL')


def write_patrol_log(snapshot: dict):
    """Tulis hasil scan patroli (sejauh ini) ke PATROL_LOG_FILE. Dipanggil
    tiap SATU meja selesai discan (bukan cuma di akhir 12 meja), jadi
    filenya terus bertambah isinya secara live selama patroli berjalan."""
    payload = {
        'updated_at':    time.time(),
        'patrol_active': patrol_active,
        'scan_results':  snapshot,
    }
    tmp_path = PATROL_LOG_FILE + '.tmp'
    try:
        with open(tmp_path, 'w') as f:
            json.dump(payload, f, indent=2)
        os.replace(tmp_path, PATROL_LOG_FILE)   # atomic, tidak korup kalau ke-interupsi
    except OSError as e:
        print(f"[PATROL] Gagal tulis {PATROL_LOG_FILE}: {e}")


def _finish_patrol():
    """Patroli selesai — simpan summary dan update Web UI."""
    global patrol_summary, patrol_done

    dirty  = {k:v for k,v in patrol_summary.items() if v.get('label')=='meja_kotor'}
    bersih = {k:v for k,v in patrol_summary.items() if v.get('label')=='meja_bersih'}
    terisi = {k:v for k,v in patrol_summary.items() if v.get('label')=='meja_terisi'}

    print(f"[PATROL] Selesai — Kotor:{len(dirty)} Bersih:{len(bersih)} Terisi:{len(terisi)}")
    for name in dirty:
        print(f"  KOTOR: {name} (conf={dirty[name]['confidence']:.2f})")

    # Update tables.json
    _update_tables_from_scan(patrol_summary)

    with patrol_lock:
        patrol_done = True
        snapshot = dict(patrol_summary)

    write_patrol_log(snapshot)   # tulis ulang, sekarang patrol_active sudah False
    print('[PATROL] Web UI siap tampilkan hasil.')


def _update_tables_from_scan(scan_results: dict):
    """Simpan/perbarui label meja di tables.json dari hasil patroli.
    Upsert: kalau meja dengan id itu belum ada di tables.json, BUAT baru.
    
    Pemetaan table_N → integer ID dibuat DINAMIS dari WAYPOINTS yang aktif,
    sehingga tidak perlu hardcode jumlah meja (tidak lagi terbatas 12)."""
    # Bangun mapping dinamis dari WAYPOINTS yang aktif saat ini
    WP_TO_ID = {
        k: int(k.split('_')[1])
        for k in WAYPOINTS
        if k.startswith('table_') and k.split('_')[1].isdigit()
    }

    tables = load_tables()
    by_id  = {t['id']: t for t in tables}
    saved  = 0
    for wp_name, result in scan_results.items():
        label = result.get('label', '')
        if label not in ('meja_bersih', 'meja_terisi', 'meja_kotor'):
            continue
        tid = WP_TO_ID.get(wp_name)
        if not tid:
            continue
        if tid in by_id:
            by_id[tid]['label']      = label
            by_id[tid]['confidence'] = result.get('confidence')
            by_id[tid]['last_scan']  = result.get('timestamp', time.time())
        else:
            new_entry = {
                'id':         tid,
                'label':      label,
                'confidence': result.get('confidence'),
                'last_scan':  result.get('timestamp', time.time()),
            }
            tables.append(new_entry)
            by_id[tid] = new_entry
        saved += 1
    if saved:
        save_tables(tables)
        print(f"[UPDATE] {saved} meja disimpan/diupdate di tables.json")


# ══════════════════════════════════════════════════════════════
# HTML WEB UI
# ══════════════════════════════════════════════════════════════
HTML_PAGE = '''<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>NEVO Table Detector</title>
<style>
  * { box-sizing: border-box; font-family: -apple-system, Arial, sans-serif; }
  body { margin:0; background:#0f1115; color:#eee; }
  header { padding:14px 20px; background:#161a22; display:flex; gap:14px;
           align-items:center; flex-wrap:wrap; border-bottom:1px solid #262c38; }
  header h1 { font-size:16px; margin:0; margin-right:auto; }
  .badge { padding:3px 10px; border-radius:12px; font-size:12px; font-weight:600; }
  .badge.blue{background:#1d3a5f;color:#7fb3ff}
  .badge.orange{background:#5f4a1d;color:#ffcf7f}
  .badge.green{background:#1d5f2f;color:#7fff9e}
  .badge.red{background:#5f1d1d;color:#ff7f7f}
  .badge.grey{background:#333;color:#aaa}
  .main { display:flex; gap:16px; padding:16px; flex-wrap:wrap; }
  .camera { flex:2; min-width:320px; }
  .camera img { width:100%; border-radius:10px; background:#000; }
  .panel { flex:1; min-width:300px; display:flex; flex-direction:column; gap:12px; }
  .card { background:#161a22; border:1px solid #262c38; border-radius:10px; padding:14px; }
  .card h3 { margin:0 0 10px 0; font-size:14px; color:#9aa; }
  .btn-row { display:flex; gap:8px; }
  button { flex:1; padding:12px; border:none; border-radius:8px; font-weight:600;
           cursor:pointer; font-size:14px; }
  .btn-green{background:#1f8f3d;color:#fff}
  .btn-red{background:#a92828;color:#fff}
  .btn-blue{background:#2860a9;color:#fff}
  button:disabled{opacity:.35;cursor:not-allowed}
  .progress-bg{background:#262c38;border-radius:6px;height:10px;overflow:hidden;margin-top:10px}
  .progress-bar{background:#2f9e4f;height:100%;width:0%;transition:width .3s}
  .progress-label{font-size:12px;color:#9aa;margin-top:6px}
  .error-banner{background:#5f1d1d;color:#ffb3b3;border-radius:8px;padding:10px 12px;
                font-size:13px;margin-top:10px;display:none}
  .meja-row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;
            border-bottom:1px solid #222836;font-size:13px}
  .meja-row .nm{color:#ddd;font-weight:600}
  .meja-row .lb{padding:2px 8px;border-radius:10px;font-size:12px}
  .dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:6px}
  .log-line{font-size:12px;color:#9aa;padding:2px 0;font-family:monospace}
  .table-btn{background:#1c2333;border:1px solid #3a4358;border-radius:8px;padding:8px 12px;
             cursor:pointer;color:#eee;text-align:center;min-width:78px;transition:.15s}
  .table-btn:hover{background:#26314a;border-color:#5499c7}
  .table-btn .nm{font-weight:700;font-size:13px}
  .table-btn .xy{font-size:9.5px;color:#8a97ac;margin-top:2px}
  .order-chip{background:#1f8f3d33;border:1px solid #1f8f3d;color:#7fff9e;
              border-radius:14px;padding:4px 10px;font-size:12px;font-weight:600}
</style>
</head>
<body>
<header>
  <h1>NEVO Table Detector</h1>
  <div>Robot: <span class="badge blue" id="st_robot">-</span></div>
  <div>WS: <span class="badge red" id="st_ws">OFFLINE</span></div>
  <div>YOLO: <span class="badge green" id="st_yolo">-</span></div>
  <div>Model: <span class="badge blue" id="st_model">-</span></div>
  <div>FPS: <span class="badge orange" id="st_fps">0</span></div>
  <div>Waypoints: <span class="badge grey" id="st_wp">0</span></div>
  <div>Map: <span class="badge blue" id="st_map">-</span></div>
</header>
<div class="main">
  <div style="display:flex; flex-direction:column; gap:15px; flex:1; max-width:640px;">
    <div class="camera" style="background:#0f1115; padding:10px; border-radius:12px; border:1px solid #333;">
      <div style="text-align:center; margin-bottom:8px; font-weight:bold; color:#aaa;">🎥 Kamera Live (Perjalanan Robot)</div>
      <img id="f" src="/video_feed" style="width:100%; border-radius:8px;">
    </div>
    <div class="camera" style="background:#0f1115; padding:10px; border-radius:12px; border:1px solid #333;">
      <div style="text-align:center; margin-bottom:8px; font-weight:bold; color:#5cff8a;">📸 Hasil Deteksi AI (Snapshot Meja)</div>
      <img id="snapshot" src="/snapshot_feed" style="width:100%; border-radius:8px; border:2px solid #1f8f3d; min-height:200px; background:#000;">
    </div>
  </div>
  <div class="panel">
    <div class="card">
      <h3>Kontrol Patroli</h3>
      <div class="btn-row">
        <button class="btn-green" id="b_start" onclick="startPatrol()">Mulai Patroli</button>
        <button class="btn-red" id="b_stop" onclick="stopPatrol()">Stop</button>
        <button class="btn-blue" id="b_home" onclick="goHome()">Home</button>
      </div>
      <div class="progress-bg"><div class="progress-bar" id="pb"></div></div>
      <div class="progress-label" id="pb_label">Belum patroli</div>
      <div class="error-banner" id="err_banner"></div>
    </div>
    <div class="card">
      <h3>Pengaturan</h3>
      <div class="progress-label" style="margin-bottom:6px">Map</div>
      <div class="btn-row" style="margin-bottom:6px">
        <select id="map_select" style="flex:1;padding:10px;border-radius:8px;background:#0f1115;color:#eee;border:1px solid #333">
          <option>Memuat daftar map...</option>
        </select>
        <button class="btn-blue" style="flex:0 0 auto;padding:10px 16px" onclick="selectMap()">Pilih</button>
      </div>
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:10px">
        <span id="wp_info" style="font-size:12px;color:#7fb3ff">— koordinat belum dimuat —</span>
        <button class="btn-blue" style="padding:6px 12px;font-size:12px;flex:0 0 auto" onclick="reloadKoordinat()">🔄 Reload Koordinat</button>
      </div>

      <div class="progress-label" style="margin-bottom:6px">
        Urutan Patroli — klik meja sesuai urutan yang diinginkan
      </div>
      <div id="table_buttons" style="display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px">
        <span style="color:#666;font-size:12px">Memuat daftar meja...</span>
      </div>

      <div class="progress-label" style="margin-bottom:6px">Urutan yang sedang disusun</div>
      <div id="order_chips" style="display:flex;flex-wrap:wrap;gap:6px;min-height:32px;
           background:#0f1115;border:1px solid #333;border-radius:8px;padding:8px;margin-bottom:10px">
        <span style="color:#666;font-size:12px">(kosong — klik tombol meja di atas)</span>
      </div>

      <div class="btn-row">
        <button class="btn-red" style="flex:1" onclick="resetOrder()">Reset</button>
        <button class="btn-red" style="flex:1" onclick="undoOrder()">Hapus Terakhir</button>
        <button class="btn-green" style="flex:1" onclick="saveOrder()">Simpan Urutan</button>
      </div>
      <div class="progress-label" id="order_current" style="margin-top:8px">Urutan tersimpan: -</div>
    </div>
    <div class="card">
      <h3>Hasil Scan (live per meja)</h3>
      <div id="meja_list"><span style="color:#666">Belum ada hasil...</span></div>
    </div>
    <div class="card">
      <h3>Log</h3>
      <div id="log_list"></div>
    </div>
  </div>

</div>
<script>
function addLog(msg){
  const el = document.getElementById('log_list');
  const line = document.createElement('div');
  line.className = 'log-line';
  const t = new Date().toLocaleTimeString();
  line.textContent = `[${t}] ${msg}`;
  el.insertBefore(line, el.firstChild);
  while (el.children.length > 30) el.removeChild(el.lastChild);
}

let lastRobotStatus = null;

async function refreshStatus(){
  try{
    const r = await fetch('/status'); const d = await r.json();
    document.getElementById('st_robot').innerText = d.robot_status.toUpperCase();
    document.getElementById('st_robot').className = 'badge ' +
      (d.robot_status === 'moving' ? 'orange' :
       d.robot_status === 'interrupted' ? 'red' : 'blue');

    document.getElementById('st_ws').innerText = d.ws_connected ? 'CONNECTED' : 'OFFLINE';
    document.getElementById('st_ws').className = 'badge ' + (d.ws_connected ? 'green' : 'red');

    document.getElementById('st_yolo').innerText = d.yolo_active ? 'AKTIF' : 'PAUSE';
    document.getElementById('st_model').innerText = d.model_name;
    document.getElementById('st_fps').innerText = d.fps.toFixed(1);
    document.getElementById('st_wp').innerText = d.waypoints_loaded;
    document.getElementById('st_wp').className = 'badge ' + (d.waypoints_loaded > 0 ? 'grey' : 'red');
    document.getElementById('st_map').innerText = d.map_name || '-';

    // Update info koordinat di panel Pengaturan
    const wpInfo = document.getElementById('wp_info');
    if (wpInfo){
      wpInfo.textContent = d.waypoints_loaded > 0
        ? `✅ ${d.waypoints_loaded} titik koordinat dimuat (map: ${d.map_name || '-'})`
        : '⚠️ Koordinat belum dimuat — pilih map dan klik Pilih';
      wpInfo.style.color = d.waypoints_loaded > 0 ? '#7fff9e' : '#ffb84d';
    }

    const pa = d.patrol_state.active;
    document.getElementById('b_start').disabled = pa;
    document.getElementById('b_stop').disabled  = (d.robot_status !== 'moving');

    const errBox = document.getElementById('err_banner');
    if (d.last_error){
      errBox.style.display = 'block';
      errBox.textContent = '⚠ ' + d.last_error;
    } else {
      errBox.style.display = 'none';
    }

    if (lastRobotStatus !== null && lastRobotStatus !== d.robot_status){
      addLog(`Status robot: ${lastRobotStatus} → ${d.robot_status}`);
    }
    lastRobotStatus = d.robot_status;
  }catch(e){ console.error(e); }
}

async function refreshPatrolResult(){
  try{
    const r = await fetch('/patrol/summary'); const d = await r.json();
    const total = d.total || 12;
    const scanned = Object.keys(d.scan_results || {}).length;
    document.getElementById('pb').style.width = (scanned/total*100)+'%';
    document.getElementById('pb_label').innerText =
      d.active ? `Scanning meja ${scanned}/${total}...` :
      (scanned>0 ? `Selesai: Kotor ${d.dirty_count} / Bersih ${d.clean_count} / Terisi ${d.occupied_count}` :
                   'Belum patroli');

    let html = '';
    const order = Object.keys(d.scan_results || {});
    for (const wp of order){
      const res = d.scan_results[wp];
      const color = res.label==='meja_kotor' ? '#ff5c5c' :
                    res.label==='meja_terisi' ? '#ffb84d' :
                    res.label==='meja_bersih' ? '#5cff8a' : '#888';
      html += `<div class="meja-row"><span class="nm"><span class="dot" style="background:${color}"></span>${wp}</span>` +
              `<span class="lb" style="background:${color}22;color:${color}">${res.label} (${(res.confidence*100).toFixed(0)}%)</span></div>`;
    }
    document.getElementById('meja_list').innerHTML = html || '<span style="color:#666">Belum ada hasil...</span>';
  }catch(e){ console.error(e); }
}

function startPatrol(){
  fetch('/patrol/start').then(r=>r.json()).then(d=>{
    addLog(d.success ? 'Patroli dimulai!' : ('Gagal mulai: ' + (d.message||'')));
  });
}
function stopPatrol(){
  fetch('/patrol/stop').then(r=>r.json()).then(d=>{
    addLog('STOP ditekan — robot dihentikan di tempat');
  });
}
function goHome(){
  fetch('/patrol/home').then(r=>r.json()).then(d=>{
    addLog(d.success ? 'Robot menuju Home' : ('Gagal pulang: ' + (d.message||'')));
  });
}

async function loadMaps(){
  try{
    const r = await fetch('/maps'); const d = await r.json();
    const sel = document.getElementById('map_select');
    sel.innerHTML = '';
    for (const m of d.maps){
      const opt = document.createElement('option');
      opt.value = m.map_id;                 // id tetap dipakai sebagai value (buat request ke backend)
      opt.textContent = m.map_name;          // yang DITAMPILKAN ke user cuma namanya
      if (String(m.map_id) === String(d.current_map_id)) opt.selected = true;
      sel.appendChild(opt);
    }
  }catch(e){ console.error(e); }
}

function selectMap(){
  const sel      = document.getElementById('map_select');
  const mapId    = sel.value;
  const mapName  = sel.options[sel.selectedIndex] ? sel.options[sel.selectedIndex].textContent : mapId;
  const btnPilih = document.querySelector('button[onclick="selectMap()"]');
  if (!mapId) return;

  // Tampilkan loading state
  if (btnPilih){ btnPilih.disabled = true; btnPilih.textContent = 'Mengambil...'; }
  addLog(`⏳ Mengambil koordinat map "${mapName}" dari API...`);

  fetch('/maps/select/' + mapId)
    .then(r => r.json())
    .then(d => {
      if (d.success){
        addLog(`✅ Map "${d.map_name || mapName}" aktif — ${d.waypoints_loaded} titik koordinat berhasil dimuat`);

        // Tampilkan ringkasan koordinat di log
        if (d.waypoints){
          const meja = Object.keys(d.waypoints).filter(k => k.startsWith('table_')).sort();
          if (meja.length > 0){
            addLog(`📍 Meja tersedia: ${meja.map(k=>k.replace('table_','')).join(', ')}`);
          }
          if (d.waypoints.home){
            const h = d.waypoints.home;
            addLog(`🏠 Home: x=${h.x.toFixed(2)}, y=${h.y.toFixed(2)}, yaw=${h.yaw.toFixed(2)}`);
          }
        }

        selectedOrder = [];
        renderChips();
        loadPatrolOrder();
      } else {
        addLog('❌ Gagal ganti map: ' + (d.message || ''));
      }
    })
    .catch(e => addLog('❌ Error koneksi: ' + e))
    .finally(() => {
      if (btnPilih){ btnPilih.disabled = false; btnPilih.textContent = 'Pilih'; }
    });
}

function reloadKoordinat(){
  const btnReload = document.querySelector('button[onclick="reloadKoordinat()"]');
  if (btnReload){ btnReload.disabled = true; btnReload.textContent = 'Reloading...'; }
  addLog('⏳ Reload koordinat map aktif dari API...');

  fetch('/maps/reload')
    .then(r => r.json())
    .then(d => {
      if (d.success){
        addLog(`✅ Koordinat di-reload — ${d.waypoints_loaded} titik dimuat untuk map "${d.map_name}"`);
        loadPatrolOrder();
      } else {
        addLog('❌ Reload gagal: ' + (d.message || ''));
      }
    })
    .catch(e => addLog('❌ Error: ' + e))
    .finally(() => {
      if (btnReload){ btnReload.disabled = false; btnReload.textContent = '🔄 Reload Koordinat'; }
    });
}

let selectedOrder = [];   // urutan yang sedang disusun user lewat klik (array angka)
let tableCoords = {};     // cache koordinat tiap meja, buat ditampilkan di tombol

async function loadPatrolOrder(){
  try{
    const r = await fetch('/patrol_order'); const d = await r.json();
    tableCoords = d.coordinates || {};

    // Render tombol satu per meja yang TERSEDIA di map aktif
    const box = document.getElementById('table_buttons');
    box.innerHTML = '';
    if (!d.available_tables || d.available_tables.length === 0){
      box.innerHTML = '<span style="color:#666;font-size:12px">Tidak ada meja di map ini</span>';
    } else {
      for (const t of d.available_tables){
        const n = t.replace('table_','');
        const c = tableCoords[t] || {};
        const btn = document.createElement('div');
        btn.className = 'table-btn';
        btn.onclick = () => addToOrder(n);
        btn.innerHTML = `<div class="nm">Meja ${n}</div>` +
          (c.x !== undefined ? `<div class="xy">${c.x.toFixed(2)}, ${c.y.toFixed(2)}</div>` : '');
        box.appendChild(btn);
      }
    }

    // Tampilkan urutan yang TERSIMPAN di server (bukan yang lagi disusun)
    const nums = d.patrol_order.map(t => t.replace('table_',''));
    document.getElementById('order_current').innerText = 'Urutan tersimpan: ' + (nums.join(', ') || '-');
  }catch(e){ console.error(e); }
}

function addToOrder(n){
  selectedOrder.push(n);
  renderChips();
}

function undoOrder(){
  selectedOrder.pop();
  renderChips();
}

function resetOrder(){
  selectedOrder = [];
  renderChips();
}

function renderChips(){
  const box = document.getElementById('order_chips');
  if (selectedOrder.length === 0){
    box.innerHTML = '<span style="color:#666;font-size:12px">(kosong — klik tombol meja di atas)</span>';
    return;
  }
  box.innerHTML = selectedOrder.map((n,i) =>
    `<span class="order-chip">${i+1}. Meja ${n}</span>`).join('');
}

function saveOrder(){
  if (selectedOrder.length === 0){
    addLog('Belum ada meja dipilih — klik tombol meja dulu');
    return;
  }
  const val = selectedOrder.join(',');
  fetch('/patrol_order/set/' + encodeURIComponent(val)).then(r=>r.json()).then(d=>{
    if (d.success){
      addLog('Urutan patroli disimpan: ' + d.patrol_order.map(t=>t.replace('table_','')).join(', '));
      selectedOrder = [];
      renderChips();
      loadPatrolOrder();
    } else {
      addLog('Gagal simpan urutan: ' + (d.message||''));
    }
  });
}

setInterval(refreshStatus, 1000);
setInterval(refreshPatrolResult, 1500);
refreshStatus();
refreshPatrolResult();
loadMaps();
loadPatrolOrder();
addLog('System started');
</script>
</body>
</html>'''

# ══════════════════════════════════════════════════════════════
# FLASK ROUTES
# ══════════════════════════════════════════════════════════════

@app.route('/')
def index():
    return HTML_PAGE


@app.route('/frame.jpg')
def frame_jpg():
    with annotated_lock:
        frame = annotated_frame.copy() if annotated_frame is not None else None
    if frame is None:
        return '', 204
    frame = cv2.resize(frame, (640, 480))
    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    from flask import Response as R
    return R(jpeg.tobytes(), mimetype='image/jpeg',
             headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})


@app.route('/video_feed')
def video_feed():
    def generate():
        last_encoded_time = 0.0
        while True:
            with annotated_lock:
                # [EXTREME OPTIMIZATION] Hilangkan .copy() karena web server HANYA BACA (Read-Only)
                frame = annotated_frame
                current_time = annotated_frame_time
            if frame is None or current_time == last_encoded_time:
                time.sleep(0.01)
                continue
            
            last_encoded_time = current_time
            _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            
    from flask import Response
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot_feed')
def snapshot_feed():
    def generate():
        last_snap = None
        while True:
            with snapshot_jpeg_lock:
                current_snap = snapshot_jpeg
            
            if current_snap is None:
                time.sleep(0.5)
                continue
                
            if current_snap != last_snap:
                last_snap = current_snap
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + current_snap + b'\r\n')
            else:
                time.sleep(0.5)
                
    from flask import Response
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/status')
def status_api():
    with status_lock:
        rs = robot_status
    with fps_lock:
        fps = fps_value
    with patrol_lock:
        pa     = patrol_active
        pi     = patrol_index
        ct     = current_target
        done   = patrol_done
    with status_lock:
        err = last_error
    return jsonify({
        'robot_status':     rs,
        'yolo_active':      rs == 'idle',
        'fps':              fps,
        'table_count':      len(list_tables()),
        'ws_connected':     ws_connected,
        'model_name':       MODEL_NAME,
        'map_name':         MAP_NAME,
        'waypoints_loaded': len(WAYPOINTS),
        'last_error':       err,
        'patrol_state': {
            'active':         pa,
            'index':          pi,
            'total':          len(PATROL_ORDER),
            'current_target': ct,
            'done':           done,
        }
    })


@app.route('/tables')
def tables_api():
    return jsonify(list_tables())


@app.route('/maps')
def maps_api():
    """Daftar semua map yang tersedia + map mana yang lagi aktif dipakai (by name)."""
    maps = fetch_map_list()
    return jsonify({'maps': maps, 'current_map_id': MAP_ID, 'current_map_name': MAP_NAME})


@app.route('/maps/select/<map_id>')
def maps_select_api(map_id):
    """Ganti map aktif, fetch koordinat dari API untuk map itu (retry 3x),
    lalu simpan pilihannya agar tetap dipakai walau main.py di-restart."""
    global MAP_ID, MAP_NAME, WAYPOINTS, last_error
    if patrol_active:
        return jsonify({'success': False, 'message': 'Tidak bisa ganti map selagi patroli berjalan'})

    old_map_id   = MAP_ID
    old_waypoints = dict(WAYPOINTS)

    MAP_ID   = map_id
    WAYPOINTS = {}

    # Retry hingga 3x — antisipasi API lambat atau koneksi sesaat putus
    MAX_RETRY = 3
    for attempt in range(1, MAX_RETRY + 1):
        fetch_waypoints()
        if WAYPOINTS:
            break
        print(f"[MAP] Fetch gagal (percobaan {attempt}/{MAX_RETRY}), coba lagi...")
        if attempt < MAX_RETRY:
            time.sleep(1.5)

    if not WAYPOINTS:
        # Gagal total setelah semua retry — rollback ke map sebelumnya
        MAP_ID    = old_map_id
        WAYPOINTS = old_waypoints
        old_name  = resolve_map_name(old_map_id)
        msg = (f"Gagal memuat koordinat map {map_id} setelah {MAX_RETRY}x percobaan. "
               f"Tetap pakai map '{old_name}'. Cek koneksi ke {API_BASE}.")
        print(f"[MAP] {msg}")
        with status_lock:
            last_error = msg
        return jsonify({'success': False, 'message': msg})

    MAP_NAME = resolve_map_name(MAP_ID)
    save_config()   # simpan pilihan map agar bertahan saat restart
    print(f"[MAP] Berpindah ke map '{MAP_NAME}' (id={MAP_ID}, {len(WAYPOINTS)} titik koordinat)")
    return jsonify({
        'success':          True,
        'map_id':           MAP_ID,
        'map_name':         MAP_NAME,
        'waypoints_loaded': len(WAYPOINTS),
        'waypoints':        WAYPOINTS,   # kirim koordinat lengkap ke UI
        'patrol_order':     PATROL_ORDER,
    })


@app.route('/maps/reload')
def maps_reload_api():
    """Fetch ulang koordinat untuk map yang sedang aktif dari API (tanpa ganti map).
    Berguna jika koordinat di server diupdate dan perlu di-refresh."""
    global WAYPOINTS, last_error
    if patrol_active:
        return jsonify({'success': False, 'message': 'Tidak bisa reload map selagi patroli berjalan'})

    old_wp = dict(WAYPOINTS)
    WAYPOINTS = {}
    fetch_waypoints()

    if not WAYPOINTS:
        WAYPOINTS = old_wp
        msg = f"Reload gagal, tetap pakai koordinat lama ({len(old_wp)} titik)."
        with status_lock:
            last_error = msg
        return jsonify({'success': False, 'message': msg})

    print(f"[MAP] Reload map '{MAP_NAME}' (id={MAP_ID}) → {len(WAYPOINTS)} titik koordinat")
    return jsonify({
        'success':          True,
        'map_id':           MAP_ID,
        'map_name':         MAP_NAME,
        'waypoints_loaded': len(WAYPOINTS),
        'waypoints':        WAYPOINTS,
    })


@app.route('/patrol_order', methods=['GET'])
def patrol_order_get_api():
    """Urutan patroli saat ini + daftar semua meja yang tersedia di map aktif,
    lengkap dengan koordinatnya masing-masing (buat ditampilkan di tombol)."""
    available = sorted(
        (k for k in WAYPOINTS if k.startswith('table_')),
        key=lambda k: int(k.split('_')[1])
    )
    coords = {t: WAYPOINTS[t] for t in available}
    return jsonify({
        'patrol_order': PATROL_ORDER,
        'available_tables': available,
        'coordinates': coords,
    })


@app.route('/patrol_order/set/<path:order_str>')
def patrol_order_set_api(order_str):
    """Atur urutan patroli custom, contoh: /patrol_order/set/1,3,2,5
    Cuma boleh berisi nomor meja yang benar-benar ada koordinatnya di map
    yang sedang aktif — supaya tidak nyoba MOVE ke meja yang tidak ada."""
    global PATROL_ORDER, last_error
    if patrol_active:
        return jsonify({'success': False, 'message': 'Tidak bisa ubah urutan selagi patroli berjalan'})

    try:
        numbers = [int(n.strip()) for n in order_str.split(',') if n.strip()]
    except ValueError:
        return jsonify({'success': False, 'message': 'Format salah, contoh yang benar: 1,3,2,5'})

    if not numbers:
        return jsonify({'success': False, 'message': 'Urutan kosong'})

    new_order = [f'table_{n}' for n in numbers]
    invalid = [t for t in new_order if t not in WAYPOINTS]
    if invalid:
        msg = f"Meja {invalid} tidak ada koordinatnya di map ini (map_id={MAP_ID})"
        with status_lock:
            last_error = msg
        return jsonify({'success': False, 'message': msg})

    PATROL_ORDER = new_order
    save_config()
    print(f"[PATROL_ORDER] Diubah manual jadi: {PATROL_ORDER}")
    return jsonify({'success': True, 'patrol_order': PATROL_ORDER})


@app.route('/patrol/start')
def patrol_start():
    global patrol_active
    with patrol_lock:
        if patrol_active:
            return jsonify({'success': False, 'message': 'Patroli sudah berjalan'})
    threading.Thread(target=start_patrol, daemon=True).start()
    return jsonify({'success': True})


@app.route('/patrol/stop')
def patrol_stop():
    """STOP beneran — kirim INTERRUPT supaya robot_api cancel semua goal
    Nav2 saat itu juga (robot berhenti persis di posisinya, TIDAK dialihkan
    ke mana pun). Robot baru gerak lagi kalau ditekan tombol Home."""
    global patrol_active
    with patrol_lock:
        patrol_active = False   # jangan kirim MOVE meja berikutnya lagi
    ws_send_sync({'code': 'INTERRUPT', 'data': {'robot_id': ROBOT_ID, 'ui_id': UI_ID}})
    print('[PATROL] STOP ditekan — kirim INTERRUPT (cancel semua goal Nav2)')
    return jsonify({'success': True})


@app.route('/patrol/home')
def patrol_home():
    """Suruh robot pulang langsung. type=HOMEBASE diterima robot_api dari
    kondisi apa pun (IDLE/MOVING/INTERRUPTED) — jadi aman dipanggil kapan
    saja, tidak perlu Stop dulu. Status 'moving' baru berubah setelah
    robot_api membalas ACK (lihat on_ws_message), bukan optimis di sini."""
    global last_error, current_target
    home = WAYPOINTS.get('home')
    if not home:
        msg = f"Koordinat 'home' belum termuat (WAYPOINTS kosong). Cek koneksi ke {API_BASE}."
        with status_lock:
            last_error = msg
        return jsonify({'success': False, 'message': msg})

    current_target = 'home'
    ws_send_sync({
        'code': 'MOVE',
        'data': {
            'robot_id':  ROBOT_ID,
            'ui_id':     UI_ID,
            'sequence':  -1,
            'type':      'HOMEBASE',
            'target':    'home',
            'dest':      home,
            'home_base': home,
            'timestamp': time.time(),
        }
    })
    print('[HOME] Kirim MOVE HOMEBASE, menunggu ACK dari robot_api...')
    return jsonify({'success': True})


@app.route('/patrol/summary')
def patrol_summary_api():
    """Hasil scan — selalu tampilkan progres LIVE (bertambah tiap satu meja
    selesai discan), bukan cuma setelah 12 meja selesai semua. Field 'done'
    menandakan apakah siklus 12 meja sudah tuntas."""
    with patrol_lock:
        ps   = dict(patrol_summary)
        done = patrol_done
        active = patrol_active
        idx  = patrol_index
    dirty  = {k:v for k,v in ps.items() if v.get('label')=='meja_kotor'}
    bersih = {k:v for k,v in ps.items() if v.get('label')=='meja_bersih'}
    terisi = {k:v for k,v in ps.items() if v.get('label')=='meja_terisi'}
    return jsonify({
        'ready':          True,
        'done':           done,
        'active':         active,
        'progress':       idx,
        'total':          len(PATROL_ORDER),
        'scan_results':   ps,
        'dirty_count':    len(dirty),
        'clean_count':    len(bersih),
        'occupied_count': len(terisi),
        'dirty_tables':   list(dirty.keys()),
    })


# ══════════════════════════════════════════════════════════════
# BACKGROUND THREADS
# ══════════════════════════════════════════════════════════════

def yolo_loop():
    """
    YOLO scan loop.
    - Saat robot MOVING: pause
    - Saat robot IDLE: scan tiap YOLO_INTERVAL detik
    - Saat force_scan=True: scan segera 1x (dipanggil saat GOAL_COMPLETE)
    """
    global last_detections
    global force_scan, patrol_summary, patrol_active, patrol_index
    last_detect = 0

    searching_mode = False
    search_start_time = 0.0

    while True:
        with status_lock:
            is_idle = robot_status == 'idle'

        if force_scan:
            force_scan = False
            searching_mode = True
            search_start_time = time.time()

        if not is_idle and not searching_mode:
            last_detections = []
            time.sleep(0.1)
            continue

        now = time.time()
        # Jika sedang mencari meja, kita proses secepat mungkin (jangan kena limit YOLO_INTERVAL)
        if now - last_detect < YOLO_INTERVAL and not searching_mode:
            time.sleep(0.05)
            continue
        last_detect = now

        with frame_lock:
            frame = latest_frame
        if frame is None:
            time.sleep(0.05)
            continue

        try:
            dets = detector.detect(frame)

            best_det = None
            for det in dets:
                if best_det is None or det['confidence'] > best_det['confidence']:
                    best_det = det

            last_detections = dets

            if searching_mode and patrol_active and current_target:
                CONFIDENCE_THRESHOLD = 0.20
                
                # Cek apakah sudah ketemu
                if best_det and best_det['confidence'] >= CONFIDENCE_THRESHOLD:
                    label      = best_det['label']
                    confidence = best_det['confidence']
                    print(f"[YOLO] MEJA TERDETEKSI di {current_target} ({label} {confidence:.2f}) - Melanjutkan patroli!")
                else:
                    # Belum ketemu meja, cek apakah timeout
                    if time.time() - search_start_time > 30.0:
                        label = 'unknown'
                        confidence = 0.0
                        print(f"[YOLO] ⚠ TIMEOUT 30 DETIK: Meja tidak ditemukan di {current_target}, lanjut paksa.")
                    else:
                        # Masih belum 30 detik, berikan visual di UI bahwa kita masih mencari
                        log_img = frame.copy()
                        cv2.putText(log_img, f"MENCARI MEJA... ({30 - int(time.time() - search_start_time)}s)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        global snapshot_jpeg
                        _, jpeg_buf = cv2.imencode('.jpg', log_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        with snapshot_jpeg_lock:
                            snapshot_jpeg = jpeg_buf.tobytes()
                        time.sleep(0.05)
                        continue # Ulangi loop tanpa menyelesaikan pencarian

                # KITA HANYA SAMPAI DI SINI JIKA (A) MEJA KETEMU, atau (B) TIMEOUT 30 DETIK
                searching_mode = False # Selesai mencari

                with patrol_lock:
                    patrol_summary[current_target] = {
                        'label':      label,
                        'confidence': round(confidence, 4),
                        'timestamp':  time.time(),
                    }
                    snapshot = dict(patrol_summary)

                write_patrol_log(snapshot)
                print(f"[YOLO] {current_target}: {label} ({confidence:.2f}) → tersimpan ke {PATROL_LOG_FILE}")

                # Simpan gambar sebagai log
                try:
                    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_logs')
                    os.makedirs(log_dir, exist_ok=True)
                    log_img = frame.copy()
                    
                    if label != 'unknown':
                        x1, y1, x2, y2 = best_det['x1'], best_det['y1'], best_det['x2'], best_det['y2']
                        cv2.rectangle(log_img, (x1, y1), (x2, y2), (50, 220, 50), 3)
                        text = f"{label.replace('_', ' ').title()} {confidence:.2f}"
                        cv2.putText(log_img, text, (x1, max(0, y1-12)), cv2.FONT_HERSHEY_DUPLEX, 0.6, (50, 220, 50), 1)
                        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                        cv2.circle(log_img, (cx, cy), 6, (255, 255, 255), -1)
                        cv2.circle(log_img, (cx, cy), 4, (50, 220, 50), -1)
                    else:
                        cv2.putText(log_img, "YOLO: GAGAL MENEMUKAN MEJA (TIMEOUT)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    _, jpeg_buf = cv2.imencode('.jpg', log_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    with snapshot_jpeg_lock:
                        snapshot_jpeg = jpeg_buf.tobytes()
                        
                    filename = f"{current_target}_{label}_{time.strftime('%H%M%S')}.jpg"
                    cv2.imwrite(os.path.join(log_dir, filename), log_img)
                except Exception as ex:
                    print(f"[YOLO] Gagal menyimpan foto log: {ex}")

                # [BYPASS TABLET] Otomatis konfirmasi tablet & panggil meja berikutnya!
                print(f"[HACK-BYPASS] Mengirim PICKUP_CONFIRM otomatis agar robot jalan ke meja berikutnya...")
                ws_send_sync({'code': 'PICKUP_CONFIRM', 'data': {'robot_id': ROBOT_ID, 'sequence': current_seq}})
                threading.Thread(target=patrol_next, daemon=True).start()

        except Exception as e:
            print(f"YOLO error: {e}")
            searching_mode = False


def render_loop():
    """Render frame dengan overlay YOLO ke annotated_frame."""
    global annotated_frame, annotated_frame_time, fps_value
    prev_time   = time.time()
    frame_count = 0
    last_rendered_time = 0.0

    while True:
        with frame_lock:
            # Hanya copy 1 KALI di sini (karena render_loop bertugas mencoret-coret/menggambar kotak)
            canvas = latest_frame.copy() if latest_frame is not None else None
            frame_time = latest_frame_time
            
        if canvas is None or frame_time == last_rendered_time:
            time.sleep(0.01)
            continue
            
        last_rendered_time = frame_time

        frame_count += 1
        now     = time.time()
        elapsed = now - prev_time
        if elapsed >= 1.0:
            with fps_lock:
                fps_value = frame_count / elapsed
            frame_count = 0
            prev_time   = now

        with status_lock:
            rs = robot_status
        is_idle = rs == 'idle'

        # [ANTI SILENT FAILURE] Peringatan visual jika kamera tidak menerima update
        if time.time() - frame_time > 3.0:
            cv2.putText(canvas, "WARNING: CAMERA FROZEN / DISCONNECTED",
                        (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # [OPSI B] TIDAK ADA penggambaran kotak YOLO di kamera live (semua kotak hanya ada di Snapshot AI)

        # [SUPER OPTIMIZATION] Perkecil ukuran gambar HANYA jika gambar dari kamera lebih besar dari 640.
        if canvas.shape[1] > 640:
            canvas = cv2.resize(canvas, (640, 480))

        with annotated_lock:
            annotated_frame = canvas
            annotated_frame_time = frame_time
        time.sleep(0.01)


# ══════════════════════════════════════════════════════════════
# ROS2 NODE (hanya untuk /image_raw dari Kinect)
# ══════════════════════════════════════════════════════════════

class KinectSubscriber(Node):
    def __init__(self):
        super().__init__('table_detector')
        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        
        ip = os.popen("hostname -I | awk '{print $1}'").read().strip()
        self.get_logger().info(f'[NEVO] Model: {MODEL_NAME}')
        self.get_logger().info(f'[NEVO] Web UI: http://{ip}:5000')
        self.get_logger().info(f'[NEVO] WebSocket → {WSL2_WS_URI}')



    def image_callback(self, msg):
        global latest_frame, latest_frame_time
        try:
            raw_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with frame_lock:
                latest_frame = raw_frame.copy()
                latest_frame_time = time.time()
        except Exception as e:
            print(f"ERROR image_callback: {e}")


# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════

def main():
    global detector, node_ref, MAP_NAME
    detector = Detector(MODEL_PATH)
    print(f"[NEVO] Model loaded: {MODEL_NAME}")

    # ── Fetch waypoints dengan retry ──────────────────────────────────────
    # Kalau API tidak terjangkau saat pertama kali nyala (misalnya jaringan
    # belum stabil), coba lagi sampai 5x sebelum fallback ke cache lokal.
    print(f"[NEVO] Menginisialisasi waypoints dari server (map_id={MAP_ID})...")
    MAX_FETCH_RETRY = 5
    FETCH_RETRY_DELAY = 3.0   # detik antar percobaan
    for attempt in range(1, MAX_FETCH_RETRY + 1):
        fetch_waypoints()
        if WAYPOINTS:
            print(f"[NEVO] Waypoints berhasil dimuat ({len(WAYPOINTS)} titik) pada percobaan ke-{attempt}")
            break
        if attempt < MAX_FETCH_RETRY:
            print(f"[NEVO] ⚠ Waypoints kosong, coba lagi dalam {FETCH_RETRY_DELAY}s "
                  f"(percobaan {attempt}/{MAX_FETCH_RETRY})...")
            time.sleep(FETCH_RETRY_DELAY)
        else:
            print(f"[NEVO] ⚠⚠ Gagal fetch waypoints setelah {MAX_FETCH_RETRY}x percobaan. "
                  "Patroli tidak akan bisa jalan sampai waypoints termuat. "
                  f"Cek koneksi ke {API_BASE}")

    MAP_NAME = resolve_map_name(MAP_ID)
    print(f"[NEVO] Map aktif: '{MAP_NAME}' (id={MAP_ID}, {len(WAYPOINTS)} waypoints)")

    rclpy.init()
    node_ref = KinectSubscriber()

    # Start threads
    threading.Thread(target=yolo_loop,   daemon=True).start()
    threading.Thread(target=render_loop, daemon=True).start()
    threading.Thread(target=start_ws_thread, daemon=True).start()
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, threaded=True),
        daemon=True).start()

    print(f"[NEVO] Semua thread berjalan — Web UI: http://0.0.0.0:{WEB_PORT}")
    try:
        rclpy.spin(node_ref)
    except KeyboardInterrupt:
        print('\n[INFO] Node dihentikan.')
    finally:
        node_ref.destroy_node()


if __name__ == '__main__':
    main()