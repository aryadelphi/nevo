# 04 — Cloud & WebSocket

Keandalan klien WebSocket, integrasi API cloud, format telemetri baterai, dan komunikasi UI cloud.

**Laporan:** 2026-08-25

---

## Sub-topik

| File | Fokus |
|------|-------|
| [ws-reconnect.md](ws-reconnect.md) | Connection stacking, guard reconnect, jalur pesan ganda |
| [battery-telemetry.md](battery-telemetry.md) | Format BATTERY JSON, field SoH, parser INA226, upload cloud |

---

## Alur Data

```
Arduino CSV → ina226_bridge.py → nevo/sensor/battery
  → cloud_upload.cpp → nevo/ws_out (setiap 60s)
    → ws_bridge_node.cpp → ws_client.cpp → WSS ke tifa-ws.forgixrobotic.com
```

---

## Masalah yang Diperbaiki

| # | Masalah | Laporan |
|---|---------|---------|
| 1 | WebSocket 5+ koneksi duplikat menumpuk setelah mode swap | 2026-08-25 |
| 2 | Flag reconnecting_ direset sebelum koneksi TCP selesai | 2026-08-25 |
| 3 | forceReconnect tidak mengambil reconnecting guard | 2026-08-25 |
| 4 | scheduleReconnect tidak memanggil ws_.stop() sebelum reconnect | 2026-08-25 |
| 5 | Jalur pesan SI ganda (WsClient + robot_api_node) | 2026-08-25 |
| 6 | Sumber pesan READY ganda (timer + networkStatusCallback) | 2026-08-25 |

## Masalah yang Diketahui

- UI cloud tidak menampilkan field baterai baru (SoH, resistansi internal, jumlah siklus) — backend tertunda
