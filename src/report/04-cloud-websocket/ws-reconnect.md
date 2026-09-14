# Reconnect WebSocket

Connection stacking klien WebSocket, guard reconnect, dan perbaikan jalur pesan ganda.

**Laporan:** 2026-08-25

---

## Masalah: Connection Stacking

UI cloud menampilkan 5+ instance koneksi duplikat yang menumpuk setelah mode swap. Setiap reconnect membuka koneksi TCP baru tanpa menutup yang lama.

---

## Akar Masalah yang Diidentifikasi (6 Masalah)

| # | Masalah | File |
|---|---------|------|
| 1 | Flag `reconnecting_` direset sebelum koneksi TCP selesai | `ws_client.cpp:256` |
| 2 | `forceReconnect()` tidak mengambil reconnecting guard | `ws_client.cpp:209` |
| 3 | `scheduleReconnect()` tidak memanggil `ws_.stop()` sebelum reconnect | `ws_client.cpp:254` |
| 4 | `scheduleReconnect()` memiliki panggilan `setUrl()` duplikat | `ws_client.cpp:242-248` |
| 5 | Jalur pesan SI ganda (WsClient + robot_api_node) | `robot_api_node.cpp:269-277` |
| 6 | Sumber pesan READY ganda (timer + networkStatusCallback) | `robot_api_node.cpp:88-96, 1487-1493` |

---

## Perbaikan yang Diterapkan

| File | Perbaikan |
|------|-----------|
| `ws_client.cpp:77` | `reconnecting_.store(false)` dipindah ke Open callback (hanya setelah TCP sukses) |
| `ws_client.cpp:255` | Ditambahkan `ws_.stop()` sebelum `ws_.setUrl()` + `ws_.start()` |
| `ws_client.cpp:209-218` | `forceReconnect()` mengambil reconnecting guard |
| `ws_client.cpp:239-251` | Menghapus `setUrl()` duplikat dari cabang if/else |
| `robot_api_node.cpp:269-277` | Menghapus jalur SI-from-ROS-path redundan pada DUPLICATE_ROBOT_ID |
| `robot_api_node.cpp:1491` | Membatalkan ready timer saat `networkStatusCallback` mengirim READY |

---

## File yang Terpengaruh

| File | Perubahan |
|------|-----------|
| `nevo_ws_bridge/src/ws_client.cpp` | Memperbaiki 4 bug reconnect/stacking |
| `nevo_robot_api/src/robot_api_node.cpp` | Menghapus jalur SI duplikat, membatalkan ready timer |
