# NEVO Session Report — 2026-08-25

## Summary

Session covering: WebSocket connection stacking fix, Arduino SoH computation, battery telemetry expansion to match TIFA format, SOH calibration fix, and cloud upload verification.

---

## Changes Made

### 1. WebSocket Connection Stacking Fix

**Problem:** Cloud UI showed 5+ duplicate connection instances stacking up after mode swaps. Each reconnect opened a new TCP connection without tearing down the old one.

**Root causes identified (6 issues):**

| # | Issue | File |
|---|-------|------|
| 1 | `reconnecting_` flag reset before TCP connect completes | `ws_client.cpp:256` |
| 2 | `forceReconnect()` doesn't acquire reconnecting guard | `ws_client.cpp:209` |
| 3 | `scheduleReconnect()` doesn't call `ws_.stop()` before reconnect | `ws_client.cpp:254` |
| 4 | `scheduleReconnect()` has duplicate `setUrl()` calls | `ws_client.cpp:242-248` |
| 5 | Dual SI message paths (WsClient + robot_api_node) | `robot_api_node.cpp:269-277` |
| 6 | Dual READY message sources (timer + networkStatusCallback) | `robot_api_node.cpp:88-96, 1487-1493` |

**Fixes applied:**

| File | Fix |
|------|-----|
| `ws_client.cpp:77` | `reconnecting_.store(false)` moved to Open callback (only after TCP success) |
| `ws_client.cpp:255` | Added `ws_.stop()` before `ws_.setUrl()` + `ws_.start()` |
| `ws_client.cpp:209-218` | `forceReconnect()` acquires `reconnecting_` guard |
| `ws_client.cpp:239-251` | Removed duplicate `setUrl()` from if/else branches |
| `robot_api_node.cpp:269-277` | Removed redundant SI-from-ROS-path on DUPLICATE_ROBOT_ID |
| `robot_api_node.cpp:1491` | Cancel ready timer when `networkStatusCallback` sends READY |

### 2. Arduino SoH Computation

**Problem:** Battery telemetry only sent SoC. No State of Health or internal resistance data.

**Hardware:** INA226 #1 (0x40) battery monitor, INA226 #2 (0x44) dock monitor, 6S LiPo (22.8-25.0V, 6000mAh).

**Fix in `INA226_Monitor.ino`:**

Added two SoH estimation methods blended via EMA:

1. **Capacity tracking:** mAh discharged per full cycle vs rated 6000mAh
   - `saved_cycle_capacity = saved_cycle_capacity * 0.7 + cycle_mAh_discharged * 0.3`
   - `SoH = saved_cycle_capacity / 6000.0 * 100`

2. **Internal resistance:** voltage sag under load vs 80mΩ baseline
   - `R_int = (V_oc - V_load) / I_load`
   - `SoH = baseline_R / measured_R * 100`

**Serial format extended from 10 to 13 fields:**
```
<BatV,BatmA,SoC,DockmA,State,BatShunt,BatPwr,DockV,DockShunt,DockPwr,SoH,Rint_mOhm,Cycles>
```

**EEPROM persistence:** SoH, cycle capacity, cycle count, v_oc_ref survive reboots (magic byte 0xB7).

### 3. SOH Calibration Bug Fix

**Problem:** SOH showed 24% on a healthy battery. The `v_oc_ref` (open-circuit voltage reference) was initialized to `VOLTAGE_MAX` (25.0V) instead of the actual battery voltage, causing the resistance calculation to think there was massive voltage sag.

**Example:** At 72% SoC, actual voltage = 24.4V, but `v_oc_ref` = 25.0V → R_int = (25.0-24.4)/2.0 = 0.3Ω → SoH = 80/300 = 26% (garbage).

**Fixes:**
- `v_oc_ref` initialized from actual battery voltage at boot (not `VOLTAGE_MAX`)
- `v_oc_ref` updated during low-current navigation (<500mA) to track resting voltage
- `v_oc_ref` only updated during charging at SoC ≥ 95% AND current < 500mA (resting)

**Result:** SOH now shows 100.0% on fresh battery, drifts down naturally over charge cycles.

### 4. NevoTelemetry.msg Update

Added 3 new fields to match SoH data from Arduino:
```
float32 soh
float32 internal_resistance_ohm
int32 cycle_count
```

### 5. ina226_bridge_node.py Parser Update

Updated to parse 13 fields (was 10):
- Added: `msg.soh`, `msg.internal_resistance_ohm`, `msg.cycle_count`
- R_int converted from mΩ (serial) to Ω (ROS message)

### 6. BATTERY JSON Aligned with TIFA Format

**Before (NEVO):**
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

**After (matches TIFA exactly):**
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

**Key changes:** Added `dock_current`, `state_code`, `bat_shunt_voltage`, `bat_power`, `dock_voltage`, `dock_shunt_voltage`, `dock_power`. Renamed `internal_resistance_ohm` → `internal_resistance_mohm` (unit: mΩ) to match TIFA.

---

## Cloud Upload Verification

**Data flow confirmed working:**
```
Arduino CSV → ina226_bridge.py → nevo/sensor/battery
  → cloud_upload.cpp → nevo/ws_out (every 60s)
    → ws_bridge_node.cpp → ws_client.cpp → WSS to tifa-ws.forgixrobotic.com
```

**Verified in logs:**
- `ina226_bridge`: "BATTERY published: battery_percent=71.3 soh=100.0" ✓
- `cloud_upload`: "Uploaded Battery to Cloud: {"code":"BATTERY","data":{...}}" ✓

**Remaining issue:** Cloud backend/frontend at `tifa-ws.forgixrobotic.com` and `tifa-web.forgixrobotic.com` only displays `battery_percent` (SoC). The new fields (`bus_voltage`, `soh`, `internal_resistance_mohm`, `cycle_count`, etc.) are being sent but not rendered in the UI. Requires cloud backend update to store and expose these fields via `/api/battery` and `/api/device-status`.

---

## Files Modified

| File | Change |
|------|--------|
| `nevo_ws_bridge/src/ws_client.cpp` | Fixed 4 reconnect/stacking bugs |
| `nevo_robot_api/src/robot_api_node.cpp` | Removed duplicate SI path, cancel ready timer |
| `nevo_bot/scripts/ina226_bridge_node.py` | Parse 13 fields, expanded BATTERY JSON |
| `nevo_msgs/msg/NevoTelemetry.msg` | Added soh, internal_resistance_ohm, cycle_count |
| `nevo_ws_bridge/src/cloud_upload.cpp` | Added temporary debug logging (removed) |
| `INA226_Monitor/INA226_Monitor.ino` | Added SoH computation, EEPROM persistence |

## Known Issues

- **Cloud UI doesn't show new fields:** Backend needs to parse/store/expose bus_voltage, soh, etc.
- **Loose CP2102 adapter:** Hardware issue causing intermittent rplidar disconnects
- **EKF "Failed to meet update rate":** Non-fatal warning during startup
