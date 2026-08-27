# NEVO Session Report — 2026-08-26

## Summary

DDS networking discovery fix between dev machine (Arch Linux distrobox) and robot (Raspberry Pi 5). FastDDS `initial_peers` configuration for WiFi hotspot (`ap0`) unicast discovery.

---

## Problem

Dev machine (Arch Linux host, Ubuntu 24 distrobox) couldn't discover ROS 2 topics from the robot (Raspberry Pi 5 at `192.168.12.195`) over WiFi hotspot (`ap0` at `192.168.12.1`). `ros2 topic list` returned only `/parameter_events` and `/rosout`.

**Key findings:**
- WiFi hotspot (`ap0`) blocks or doesn't support multicast — FastDDS default discovery mechanism
- Distrobox uses host network namespace, so network reachability is fine (ping works)
- `ros2 topic list --spin-time 10` works — DDS just needs more time for unicast discovery

## Solution

Added `initial_peers` to the dev machine's `~/fastdds.xml` to force unicast discovery to the robot:

```xml
<initial_peers>
  <locator>
    <udpv4>
      <address>192.168.12.195</address>
    </udpv4>
  </locator>
</initial_peers>
```

**Important:** Robot-side `fastdds.xml` does NOT need `initial_peers` — adding it broke inbound discovery. The robot broadcasts normally via multicast on `ap0`; only the dev machine needs explicit peer configuration.

## Environment Setup (Dev Machine)

Required env vars for ROS 2 topic access from distrobox:

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="file:///home/$(whoami)/fastdds.xml"
```

Usage: `ros2 topic list --spin-time 10` (default spin time is too short for WiFi unicast)

## Files Modified

| File | Change |
|------|--------|
| `~/fastdds.xml` (dev machine) | Added `initial_peers` pointing to robot `192.168.12.195` |
| `/home/rspx1/fastdds.xml` (robot) | Reverted to original (no `initial_peers`) |
