#!/bin/bash
# NEVO — udev rule generator for Raspberry Pi 5 (Ubuntu 24.04 / ROS 2 Jazzy).
#
# The old Pi 4 rules matched devices by physical USB port position
# (KERNELS=="1-1.x" / ID_PATH platform-fd500000.pcie). On the Pi 5 the
# USB controller enumerates differently (e.g. platform-1000120000.pci or
# xhci-hcd paths), so port positions MUST be re-discovered on the new board.
#
# This helper prints, for every currently plugged serial device, the
# attributes needed to write stable udev rules, plus a ready-made rule
# using the most stable available strategy:
#   1. serial number (best — CP2102 lidar usually has one)
#   2. physical port path (works when each device sits in a fixed slot)
#
# Usage:
#   bash gen_udev_rules.sh            # inspect + print suggested rules
#   bash gen_udev_rules.sh | sudo tee /etc/udev/rules.d/99-nevo.rules

set -euo pipefail

echo "# ============================================================="
echo "# Suggested NEVO udev rules — generated $(date '+%F %T')"
echo "# Verify symlinks afterwards: ls -la /dev/ttyLIDAR /dev/ttyMOTOR /dev/ttyBATERAI"
echo "# ============================================================="

for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  base=$(basename "$dev")
  sys=$(udevadm info -q path -n "$dev")

  vendor=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_VENDOR_ID=//p')
  model=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_MODEL_ID=//p')
  serial=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_SERIAL_SHORT=//p')
  idpath=$(udevadm info -q property -n "$dev" | sed -n 's/^ID_PATH=//p')

  echo ""
  echo "# --- $base : vendor=$vendor model=$model serial=${serial:-none} path=$idpath"

  if [ -n "$serial" ]; then
    echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vendor\", ATTRS{idProduct}==\"$model\", ATTRS{serial}==\"$serial\", SYMLINK+=\"ttyXXX\", MODE=\"0666\""
  else
    echo "SUBSYSTEM==\"tty\", ENV{ID_VENDOR_ID}==\"$vendor\", ENV{ID_MODEL_ID}==\"$model\", ENV{ID_PATH}==\"$idpath\", SYMLINK+=\"ttyXXX\", MODE=\"0666\""
  fi
done

cat <<'EOF'

# After editing SYMLINK targets (ttyXXX -> ttyLIDAR / ttyMOTOR / ttyBATERAI):
#   sudo udevadm control --reload-rules && sudo udevadm trigger
# Then re-plug devices or reboot and verify with: ls -la /dev/tty*
EOF
