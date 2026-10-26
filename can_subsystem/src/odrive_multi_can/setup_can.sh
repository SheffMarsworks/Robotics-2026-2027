#!/bin/bash
# Brings up a SocketCAN interface for the ODrive bus.
# Usage: ./setup_can.sh [interface] [bitrate]
#   defaults: can0 500000
# The bitrate MUST equal can.config.baud_rate on every ODrive.
# 500 kbps is the safe choice for an MCP2515 hat with several ODrives streaming.
set -e

IFACE=${1:-can0}
BITRATE=${2:-500000}

echo "Bringing up $IFACE at $BITRATE bit/s..."
sudo ip link set "$IFACE" down 2>/dev/null || true
# restart-ms: recover automatically from bus-off instead of staying dead
sudo ip link set "$IFACE" up type can bitrate "$BITRATE" restart-ms 100
sudo ip link set "$IFACE" txqueuelen 1000

ip -details -statistics link show "$IFACE"
echo "$IFACE is up at $BITRATE bit/s."
