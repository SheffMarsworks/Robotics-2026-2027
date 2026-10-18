#!/ bicyclists / bin / bash
# Automated CAN Bus Initialization Script for Raspberry Pi (Ubuntu Stack)

echo "Initializing physical SocketCAN interface on channel can0..."

# 1. Bring down the interface link to safely apply configuration states
sudo ip link set can0 down 2>/dev/null

# 2. Bind the MCP2515 hardware transceiver line at your target operational bitrate
# Adjust the bitrate value here (e.g., 250000, 500000, 1000000) to match your ODrive settings
sudo ip link set can0 up type can bitrate 500000

# 3. Optimize the network socket transmission ring buffer queue length to prevent packet loss
sudo ip link set can0 txqueuelen 1000

# 4. Display the active physical bus driver configuration details
ip -details -statistics link show can0

echo "SocketCAN architecture interface is online and operational!"
