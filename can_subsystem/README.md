# CAN Bus Motor Control Subsystem

This subdirectory houses the centralized Controller Area Network (CAN) communication matrix, node routing infrastructure, and electronic speed controller configurations for the mobile platform.

## 📡 Physical Network Topology

```text
  +------------------+          +-------------------+          +-------------------+

  |  Raspberry Pi 4  |          |   ODrive Axis 0   |          |   ODrive Axis 1   |
  |  [ Ubuntu OS ]   |          |  Node ID: 1       |          |  Node ID: 2       |
  |                  |  CAN_H   |                   |  CAN_H   |                   |
  | +--------------+ |----------|-------------------|----------|-------------------|
  | |   CAN Hat    | |          |  120Ω Terminator  |          |  120Ω Terminator  |
  | +--------------+ |----------|-------------------|----------|-------------------|
  +------------------+  CAN_L   +-------------------+  CAN_L   +-------------------+
```

### Hardware Prerequisites
- **Host Controller:** Raspberry Pi running an active 64-bit Linux distribution.
- **Transceiver Layer:** Dedicated MCP2515/Waveshare CAN Hat pinned to the SPI bus interface.
- **Actuators:** ODrive Brushless Motor Controllers configured with unique CAN Node IDs.
- **Bus Medium:** Dedicated twisted-pair copper wire carrying differential high (`CAN_H`) and low (`CAN_L`) signals, capped at both physical endpoints with 120Ω parallel termination resistors.

---

## 📂 Subsystem Workspace Architecture

```text
can_subsystem/
├── cyclone_config.xml       # Middleware configurations optimizing Eclipse Cyclone DDS profiles
├── config/                  # Centralized operational configuration store
│   └── motors.yaml          # Motor parameter allocation metrics (Baud rates, Node IDs, limits)
└── src/                     # Master source compilation area
    ├── can_interface/       # Core ROS 2 abstraction layer package (Python-based execution)
    │   ├── package.xml      # Automated package dependencies and hardware bindings index
    │   ├── setup.py         # Entry point definitions registering executable terminal scripts
    │   └── can_interface/
    │       ├── __init__.py  # Structural anchor for internal namespace compilation
    │       ├── can_ros2_node.py # Core lifecycle execution script mapping topics to frames
    │       └── can_test.py  # Standalone SocketCAN loopback diagnostic test runner
    └── ros_odrive/          # Standard ODrive hardware interface and protocol engine package
```

---

## 🛠️ Remote Target Deployment Execution

Follow these step-by-step instructions to pull only the necessary files down to your field-deployed Raspberry Pi, bring up the physical network link, and broadcast control targets.

### 1. Sparse-Clone Only the CAN Subsystem files to the Pi
To prevent unnecessary storage consumption on the embedded host computer, run a sparse-checkout to pull down **only** the `can_subsystem` architecture from your repository root:

```bash
# Initialize an empty git index on the Raspberry Pi target directory
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws
git init

# Attach your remote tracking address
git remote add origin https://github.com

# Restrict the workspace scope down to the CAN environment path
git config core.sparseCheckout true
echo "can_subsystem/*" >> .git/info/sparse-checkout

# Pull down the clean subsystem assets
git pull origin main
```

### 2. Configure the Linux SocketCAN Device
The underlying operating system must register the physical hardware transceiver hat as a standard network device socket. Bring up the network link (`can0`) at your target baud rate match (e.g., 250000, 500000, or 1000000 bps):

```bash
sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
```
*Verify the active physical driver state by querying: `ifconfig can0` or `ip -details -statistics link show can0`.*

### 3. Automatically Install System Dependencies
Leverage `rosdep` to parse your `package.xml` entries and automatically download missing prerequisite tools, message models, or hardware extensions:

```bash
cd ~/robotics_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Compile the Subsystem Workspace
Trigger a top-level compilation routine utilizing the custom build space arguments:

```bash
colcon build --symlink-install
```

---

## 🕹️ System Verification & Functional Execution

### A. Run Local Loopback Validation Tests
Before spinning up full ROS 2 messaging structures, run your standalone diagnosis pipeline loop directly on the SocketCAN stack to verify hardware transmission channels:

```bash
source install/setup.bash
ros2 run can_interface can_test
```

### B. Launch the Production Node Matrix
Load your custom global environment variables to direct the middleware configurations, then boot up your interface routing engine:

```bash
source install/setup.bash
export CYCLONE_DDS_URI=\$(pwd)/can_subsystem/cyclone_config.xml

# Run the centralized communication channel mapping loop
ros2 run can_interface can_node
```

### C. Command Motors & Read Feedback States via ROS 2 Topics
Your custom package transforms raw CAN data frame registers into clear, readable ROS 2 topics. 

- **Monitor Continuous Real-Time Telemetry Data:**
  Open a parallel monitor shell on your Pi or laptop to read incoming velocity, encoder counts, error logs, and temperature arrays streaming back continuously from all active node indices:
  ```bash
  ros2 topic echo /motor_telemetry
  ```

- **Broadcast Vector Movement Targets:**
  Publish velocity commands directly into the communication matrix topic pipeline to watch the physical motor shafts execute the targeted setpoints:
  ```bash
  ros2 topic pub --once /motor_commands can_msgs/msg/Frame "{id: 1, is_rtr: false, is_extended: false, is_error: false, dlc: 8, data:}"
  ```
