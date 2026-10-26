# odrive_multi_can: multi-motor ODrive CAN package (proposal)

Drives any number of ODrive motor controllers over one CAN bus (ODrive CANSimple protocol), from a Raspberry Pi today and a Jetson + CAN hat later. Works with ODrive 3.6, S1, Pro and Micro.

> **Status: proposal.** This is a new package added next to the existing `can_interface` code. None of the existing files were changed. It is a proposed replacement for `can_interface/can_ros2_node.py` and `can_test.py`. The reasons are in the code review.
> Both packages use the same `/<name>/command` and `/<name>/state` topics, so don't run both at once.

## Physical network

```text
 Host (Pi / Jetson)          ODrive node 0          ODrive node 1   ...   ODrive node N
 +--------------+            +-----------+          +-----------+         +-----------+
 |   CAN hat    |--CAN_H-----|-----------|----------|-----------|---------|-----------|
 |  [120 Ω ON]  |--CAN_L-----|-----------|----------|-----------|---------|-----------|
 +--------------+            +-----------+          +-----------+         | [120 Ω ON]|
                                                                          +-----------+
```

- Twisted pair for CAN_H / CAN_L, plus a shared GND between host and ODrives.
- **Exactly two 120 Ω terminators, one at each physical end of the bus.** Turn off the terminators on every ODrive in the middle. You can check this with the power off: measuring between CAN_H and CAN_L should read about 60 Ω.
- Every ODrive needs a **unique** `axis0.config.can.node_id`, and they all need the **same** `can.config.baud_rate`, which must match `setup_can.sh`.

## Layout

```text
can_subsystem/src/odrive_multi_can/      # new ROS 2 package (Python)
├── setup_can.sh               # brings up can0 (args: interface, bitrate; default can0 500000)
├── package.xml, setup.py, setup.cfg, resource/
├── config/motors.yaml         # CAN interface + one entry per motor  <-- edit this
├── odrive_multi_can/
│   ├── odrive_can.py          # CAN IDs, encode/decode (no ROS)
│   ├── motor_manager.py       # multi-motor driver: RX thread, enable, timeout (no ROS)
│   ├── multi_motor_node.py    # ROS 2 node (topics below)
│   └── multi_motor_test.py    # standalone demo, 1 motor or many
└── test/                      # pytest against simulated ODrives, no hardware needed
```

## How multi-motor works

`MotorManager` owns the bus. A background thread reads **every** incoming frame and stores the latest heartbeat (state and errors) and encoder estimate for each node ID. The control loop runs at 50 Hz and sends the current setpoint of every motor back-to-back. If a motor hasn't received a new command within `command_timeout` (0.5 s), its setpoint is set to 0, so the rover stops if the controlling laptop or the link drops. On shutdown, every motor is set to 0 and then IDLE.

To add a motor, add an entry to `config/motors.yaml`. You don't need to change any code.

## 1. Bring up the bus

```bash
cd can_subsystem/src/odrive_multi_can
./setup_can.sh                # can0 at 500 kbps
./setup_can.sh can0 1000000   # if your ODrives are set to 1 Mbps
```

## 2. Demo without ROS (start here)

You need `pip install python-can pyyaml`. Run these from `can_subsystem/src/odrive_multi_can`:

```bash
# Is anything there? Prints state/errors per node, does not move.
python3 -m odrive_multi_can.multi_motor_test --nodes 0 --listen

# 1 motor: node 0 at 1 turn/s for 2 s
python3 -m odrive_multi_can.multi_motor_test --nodes 0 --vel 1.0 --time 2

# several motors at the same time
python3 -m odrive_multi_can.multi_motor_test --nodes 0 1 2 3 --vel 1.0 --time 3

# every motor in the config
python3 -m odrive_multi_can.multi_motor_test --config config/motors.yaml
```

If a node shows `NO HEARTBEAT`, check the wiring, termination, node ID, and that the bitrate matches. If it shows `ERR 0x...`, look up the code in the ODrive docs for your firmware.

## 3. Build and run the ROS 2 node

```bash
cd ~/robotics_ws          # workspace containing can_subsystem/
rosdep install --from-paths can_subsystem/src/odrive_multi_can --ignore-src -r -y
colcon build --symlink-install --packages-select odrive_multi_can
source install/setup.bash

ros2 run odrive_multi_can multi_motor_node
# or with a different config:
ros2 run odrive_multi_can multi_motor_node --ros-args -p config_file:=/path/to/motors.yaml
```

### Topics

| Topic | Type | Meaning |
|---|---|---|
| `/<name>/command` | `std_msgs/Float32` | velocity setpoint for one motor, turns/s |
| `/<name>/state` | `sensor_msgs/JointState` | position (turns), velocity (turns/s) |
| `/motors/command` | `sensor_msgs/JointState` | `name[]` + `velocity[]`: set several motors in one message |
| `/motors/state` | `sensor_msgs/JointState` | all motors, in config order |

```bash
ros2 topic pub -r 10 /joint1/command std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub -r 10 /motors/command sensor_msgs/msg/JointState \
  "{name: [front_left, front_right], velocity: [1.0, 1.0]}"
ros2 topic echo /motors/state
```

Commands have to keep arriving (`-r 10` = 10 Hz). A single `--once` command stops after `command_timeout`. That's deliberate.

## Tests (no hardware)

```bash
cd can_subsystem/src/odrive_multi_can
python3 -m pytest test/
```

These tests run the manager against 6 simulated ODrives on a python-can virtual bus.

## Moving to the Jetson

- Set `can.interface` in `motors.yaml` (and the argument to `setup_can.sh`) to whatever `ip link` shows for the hat, usually `can0`. The code doesn't change.
- `cyclone_config.xml` has fixed IP addresses for the current machines, so update them, or use one copy per machine.

## Recommended ODrive settings

- `axis0.config.can.node_id`: unique per ODrive.
- `can.config.baud_rate`: same as `setup_can.sh`.
- Heartbeat and encoder messages at 10 ms. On the S1/Pro: `axis0.config.can.heartbeat_msg_rate_ms = 10`, `encoder_msg_rate_ms = 10`. On the 3.6, `encoder_rate_ms` must be set or no position feedback is sent.
- Optional second safety layer: enable the ODrive watchdog (`axis0.config.enable_watchdog = True`, `watchdog_timeout = 0.5`). The node re-sends setpoints at 50 Hz, which keeps it fed.
