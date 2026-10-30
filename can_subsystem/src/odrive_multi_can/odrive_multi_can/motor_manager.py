"""Drives any number of ODrive axes on one CAN bus.

- One background thread (python-can Notifier) reads EVERY frame as it arrives
  and routes it to the right axis by node ID, so feedback never lags behind.
- Setpoints are stored per axis and re-sent together by send_setpoints(),
  which the caller runs at a fixed rate (e.g. 50 Hz). Re-sending also keeps
  the ODrive's own watchdog fed, if you enable it.
- A command timeout zeroes any axis that has not had a new setpoint recently,
  so the rover stops if the controller / Wi-Fi drops out.

No ROS imports here: multi_motor_test.py uses this directly, the ROS node wraps it.
"""

import threading
import time
from dataclasses import dataclass, field

import can

from odrive_multi_can import odrive_can as oc


@dataclass
class MotorConfig:
    name: str
    node_id: int
    max_velocity: float = 2.0  # turns/s, commands are clamped to +/- this
    invert: bool = False  # flip sign, e.g. for wheels on the left side


@dataclass
class AxisStatus:
    position: float = 0.0  # turns
    velocity: float = 0.0  # turns/s
    axis_state: int = oc.AxisState.UNDEFINED
    axis_error: int = 0
    last_heartbeat: float = 0.0  # time.monotonic(), 0 = never seen
    last_encoder: float = 0.0


@dataclass
class ODriveAxis:
    cfg: MotorConfig
    status: AxisStatus = field(default_factory=AxisStatus)
    setpoint: float = 0.0
    last_command: float = 0.0  # time.monotonic() of last set_velocity()

    def online(self, now: float, timeout: float = 0.5) -> bool:
        return now - self.status.last_heartbeat < timeout


class MotorManager(can.Listener):
    def __init__(self, motors: list[MotorConfig], interface: str = 'can0',
                 bustype: str = 'socketcan', command_timeout: float = 0.5,
                 bus: can.BusABC | None = None):
        ids = [m.node_id for m in motors]
        if len(ids) != len(set(ids)):
            raise ValueError(f'Duplicate node_id in motor config: {ids}')

        self.axes: dict[str, ODriveAxis] = {m.name: ODriveAxis(m) for m in motors}
        self._by_node = {a.cfg.node_id: a for a in self.axes.values()}
        self.command_timeout = command_timeout
        self._lock = threading.Lock()

        self.bus = bus or can.interface.Bus(channel=interface, interface=bustype)
        self._notifier = can.Notifier(self.bus, [self])

    # ---- receive side (runs on the Notifier thread) ----

    def on_message_received(self, msg: can.Message):
        if msg.is_error_frame or msg.is_remote_frame:
            return
        node_id, cmd = oc.split_id(msg.arbitration_id)
        axis = self._by_node.get(node_id)
        if axis is None:
            return
        now = time.monotonic()
        with self._lock:
            if cmd == oc.Cmd.HEARTBEAT and len(msg.data) >= 5:
                axis.status.axis_error, axis.status.axis_state = oc.decode_heartbeat(msg.data)
                axis.status.last_heartbeat = now
            elif cmd == oc.Cmd.GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
                pos, vel = oc.decode_encoder_estimates(msg.data)
                sign = -1.0 if axis.cfg.invert else 1.0
                axis.status.position = sign * pos
                axis.status.velocity = sign * vel
                axis.status.last_encoder = now

    # ---- send side ----

    def _send(self, frame):
        arb_id, data = frame
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def wait_for_heartbeats(self, timeout: float = 2.0) -> list[str]:
        """Waits until every axis has sent a heartbeat. Returns names still missing."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            missing = self.missing_axes()
            if not missing:
                return []
            time.sleep(0.05)
        return self.missing_axes()

    def missing_axes(self) -> list[str]:
        now = time.monotonic()
        with self._lock:
            return [n for n, a in self.axes.items() if not a.online(now)]

    def enable_all(self, input_mode: oc.InputMode = oc.InputMode.VEL_RAMP,
                   timeout: float = 2.0) -> dict[str, bool]:
        """Clears errors, sets velocity mode and requests closed loop on every axis
        at once, then waits for each to report CLOSED_LOOP_CONTROL in its heartbeat.
        Returns {name: success}."""
        with self._lock:
            for axis in self.axes.values():
                axis.setpoint = 0.0
        for axis in self.axes.values():
            nid = axis.cfg.node_id
            self._send(oc.clear_errors(nid))
            self._send(oc.set_controller_mode(nid, oc.ControlMode.VELOCITY_CONTROL, input_mode))
            self._send(oc.set_input_vel(nid, 0.0))
            self._send(oc.set_axis_state(nid, oc.AxisState.CLOSED_LOOP_CONTROL))

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(s == oc.AxisState.CLOSED_LOOP_CONTROL for s in self._states().values()):
                break
            time.sleep(0.05)
        return {n: s == oc.AxisState.CLOSED_LOOP_CONTROL for n, s in self._states().items()}

    def _states(self) -> dict[str, int]:
        with self._lock:
            return {n: a.status.axis_state for n, a in self.axes.items()}

    def set_velocity(self, name: str, velocity: float):
        """Stores a setpoint (turns/s). It is sent by the next send_setpoints()."""
        axis = self.axes[name]
        lim = axis.cfg.max_velocity
        v = max(-lim, min(lim, float(velocity)))
        with self._lock:
            axis.setpoint = -v if axis.cfg.invert else v
            axis.last_command = time.monotonic()

    def set_velocities(self, velocities: dict[str, float]):
        """Sets several motors in one call; unknown names raise KeyError."""
        for name, vel in velocities.items():
            self.set_velocity(name, vel)

    def send_setpoints(self) -> list[str]:
        """Sends the current setpoint of every axis back-to-back. Any axis whose
        setpoint is older than command_timeout is zeroed. Returns names that
        timed out on this call (so the caller can log it once)."""
        now = time.monotonic()
        timed_out = []
        frames = []
        with self._lock:
            for name, axis in self.axes.items():
                if axis.setpoint != 0.0 and now - axis.last_command > self.command_timeout:
                    axis.setpoint = 0.0
                    timed_out.append(name)
                frames.append(oc.set_input_vel(axis.cfg.node_id, axis.setpoint))
        for f in frames:
            self._send(f)
        return timed_out

    def stop_all(self, idle: bool = True):
        """Zero velocity on every axis, then (by default) put them in IDLE."""
        with self._lock:
            for axis in self.axes.values():
                axis.setpoint = 0.0
        for axis in self.axes.values():
            self._send(oc.set_input_vel(axis.cfg.node_id, 0.0))
        if idle:
            for axis in self.axes.values():
                self._send(oc.set_axis_state(axis.cfg.node_id, oc.AxisState.IDLE))

    def snapshot(self) -> dict[str, AxisStatus]:
        """Thread-safe copy of every axis' latest feedback."""
        with self._lock:
            return {n: AxisStatus(**vars(a.status)) for n, a in self.axes.items()}

    def shutdown(self):
        try:
            self.stop_all(idle=True)
        finally:
            self._notifier.stop()
            self.bus.shutdown()


def load_config(path: str) -> dict:
    """Reads motors.yaml. Returns dict with 'can' settings and 'motors' list."""
    import yaml

    with open(path, 'r') as f:
        raw = yaml.safe_load(f)
    can_cfg = raw.get('can', {})
    motors = [MotorConfig(**m) for m in raw['motors']]
    return {
        'interface': can_cfg.get('interface', 'can0'),
        'bustype': can_cfg.get('bustype', 'socketcan'),
        'command_timeout': float(can_cfg.get('command_timeout', 0.5)),
        'control_rate': float(can_cfg.get('control_rate', 50.0)),
        # vel_ramp: ODrive smooths speed changes using its vel_ramp_rate (recommended)
        # passthrough: setpoints are applied instantly
        'input_mode': oc.InputMode[can_cfg.get('input_mode', 'vel_ramp').upper()],
        'motors': motors,
    }
