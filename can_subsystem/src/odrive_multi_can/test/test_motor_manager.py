"""Runs MotorManager against simulated ODrives on a python-can virtual bus.
No hardware or ROS needed:  python3 -m pytest test/
"""

import struct
import threading
import time

import can
import pytest

from odrive_multi_can import odrive_can as oc
from odrive_multi_can.motor_manager import MotorConfig, MotorManager


class FakeODrive(threading.Thread):
    """Answers like an ODrive: heartbeat + encoder estimates every 10 ms,
    follows Set_Axis_State / Set_Input_Vel / Clear_Errors."""

    def __init__(self, channel, node_id, error=0):
        super().__init__(daemon=True)
        self.bus = can.interface.Bus(channel=channel, interface='virtual')
        self.node_id, self.error = node_id, error
        self.state, self.vel, self.pos = oc.AxisState.IDLE, 0.0, 0.0
        self.running = True

    def run(self):
        next_tx = time.monotonic()
        while self.running:
            msg = self.bus.recv(timeout=0.002)
            if msg is not None:
                nid, cmd = oc.split_id(msg.arbitration_id)
                if nid == self.node_id:
                    if cmd == oc.Cmd.SET_AXIS_STATE:
                        req = struct.unpack('<I', msg.data)[0]
                        if not (self.error and req == oc.AxisState.CLOSED_LOOP_CONTROL):
                            self.state = req
                    elif cmd == oc.Cmd.SET_INPUT_VEL:
                        self.vel = struct.unpack('<ff', msg.data)[0]
                    elif cmd == oc.Cmd.CLEAR_ERRORS:
                        pass  # sticky error in this sim: clear does not fix it
            if time.monotonic() >= next_tx:
                next_tx += 0.01
                moving = self.state == oc.AxisState.CLOSED_LOOP_CONTROL
                v = self.vel if moving else 0.0
                self.pos += v * 0.01
                for cmd, data in (
                        (oc.Cmd.HEARTBEAT, struct.pack('<IBBBB', self.error, self.state, 0, 0, 0)),
                        (oc.Cmd.GET_ENCODER_ESTIMATES, struct.pack('<ff', self.pos, v))):
                    self.bus.send(can.Message(arbitration_id=oc.make_id(self.node_id, cmd),
                                              data=data, is_extended_id=False))

    def stop(self):
        self.running = False
        self.join()
        self.bus.shutdown()


@pytest.fixture
def channel(request):
    return f'test_{request.node.name}'


def make(channel, motors, **kw):
    bus = can.interface.Bus(channel=channel, interface='virtual')
    return MotorManager(motors, bus=bus, **kw)


def test_six_motors_enable_command_and_feedback(channel):
    nodes = [1, 2, 3, 4, 5, 6]
    fakes = [FakeODrive(channel, n) for n in nodes]
    for f in fakes:
        f.start()
    motors = [MotorConfig(name=f'm{n}', node_id=n, max_velocity=2.0, invert=(n <= 3))
              for n in nodes]
    mgr = make(channel, motors)
    try:
        assert mgr.wait_for_heartbeats() == []
        assert all(mgr.enable_all().values())
        mgr.set_velocities({m.name: 1.5 for m in motors})
        for _ in range(10):
            mgr.send_setpoints()
            time.sleep(0.02)
        snap = mgr.snapshot()
        for f in fakes:
            expected_raw = -1.5 if f.node_id <= 3 else 1.5  # inverted on the wire
            assert f.vel == pytest.approx(expected_raw)
            # feedback is un-inverted back to the commanded sign
            assert snap[f'm{f.node_id}'].velocity == pytest.approx(1.5)
            assert snap[f'm{f.node_id}'].position > 0
    finally:
        mgr.shutdown()
        time.sleep(0.05)
        for f in fakes:
            assert f.state == oc.AxisState.IDLE and f.vel == 0.0
            f.stop()


def test_velocity_is_clamped(channel):
    f = FakeODrive(channel, 0)
    f.start()
    mgr = make(channel, [MotorConfig('a', 0, max_velocity=1.0)])
    try:
        mgr.enable_all()
        mgr.set_velocity('a', 50.0)
        mgr.send_setpoints()
        time.sleep(0.05)
        assert f.vel == pytest.approx(1.0)
    finally:
        mgr.shutdown()
        f.stop()


def test_command_timeout_stops_motor(channel):
    f = FakeODrive(channel, 0)
    f.start()
    mgr = make(channel, [MotorConfig('a', 0)], command_timeout=0.1)
    try:
        mgr.enable_all()
        mgr.set_velocity('a', 1.0)
        assert mgr.send_setpoints() == []
        time.sleep(0.15)
        assert mgr.send_setpoints() == ['a']
        time.sleep(0.05)
        assert f.vel == 0.0
    finally:
        mgr.shutdown()
        f.stop()


def test_reports_axis_that_fails_closed_loop(channel):
    good, bad = FakeODrive(channel, 0), FakeODrive(channel, 1, error=0x40)
    good.start()
    bad.start()
    mgr = make(channel, [MotorConfig('good', 0), MotorConfig('bad', 1)])
    try:
        result = mgr.enable_all(timeout=0.5)
        assert result == {'good': True, 'bad': False}
        assert mgr.snapshot()['bad'].axis_error == 0x40
    finally:
        mgr.shutdown()
        good.stop()
        bad.stop()


def test_missing_node_detected(channel):
    f = FakeODrive(channel, 0)
    f.start()
    mgr = make(channel, [MotorConfig('here', 0), MotorConfig('gone', 7)])
    try:
        assert mgr.wait_for_heartbeats(timeout=0.3) == ['gone']
    finally:
        mgr.shutdown()
        f.stop()


def test_duplicate_node_ids_rejected(channel):
    with pytest.raises(ValueError):
        make(channel, [MotorConfig('a', 1), MotorConfig('b', 1)])
