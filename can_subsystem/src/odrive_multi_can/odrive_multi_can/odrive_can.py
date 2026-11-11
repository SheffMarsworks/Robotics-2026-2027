"""ODrive CANSimple protocol helpers.

No ROS imports here, so this file can be tested on any laptop with python-can.
Works with ODrive 3.6 (firmware 0.5.x) and ODrive S1 / Pro / Micro (0.6.x):
every message used below has the same ID and layout on both.

CAN ID layout (11-bit standard frame): (node_id << 5) | cmd_id
"""

import struct
from enum import IntEnum


class Cmd(IntEnum):
    HEARTBEAT = 0x001
    ESTOP = 0x002
    SET_AXIS_STATE = 0x007
    GET_ENCODER_ESTIMATES = 0x009
    SET_CONTROLLER_MODE = 0x00B
    SET_INPUT_VEL = 0x00D
    CLEAR_ERRORS = 0x018


class AxisState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    CLOSED_LOOP_CONTROL = 8


class ControlMode(IntEnum):
    VELOCITY_CONTROL = 2


class InputMode(IntEnum):
    PASSTHROUGH = 1
    VEL_RAMP = 2


def make_id(node_id: int, cmd: int) -> int:
    return (node_id << 5) | cmd


def split_id(arbitration_id: int) -> tuple[int, int]:
    """Returns (node_id, cmd_id)."""
    return arbitration_id >> 5, arbitration_id & 0x1F


# ---- Encoders: return (arbitration_id, data bytes) ----

def set_axis_state(node_id: int, state: AxisState):
    return make_id(node_id, Cmd.SET_AXIS_STATE), struct.pack('<I', state)


def set_controller_mode(node_id: int, control_mode: ControlMode, input_mode: InputMode):
    return make_id(node_id, Cmd.SET_CONTROLLER_MODE), struct.pack('<II', control_mode, input_mode)


def set_input_vel(node_id: int, vel: float, torque_ff: float = 0.0):
    return make_id(node_id, Cmd.SET_INPUT_VEL), struct.pack('<ff', vel, torque_ff)


def clear_errors(node_id: int):
    # S1/Pro read one "identify" byte; 3.6 ignores the payload.
    return make_id(node_id, Cmd.CLEAR_ERRORS), b'\x00'


def estop(node_id: int):
    return make_id(node_id, Cmd.ESTOP), b''


# ---- Decoders ----

def decode_heartbeat(data: bytes) -> tuple[int, int]:
    """Returns (axis_error, axis_state)."""
    axis_error, axis_state = struct.unpack_from('<IB', data)
    return axis_error, axis_state


def decode_encoder_estimates(data: bytes) -> tuple[float, float]:
    """Returns (position [turns], velocity [turns/s])."""
    return struct.unpack_from('<ff', data)
