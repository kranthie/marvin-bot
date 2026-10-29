"""Robstride actuator CAN protocol (the motors' default "private" protocol), and a SocketCAN bus.

Frame layout and scaling are from RobStride's English user manuals (CAN 2.0 versions, section 4
"Driver protocol and instructions", RS00/RS04/RS06 revised 2026-07-13, RS03 2026-10-13):
https://github.com/RobStride/Product_Information

Classic CAN 2.0B at 1 Mbit/s with 29-bit identifiers:

    bits 28-24  communication type
    bits 23-8   data area 2 (the host CAN ID in bits 15-8 for most types; the feed-forward
                torque for type 1; status bits for type 2 replies)
    bits 7-0    destination CAN ID
    8 data bytes

Operation control (type 1) packs target angle, angular velocity, kp and kd as big-endian
uint16 values linearly mapped over each model's range; the feed-forward torque goes in the
identifier. Every command is answered with a feedback frame (type 2): angle, velocity, torque
(big-endian uint16) and temperature x 10, with fault and mode bits in the identifier. Parameter
reads and writes (types 17 and 18) carry a little-endian index and value.

The encoding functions are pure and covered by tests/test_robstride.py. ``RobstrideBus`` talks
to real motors through python-can and SocketCAN; it is UNTESTED until the phase 2 bench test.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass

# communication types (manual section 4.1)
GET_ID = 0
CONTROL = 1
FEEDBACK = 2
ENABLE = 3
STOP = 4
SET_ZERO = 6
SET_CAN_ID = 7
PARAM_READ = 17
PARAM_WRITE = 18
FAULT = 21

# parameters (manual section 4.1.13)
RUN_MODE = 0x7005  # 0 operation control, 1 position PP, 2 velocity, 3 current, 5 position CSP
LIMIT_TORQUE = 0x700B
LIMIT_SPD = 0x7017
LIMIT_CUR = 0x7018

FAULT_BITS = {
    16: "undervoltage",
    17: "three-phase overcurrent",
    18: "overtemperature",
    19: "magnetic encoder fault",
    20: "stall overload",
    21: "uncalibrated",
}
MODES = {0: "reset", 1: "calibration", 2: "run"}


@dataclass(frozen=True)
class Limits:
    """Ranges that the type 1 command and type 2 feedback map 0..65535 onto."""

    position: float  # rad, symmetric
    velocity: float  # rad/s, symmetric
    torque: float  # N·m, symmetric
    kp: float  # N·m/rad, 0..kp
    kd: float  # N·m·s/rad, 0..kd


MODELS = {
    "RS00": Limits(4 * math.pi, 33.0, 14.0, 500.0, 5.0),
    "RS03": Limits(4 * math.pi, 20.0, 60.0, 5000.0, 100.0),
    "RS04": Limits(4 * math.pi, 15.0, 120.0, 5000.0, 100.0),
    "RS06": Limits(4 * math.pi, 50.0, 36.0, 5000.0, 100.0),
}


def float_to_uint(x: float, lo: float, hi: float) -> int:
    x = min(max(x, lo), hi)
    return int(round((x - lo) * 65535.0 / (hi - lo)))


def uint_to_float(u: int, lo: float, hi: float) -> float:
    return lo + u * (hi - lo) / 65535.0


def arbitration_id(kind: int, data2: int, motor_id: int) -> int:
    return ((kind & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (motor_id & 0xFF)


def split_id(arbitration: int) -> tuple[int, int, int]:
    """(communication type, data area 2, destination ID) of a 29-bit identifier."""
    return (arbitration >> 24) & 0x1F, (arbitration >> 8) & 0xFFFF, arbitration & 0xFF


def control_frame(
    model: str,
    motor_id: int,
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque: float,
) -> tuple[int, bytes]:
    """Operation control (type 1): tau = kp (position - q) + kd (velocity - dq) + torque."""
    m = MODELS[model]
    tid = float_to_uint(torque, -m.torque, m.torque)
    data = struct.pack(
        ">HHHH",
        float_to_uint(position, -m.position, m.position),
        float_to_uint(velocity, -m.velocity, m.velocity),
        float_to_uint(kp, 0.0, m.kp),
        float_to_uint(kd, 0.0, m.kd),
    )
    return arbitration_id(CONTROL, tid, motor_id), data


def simple_frame(kind: int, motor_id: int, host_id: int, byte0: int = 0) -> tuple[int, bytes]:
    """Enable (3), stop (4; byte0 = 1 also clears faults) or set zero (6; byte0 = 1)."""
    return arbitration_id(kind, host_id, motor_id), bytes([byte0, 0, 0, 0, 0, 0, 0, 0])


def param_write_frame(motor_id: int, host_id: int, index: int, value: float | int, kind: str = "f"):
    """Single parameter write (type 18, lost on power-off): index and value little-endian.
    ``kind`` is the struct code of the value: "f" for float parameters, "B" for uint8."""
    raw = struct.pack("<" + kind, value).ljust(4, b"\0")
    return arbitration_id(PARAM_WRITE, host_id, motor_id), struct.pack("<HH", index, 0) + raw


def param_read_frame(motor_id: int, host_id: int, index: int) -> tuple[int, bytes]:
    return arbitration_id(PARAM_READ, host_id, motor_id), struct.pack("<HHI", index, 0, 0)


@dataclass
class Feedback:
    motor_id: int
    position: float  # rad
    velocity: float  # rad/s
    torque: float  # N·m
    temperature: float  # °C
    faults: list[str]
    mode: str
    stamp: float = 0.0


def parse_feedback(model: str, arbitration: int, data: bytes, stamp: float = 0.0) -> Feedback:
    """Decode a type 2 feedback frame."""
    kind, data2, _host = split_id(arbitration)
    if kind != FEEDBACK:
        raise ValueError(f"not a feedback frame (type {kind})")
    m = MODELS[model]
    pos, vel, tq, temp = struct.unpack(">HHHH", bytes(data[:8]))
    status = data2 << 8  # put data area 2 back at bits 23-8
    faults = [name for bit, name in FAULT_BITS.items() if status & (1 << bit)]
    return Feedback(
        motor_id=data2 & 0xFF,
        position=uint_to_float(pos, -m.position, m.position),
        velocity=uint_to_float(vel, -m.velocity, m.velocity),
        torque=uint_to_float(tq, -m.torque, m.torque),
        temperature=temp / 10.0,
        faults=faults,
        mode=MODES.get((status >> 22) & 0x3, "unknown"),
        stamp=stamp,
    )


class RobstrideBus:
    """One SocketCAN bus of Robstride motors. UNTESTED on hardware.

    ``motors`` maps CAN ID to model name, e.g. {1: "RS04", 2: "RS04", 3: "RS03"}.
    """

    def __init__(self, channel: str, motors: dict[int, str], host_id: int = 0xFD):
        import can  # python-can, only needed on the robot or the bench

        self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=1_000_000)
        self.can = can
        self.motors = motors
        self.host_id = host_id
        self.state: dict[int, Feedback] = {}

    def _send(self, frame: tuple[int, bytes]) -> None:
        arb, data = frame
        self.bus.send(self.can.Message(arbitration_id=arb, data=data, is_extended_id=True))

    def enable(self, motor_id: int) -> None:
        self._send(param_write_frame(motor_id, self.host_id, RUN_MODE, 0, "B"))
        self._send(simple_frame(ENABLE, motor_id, self.host_id))

    def stop(self, motor_id: int, clear_faults: bool = False) -> None:
        self._send(simple_frame(STOP, motor_id, self.host_id, 1 if clear_faults else 0))

    def set_zero(self, motor_id: int) -> None:
        self._send(simple_frame(SET_ZERO, motor_id, self.host_id, 1))

    def command(self, motor_id: int, position, velocity, kp, kd, torque) -> None:
        model = self.motors[motor_id]
        self._send(control_frame(model, motor_id, position, velocity, kp, kd, torque))

    def poll(self, timeout: float = 0.002) -> list[Feedback]:
        """Read every frame waiting on the bus; keep the latest feedback per motor."""
        out = []
        end = time.monotonic() + timeout
        while (msg := self.bus.recv(max(0.0, end - time.monotonic()))) is not None:
            kind, data2, _ = split_id(msg.arbitration_id)
            if kind == FEEDBACK and (data2 & 0xFF) in self.motors:
                fb = parse_feedback(
                    self.motors[data2 & 0xFF], msg.arbitration_id, msg.data, msg.timestamp
                )
                self.state[fb.motor_id] = fb
                out.append(fb)
        return out

    def close(self) -> None:
        self.bus.shutdown()
