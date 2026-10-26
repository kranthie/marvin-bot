"""Tests for the Robstride frame encoding (robot/drivers/robstride.py), no hardware needed."""

import math
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot" / "drivers"))
import robstride as rs  # noqa: E402


def test_identifier_layout():
    arb = rs.arbitration_id(rs.ENABLE, 0xFD << 8, 0x7F)
    assert arb == (3 << 24) | (0xFD << 16) | 0x7F
    assert rs.split_id(arb) == (3, 0xFD00, 0x7F)


def test_uint_mapping_ends_and_middle():
    assert rs.float_to_uint(-120.0, -120.0, 120.0) == 0
    assert rs.float_to_uint(120.0, -120.0, 120.0) == 65535
    assert rs.float_to_uint(0.0, -120.0, 120.0) in (32767, 32768)
    assert rs.float_to_uint(500.0, -120.0, 120.0) == 65535  # clipped
    assert rs.uint_to_float(0, -15.0, 15.0) == -15.0
    assert rs.uint_to_float(65535, -15.0, 15.0) == 15.0


@pytest.mark.parametrize("model", sorted(rs.MODELS))
def test_control_frame_round_trip(model):
    m = rs.MODELS[model]
    pos, vel, kp, kd, tq = 1.234, -0.4 * m.velocity, 0.3 * m.kp, 0.05 * m.kd, -0.25 * m.torque
    arb, data = rs.control_frame(model, 5, pos, vel, kp, kd, tq)
    kind, data2, dest = rs.split_id(arb)
    assert (kind, dest, len(data)) == (rs.CONTROL, 5, 8)
    p, v, k, d = struct.unpack(">HHHH", data)
    # one count of quantisation on each value
    assert rs.uint_to_float(p, -m.position, m.position) == pytest.approx(
        pos, abs=2 * m.position / 65535
    )
    assert rs.uint_to_float(v, -m.velocity, m.velocity) == pytest.approx(
        vel, abs=2 * m.velocity / 65535
    )
    assert rs.uint_to_float(k, 0, m.kp) == pytest.approx(kp, abs=m.kp / 65535)
    assert rs.uint_to_float(d, 0, m.kd) == pytest.approx(kd, abs=m.kd / 65535)
    assert rs.uint_to_float(data2, -m.torque, m.torque) == pytest.approx(
        tq, abs=2 * m.torque / 65535
    )


def test_feedback_parse():
    m = rs.MODELS["RS04"]
    status = (2 << 22) | (1 << 18) | (0x07 << 8)  # run mode, overtemperature, motor 7
    arb = (rs.FEEDBACK << 24) | (status & 0xFFFF00) | 0xFD
    data = struct.pack(
        ">HHHH",
        rs.float_to_uint(math.pi / 2, -m.position, m.position),
        rs.float_to_uint(3.0, -m.velocity, m.velocity),
        rs.float_to_uint(-40.0, -m.torque, m.torque),
        456,
    )
    fb = rs.parse_feedback("RS04", arb, data)
    assert fb.motor_id == 7
    assert fb.position == pytest.approx(math.pi / 2, abs=1e-3)
    assert fb.velocity == pytest.approx(3.0, abs=1e-3)
    assert fb.torque == pytest.approx(-40.0, abs=5e-3)
    assert fb.temperature == pytest.approx(45.6)
    assert fb.faults == ["overtemperature"]
    assert fb.mode == "run"


def test_param_frames():
    arb, data = rs.param_write_frame(3, 0xFD, rs.RUN_MODE, 0, "B")
    assert rs.split_id(arb) == (rs.PARAM_WRITE, 0xFD00, 3)
    assert data == bytes([0x05, 0x70, 0, 0, 0, 0, 0, 0])
    arb, data = rs.param_write_frame(3, 0xFD, rs.LIMIT_SPD, 2.5)
    assert struct.unpack("<HHf", data) == (rs.LIMIT_SPD, 0, 2.5)
    arb, data = rs.param_read_frame(3, 0xFD, rs.LIMIT_CUR)
    assert rs.split_id(arb)[0] == rs.PARAM_READ and data[:2] == bytes([0x18, 0x70])


def test_models_match_actuators_yaml():
    """The protocol torque ranges equal the peak torques in description/actuators.yaml."""
    import yaml

    acts = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "description" / "actuators.yaml").read_text()
    )
    for model, lim in rs.MODELS.items():
        assert lim.torque == acts[model]["peak_torque"]
