"""Motor mounting features from the actuators' real outlines (description/actuators.yaml).

A RobStride actuator is a body with its output on the front face and a smaller driver cap (with
the cable exit) standing proud of the rear face. Its stator bolts through its body's front or
rear face on the stator hole circle; whatever it drives bolts to the output flange's hole
circle. Plates stay flat (cad/b3d.py), so:

- `pillar_ring`: on the body's rear face, a pillar round each stator bolt, standing past the
  driver cap and joined by a thin ring at the far end, where a flat plate bolts on; the stator
  bolts run through the plate, the ring and the pillars into the motor. (The first pass used a
  ring on the rear face and three posts to the plate, which carried a joint's whole load
  through three 8 mm posts: finite elements put those at SF 0.2-1.3.)
- `output_plate`: a plate bolted to the output flange's hole circle, relieved 1 mm on the motor
  side outside the rotating flange so it can't rub the fixed face.

Positions are along the motor's axis in its link frame (mm): the motor centre is the middle of
its overall length (envelope), so the output face is at +t/2 along the output direction, the
cap's end at -t/2, and the body's rear face at -(t/2 - cap length).
"""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
_acts = yaml.safe_load((ROOT / "description" / "actuators.yaml").read_text())
CLEAR = {"M3": 3.4, "M4": 4.5, "M5": 5.5}  # bolt clearance holes


def spec(model: str) -> dict:
    a = _acts[model]
    d, _, t = a["envelope_mm"]
    sh = a.get("shape") or {}
    cap = sh.get("cap_len") or 0.0
    return dict(
        d=d,
        t=t,
        body_d=sh.get("body_d", d),
        cap_d=sh.get("cap_d"),
        cap=cap,
        output_d=sh.get("output_d", d / 2),
        rear=t / 2 - cap,  # body rear face, from the centre
        holes=a.get("holes", {}),
    )


def _vec(axis: str, sign: int, dist: float):
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = sign * dist
    return v


def _at(centre, axis, sign, dist):
    return [c + o for c, o in zip(centre, _vec(axis, sign, dist), strict=True)]


# --- interfaces: where a part is driven or carries the next link, for cad/fea.py. Each is a dict:
# the joint it serves, and a patch of the part's surface (cad.fea.Patch's fields): "face", an
# annulus r0-r1 on the plane through `centre` normal to `axis`; "cyl", a cylinder of radius r1
# about `axis` through `centre`, `span` along it.
_AX = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def face(joint: str, centre, axis: str, r0: float, r1: float, rigid: bool = True) -> dict:
    """`rigid` False: a load spread over the face (the ground), not a bolted part."""
    out = dict(joint=joint, kind="face", centre=tuple(centre), axis=_AX[axis], r0=r0, r1=r1)
    return out if rigid else {**out, "rigid": False}


def bore(joint: str, centre, axis: str, d: float, length: float) -> dict:
    """A bearing bore or pin hole of diameter d through a plate `length` thick."""
    span = (-length / 2, length / 2)
    return dict(joint=joint, kind="cyl", centre=tuple(centre), axis=_AX[axis], r1=d / 2, span=span)


def output_face(joint: str, model: str, centre, axis: str, sign: int) -> dict:
    """The contact face of a part bolted to a motor's output flange."""
    s = spec(model)
    return face(joint, _at(centre, axis, sign, s["t"] / 2), axis, 0.0, s["output_d"] / 2)


def stator_face(joint: str, model: str, centre, axis: str, sign: int) -> dict:
    """The contact face of a rear ring on a motor's body (its stator bolt circle)."""
    s = spec(model)
    r0 = (s["cap_d"] + 2) / 2 if s["cap_d"] else s["body_d"] / 2 - 10
    r1 = s["holes"]["stator"]["pcd"] / 2 + 7 if "stator" in s["holes"] else s["body_d"] / 2
    return face(joint, _at(centre, axis, -sign, s["rear"]), axis, r0, r1)


__all__ = [
    "bore",
    "face",
    "output_face",
    "output_holes",
    "output_plate",
    "pillar_ring",
    "ring_band",
    "ring_od",
    "spec",
    "stator_face",
    "stator_holes",
]


# --- build123d mounts (cad/b3d.py): plates stay flat; motors stand off them on pillar rings --


def _uvc(axis: str, centre):
    from . import b3d  # noqa: PLC0415

    return b3d.uv(axis, centre)


def stator_holes(model: str, centre, axis: str):
    """2D clearance holes for a motor's stator bolts, in the plane normal to `axis`."""
    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["stator"]
    return b3d.circles(CLEAR[h["thread"]], b3d.circle_pts(h["n"], h["pcd"], _uvc(axis, centre)))


def output_holes(model: str, centre, axis: str):
    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["output"]
    return b3d.circles(CLEAR[h["thread"]], b3d.circle_pts(h["n"], h["pcd"], _uvc(axis, centre)))


def ring_od(model: str) -> float:
    """The outside diameter of a stator mounting: the body, or round the bolt ears."""
    s = spec(model)
    return max(s["body_d"], s["holes"]["stator"]["pcd"] + 14)


def pillar_ring(
    model: str,
    centre,
    axis: str,
    sign: int,
    base: float = 2.5,
    clear: float = 1.0,
    od: float | None = None,
):
    """A motor's stator mounting: a pillar round each stator bolt, from the body's rear face
    out past the driver cap (`clear` mm), joined by a ring `base` thick at the far end, where a
    flat plate bolts on. The stator bolts run through the plate, the ring and the pillars into
    the motor, clamping them together: (B-rep, the plate's near face from the motor centre,
    along -sign); `od` trims the ring's outside diameter where space is short. Between the
    pillars, room for the cable exit and air round the cap."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    h = s["holes"]["stator"]
    c = _uvc(axis, centre)
    r_in = (s["cap_d"] + 2) / 2 if s["cap_d"] else s["body_d"] / 2 - 10
    r_out = (od or ring_od(model)) / 2
    w = CLEAR[h["thread"]] + 7  # 3.5 mm of wall either side of the bolt
    # each pillar a radial strip `w` wide across the ring; the windows between them run right
    # through the ring's width (straight sides, no corners to round, nothing tangent)
    pillars = []
    for k in range(h["n"]):
        a = 360.0 * k / h["n"]
        strip = b3d.rect(r_in - 3, r_out + 3, -w / 2, w / 2, r=0)
        pillars.append(b3d.bd.Pos(*c) * (b3d.bd.Rot(0, 0, a) * strip))
    holes = stator_holes(model, centre, axis)
    ring = b3d.cut(b3d.disc(2 * r_out, c), b3d.disc(2 * r_in, c), holes)
    # milled from one piece: the ring's full height, with windows cut between the pillars
    windows = b3d.cut(b3d.disc(2 * r_out + 2, c), b3d.disc(2 * r_in - 2, c), *pillars)
    i = "xyz".index(axis)
    rear = centre[i] - sign * s["rear"]
    near = s["rear"] + s["cap"] + clear  # the plate's near face, from the motor centre
    far = centre[i] - sign * near
    body = b3d.prism(ring, axis, rear, far)
    body = body - b3d.prism(windows, axis, rear + sign, far + sign * base)
    return body, near


def output_plate(model: str, outline, centre, axis: str, sign: int, t: float):
    """A plate of outline `outline` bolted to a motor's output flange: bolt holes on the output
    circle, and relieved 1 mm on the motor side outside the rotating flange so it can't rub the
    fixed face. (B-rep, the plate's mid-plane along `axis`)."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    i = "xyz".index(axis)
    face = centre[i] + sign * s["t"] / 2
    mid = face + sign * t / 2
    body = b3d.plate(b3d.cut(outline, output_holes(model, centre, axis)), axis, mid, t)
    c = _uvc(axis, centre)
    relief = b3d.cut(b3d.offset_box(outline, c), b3d.disc(s["output_d"] + 2, c))
    body = body - b3d.prism(relief, axis, face - sign, face + sign)
    return body, mid


def ring_band(model: str, centre, axis: str):
    """The band of a plate under a motor's pillar ring (the stator bolts), to keep solid when
    the plate is pocketed; the plate inside it, over the driver cap, can be pocketed too."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    c = _uvc(axis, centre)
    r_in = (s["cap_d"] + 2) / 2 if s["cap_d"] else s["body_d"] / 2 - 10
    return b3d.cut(b3d.disc(ring_od(model) + 2, c), b3d.disc(2 * r_in, c))
