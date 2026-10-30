"""Motor mounting features from the actuators' real outlines (description/actuators.yaml).

A RobStride actuator is a body with its output on the front face and a smaller driver cap (with
the cable exit) standing proud of the rear face. Its stator bolts through its body's front or
rear face on the stator hole circle; whatever it drives bolts to the output flange's hole
circle. So:

- `rear_ring`: a ring plate on the body's rear face, with a clearance hole for the cap and the
  stator holes; where a far-side bearing sits behind the motor on its axis, `posts` join the
  ring to an outer plate over the cap that carries the bearing's stub axle;
- `output_flange`: a plate on the output flange, its bolt holes on the output hole circle, and
  relieved 1 mm on the motor side outside the rotating flange so it can't rub the fixed face.

Positions are along the motor's axis in its link frame (mm): the motor centre is the middle of
its overall length (envelope), so the output face is at +t/2 along the output direction, the
cap's end at -t/2, and the body's rear face at -(t/2 - cap length).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .lib import box, cyl, hull, union

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


def bolt_holes(n: int, thread: str, pcd: float, axis: str, centre, length: float = 60.0, phase=0.0):
    """Clearance holes for n bolts on a circle of diameter pcd about `axis` through `centre`."""
    i = "xyz".index(axis)
    u, v = [k for k in range(3) if k != i]
    holes = []
    for k in range(n):
        a = phase + 2 * np.pi * k / n
        c = list(centre)
        c[u] += pcd / 2 * np.cos(a)
        c[v] += pcd / 2 * np.sin(a)
        holes.append(cyl(CLEAR[thread], length, axis, c))
    return union(*holes)


def rear_ring(model: str, centre, axis: str, sign: int, t: float = 4.0, outer: float | None = None):
    """A ring on the body's rear face of a motor centred at `centre` with its output toward
    `sign` along `axis`: (solid, plate centre)."""
    s = spec(model)
    pc = _at(centre, axis, -sign, s["rear"] + t / 2)
    od = outer or (s["holes"]["stator"]["pcd"] + 14 if "stator" in s["holes"] else s["body_d"])
    ring = cyl(od, t, axis, pc)
    if s["cap_d"]:
        ring = ring - cyl(s["cap_d"] + 2, t + 2, axis, pc)
    if "stator" in s["holes"]:
        h = s["holes"]["stator"]
        ring = ring - bolt_holes(h["n"], h["thread"], h["pcd"], axis, pc, t + 2)
    return ring, pc


def post_points(model: str, centre, axis: str, n: int = 3):
    """Where the posts stand, in the plane through `centre` normal to `axis`."""
    s = spec(model)
    r = (s["cap_d"] + s["body_d"]) / 4 + 2 if s["cap_d"] else s["body_d"] / 2 - 6
    i = "xyz".index(axis)
    u, v = [k for k in range(3) if k != i]
    pts = []
    for k in range(n):
        ang = 2 * np.pi * (k + 0.25) / n
        c = list(centre)
        c[u] += r * np.cos(ang)
        c[v] += r * np.sin(ang)
        pts.append(c)
    return pts


def spider(
    model: str,
    centre,
    axis: str,
    sign: int,
    at: float,
    t: float,
    width: float = 16.0,
    hub: float = 40.0,
):
    """A light outer plate over the cap: a hub on the motor's axis (for a bearing stub) and
    three spokes out to the posts. `at` is its near face, from the motor centre (along -sign)."""
    pc = _at(centre, axis, -sign, at + t / 2)
    i = "xyz".index(axis)
    spokes = []
    for p in post_points(model, centre, axis):
        p = list(p)
        p[i] = pc[i]
        spokes.append(hull(cyl(width, t, axis, pc), cyl(width, t, axis, p)))
    return union(cyl(hub, t, axis, pc), *spokes)


def posts(
    model: str,
    centre,
    axis: str,
    sign: int,
    ring_t: float,
    outer_at: float,
    n: int = 3,
    d: float = 8.0,
):
    """Posts from a rear ring out to an outer plate whose near face is `outer_at` from the
    motor centre (along -sign): between the cap and the body's rim."""
    s = spec(model)
    r = (s["cap_d"] + s["body_d"]) / 4 + 2 if s["cap_d"] else s["body_d"] / 2 - 6
    a0 = s["rear"] + ring_t
    i = "xyz".index(axis)
    u, v = [k for k in range(3) if k != i]
    out = []
    for k in range(n):
        ang = 2 * np.pi * (k + 0.25) / n
        c = list(centre)
        c[u] += r * np.cos(ang)
        c[v] += r * np.sin(ang)
        c[i] += -sign * (a0 + outer_at) / 2
        out.append(cyl(d, outer_at - a0, axis, c))
    return union(*out)


def output_flange(model: str, centre, axis: str, sign: int, d: float, t: float = 6.0):
    """A plate of diameter d on the output face: (solid, plate centre). Bolt holes on the
    output circle; relieved 1 mm on the motor side outside the rotating flange."""
    s = spec(model)
    pc = _at(centre, axis, sign, s["t"] / 2 + t / 2)
    plate = cyl(d, t, axis, pc)
    relief = cyl(d + 2, 1.0, axis, _at(centre, axis, sign, s["t"] / 2 + 0.5)) - cyl(
        s["output_d"] + 2, 3.0, axis, _at(centre, axis, sign, s["t"] / 2 + 0.5)
    )
    plate = plate - relief
    if "output" in s["holes"]:
        h = s["holes"]["output"]
        plate = plate - bolt_holes(h["n"], h["thread"], h["pcd"], axis, pc, t + 2)
        # lightening holes between the bolt circle and the rim, where there is room
        rim_r = d / 2 - 6
        in_r = h["pcd"] / 2 + 6
        if rim_r - in_r >= 8:
            hd = min(14.0, rim_r - in_r)
            n = int(np.pi * (rim_r + in_r) / (hd + 6))
            ring = []
            i = "xyz".index(axis)
            u, v = [k for k in range(3) if k != i]
            for k in range(n):
                a_ = 2 * np.pi * (k + 0.5) / n
                c = list(pc)
                c[u] += (rim_r + in_r) / 2 * np.cos(a_)
                c[v] += (rim_r + in_r) / 2 * np.sin(a_)
                ring.append(cyl(hd, t + 2, axis, c))
            plate = plate - union(*ring)
    return plate, pc


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
    "box",
    "bolt_holes",
    "face",
    "output_face",
    "stator_face",
    "output_flange",
    "post_points",
    "posts",
    "rear_ring",
    "spec",
    "spider",
]
