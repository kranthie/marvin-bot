"""Clip-on covers: an optional add-on over the bare structure.

The robot is designed to look finished without them (graphite printed shells, aluminium
plates, the actuators' own housings). The covers add light panels in the same design language
and some fall protection. Each cover is a 2 mm printed skin following its link's parts and
motors at a 4 mm gap: the hull of everything on the link, offset outward, kept only on the
side the cover faces, and trimmed short of the joints so it never touches the next link.
Covers clip onto the parts under them (clip and boss details come with the fastener pass), so
every one comes off without tools, and none carries load.

They stay out of the physics model by default: tools/build_urdf.py writes them, with their
mass, only into description/urdf/marvin_bot_covers.urdf.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml

from . import arm, head, leg, torso
from .lib import Part, box, cyl, hull

ROOT = Path(__file__).resolve().parent.parent
GAP, SKIN = 4.0, 2.0
_DIRS = [
    np.array(d, float)
    for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
]
_DIRS += [
    np.array((a, b, c), float) / np.sqrt(3) for a in (1, -1) for b in (1, -1) for c in (1, -1)
]
BIG = 1000.0


def motors_on(link: str):
    """The motor housings carried by `link`, as solids (mm, link frame)."""
    sys.path.insert(0, str(ROOT / "tools"))
    import build_urdf

    out = []
    for m in build_urdf.motor_placements():
        if m["link"] != link:
            continue
        d, _, t = (x * 1000 for x in m["envelope"])
        ax = "xyz"[int(np.argmax(np.abs(m["axis"])))]
        out.append(cyl(d, t, ax, [x * 1000 for x in m["xyz"]]))
    return out


def offset_hull(solids, r: float):
    """Hull of `solids` grown by about r in every direction."""
    h = hull(*solids)
    return hull(*[h.translate(list(r * d)) for d in _DIRS])


def wrap(solids, keep, cut=()):
    """A cover skin around `solids`, kept inside the `keep` region, minus the `cut` regions."""
    skin = offset_hull(solids, GAP + SKIN) - offset_hull(solids, GAP)
    skin = skin ^ keep
    for c in cut:
        skin = skin - c
    return skin


def half(axis: str, sign: int, at: float = 0.0, lo=None, hi=None):
    """The half-space beyond `at` along `axis` (sign +1: coordinate > at; -1: coordinate < at),
    optionally cut to lo < z < hi."""
    i = "xyz".index(axis)
    centre = [0.0, 0.0, 0.0]
    centre[i] = at + sign * BIG / 2
    region = box((BIG, BIG, BIG), centre)
    if lo is not None or hi is not None:
        lo = -BIG if lo is None else lo
        hi = BIG if hi is None else hi
        region = region ^ box((BIG, BIG, hi - lo), (0, 0, (lo + hi) / 2))
    return region


def parts() -> list[Part]:
    by_link: dict[str, list] = {}
    for p in leg.parts() + torso.parts() + arm.parts() + head.parts():
        by_link.setdefault(p.link, []).append(p.solid)

    skel = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())
    # equipment still drawn as estimate boxes (computer, battery, electronics) also gets covered
    for b in skel["bodies"]:
        if b["budget"] in ("compute", "battery", "keep_alive", "electronics"):
            by_link.setdefault(b["link"], []).append(
                box([x * 1000 for x in b["size"]], [x * 1000 for x in b["xyz"]])
            )

    def on(link):
        return by_link.get(link, []) + motors_on(link)

    out: list[Part] = []

    def add(name, link, solid):
        out.append(Part(name, link, "pa_cf", solid, "cover"))

    # thigh: front and outer side, from below the hip yaw motor down over the knee motor's front
    j = {k: [x * 1000 for x in v["xyz"]] for k, v in skel["joints"].items()}
    knee_z = j["left_knee"][2]
    add(
        "left_thigh_cover",
        "left_thigh_link",
        wrap(on("left_thigh_link"), half("x", 1, -10, knee_z - 10, -32) - half("y", -1, -20)),
    )
    # shin guard: front, from below the knee motor to above the ankle
    ankle_z = j["left_ankle_pitch"][2]
    add(
        "left_shin_guard",
        "left_shin_link",
        wrap(on("left_shin_link"), half("x", 1, 5, ankle_z + 70, -70)),
    )
    # hip cap: outer side of the hip pitch motor and the yoke behind it
    add(
        "left_hip_cap",
        "left_hip_pitch_link",
        wrap(on("left_hip_pitch_link"), half("y", 1, 12, -60, 55) - half("x", -1, -45)),
    )
    # shoulder cap: over the top and outer side of the shoulder roll motor
    add(
        "left_shoulder_cap",
        "left_shoulder_pitch_link",
        wrap(on("left_shoulder_pitch_link"), half("y", 1, 20, -75, 60) - half("x", -1, -40)),
    )
    # pelvis: front and back panels between the hips
    pel = on("pelvis")
    add(
        "pelvis_front_cover",
        "pelvis",
        wrap(pel, half("x", 1, 30, -40, 80) ^ half("y", 1, -60) ^ half("y", -1, 60)),
    )
    # torso: a back panel over the battery bay (the battery slides out under it) and a collar
    # round the neck
    tor = on("torso_link")
    add(
        "torso_back_cover",
        "torso_link",
        wrap(tor, half("x", -1, -20, 70, 270) ^ half("y", 1, -80) ^ half("y", -1, 80)),
    )
    add(
        "torso_chest_cover",
        "torso_link",
        wrap(tor, half("x", 1, 40, 60, 262) ^ half("y", 1, -78) ^ half("y", -1, 78)),
    )
    add(
        "torso_collar",
        "torso_link",
        wrap(tor, half("z", 1, 262, None, 300) ^ half("y", 1, -80) ^ half("y", -1, 80)),
    )
    return out


def all_parts() -> list[Part]:
    ps = parts()
    return ps + [p.mirrored() for p in ps if p.name.startswith("left_")]
