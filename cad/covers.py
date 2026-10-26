"""Clip-on body shells: an optional add-on over the bare structure.

The robot is designed to look finished without them (graphite printed shells, aluminium
plates, the actuators' own housings). The shells give it a finished body: a smooth light
torso tapering to the waist, a dark hip girdle, long leg shells, arm sleeves, toe caps and a
dark neck sleeve under the helmet head, with the joints left visible between them. They add
some fall protection.

Each shell is a 1.6 mm printed skin following its link's parts and motors at a 4 mm gap: the
hull of everything on the link, grown outward (rounded), kept only where it cannot reach the
next link, and split where it must come off for service. Shells clip onto the parts under them
(clip and boss details come with the fastener pass), so every one comes off without tools,
and none carries load. `python -m cad.covers --check` moves every joint through its range and
fails if a shell touches a part of another link.

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
GAP, SKIN = 4.0, 1.6


def _sphere_dirs() -> list[np.ndarray]:
    """42 directions spread over the sphere (an icosahedron, subdivided once): offsetting a
    hull along them rounds its edges."""
    t = (1 + 5**0.5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t)]
    v += [(0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    pts = [np.array(p, float) / np.linalg.norm(p) for p in v]
    edge = min(np.linalg.norm(pts[0] - q) for q in pts[1:])
    mids = []
    for i, a in enumerate(pts):
        for b in pts[i + 1 :]:
            if np.linalg.norm(a - b) < edge * 1.01:
                m = (a + b) / 2
                mids.append(m / np.linalg.norm(m))
    return pts + mids


_DIRS = _sphere_dirs()
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


def lofted(solids, gap: float, corner: float, step: float = 15.0, style=None):
    """A smooth body round `solids`: at each height, a rectangle with large rounded corners
    (radius `corner`) just containing everything in that slice plus `gap`; the hull of all of
    them, closed at the top by a smaller section so the shoulders round over too. `style(z)`
    may give a larger rectangle (x0, y0, x1, y1) at a height, to shape the body beyond what it
    must contain."""
    h = hull(*solids)
    lo, hi = h.bounding_box()[2], h.bounding_box()[5]

    def section(x0, y0, x1, y1, zc, r):
        r = min(r, (x1 - x0) / 2 - 1, (y1 - y0) / 2 - 1)
        return hull(
            *[cyl(2 * r, 2.0, "z", (x, y, zc)) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)]
        )

    sections, top = [], None
    for z in np.arange(lo + step / 2, hi, step):
        sl = h ^ box((BIG, BIG, step), (0, 0, z))
        if sl.is_empty():
            continue
        b = sl.bounding_box()
        pull = gap + corner * (1 - 0.5**0.5)  # the rounded corner still contains the slice's
        rect = (b[0] - pull, b[1] - pull, b[3] + pull, b[4] + pull)
        if style is not None:
            sx0, sy0, sx1, sy1 = style(z)
            rect = (min(rect[0], sx0), min(rect[1], sy0), max(rect[2], sx1), max(rect[3], sy1))
        for zz in (z - step / 2, z + step / 2):
            sections.append(section(*rect, zz, corner))
        top = rect
    x0, y0, x1, y1 = top
    inset = 0.6 * corner
    sections.append(section(x0 + inset, y0 + inset, x1 - inset, y1 - inset, hi + gap, corner))
    return hull(*sections)


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

    j = {k: [x * 1000 for x in v["xyz"]] for k, v in skel["joints"].items()}
    acts = yaml.safe_load((ROOT / "description" / "actuators.yaml").read_text())

    def env(model):
        d, _, t = acts[model]["envelope_mm"]
        return d, t

    d04, t04 = env("RS04")
    d03, t03 = env("RS03")
    d06, t06 = env("RS06")
    d00, t00 = env("RS00")

    # --- torso: a smooth shell from the waist to the shoulders, in a front half and a back half
    # (the back half comes off for the battery). It stops short of the shoulder flanges, leaves
    # a round opening for the neck sleeve, and its bottom edge clears the hip girdle.
    tor = on("torso_link")
    sh_y = j["left_shoulder_pitch"][1] - 8  # inside the shoulder pitch output flange
    sh = j["left_shoulder_pitch"]
    body = (
        box((BIG, BIG, BIG), (0, 0, 0))
        - half("z", -1, 26.0)
        - cyl(108, BIG, "y", (sh[0], 0, sh[2]))  # round openings for the shoulder motors
        - cyl(104, 200, "z", (0, 0, j["neck_yaw"][2]))
        - cyl(150, 100, "z", (0, 0, j["neck_yaw"][2] + 3 + 50))  # a shallow collar: chin room
    )

    def chest(z, grow=0.0):  # a chest that widens from the waist to the shoulders
        k = float(np.clip((z - 40.0) / 180.0, 0.0, 1.0))
        k = np.sin(k * np.pi / 2)  # full width well below the shoulders
        half_w = 80.0 + 50.0 * k + grow
        return (-70.0 - 28.0 * k - grow, -half_w, 72.0 + 30.0 * k + grow, half_w)

    # flat side walls short of the shoulder flanges close the sides
    outer_b = lofted(tor, GAP + SKIN, 34.0, style=lambda z: chest(z, SKIN))
    inner_b = lofted(tor, GAP, 34.0 - SKIN, style=chest)
    skin = (outer_b ^ box((BIG, 2 * sh_y, BIG))) - (inner_b ^ box((BIG, 2 * (sh_y - SKIN), BIG)))
    # each half in four pieces (left and right, chest and waist): a whole half is too big to
    # print; the seams run down the middle and round the body under the chest
    seam_z = 150.0
    for name, s_ in (("front", 1), ("back", -1)):
        part = skin ^ body ^ half("x", s_, -20)
        for side, sy in (("l", 1), ("r", -1)):
            q = part ^ half("y", sy, 0)
            add(f"torso_cover_{name}_{side}", "torso_link", q ^ half("z", 1, seam_z))
            add(f"torso_cover_{name}_{side}_waist", "torso_link", q ^ half("z", -1, seam_z))

    # --- neck sleeve (dark, flexible print): from inside the torso shell's neck opening up
    # into the helmet, turning with neck yaw; the head pitches over it
    nz = j["neck_pitch"][2]
    sleeve = hull(cyl(76, 2, "z", (0, 0, -4)), cyl(68, 2, "z", (0, 0, nz + 8))) - hull(
        cyl(72, 2, "z", (0, 0, -6)), cyl(64, 2, "z", (0, 0, nz + 10))
    )  # starts just above the torso bridge
    add("neck_cover", "neck_link", sleeve)

    # --- hip girdle (dark): round the pelvis frame and the waist motor, between the hips; in
    # front it reaches over the hip pitch motors, behind it stays clear of the hip roll motors
    # that swing round the hip axis
    pel = on("pelvis")
    waist_top = j["waist_yaw"][2] + 18.0  # below the torso shell's bottom edge
    centre = half("y", 1, -40) ^ half("y", -1, 40)
    hip_face = j["left_hip_pitch"][1] + t04 / 2 - 4  # short of the hip pitch output flanges
    front = half("x", 1, 30, -45, None) ^ half("y", 1, -hip_face) ^ half("y", -1, hip_face)
    add(
        "pelvis_cover",
        "pelvis",
        wrap(pel, (centre + front) ^ half("z", -1, waist_top) ^ half("z", 1, -75)),
    )

    # --- thigh shell: all round the thigh from under the hip yaw motor to the knee, except a
    # strip at the back of the inner side (harness); over the knee motor only in front
    knee_z = j["left_knee"][2]
    yaw_out = -t03 / 2
    thigh = on("left_thigh_link")
    upper = half("z", -1, yaw_out - 10) ^ half("z", 1, knee_z + d04 / 2 + 10)
    kneecap = half("z", 1, knee_z - 10, None, knee_z + d04 / 2 + 12) ^ half("x", 1, 40)
    add(
        "left_thigh_cover",
        "left_thigh_link",
        wrap(thigh, (upper + kneecap) - (half("x", -1, -25) ^ half("y", -1, -10))),
    )

    # --- shin shell: front and sides, from below the knee to above the ankle; open at the back
    # where the cranks and push rods move
    ankle_z = j["left_ankle_pitch"][2]
    shin = on("left_shin_link")
    add(
        "left_shin_cover",
        "left_shin_link",
        wrap(shin, half("x", 1, 0, ankle_z + 55, -d04 / 2 - 12)),
    )

    # --- hip cap: outer side of the hip pitch motor's yoke
    add(
        "left_hip_cover",
        "left_hip_pitch_link",
        wrap(on("left_hip_pitch_link"), half("y", 1, 36, -60, 55) - half("x", -1, -45)),
    )
    # --- shoulder cap: over the top of the shoulder roll motor. The roll link's parts swing
    # round the roll axis from straight down to 30° above horizontal, so the cap keeps above a
    # plane through that axis at 40°, and behind the roll output face
    ro = j["left_shoulder_roll"]
    up = box((BIG, BIG, BIG), (0, 0, BIG / 2)).rotate([40, 0, 0]).translate([0, ro[1], ro[2]])
    add(
        "left_shoulder_cover",
        "left_shoulder_pitch_link",
        wrap(
            on("left_shoulder_pitch_link"),
            up ^ half("x", -1, t03 / 2 - 4) ^ half("x", 1, -40) ^ half("y", 1, 14),
        ),
    )

    # --- arm sleeves: the upper arm from the yaw output to above the elbow motor, the forearm
    # from below the elbow motor to the wrist output
    el = j["left_elbow"]
    add(
        "left_upper_arm_cover",
        "left_upper_arm_link",
        wrap(
            on("left_upper_arm_link"),
            half("z", -1, -t06 / 2 - 8) ^ half("z", 1, el[2] + d06 / 2 + 6),
        ),
    )
    wr = j["left_wrist_yaw"]
    add(
        "left_forearm_cover",
        "left_forearm_link",
        wrap(
            on("left_forearm_link"),
            half("z", -1, -d06 / 2 - 26) ^ half("z", 1, wr[2] - t00 / 2 + 2),
        ),
    )

    # --- toe cap: over the front of the foot, above the sole
    foot = on("left_foot_link")
    add("left_toe_cover", "left_foot_link", wrap(foot, half("x", 1, 55) ^ half("z", 1, -54)))
    return out


def all_parts() -> list[Part]:
    ps = parts()
    return ps + [p.mirrored() for p in ps if p.name.startswith("left_")]


# ------------------------------------------------------------------------- clearance check

SIDE_GROUPS = ("leg", "arm")  # shells on one limb are not checked against the other limbs


def _group(link: str) -> str:
    if link.startswith(("left_", "right_")):
        side, rest = link.split("_", 1)
        limb = (
            "arm"
            if rest.startswith(("shoulder", "upper_arm", "forearm", "wrist", "gripper"))
            else "leg"
        )
        return f"{side}_{limb}"
    return "body"


def poses(joints: dict, ranges: dict) -> list[tuple[str, dict]]:
    """Each joint at both ends and half of each end of its range (the others at zero), and
    some combined poses."""
    out = [("zero", {})]
    for name in joints:
        lo, hi = ranges[name]
        for q in (lo, lo / 2, hi / 2, hi):
            if q:
                out.append((f"{name} {q:+.0f}", {name: q}))
    for side in ("left", "right"):
        out += [
            (
                f"{side} deep squat",
                {f"{side}_hip_pitch": 70, f"{side}_knee": 125, f"{side}_ankle_pitch": 40},
            ),
            (f"{side} knee up", {f"{side}_hip_pitch": 110, f"{side}_knee": 110}),
            (
                f"{side} step back",
                {f"{side}_hip_pitch": -30, f"{side}_knee": 60, f"{side}_ankle_pitch": -30},
            ),
            (
                f"{side} arm forward, elbow bent",
                {f"{side}_shoulder_pitch": 90, f"{side}_elbow": 135},
            ),
            (f"{side} arm up", {f"{side}_shoulder_pitch": 180, f"{side}_shoulder_roll": 10}),
            (
                f"{side} arm out, yawed",
                {f"{side}_shoulder_roll": 90, f"{side}_shoulder_yaw": 90, f"{side}_elbow": 90},
            ),
            (
                f"{side} hand to chest",
                {f"{side}_shoulder_pitch": 60, f"{side}_shoulder_yaw": -90, f"{side}_elbow": 120},
            ),
        ]
    out += [
        ("look down, turned", {"neck_pitch": 40, "neck_yaw": 60}),
        ("look up, turned", {"neck_pitch": -20, "neck_yaw": -60}),
        ("waist turned, arms down", {"waist_yaw": 90}),
    ]
    return out


def _rot(axis, q: float) -> np.ndarray:
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def check(min_cm3: float = 0.5) -> list[str]:
    """Shell-to-part contacts (overlap above min_cm3) over the poses, worst per pair."""
    sys.path.insert(0, str(ROOT / "tools"))
    import build_urdf as bu

    robot = yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())
    skel = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())
    joints = bu.mirrored(skel["joints"], key_is_name=True)
    ranges = {j["name"]: j["range"] for j in bu.expand(robot["joints"])}
    body: dict[str, list] = {}
    for p in leg.all_parts() + torso.parts() + arm.all_parts() + head.parts():
        body.setdefault(p.link, []).append(p.solid)
    for m in bu.motor_placements():
        d, _, t = (x * 1000 for x in m["envelope"])
        ax = "xyz"[int(np.argmax(np.abs(m["axis"])))]
        body.setdefault(m["link"], []).append(cyl(d, t, ax, [x * 1000 for x in m["xyz"]]))
    for b in bu.mirrored(skel["bodies"], key_is_name=False):
        if b["budget"] in ("compute", "battery", "keep_alive", "electronics"):
            body[b["link"]].append(box([x * 1000 for x in b["size"]], [x * 1000 for x in b["xyz"]]))
    shells: dict[str, list] = {}
    for p in all_parts():
        shells.setdefault(p.link, []).append(p.solid)
    body_u = {k: hull_free_union(v) for k, v in body.items()}
    shell_u = {k: hull_free_union(v) for k, v in shells.items()}
    worst: dict[tuple, tuple] = {}
    for label, q in poses(joints, ranges):
        world = {"pelvis": (np.eye(3), np.zeros(3))}
        pending = dict(joints)
        while pending:
            for name, jt in list(pending.items()):
                if jt["parent"] in world:
                    r, t = world[jt["parent"]]
                    rj = r @ _rot(jt["axis"], np.radians(q.get(name, 0.0)))
                    world[jt["child"]] = (rj, t + r @ (np.array(jt["xyz"]) * 1000))
                    del pending[name]

        def placed(m, link, world=world):
            r, t = world[link]
            return m.transform(np.hstack([r, t[:, None]]).tolist())

        cache = {}
        for a, sh in shell_u.items():
            sa = placed(sh, a)
            ba = np.array(sa.bounding_box())
            for b in world:
                if b == a:
                    continue
                ga, gb = _group(a), _group(b)
                if ga != "body" and gb != "body" and ga != gb:
                    continue
                others = [x for x in (body_u.get(b), shell_u.get(b)) if x is not None]
                for kind, other in zip(("part", "shell"), others, strict=False):
                    key = (b, kind)
                    if key not in cache:
                        cache[key] = placed(other, b)
                    ob = np.array(cache[key].bounding_box())
                    if np.any(ba[3:] < ob[:3]) or np.any(ob[3:] < ba[:3]):
                        continue
                    v = (sa ^ cache[key]).volume() / 1000
                    if v > min_cm3 and v > worst.get((a, b), (0, ""))[0]:
                        worst[(a, b)] = (v, label)
    return [f"shell on {a:24} x {b:24} {v:6.1f} cm³ at {lbl}" for (a, b), (v, lbl) in worst.items()]


def hull_free_union(solids):
    from .lib import union

    return union(*solids)


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="clearance check of the clip-on shells")
    ap.add_argument("--check", action="store_true")
    ap.parse_args()
    hits = check()
    for h in hits:
        print(h)
    print(f"{len(hits)} shell contacts")
    sys.exit(1 if hits else 0)
