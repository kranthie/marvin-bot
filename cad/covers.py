"""Clip-on body shells: an optional add-on over the bare structure.

The robot is designed to look finished without them (graphite printed shells, aluminium
plates, the actuators' own housings). The shells give it a finished body: a smooth light
torso tapering to the waist, a dark hip girdle, long leg shells, arm sleeves, toe caps and a
dark neck sleeve under the helmet head, with the joints left visible between them. They add
some fall protection.

Each shell is a 1.2 mm printed skin following its link's parts and motors at a 4 mm gap: the
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
from .lib import Part, box, cyl, hull, union

ROOT = Path(__file__).resolve().parent.parent
GAP, SKIN = 4.0, 1.2


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

    return [motor_solid(m) for m in build_urdf.motor_placements() if m["link"] == link]


def motor_solid(m: dict):
    """A placed motor's housing (tools/build_urdf.py motor_primitives) as one solid, mm."""
    sys.path.insert(0, str(ROOT / "tools"))
    import build_urdf

    solids = []
    for p in build_urdf.motor_primitives(m):
        c = [x * 1000 for x in p["centre"]]
        if p["kind"] == "box":
            solids.append(box([x * 1000 for x in p["size"]], c))
        else:
            ax = "xyz"[int(np.argmax(np.abs(p["axis"])))]
            solids.append(cyl(p["d"] * 1000, p["length"] * 1000, ax, c))
    return union(*solids)


def offset_hull(solids, r: float):
    """Hull of `solids` grown by about r in every direction."""
    h = hull(*solids)
    return hull(*[h.translate(list(r * d)) for d in _DIRS])


def lofted(solids, gap: float, corner: float, step: float = 10.0, neck=None, slope: float = 0.7):
    """A smooth body round `solids` that follows them slice by slice: at each height, a
    rectangle with rounded corners (radius `corner`) containing everything in that slice plus
    `gap`, widened only as far as needed for the slices above and below to blend at `slope`
    (mm across per mm up), each joined to the next by their hull; closed at the top by a
    smaller section so the shoulders round over. `neck` = (half depth, half width, height)
    adds a small top section that the shoulders slope up to."""
    h = union(*solids)
    lo, hi = h.bounding_box()[2], h.bounding_box()[5]

    def section(x0, y0, x1, y1, zc, r):
        r = min(r, (x1 - x0) / 2 - 1, (y1 - y0) / 2 - 1)
        return hull(
            *[cyl(2 * r, 1.0, "z", (x, y, zc)) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)]
        )

    pull = gap + corner * (1 - 0.5**0.5)  # the rounded corner still contains the slice's
    zs, rects = [], []
    for z in np.arange(lo + step / 2, hi, step):
        sl = h ^ box((BIG, BIG, step), (0, 0, z))
        if sl.is_empty():
            continue
        b = sl.bounding_box()
        zs.append(z)
        rects.append(np.array([b[0] - pull, b[1] - pull, b[3] + pull, b[4] + pull]))
    zs, rects = np.array(zs), np.array(rects)
    smooth = rects.copy()
    for i, z in enumerate(zs):
        d = slope * np.abs(zs - z)
        smooth[i, :2] = np.min(rects[:, :2] + d[:, None], axis=0)
        smooth[i, 2:] = np.max(rects[:, 2:] - d[:, None], axis=0)
    chain = [section(*smooth[i], z, corner) for i, z in enumerate(zs)]
    x0, y0, x1, y1 = smooth[-1]
    inset = 0.6 * corner
    cap = [chain[-1], section(x0 + inset, y0 + inset, x1 - inset, y1 - inset, hi + gap, corner)]
    if neck is not None:
        hx, hy, nz = neck
        cap.append(section(-hx, -hy, hx, hy, nz, corner))
    bottom = section(*smooth[0], lo - gap, corner)
    return union(
        hull(bottom, chain[0]),
        *[hull(a, b) for a, b in zip(chain, chain[1:], strict=False)],
        hull(*cap),
    )


def wrap(solids, keep, cut=(), extra: float = 0.0):
    """A cover skin around `solids`, kept inside the `keep` region, minus the `cut` regions;
    `extra` mm more gap gives a fuller, rounder shape over boxy parts."""
    g = GAP + extra
    skin = offset_hull(solids, g + SKIN) - offset_hull(solids, g)
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
    # the shell covers the inner part of the shoulder pitch motors: broad shoulders over a
    # narrower chest and waist; the motors' outer ends pass through the side walls
    tor = on("torso_link")
    sh_y = j["left_shoulder_pitch"][1] - 8  # inside the shoulder pitch output flange
    sh = j["left_shoulder_pitch"]
    neck_z = j["neck_yaw"][2]
    body = (
        box((BIG, BIG, BIG), (0, 0, 0))
        - half("z", -1, 26.0)
        - (cyl(108, BIG, "y", (sh[0], 0, sh[2])) - box((BIG, 2 * 110, BIG)))  # shoulder motors
        - cyl(100, 200, "z", (0, 0, neck_z))  # the neck opening
    )
    # the body follows its contents; the shoulders slope up into a collar round the neck
    collar = (60.0, 70.0, neck_z + 4)
    # flat side walls short of the shoulder flanges close the sides
    outer_b = lofted(tor, GAP + SKIN, 24.0, neck=(collar[0] + SKIN, collar[1] + SKIN, collar[2]))
    inner_b = lofted(tor, GAP, 24.0 - SKIN, neck=(collar[0], collar[1], collar[2] - SKIN))
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
    bot = -22.0  # just above the torso bridge; flaring towards the collar

    # oval at the top: wide side to side for the head frame's plates, narrow front to back so
    # the chin clears it looking down
    def oval(w, d, z):
        return cyl(w, 2, "z").scale([d / w, 1.0, 1.0]).translate([0, 0, z])

    sleeve = hull(cyl(92, 2, "z", (0, 0, bot)), oval(84, 60, nz + 14)) - hull(
        cyl(88, 2, "z", (0, 0, bot - 2)), oval(80, 56, nz + 16)
    )
    add("neck_cover", "neck_link", sleeve)

    # --- hip girdle (dark): round the pelvis frame and the waist motor, between the hips
    # (the hip pitch motors swing round the roll axes on either side of it)
    pel = [m_ for m_ in on("pelvis") if abs((m_.bounding_box()[1] + m_.bounding_box()[4]) / 2) < 50]
    waist_top = j["waist_yaw"][2] + 18.0  # below the torso shell's bottom edge
    centre = half("y", 1, -28) ^ half("y", -1, 28)
    add(
        "pelvis_cover",
        "pelvis",
        wrap(pel, centre ^ half("z", -1, waist_top) ^ half("z", 1, -75)),
    )

    # --- thigh shell: all round the thigh from under the hip yaw motor to the knee, except a
    # strip at the back of the inner side (harness); over the knee motor only in front
    knee_z = j["left_knee"][2]
    yaw_out = -t03 / 2
    thigh = on("left_thigh_link")
    upper = half("z", -1, yaw_out - 5) ^ half("z", 1, knee_z + d04 / 2 + 10)
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

    # (no shell over the hip roll motors behind the hips: they are left bare)
    # --- upper thigh: round the hip pitch yoke and the yaw motor, between the pitch motor
    # (which the yoke turns round) and the yaw output, where the thigh shell starts
    yaw = j["left_hip_yaw"]
    add(
        "left_upper_thigh_cover",
        "left_hip_pitch_link",
        wrap(
            on("left_hip_pitch_link"),
            half("z", -1, -d04 / 2 - 8)
            ^ half("z", 1, yaw[2] - t03 / 2 + 2)
            ^ half("x", 1, -45)  # the hip roll motor is behind it when the hip extends
            ^ half("y", 1, -30),  # and the pelvis inboard of it when the hip flexes
        ),
    )
    # --- shoulder cap: over the top of the shoulder roll motor. The roll link's parts swing
    # round the roll axis from straight down to 30° above horizontal, so the cap keeps above a
    # plane through that axis at 40°, and behind the roll output face
    ro = j["left_shoulder_roll"]
    up = box((BIG, BIG, BIG), (0, 0, BIG / 2)).rotate([45, 0, 0]).translate([0, ro[1], ro[2]])
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
    band = half("z", -1, -t06 / 2 - 8) ^ half("z", 1, el[2] + d06 / 2 + 6)
    # and down the back of the elbow motor: the forearm swings forward, never back
    elbow_back = half("z", -1, -t06 / 2 - 8) ^ half("z", 1, el[2]) ^ half("x", -1, -24)
    add(
        "left_upper_arm_cover",
        "left_upper_arm_link",
        wrap(on("left_upper_arm_link"), band + elbow_back),
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
    return "pelvis" if link == "pelvis" else "upper"


def _checked(a: str, b: str) -> bool:
    """Pairs that must never touch: within a limb, a limb and the body it hangs from (legs from
    the pelvis, arms from the upper body), the pelvis and the upper body. A hand against a hip
    or a leg against the other leg is the controller's business, as in check_packaging.py."""
    ga, gb = _group(a), _group(b)
    if ga == gb or {ga, gb} == {"pelvis", "upper"}:
        return True
    hangs = {"leg": "pelvis", "arm": "upper"}
    return any(
        x.endswith(("leg", "arm")) and hangs[x.split("_")[1]] == y for x, y in ((ga, gb), (gb, ga))
    )


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
        for ap in (-40, 40):  # the ankle's corners, where the linkage is most stretched
            for ar in (-20, 20):
                out.append(
                    (
                        f"{side} ankle {ap:+d}/{ar:+d}",
                        {f"{side}_ankle_pitch": ap, f"{side}_ankle_roll": ar},
                    )
                )
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


def check(min_cm3: float = 0.5, parts_too: bool = True) -> list[str]:
    """Shell-to-part contacts (overlap above min_cm3) over the poses, worst per pair; with
    `parts_too`, also part-to-part contacts between links beyond the overlap they already
    have at rest (where parts bolt to the next link's motor)."""
    sys.path.insert(0, str(ROOT / "tools"))
    import build_urdf as bu

    robot = yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())
    skel = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())
    joints = bu.mirrored(skel["joints"], key_is_name=True)
    ranges = {j["name"]: j["range"] for j in bu.expand(robot["joints"])}
    body: dict[str, list] = {}
    for p in leg.all_parts() + torso.parts() + arm.all_parts() + head.parts():
        if "ankle_rod" in p.name or "ankle_crank" in p.name:
            continue  # drawn at the neutral pose only; they move with the ankle
        body.setdefault(p.link, []).append(p.solid)
    for m in bu.motor_placements():
        body.setdefault(m["link"], []).append(motor_solid(m))
    for b in bu.mirrored(skel["bodies"], key_is_name=False):
        if b["budget"] in ("compute", "battery", "keep_alive", "electronics"):
            body[b["link"]].append(box([x * 1000 for x in b["size"]], [x * 1000 for x in b["xyz"]]))
    shells: dict[str, list] = {}
    for p in all_parts():
        shells.setdefault(p.link, []).append(p.solid)
    body_u = {k: hull_free_union(v) for k, v in body.items()}
    shell_u = {k: hull_free_union(v) for k, v in shells.items()}
    worst: dict[tuple, tuple] = {}
    rest: dict[tuple, float] = {}
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
        for side in ("left", "right"):
            pq = np.radians(q.get(f"{side}_ankle_pitch", 0.0))
            rq = np.radians(q.get(f"{side}_ankle_roll", 0.0))
            for name, solid in ankle_linkage_solids(side, pq, rq):
                lw = placed(solid, f"{side}_shin_link")
                lb = np.array(lw.bounding_box())
                for b in (
                    f"{side}_shin_link",
                    f"{side}_foot_link",
                    f"{side}_thigh_link",
                    f"{side}_ankle_link",
                ):
                    if b not in body_u:
                        continue
                    other = placed(body_u[b], b)
                    ob = np.array(other.bounding_box())
                    if np.any(lb[3:] < ob[:3]) or np.any(ob[3:] < lb[:3]):
                        continue
                    v = (lw ^ other).volume() / 1000
                    key = (f"{side}_{name}", b, "part")
                    if label == "zero":
                        rest[key] = v
                    v -= rest.get(key, 0.0)
                    if v > min_cm3 and v > worst.get(key, (0, ""))[0]:
                        worst[key] = (v, label)
        if parts_too:
            placed_b = {k: placed(v, k) for k, v in body_u.items()}
            for a in body_u:
                ba = np.array(placed_b[a].bounding_box())
                for b in body_u:
                    if b <= a or not _checked(a, b):
                        continue
                    ob = np.array(placed_b[b].bounding_box())
                    if np.any(ba[3:] < ob[:3]) or np.any(ob[3:] < ba[:3]):
                        continue
                    v = (placed_b[a] ^ placed_b[b]).volume() / 1000
                    if label == "zero":
                        rest[(a, b)] = v
                    v -= rest.get((a, b), 0.0)
                    if v > min_cm3 and v > worst.get((a, b, "part"), (0, ""))[0]:
                        worst[(a, b, "part")] = (v, label)
        for a, sh in shell_u.items():
            sa = placed(sh, a)
            ba = np.array(sa.bounding_box())
            for b in world:
                if b == a:
                    continue
                if not _checked(a, b):
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
    return [
        f"{'part' if len(k) == 3 else 'shell'} on {k[0]:24} x {k[1]:24} {v:6.1f} cm³ at {lbl}"
        for k, (v, lbl) in worst.items()
    ]


def ankle_linkage_solids(side: str, pitch: float, roll: float):
    """The two cranks and push rods of one ankle at a joint pose (radians), solved by
    tools/ankle_linkage.py, as solids in the shin frame (mm): [(name, solid)]."""
    sys.path.insert(0, str(ROOT / "tools"))
    import ankle_linkage as al

    skel = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())
    lk = al.Linkage(skel["ankle_linkage"]["geometry"])
    az = skel["joints"]["left_ankle_pitch"]["xyz"][2] * 1000  # ankle axis in the shin frame
    sgn = 1 if side == "left" else -1
    out = []
    for i, name in enumerate(("a", "b")):
        th = lk.crank(i, pitch, roll, lk.theta0)
        if th is None:
            continue
        m = lk.motor[i] * 1000
        tip = lk.tip(i, th) * 1000
        foot = lk.foot_point(i, pitch, roll) * 1000
        pts = [np.array([p_[0], sgn * p_[1], p_[2] + az]) for p_ in (m, tip, foot)]
        crank = hull(cyl(36, 6, "y", pts[0]), cyl(18, 6, "y", pts[1]))
        d = pts[2] - pts[1]
        rod = hull(*[box((8, 8, 8), pts[1] + d * f) for f in (0.08, 0.92)])  # clear of the ends
        out += [(f"crank_{name}", crank), (f"rod_{name}", rod)]
    return out


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
