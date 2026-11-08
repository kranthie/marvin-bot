"""Lite's wheeled base: Marvin's upper body (waist motor and up) on a bought standing-desk
column, on a round omnidirectional base. Design pass 1: not built or tested.

Base frame (mm): origin on the floor at the base's centre, x forward, z up. The column stands at
the centre; four 152 mm double omni wheels sit at 45, 135, 225 and 315 degrees (an X), each
driven directly by an RS00 (description/actuators.yaml).

Skeleton: a 6 mm floor plate (a flat plate past the mill's travel, cut by an online laser or
water-jet service) and a 4 mm deck plate, held apart by eight aluminium standoffs and clamped
together through the column: the column's foot bolts to the floor plate through a foot plate,
and a collar on the deck plate holds the column 130 mm higher, so its bending moment reaches
the base as a couple between the two plates rather than through the floor plate alone.

Wheel pods: each wheel turns between two plates, driven at its inner face by the RS00's output
through a hub, and carried at its outer face by a 6901 bearing on a stub axle, so the motor's
own bearing carries the torque and little of the wheel's load (the arm's yoke idea). The motor
bolts to the inner plate by its front ears. The pod is a trailing arm: it pivots on a pin in a
clevis block on the floor plate at one end, and a compression spring on a guide post presses
its other end down, so all four wheels stay loaded on an uneven floor (about +-8 mm at the
wheel). A urethane bump stop limits upward travel, a collar on the post the downward travel.

Shell: printed in eight 45 degree segments (each fits a 250 mm printer), a 2.5 mm skin over the
skeleton with no load on it. Each segment snaps onto the floor plate's edge with two cantilever
hooks (released from below with a flat screwdriver) and locates on the deck plate with two pins
in push-in rubber grommets; neighbours overlap on a lap strip. A TPU bumper (eight arcs) keys
into a groove low on the skirt, a diffuser strip into a groove under the dome's shoulder (an LED
strip behind it), and a 1 mm polycarbonate window, bent to the skirt, sits in a groove across
the front for the lidar. A two-piece dark deck cover goes round the column.

usage: python -m cad.lite_base [--check] [--bom]
"""

from __future__ import annotations

import math
import sys

import build123d as bd
import manifold3d as mf
import numpy as np

from . import b3d
from .b3d import circle_pts, circles, cut, disc, lighten, made, rect, rounded
from .lib import Part, box, cyl, hull, union
from .mounts import CLEAR, HEAD, front_ring, spec

LINK = "lite_base"

# ---- layout (mm) ----
R_SHELL = 275.0  # shell outside radius: 0.55 m across
Z_SHELL = (22.0, 185.0)  # shell underside, top
DOME = 55.0  # the dome's shoulder radius
SKIN = 2.5
WHEEL_D, WHEEL_W = 152.0, 29.0  # 6-inch double omni wheel (40 kg rating, reseller figure)
RC = 233.0  # wheel centre from the base centre
ZW = WHEEL_D / 2  # wheel axle height
POD_ANG = (45.0, 135.0, 225.0, 315.0)
CORNERS = {45: "front_left", 135: "rear_left", 225: "rear_right", 315: "front_right"}
SPINNING = ("pod_hub", "pod_axle", "pod_wheel")  # the parts that turn with each wheel
FLOOR_Z, FLOOR_T, FLOOR_R = 30.0, 6.0, 262.0
DECK_Z, DECK_T, DECK_R = 162.0, 4.0, 245.0
POST_D = 16.0
PLATE_T = 6.0
# the column: a LINAK DL6-size three-stage desk column, 80 x 50 mm base stage (80 fore-aft)
# the DL6's outer profile (x, y): its 80 mm side across the robot, so its motor housing (along
# the 50 mm side; cad/lite_column.py) overhangs to the back, under the torso
COL = (50.0, 80.0)
COL_OUTER_LEN = 498.0  # the outer profile's length (data sheet)
# its bottom plate's tapped holes (data sheet): 4 x M6 on a 34 x 50 rectangle, 2 x M8 58 apart,
# both along the 80 mm side
COL_M6 = [(sx * 17.0, sy * 25.0) for sx in (1, -1) for sy in (1, -1)]
COL_M8 = [(0.0, 29.0), (0.0, -29.0)]
COL_RETRACTED, COL_STROKE = 560.0, 450.0  # DL6: 560 retracted, 650 stroke, used to 450 (overlap)
FOOT_T = 18.0  # the column bolts straight to it (counterbored from below)
TOP = FLOOR_Z + FLOOR_T + FOOT_T + COL_RETRACTED  # the column top, retracted

# pod frame: the wheel's axle along +x (radially out), u = y (the rolling direction), z up
X_IN = RC - WHEEL_W / 2 - 8.5  # the inner plate's outer face (the hub between it and the wheel)
X_OUT = RC + WHEEL_W / 2 + 4.5  # the outer plate's inner face
PIVOT = (80.0, 62.0)  # (u, z) of the pod's pivot pin
SPRING = (-60.0, 100.0)  # (u, z) of the spring tab (a bracket on the inner plate's inner face)
# the pod spring (music wire), sized by spring_rates(): mean diameter, wire, active coils, free
# length (mm). Its pocket runs from the tab up to a cap under the deck.
COIL_D, COIL_WIRE, COIL_N, COIL_FREE = 13.0, 1.8, 11.6, 60.0
BUMP_H = 32.0  # the urethane bump stop inside it: it meets the cap when the tab has risen 14 mm
TAB_X = 16.0  # the spring post, inboard of the inner plate's inner face
CORNER = 64.0  # the two spacers joining the plates, at (+-CORNER, ZW + 58)


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array(
        [[math.cos(a), -math.sin(a), 0, 0], [math.sin(a), math.cos(a), 0, 0], [0, 0, 1, 0]]
    )


def turned(m: mf.Manifold, deg: float) -> mf.Manifold:
    return m.transform(rot_z(deg).tolist())


def brep_turned(shape, deg: float):
    return bd.Rot(0, 0, deg) * shape


def capsule(a, b, r: float) -> mf.Manifold:
    s = mf.Manifold.sphere(r, 16)
    return hull(s.translate(list(a)), s.translate(list(b)))


def revolve(profile, deg: float = 360.0, segments: int = 180) -> mf.Manifold:
    """A closed (r, z) profile turned about z, from +x anticlockwise through `deg`."""
    cs = mf.CrossSection([list(map(list, profile))], mf.FillRule.Positive)
    m = mf.Manifold.revolve(cs, max(8, int(segments * deg / 360)), deg)
    return m


def arc(cx, cz, r, a0, a1, n=12):
    return [(cx + r * math.cos(a), cz + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


# ---- bought parts, drawn simply ----
def omni_wheel() -> mf.Manifold:
    """A double omni wheel in the pod frame: hub plates and two rows of rollers."""
    r, w = WHEEL_D / 2, WHEEL_W
    c = np.array([RC, 0.0, ZW])
    parts = [cyl(r * 1.55, 4, "x", c + [s * (w / 2 - 6), 0, 0]) for s in (1, -1)]
    parts.append(cyl(40, w, "x", c) - cyl(12.0, w + 2, "x", c))
    n, rr = 14, 9.0
    for row, s in enumerate((-1, 1)):
        for k in range(n):
            th = 2 * math.pi * (k + 0.5 * row) / n
            rad = np.array([0, math.sin(th), math.cos(th)])
            tan = np.array([0, math.cos(th), -math.sin(th)])
            p = c + rad * (r - rr) + [s * w * 0.24, 0, 0]
            half = 2 * math.pi * (r - rr) / n * 0.30
            parts.append(capsule(p - tan * half, p + tan * half, rr))
    return union(*parts) - cyl(12.0, w + 20, "x", c)


def rs00(centre) -> mf.Manifold:
    s = spec("RS00")
    x0 = centre[0] - s["t"] / 2
    body = cyl(s["body_d"], s["t"] - 1, "x", (x0 + (s["t"] - 1) / 2, centre[1], centre[2]))
    out = cyl(s["output_d"], 1, "x", (x0 + s["t"] - 0.5, centre[1], centre[2]))
    return union(body, out)


def motor_centre():
    s = spec("RS00")
    return (X_IN - PLATE_T - s["ear"], 0.0, ZW)


# ---- one wheel pod, in the pod frame ----
def pod_parts() -> list[tuple[str, object, str, str, dict]]:
    """(name, shape, material, process, extra): shapes are B-reps for made parts, manifolds
    for bought ones. Built for the pod at 0 degrees (its axle along +x)."""
    out = []
    mc = motor_centre()
    s = spec("RS00")
    # inner plate: the RS00's front ring (the plate is the ring), long enough to reach the
    # pivot and the spring tab, so the pod is one stiff arm between them
    pu, pz = PIVOT
    su, sz = SPRING
    outline = rounded(
        b3d.union(
            disc(70.0, (0.0, ZW)),
            b3d.slot(26.0, (pu, pz), (0.0, ZW - 8), (su, sz - 12)),
            b3d.slot(20.0, (-CORNER, ZW + 58), (0.0, ZW), (CORNER, ZW + 58)),
        ),
        inside=12.0,
    )
    # the plate's outline in its (u, v) = (y, z) plane; front_ring cuts the bore, the stator
    # holes and their counterbores, from the ear face to the plate's outer face
    ring, _ = front_ring("RS00", mc, "x", 1, t=PLATE_T, outline=outline, pocket=False)
    holes = b3d.prism(
        b3d.union(
            disc(8.2, (pu, pz)),  # the pivot pin's flanged bushing
            circles(CLEAR["M4"], [(su - 8, sz - 12), (su + 8, sz - 12)]),  # the spring tab
            circles(CLEAR["M5"], [(-CORNER, ZW + 58), (CORNER, ZW + 58)]),
        ),
        "x",
        X_IN - PLATE_T - 1,
        X_IN + 1,
    )
    out.append(("pod_inner", ring - holes, "al6061", "cnc", {}))
    # outer plate: round the 6901 bearing (24 mm OD), arms up to the two spacers
    bo = rounded(
        b3d.union(
            disc(44.0, (0.0, ZW)),
            b3d.slot(20.0, (-CORNER, ZW + 58), (0.0, ZW), (CORNER, ZW + 58)),
        ),
        inside=10.0,
    )
    bo = cut(
        bo, disc(24.0, (0.0, ZW)), circles(CLEAR["M5"], [(-CORNER, ZW + 58), (CORNER, ZW + 58)])
    )
    outer = b3d.plate(bo, "x", X_OUT + PLATE_T / 2, PLATE_T)
    # the bearing's shoulder: a 20 mm hole through the last millimetre
    out.append(("pod_outer", outer, "al6061", "cnc", {}))
    # two spacers between the plates (M5 through bolts)
    for k, u in enumerate((-CORNER, CORNER)):
        sp = b3d.prism(
            b3d.cut(disc(12.0, (u, ZW + 58)), disc(CLEAR["M5"], (u, ZW + 58))), "x", X_IN, X_OUT
        )
        out.append((f"pod_spacer_{k}", sp, "al6061", "cnc", {}))
    # hub: a spigot through the plate's bore onto the output flange, a flange the wheel bolts to
    out_f = mc[0] + s["t"] / 2
    hub_sk = cut(
        disc(s["output_d"], (0, ZW)),
        circles(CLEAR["M3"], circle_pts(6, 27.0, (0, ZW))),
        disc(12.0, (0, ZW)),
    )
    hub = b3d.prism(hub_sk, "x", out_f, X_IN + 0.5)
    fl = cut(
        disc(64.0, (0, ZW)),
        disc(12.0, (0, ZW)),
        circles(CLEAR["M4"], circle_pts(4, 48.0, (0, ZW), phase=math.pi / 4)),
    )
    hub = hub + b3d.prism(fl, "x", X_IN + 0.5, RC - WHEEL_W / 2)
    out.append(("pod_hub", hub, "al7075", "cnc", {}))
    # the stub axle: a 12 mm steel pin pressed into the hub, through the wheel to the bearing
    axle = cyl(12.0, X_OUT + PLATE_T - (out_f + 2), "x", ((out_f + 2 + X_OUT + PLATE_T) / 2, 0, ZW))
    out.append(("pod_axle", axle, "steel", "bought", {"mass_g": None}))
    out.append(
        (
            "pod_bearing",
            cyl(24, 6, "x", (X_OUT + 3, 0, ZW)) - cyl(12, 7, "x", (X_OUT + 3, 0, ZW)),
            "bought",
            "bought",
            {"mass_g": 11.0},
        )
    )
    out.append(("pod_wheel", omni_wheel(), "bought", "bought", {"mass_g": 630.0}))
    out.append(("pod_motor", rs00(mc), "bought", "bought", {"mass_g": 310.0}))
    return out


def pod_fixed_parts() -> list[tuple[str, object, str, str, dict]]:
    """The parts that hold a pod: the clevis block on the floor plate and the pivot pin (single
    shear: a shoulder bolt through the block and the inner plate's bushing), the spring tab on
    the inner plate, and the guide post with its spring, bump stop and collar."""
    out = []
    pu, pz = PIVOT
    su, sz = SPRING
    xi = X_IN - PLATE_T  # the inner plate's inner face
    c0, c1 = xi - 1 - 18, xi - 1
    blk = b3d.prism(rect(pu - 14, pu + 14, FLOOR_Z + FLOOR_T, pz + 13, r=4.0), "x", c0, c1)
    blk = blk - b3d.prism(disc(6.2, (pu, pz)), "x", c0 - 1, c1 + 1)
    out.append(("pod_clevis", blk, "al6061", "cnc", {}))
    out.append(
        (
            "pod_pivot_pin",
            cyl(6, X_IN + 4 - c0, "x", ((c0 + X_IN + 4) / 2, pu, pz)),
            "steel",
            "bought",
            {"mass_g": 14.0},
        )
    )
    # spring tab: an L, its leg bolted to the plate's inner face, its flange under the spring
    xp = xi - TAB_X
    leg = b3d.prism(rect(su - 15, su + 15, sz - 22, sz + 3, r=3.0), "x", xi - 5, xi)
    fl = b3d.prism(
        cut(rect(xp - 12, xi - 2, su - 15, su + 15, r=3.0), disc(8.4, (xp, su))),
        "z",
        sz - 3,
        sz + 3,
    )
    out.append(("pod_spring_tab", leg + fl, "al6061", "cnc", {}))
    zf = FLOOR_Z + FLOOR_T
    out.append(
        (
            "pod_post",
            cyl(6, DECK_Z - 8 - zf, "z", (xp, su, (zf + DECK_Z - 8) / 2)),
            "steel",
            "bought",
            {"mass_g": 25.0},
        )
    )
    # the post's base standoff, its top the down-stop under the tab (the tab moves 1.75x the
    # wheel: about +-14 mm)
    zc = sz - 3 - 15
    out.append(
        (
            "pod_post_base",
            cyl(14, zc - zf, "z", (xp, su, (zf + zc) / 2)) - cyl(6.5, 400, "z", (xp, su, 100)),
            "al6061",
            "cnc",
            {},
        )
    )
    # spring above the tab, pressing the pod down; urethane bump stop on the tab inside it
    z0, z1 = sz + 3, DECK_Z - 13
    coil = []
    turns, n, rc, hw = COIL_N + 2, 240, COIL_D / 2, COIL_WIRE / 2
    for i in range(n):
        a0, a1 = 2 * math.pi * turns * i / n, 2 * math.pi * turns * (i + 1) / n
        h0, h1 = (z0 + hw + (z1 - z0 - 2 * hw) * k / n for k in (i, i + 1))
        p0 = (xp + rc * math.cos(a0), su + rc * math.sin(a0), h0)
        p1 = (xp + rc * math.cos(a1), su + rc * math.sin(a1), h1)
        coil.append(capsule(p0, p1, hw))
    out.append(("pod_spring", union(*coil), "bought", "bought", {"mass_g": 15.0}))
    out.append(
        (
            "pod_bump_stop",
            cyl(10, BUMP_H, "z", (xp, su, z0 + BUMP_H / 2))
            - cyl(6.5, BUMP_H + 2, "z", (xp, su, z0 + BUMP_H / 2)),
            "bought",
            "bought",
            {"mass_g": 3.0},
        )
    )
    out.append(
        (
            "pod_spring_cap",
            cyl(20, 2, "z", (xp, su, z1 + 1)) - cyl(6.5, 3, "z", (xp, su, z1 + 1)),
            "steel",
            "bought",
            {"mass_g": 4.0},
        )
    )
    return out


def spring_rates(mass: float = 40.0) -> dict:
    """The pod spring against the pod's geometry: its rate at the tab and at the wheel (the tab is
    1.75x as far from the pivot as the wheel), the wheel load at mid travel and at the ends, the
    wire's shear stress at the bump stop and the spring's length there against its solid length.
    `mass`: Lite's (kg), shared by four wheels."""
    g_mod = 79.3e3  # MPa, music wire
    pu, sz = PIVOT[0], SPRING[1]
    ratio = (pu - SPRING[0]) / pu
    k = g_mod * COIL_WIRE**4 / (8 * COIL_D**3 * COIL_N)  # N/mm at the tab
    installed = (DECK_Z - 13) - (sz + 3)
    up = installed - BUMP_H  # tab travel to the bump stop
    down = 15.0  # to the post base (pod_parts)
    f_mid = k * (COIL_FREE - installed)
    c = COIL_D / COIL_WIRE
    wahl = (4 * c - 1) / (4 * c - 4) + 0.615 / c
    f_max = f_mid + k * up
    return dict(
        rate_tab=k,
        rate_wheel=k * ratio**2,
        wheel_mid=f_mid * ratio,
        wheel_bump=f_max * ratio,
        wheel_droop=max(0.0, f_mid - k * down) * ratio,
        share=mass * 9.81 / 4,
        travel_wheel=(up / ratio, down / ratio),
        stress=8 * f_max * COIL_D / (math.pi * COIL_WIRE**3) * wahl,
        length_bump=installed - up,
        solid=(COIL_N + 2) * COIL_WIRE,
    )


# ---- skeleton ----
def wheel_cutout(ang: float) -> bd.Sketch:
    """The floor plate's opening for a wheel and its travel (z: the wheel's chord at the plate).
    Open at the rim: a closed slot this long would leave the rim outside it as a loose sliver
    (its corners are outside the plate)."""
    c = np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang))])
    t = np.array([-c[1], c[0]])
    pts = []
    out = FLOOR_R - RC + 10
    for dx, du in (
        (-WHEEL_W / 2 - 8, -72),
        (out, -72),
        (out, 72),
        (-WHEEL_W / 2 - 8, 72),
    ):
        p = c * (RC + dx) + t * du
        pts.append((float(p[0]), float(p[1])))
    return rounded(bd.Sketch() + bd.Polygon(*pts, align=None), inside=0.1)


def pod_world(ang: float, u: float, x: float):
    """A pod-frame point (x radial, u along y) in the base frame's (x, y)."""
    a = math.radians(ang)
    return (x * math.cos(a) - u * math.sin(a), x * math.sin(a) + u * math.cos(a))


POSTS = [
    (245.0 * math.cos(math.radians(a)), 245.0 * math.sin(math.radians(a)))
    for a in (14, 78, 102, 166, 194, 258, 282, 346)
]
POSTS += [(125.0 * math.cos(math.radians(a)), 125.0 * math.sin(math.radians(a))) for a in POD_ANG]
# floor-edge hooks, two per segment; over a wheel, clear of the floor plate's open wheel slot
SNAPS = [
    a
    for k in range(8)
    for a in (
        (22.5 + 45 * k + 4, 22.5 + 45 * k + 41)
        if k % 2 == 0
        else (22.5 + 45 * k + 12, 22.5 + 45 * k + 33)
    )
]
WINDOWS = [(-45.0, -25.0), (-20.0, 20.0), (25.0, 45.0)]  # lidar openings (degrees)
GROMMETS = [a for k in range(8) for a in (22.5 + 45 * k + 15, 22.5 + 45 * k + 30)]  # deck pins


def skeleton_parts():
    out = []
    # floor plate
    fl = disc(2 * FLOOR_R)
    fl = cut(fl, *[wheel_cutout(a) for a in POD_ANG])
    holes = [circles(CLEAR["M6"] if "M6" in CLEAR else 6.6, POSTS)]
    for a in POD_ANG:  # clevis and post base screws
        holes.append(
            circles(CLEAR["M5"], [pod_world(a, PIVOT[0] + d, X_IN - PLATE_T - 10) for d in (-8, 8)])
        )
        holes.append(circles(6.6, [pod_world(a, SPRING[0], X_IN - PLATE_T - TAB_X)]))
    holes.append(
        circles(CLEAR["M5"], [(x, y) for x in (-80, -27, 27, 80) for y in (-60, 60)])
    )  # foot plate
    keep = [disc(30, p) for p in POSTS] + [rect(-95, 95, -75, 75)]
    keep += [disc(34, pod_world(a, PIVOT[0], X_IN - PLATE_T - 10)) for a in POD_ANG]
    keep += [disc(24, pod_world(a, SPRING[0], X_IN - PLATE_T - TAB_X)) for a in POD_ANG]
    keep += [wheel_cutout(a) for a in POD_ANG]
    # the shell's snap hooks catch the plate's rim: keep a solid band round it
    keep.append(cut(disc(2 * FLOOR_R), disc(2 * (FLOOR_R - 22))))
    fl_pk = lighten(fl, keep=keep, pitch=26, rim=8, rib=8)
    fl = cut(fl, *holes, fl_pk)
    out.append(
        ("floor_plate", b3d.plate(fl, "z", FLOOR_Z + FLOOR_T / 2, FLOOR_T), "al6061", "service")
    )
    # deck plate: column opening, the column collar's screws, grommet holes for the shell pins
    dk = disc(2 * DECK_R)
    op = rect(-COL[0] / 2 - 2, COL[0] / 2 + 2, -COL[1] / 2 - 2, COL[1] / 2 + 2, r=4.0)
    gr = [
        ((DECK_R - 9) * math.cos(math.radians(a)), (DECK_R - 9) * math.sin(math.radians(a)))
        for a in GROMMETS
    ]
    collar_pts = [(x, y) for x in (-38, 0, 38) for y in (-55, 55)]
    estop = (-200.0, 0.0)
    dk_holes = [
        op,
        circles(6.6, POSTS),
        circles(8.0, gr),
        circles(CLEAR["M4"], collar_pts),
        disc(22.5, estop),
    ]
    dkeep = [disc(30, p) for p in POSTS] + [rect(-72, 72, -50, 50), disc(40, estop)]
    dkeep.append(cut(disc(2 * DECK_R), disc(2 * (DECK_R - 20))))
    dk_pk = lighten(dk, keep=dkeep, pitch=26, rim=8, rib=7)
    dk = cut(dk, *dk_holes, dk_pk)
    out.append(("deck_plate", b3d.plate(dk, "z", DECK_Z + DECK_T / 2, DECK_T), "al6061", "service"))
    # standoffs: M6 tapped both ends
    for k, p in enumerate(POSTS):
        sk = cut(disc(POST_D, p), disc(5.0, p))
        out.append(
            (f"standoff_{k}", b3d.prism(sk, "z", FLOOR_Z + FLOOR_T, DECK_Z), "al6061", "cnc")
        )
    # column foot plate: the column's bottom plate (4 x M6, 2 x M8, screws counterbored from
    # below) to the floor plate (8 x M5)
    fp = rect(-95, 95, -72, 72, r=8.0)
    fp_holes = circles(CLEAR["M5"], [(x, y) for x in (-80, -27, 27, 80) for y in (-60, 60)])
    fp = cut(fp, fp_holes, circles(6.6, COL_M6), circles(8.6, COL_M8))
    fp = cut(
        fp,
        lighten(
            fp,
            keep=[rect(-62, 62, -42, 42)]
            + [disc(16, (x, y)) for x in (-80, -27, 27, 80) for y in (-60, 60)],
            pitch=18,
            rim=6,
            rib=6,
        ),
    )
    out.append(
        ("column_foot", b3d.plate(fp, "z", FLOOR_Z + FLOOR_T + FOOT_T / 2, FOOT_T), "al6061", "cnc")
    )
    # column collar on the deck: a 12 mm block round the column, split at the front so a screw
    # across it clamps it (printed TPU shims take up the column's tolerance)
    zc0 = DECK_Z + DECK_T
    cl = rect(-48, 48, -68, 68, r=10.0)
    cl = cut(
        cl,
        rect(-COL[0] / 2 - 1.5, COL[0] / 2 + 1.5, -COL[1] / 2 - 1.5, COL[1] / 2 + 1.5, r=3.0),
        rect(COL[0] / 2, 70, -1.0, 1.0, r=0.0),
        circles(CLEAR["M4"], collar_pts),
    )
    collar = b3d.prism(cl, "z", zc0, zc0 + 12)
    out.append(("column_collar", collar, "al6061", "cnc"))
    return out


# ---- shell ----
def shell_profile(inset: float = 0.0):
    """The shell's outer (inset 0) or inner (inset SKIN) surface as an (r, z) polyline, from the
    bottom edge up and over the dome to the deck cover's edge at r = 120."""
    z0, z1 = Z_SHELL
    R = R_SHELL - inset
    pts = [(R - 10, z0)] + arc(R - 10, z0 + 10, 10, -math.pi / 2, 0, 6)[1:]
    pts += arc(R_SHELL - DOME, z1 - DOME, DOME - inset, 0, math.pi / 2, 20)
    pts += [(120.0, z1 - inset)]
    return pts


def shell_solid() -> mf.Manifold:
    outer = shell_profile(0.0)
    inner = shell_profile(SKIN)
    prof = outer + inner[::-1]
    sh = revolve(prof)
    # the bumper groove (5 mm deep, 10 high) low on the skirt, the light groove under the dome
    zb = Z_SHELL[0] + 18
    sh = sh - revolve(
        [(R_SHELL - 1.5, zb), (R_SHELL + 1, zb), (R_SHELL + 1, zb + 10), (R_SHELL - 1.5, zb + 10)]
    )
    zl = Z_SHELL[1] - DOME - 6
    sh = sh - revolve(
        [(R_SHELL - 1.2, zl), (R_SHELL + 1, zl), (R_SHELL + 1, zl + 7), (R_SHELL - 1.2, zl + 7)]
    )
    # the lidar's view across the front: three openings (posts at the seams keep each segment's
    # skirt joined), in a 1 mm recess for the window strips
    zw = Z_SHELL[0] + 46
    rec = revolve(
        [(R_SHELL - 1, zw), (R_SHELL + 1, zw), (R_SHELL + 1, zw + 22), (R_SHELL - 1, zw + 22)], 90
    )
    sh = sh - turned(rec, -45)
    for a0, a1 in WINDOWS:
        op = revolve(
            [
                (R_SHELL - 4, zw + 2),
                (R_SHELL + 1, zw + 2),
                (R_SHELL + 1, zw + 20),
                (R_SHELL - 4, zw + 20),
            ],
            a1 - a0,
        )
        sh = sh - turned(op, a0)
    sh = sh - cyl(23, 80, "z", (-200, 0, Z_SHELL[1]))  # the e-stop
    for s_ in (1, -1):  # the charge contacts
        sh = sh - box((20, 20, 16), (-R_SHELL, s_ * 22, Z_SHELL[0] + 60))
    return sh


def wedge(a0: float, a1: float, r: float = 400.0) -> mf.Manifold:
    pts = [(0.0, 0.0)] + [
        (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
        for a in np.linspace(a0, a1, 12)
    ]
    cs = mf.CrossSection([pts], mf.FillRule.Positive)
    return mf.Manifold.extrude(cs, 400).translate([0, 0, -50])


def shell_segments() -> list[tuple[str, mf.Manifold]]:
    """Eight segments, the seams at 22.5 + 45k degrees; each with a lap strip on its
    anticlockwise edge (under the next segment), two snap hooks and two locating pins."""
    sh = shell_solid()
    inner = shell_profile(SKIN)
    out = []
    for k in range(8):
        a0 = -22.5 + 45 * k
        seg = sh ^ wedge(a0, a0 + 45)
        # lap strip: 10 mm wide, 2 mm thick, inside the skin past the seam
        lap_prof = [(r - 0.05, z) for r, z in inner]
        lap_in = [(r - 2.0, z) for r, z in inner]
        lap = revolve(lap_prof + lap_in[::-1]) ^ wedge(a0 + 45 - 0.5, a0 + 45 + 2.2)
        lap = lap - box((800, 800, 60), (0, 0, Z_SHELL[0] + 2))  # stops above the floor plate
        seg = seg + lap
        # snap hooks: a 12 mm wide arm hanging inside the skirt to below the floor plate, its
        # barb under the plate's edge
        for a in SNAPS:
            if (a - a0) % 360 >= 45:
                continue
            # (the arm 1 mm off the plate's edge; the barb reaches 3 mm under it, so the arm bends
            # 3 mm going on: 0.8 % strain in PETG at this length)
            arm_x = FLOOR_R + 1 + 1.5
            arm = box((3.0, 12, 42), (arm_x, 0, FLOOR_Z + 16))
            web_w = R_SHELL - SKIN + 0.5 - (arm_x + 1.5)
            web = box(
                (web_w, 12, 8), (arm_x + 1.5 + web_w / 2, 0, FLOOR_Z + 33)
            )  # joins arm and skin
            barb = hull(
                box((0.1, 12, 5), (arm_x - 1.5, 0, FLOOR_Z - 2.6)),
                box((3.0, 12, 0.1), (arm_x - 1.5 - 1.5, 0, FLOOR_Z - 0.3)),
            )
            seg = seg + turned(union(arm, web, barb.translate([-0.0, 0, 0])), a)
        # locating pins into rubber grommets in the deck plate
        for a in GROMMETS:
            if (a - a0) % 360 >= 45:
                continue
            r = DECK_R - 9
            top = Z_SHELL[1] - SKIN + 0.5
            rz = _inner_z(inner, r)
            zt = DECK_Z + DECK_T + 0.5
            pin = cyl(10, rz + 1 - zt, "z", (r, 0, (rz + 1 + zt) / 2))
            tip = cyl(5.5, 10, "z", (r, 0, zt - 5))
            del top
            seg = seg + turned(union(pin, tip), a)
        out.append((f"shell_{k}", seg))
    return out


def _inner_z(inner, r: float) -> float:
    """The shell's inner surface height above radius r (on the dome)."""
    pts = [(pr, pz) for pr, pz in inner if pz > Z_SHELL[0] + 20]
    pts.sort()
    rs, zs = zip(*pts, strict=True)
    return float(np.interp(r, rs, zs))


def trim_parts() -> list[Part]:
    out = []
    zb = Z_SHELL[0] + 18
    bump = revolve(
        [
            (R_SHELL - 1.0, zb + 0.5),
            (R_SHELL + 6, zb - 2),
            (R_SHELL + 8, zb + 5),
            (R_SHELL + 6, zb + 12),
            (R_SHELL - 1.0, zb + 9.5),
        ]
    )
    for k in range(8):
        a0 = -22.5 + 45 * k
        out.append(
            Part(
                f"bumper_{k}",
                LINK,
                "tpu",
                bump ^ wedge(a0 + 0.3, a0 + 44.7),
                "print",
                notes="TPU bumper arc, keyed into the shell's groove",
            )
        )
    zl = Z_SHELL[1] - DOME - 6
    diff = revolve(
        [
            (R_SHELL - 1.0, zl + 0.5),
            (R_SHELL + 0.6, zl + 0.5),
            (R_SHELL + 0.6, zl + 6.5),
            (R_SHELL - 1.0, zl + 6.5),
        ]
    )
    for k in range(8):
        a0 = -22.5 + 45 * k
        out.append(
            Part(
                f"light_diffuser_{k}",
                LINK,
                "petg",
                diff ^ wedge(a0 + 0.2, a0 + 44.8),
                "print",
                notes="clear PETG strip; the LED strip sits behind it",
            )
        )
    zw = Z_SHELL[0] + 46
    for k, (a0, a1) in enumerate(WINDOWS):
        b0, b1 = max(a0 - 2.0, -44.8), min(a1 + 2.0, 44.8)
        win = turned(
            revolve(
                [
                    (R_SHELL - 1.0, zw + 0.5),
                    (R_SHELL - 0.1, zw + 0.5),
                    (R_SHELL - 0.1, zw + 21.5),
                    (R_SHELL - 1.0, zw + 21.5),
                ],
                b1 - b0,
            ),
            b0,
        )
        out.append(
            Part(
                f"lidar_window_{k}",
                LINK,
                "bought",
                win,
                "bought",
                mass_g=4.0,
                notes="1 mm IR-clear polycarbonate strip, bent into the recess, bonded at its ends",
            )
        )
    # dark deck cover round the column, two halves
    zt = Z_SHELL[1] - 2.0
    cov = revolve([(0.0, zt - 3), (119.5, zt - 3), (119.5, zt + 0.5), (0.0, zt + 0.5)])
    cov = cov - box((COL[0] + 4, COL[1] + 4, 20), (0, 0, zt))
    cov = cov - cyl(23, 20, "z", (-200, 0, zt))  # (outside it: the e-stop is in the shell)
    for k, (name, sgn) in enumerate((("front", 1), ("back", -1))):
        half = cov ^ box((300, 300, 40), (sgn * 150, 0, zt))
        out.append(
            Part(
                f"deck_cover_{name}",
                LINK,
                "petg",
                half,
                "print",
                notes="two halves round the column: tongue and groove, snap to the collar",
            )
        )
        del k
    # column trim ring where the column leaves the deck cover
    ring = mf.Manifold.extrude(mf.CrossSection([_rr(COL[0] + 22, COL[1] + 22, 10)]), 10).translate(
        [0, 0, zt + 0.5]
    )
    ring = ring - box((COL[0] + 2, COL[1] + 2, 30), (0, 0, zt + 5))
    out.append(
        Part("column_trim", LINK, "petg", ring, "print", notes="two halves, snaps round the column")
    )
    return out


def _rr(dx: float, dy: float, r: float, n: int = 8):
    pts = []
    for cx, cy, a0 in (
        (dx / 2 - r, dy / 2 - r, 0),
        (-dx / 2 + r, dy / 2 - r, 90),
        (-dx / 2 + r, -dy / 2 + r, 180),
        (dx / 2 - r, -dy / 2 + r, 270),
    ):
        for a in np.radians(np.linspace(a0, a0 + 90, n)):
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


# ---- inside: battery, electronics, lidar, e-stop, column ----
def inside_parts() -> list[Part]:
    out = []
    zf = FLOOR_Z + FLOOR_T
    # the 13S3P battery (moved from the torso) as two bricks in series (7S3P and 6S3P, cells
    # standing, one BMS), in the side gaps between the pods: low and on the centre line
    for name, y in (("l", 1), ("r", -1)):
        bat = box((66, 150, 74), (0, y * 170, zf + 3 + 37))
        out.append(
            Part(
                f"battery_{name}",
                LINK,
                "bought",
                bat,
                "bought",
                mass_g=1700.0,
                notes="half the 13S3P 21700 pack (7S3P / 6S3P), housing; the BMS on the left",
            )
        )
        tray = box((74, 158, 3), (0, y * 170, zf + 1.5))
        tray = (
            tray
            + box((3, 158, 22), (-35.5, y * 170, zf + 11))
            + box((3, 158, 22), (35.5, y * 170, zf + 11))
        )
        out.append(
            Part(
                f"battery_tray_{name}",
                LINK,
                "pa_cf",
                tray,
                "print",
                notes="slots for two hook-and-loop straps",
            )
        )
    # power electronics behind the column, under the e-stop: distribution board with the main
    # fuse and contactor, 48-24 V converter, the column's H-bridge and encoder input
    out.append(
        Part(
            "elec_power",
            LINK,
            "bought",
            box((90, 100, 40), (-150, 0, zf + 3 + 20)),
            "bought",
            mass_g=450.0,
            notes="power distribution, contactor, DC-DC 48-24 V, column driver",
        )
    )
    # 2D lidar behind the front window
    lid = cyl(70, 40, "z", (210, 0, Z_SHELL[0] + 46 + 11 - 6)) + cyl(
        56, 8, "z", (210, 0, Z_SHELL[0] + 46 + 11 + 18)
    )
    out.append(
        Part(
            "lidar",
            LINK,
            "bought",
            lid,
            "bought",
            mass_g=190.0,
            notes="2D lidar, behind the window",
        )
    )
    lbr = box(
        (70, 70, Z_SHELL[0] + 46 + 11 - 26 - zf), (210, 0, (zf + Z_SHELL[0] + 46 + 11 - 26) / 2)
    )
    lbr = lbr - box((50, 50, 100), (210, 0, 60))
    out.append(Part("lidar_bracket", LINK, "pa_cf", lbr, "print"))
    # e-stop on the deck at the back, through the shell
    es = cyl(22, 30, "z", (-200, 0, DECK_Z + DECK_T + 15)) + cyl(
        40, 16, "z", (-200, 0, Z_SHELL[1] + 8)
    )
    out.append(
        Part("estop", LINK, "bought", es, "bought", mass_g=90.0, notes="22 mm e-stop, red mushroom")
    )
    # charge contacts at the back of the skirt
    for s in (1, -1):
        cc = box((6, 18, 14), (-R_SHELL + 1, s * 22, Z_SHELL[0] + 60))
        out.append(
            Part(
                f"charge_contact_{'l' if s > 0 else 'r'}", LINK, "bought", cc, "bought", mass_g=20.0
            )
        )
    return out


def column_solid(top: float, outer_only: bool = False) -> mf.Manifold:
    """The desk column's outer profile (bought), on the foot plate, the part that stays on the
    base (its moving profiles and motor housing are in cad/lite_column.py). `top` and
    `outer_only` are kept for the renders' callers."""
    zb = FLOOR_Z + FLOOR_T + FOOT_T
    outer = mf.Manifold.extrude(mf.CrossSection([_rr(*COL, 6)]), COL_OUTER_LEN)
    return outer.translate([0, 0, zb])


# ---- fasteners ----
def screw(p, axis: str, sign: int, d: float, length: float, head: str = "socket") -> mf.Manifold:
    """A screw with its head at p, its shank running `length` from p along -sign*axis."""
    v = np.zeros(3)
    v["xyz".index(axis)] = sign
    hd, hh = {
        "socket": (1.6 * d, d),
        "low": (HEAD.get(f"M{int(d)}", (1.8 * d, 0.6 * d))[0], 0.6 * d),
        "button": (1.9 * d, 0.55 * d),
    }[head]
    h = cyl(hd, hh, axis, np.asarray(p) + v * hh / 2)
    s = cyl(d * 0.9, length, axis, np.asarray(p) - v * length / 2)
    return union(h, s)


def fastener_parts() -> tuple[list[Part], list[tuple[str, int, str]]]:
    """Screws drawn where they go, and the fastener list: (item, count, where)."""
    bom: list[tuple[str, int, str]] = []
    out = []
    s = spec("RS00")
    pod = []
    # RS00 to the inner plate: 6 x M3 low-head from the outer face
    for u, z in circle_pts(6, s["holes"]["stator"]["pcd"], (0, ZW)):
        pod.append(screw((X_IN - HEAD["M3"][1], u, z), "x", 1, 3, 12, "low"))
    # hub to the output: 6 x M3 from the flange side
    for u, z in circle_pts(6, 27.0, (0, ZW)):
        pod.append(screw((X_IN + 0.5 + 3, u, z), "x", 1, 3, 14, "low"))
    # wheel to the hub: 4 x M4 through the wheel's hub plate (the bought wheel's own pattern)
    for u, z in circle_pts(4, 48.0, (0, ZW), phase=math.pi / 4):
        pod.append(screw((RC - WHEEL_W / 2 + 6, u, z), "x", 1, 4, 14, "button"))
    # spacers: 2 x M5 through bolts, nyloc nuts
    for u in (-CORNER, CORNER):
        pod.append(
            screw(
                (X_OUT + PLATE_T, u, ZW + 58),
                "x",
                1,
                5,
                X_OUT + PLATE_T - X_IN + PLATE_T + 6,
                "socket",
            )
        )
    # the axle's retaining screw at the bearing (M5 into the axle end, with a washer)
    pod.append(screw((X_OUT + PLATE_T + 1, 0, ZW), "x", 1, 5, 12, "button"))
    # clevis to the floor plate: 2 x M5 from below
    for d in (-8, 8):
        pod.append(screw((X_IN - PLATE_T - 10, PIVOT[0] + d, FLOOR_Z), "z", -1, 5, 16, "low"))
        pod.append(screw((X_IN + 0.0, SPRING[0] + d, SPRING[1] - 12), "x", 1, 4, 12, "button"))
    pod_m = union(*pod)
    for a in POD_ANG:
        out.append(
            Part(f"screws_pod_{int(a)}", LINK, "bought", turned(pod_m, a), "bought", mass_g=60.0)
        )
    bom += [
        ("M3 x 12 low-head (DIN 7984), RS00 to inner plate", 24, "pods"),
        ("M3 x 14 low-head, hub to RS00 output", 24, "pods"),
        ("M4 x 14 button head, wheel to hub", 16, "pods"),
        ("M5 x 60 socket head + nyloc nut, pod spacers", 8, "pods"),
        ("M5 x 12 button head + washer, axle retainer", 4, "pods"),
        ("M5 x 16 low-head, clevis to floor plate", 8, "floor"),
        ("M4 x 12 button head, spring tab to inner plate (tapped tab)", 8, "pods"),
        ("flanged bronze bushing 8 x 6 x 6, pivot", 4, "pods"),
        ("M6 shoulder bolt 6 x 30 + nyloc nut, pod pivot pin", 4, "pods"),
        ("6901-2RS bearing 12 x 24 x 6", 4, "pods"),
        ("12 mm steel pin x 50, stub axle (pressed in hub)", 4, "pods"),
        ("M6 stud x 110 + nyloc nut + washer, spring post", 4, "pods"),
        (
            f"compression spring, music wire {COIL_WIRE} mm, mean dia {COIL_D:.0f} mm, "
            f"{COIL_N} active coils, free {COIL_FREE:.0f} mm, ground ends (spring_rates())",
            4,
            "pods",
        ),
        ("urethane bump stop OD 11 x 8", 4, "pods"),
    ]
    sk = []
    for p in POSTS:  # standoffs: M6 from below and above
        sk.append(screw((p[0], p[1], FLOOR_Z), "z", -1, 6, 12, "button"))
        sk.append(screw((p[0], p[1], DECK_Z + DECK_T), "z", 1, 6, 12, "button"))
    for x in (-80, -27, 27, 80):  # foot plate to floor plate
        for y in (-60, 60):
            sk.append(screw((x, y, FLOOR_Z), "z", -1, 5, 16, "low"))
    zf8 = FLOOR_Z + FLOOR_T + 8  # the foot plate's counterbores, 8 deep from its underside
    for x, y in COL_M6:  # column to foot plate, from below
        sk.append(screw((x, y, zf8), "z", -1, 6, 20, "socket"))
    for x, y in COL_M8:
        sk.append(screw((x, y, zf8), "z", -1, 8, 20, "socket"))
    for x in (-38, 0, 38):  # collar to deck
        for y in (-55, 55):
            sk.append(screw((x, y, DECK_Z + DECK_T + 12), "z", 1, 4, 16, "socket"))
    sk.append(screw((38, 20, DECK_Z + DECK_T + 6), "y", 1, 4, 40, "socket"))  # collar clamp
    out.append(Part("screws_frame", LINK, "bought", union(*sk), "bought", mass_g=120.0))
    bom += [
        ("M6 x 12 button head, standoffs", 2 * len(POSTS), "frame"),
        ("M5 x 16 low-head, column foot plate to floor plate", 8, "frame"),
        ("M6 x 20 socket head, column to foot plate (from below)", 4, "frame"),
        ("M8 x 20 socket head, column to foot plate (from below)", 2, "frame"),
        ("M4 x 16 socket head, collar to deck", 6, "frame"),
        ("M4 x 30 socket head + nut, collar clamp", 1, "frame"),
        ("push-in rubber grommet, 8 mm panel hole", 16, "deck: shell pins"),
        ("hook-and-loop strap 25 mm, battery", 4, "battery trays"),
        ("snap-in cable-tie mounts (fir tree, 6.5 mm hole)", 12, "harness"),
    ]
    return out, bom


# ---- assembly ----
def parts() -> list[Part]:
    out: list[Part] = []
    for name, shape, mat, proc in skeleton_parts():
        out.append(made(name, LINK, mat, shape, proc))
    pod = pod_parts() + pod_fixed_parts()
    for a in POD_ANG:
        tag = int(a)
        for name, shape, mat, proc, extra in pod:
            if isinstance(shape, mf.Manifold):
                mass = extra.get("mass_g")
                p = Part(f"{name}_{tag}", LINK, mat, turned(shape, a), proc, mass_g=mass)
                b = np.array(shape.bounding_box())
            else:
                p = made(f"{name}_{tag}", LINK, mat, brep_turned(shape, a), proc)
                b = np.array(b3d.to_manifold(shape).bounding_box())
            p.extra["fit_mm"] = b[3:] - b[:3]  # the size to check, in the pod's own frame
            out.append(p)
    for name, seg in shell_segments():
        out.append(
            Part(name, LINK, "petg", seg, "print", notes="shell segment: snaps on, no tools")
        )
    out += trim_parts()
    out += inside_parts()
    out.append(
        Part(
            "desk_column",
            LINK,
            "bought",
            column_solid(TOP, outer_only=True),
            "bought",
            mass_g=2500.0,
            notes="LINAK DL6 three-stage desk column: its outer profile and bottom plate (the "
            "moving profiles, spindle and motor housing are in cad/lite_column.py; the "
            "data sheet's 7.5 kg split by estimate)",
        )
    )
    fs, _ = fastener_parts()
    out += fs
    return out


def link_parts() -> list[Part]:
    """parts(), with each wheel's spinning parts moved into its wheel link's frame (origin at
    the wheel's centre, x out along its axle: description/skeleton.yaml variants.lite), for the
    URDF; everything else stays in the base frame."""
    out = []
    for p in parts():
        stem, _, tag = p.name.rpartition("_")
        if stem not in SPINNING:
            out.append(p)
            continue
        a = float(tag)
        solid = p.solid.rotate([0, 0, -a]).translate([-RC, 0, -ZW])
        brep = None if p.brep is None else bd.Pos(-RC, 0, -ZW) * (bd.Rot(0, 0, -a) * p.brep)
        out.append(
            Part(
                p.name,
                f"wheel_{CORNERS[int(a)]}_link",
                p.material,
                solid,
                p.process,
                mass_g=p.mass_g,
                notes=p.notes,
                extra=p.extra,
                brep=brep,
            )
        )
    return out


def interferences(ps: list[Part], min_mm3: float = 12.0) -> list[str]:
    """Pairs of parts that overlap by more than `min_mm3` (bolted faces only touch; the
    revolved parts' facets leave a few mm3 where they nest)."""
    skip = ("screws_",)
    items = [p for p in ps if not p.name.startswith(skip)]
    bbs = [np.array(p.solid.bounding_box()) for p in items]
    bad = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = bbs[i], bbs[j]
            if np.any(a[3:] < b[:3]) or np.any(b[3:] < a[:3]):
                continue
            v = (items[i].solid ^ items[j].solid).volume()
            if v > min_mm3:
                bad.append(f"{items[i].name} x {items[j].name}: {v:.0f} mm3")
    return bad


def main() -> None:
    from .build import MILL_MM, PRINT_MM, fits  # noqa: PLC0415

    ps = parts()
    tot = sum(p.mass() for p in ps)
    groups: dict[str, float] = {}
    for p in ps:
        g = p.name.split("_")[0]
        groups[g] = groups.get(g, 0.0) + p.mass()
    print(f"{len(ps)} parts, {tot:.2f} kg")
    for g, m in sorted(groups.items(), key=lambda kv: -kv[1]):
        print(f"  {g:14} {m:6.2f} kg")
    canon = {}
    for name, shape, _mat, proc, _ in pod_parts() + pod_fixed_parts():
        if proc in ("cnc", "print"):
            canon[name] = b3d.to_manifold(shape) if not isinstance(shape, mf.Manifold) else shape
    for name, sol in canon.items():
        b = np.array(sol.bounding_box())
        if not fits(b[3:] - b[:3], MILL_MM):
            print(f"  does not fit the mill: {name} {np.round(b[3:] - b[:3])}")
    for p in ps:
        if p.name.rsplit("_", 1)[0] in canon:
            continue
        size = p.bounds()
        if p.process == "cnc" and not fits(size, MILL_MM):
            print(f"  does not fit the mill: {p.name} {np.round(size)}")
        if p.process == "print" and not fits(size, PRINT_MM):
            print(f"  does not fit the printer: {p.name} {np.round(size)}")
    if "--bom" in sys.argv:
        _, bom = fastener_parts()
        print("fasteners and small bought parts:")
        for item, n, where in bom:
            print(f"  {n:3d} x {item}  ({where})")
    if "--check" in sys.argv:
        bad = interferences(ps)
        print(f"{len(bad)} interferences")
        for b in bad:
            print("  " + b)
        r = spring_rates()
        print(
            f"pod spring: {r['rate_tab']:.2f} N/mm at the tab, {r['rate_wheel']:.1f} at the wheel;"
            f" wheel load {r['wheel_droop']:.0f} / {r['wheel_mid']:.0f} / {r['wheel_bump']:.0f} N"
            f" (droop / mid / bump; a wheel's share {r['share']:.0f} N) over"
            f" -{r['travel_wheel'][1]:.1f} / +{r['travel_wheel'][0]:.1f} mm;"
            f" {r['stress']:.0f} MPa at the bump; {r['length_bump']:.1f} mm long there,"
            f" solid {r['solid']:.1f}"
        )
        if r["length_bump"] < r["solid"] + 2 or r["stress"] > 0.45 * 1850:
            bad.append("pod spring")
        if bad:
            sys.exit(1)


if __name__ == "__main__":
    main()


# ---- strength check (cad/fea.py --variant lite) ----
def _with(p: Part, *ifaces: dict) -> Part:
    p.extra["ifaces"] = list(ifaces)
    return p


def _cyl_iface(joint: str, centre, axis, r: float, half: float) -> dict:
    return dict(
        joint=joint, kind="cyl", centre=tuple(centre), axis=tuple(axis), r1=r, span=(-half, half)
    )


def _face_iface(joint: str, centre, axis, r0: float, r1: float) -> dict:
    return dict(joint=joint, kind="face", centre=tuple(centre), axis=tuple(axis), r0=r0, r1=r1)


def fea_groups(mass: float, mass_above: float, palm) -> list[tuple[str, list, list]]:
    """(group, parts with their interfaces, load cases) for cad/fea.py. `mass`: Lite's (kg);
    `mass_above`: what the column carries (its moving stages and the upper body, kg); `palm`:
    the left palm's position in the column link's frame (mm), arms hanging.

    - pod: one wheel pod in its own frame (axle along x), held at its pivot bushing and its
      spring tab, loaded at the wheel through the RS00's ears and the outer bearing as one rigid
      body: a bump (3 x a wheel's share up), a sideways knock (half its share along the axle),
      and the RS00's peak torque reacted at the floor;
    - base: the floor and deck plates with the standoffs, column foot and collar, clevis blocks
      and post bases, held at the four pods' pins and spring posts, loaded at the column's foot
      and the collar: a 2 g kerb drop of everything on the column, and 1.5 x the largest moment
      the floor can put on Lite before it tips, fore-aft and sideways;
    - floor: the floor plate alone, held at the clevis blocks and post bases, with the base's
      cases at the column foot's bolts (all of the column's moment through the foot);
    - column_head: the head plate, cheeks, waist ring and cage, held where the plate bears on
      the column's motor housing and at the cheeks' M6 holes into its sides, loaded
      at the waist: the hand catch (Max's case: 0.5 x body weight up at the palm, 0.25 x fore-aft,
      0.15 x sideways) and the waist RS03's peak torque."""
    from . import lite_column as lc  # noqa: PLC0415
    from .mounts import ear_face  # noqa: PLC0415

    g = 9.81
    w, w_up = mass * g, (mass_above + 2.5) * g  # and the column's outer profile, on its foot
    share = w / 4
    out = []

    # pod
    pod = {n: (s, m, pr) for n, s, m, pr, _ in pod_parts() + pod_fixed_parts()}
    pu, pz = PIVOT
    su, sz = SPRING
    xi = X_IN - PLATE_T
    xp = xi - TAB_X
    mc = motor_centre()
    parts = [
        _with(
            made("pod_inner", "pod", "al6061", pod["pod_inner"][0]),
            ear_face("wheel", "RS00", mc, "x", 1),
            _cyl_iface("pivot", (X_IN - PLATE_T / 2, pu, pz), (1, 0, 0), 4.1, PLATE_T / 2),
        ),
        _with(
            made("pod_outer", "pod", "al6061", pod["pod_outer"][0]),
            _cyl_iface("wheel", (X_OUT + 3, 0, ZW), (1, 0, 0), 12.0, 3.0),
        ),
        made("pod_spacer_0", "pod", "al6061", pod["pod_spacer_0"][0]),
        made("pod_spacer_1", "pod", "al6061", pod["pod_spacer_1"][0]),
        _with(
            made("pod_spring_tab", "pod", "al6061", pod["pod_spring_tab"][0]),
            _face_iface("spring", (xp, su, sz + 3), (0, 0, 1), 4.2, 9.0),
        ),
    ]
    contact = (RC, 0.0, 0.0)
    out.append(
        (
            "pod",
            parts,
            [
                (
                    "bump (3 x)",
                    ["pivot", "spring"],
                    ["wheel"],
                    np.array([0, 0, 3 * share]),
                    np.zeros(3),
                    contact,
                ),
                (
                    "side knock",
                    ["pivot", "spring"],
                    ["wheel"],
                    np.array([-0.5 * share, 0, share]),
                    np.zeros(3),
                    contact,
                ),
                (
                    "drive (RS00 peak)",
                    ["pivot", "spring"],
                    ["wheel"],
                    np.array([0, 14.0 / (ZW / 1000), share]),
                    np.zeros(3),
                    contact,
                ),
            ],
        )
    )

    # base
    names = ("floor_plate", "deck_plate", "column_foot", "column_collar")
    sk = {n: (s, m, pr) for n, s, m, pr in skeleton_parts()}
    zf = FLOOR_Z + FLOOR_T
    zc0 = DECK_Z + DECK_T
    col = [_face_iface("column", (0, 0, zf + FOOT_T), (0, 0, 1), 0.0, 38.0)]
    for s in (1, -1):
        col.append(
            _face_iface("column", (s * (COL[0] / 2 + 1.5), 0, zc0 + 6), (1, 0, 0), 0.0, 18.0)
        )
        col.append(
            _face_iface("column", (0, s * (COL[1] / 2 + 1.5), zc0 + 6), (0, 1, 0), 0.0, 18.0)
        )
    parts = []
    for n in names:
        ifs = (
            [i for i in col if (n == "column_foot") == (i["axis"] == (0, 0, 1))]
            if n.startswith("column")
            else []
        )
        parts.append(_with(made(n, "base", sk[n][1], sk[n][0], sk[n][2]), *ifs))
    parts += [made(n, "base", m, s, pr) for n, (s, m, pr) in sk.items() if n.startswith("standoff")]
    for a in POD_ANG:
        c0, c1 = xi - 1 - 18, xi - 1
        r = np.radians(a)
        radial = (math.cos(r), math.sin(r), 0.0)
        pin = pod_world(a, pu, (c0 + c1) / 2)
        post = pod_world(a, su, xp)
        zc = sz - 3 - 15
        parts.append(
            _with(
                made(
                    f"pod_clevis_{int(a)}", "base", "al6061", brep_turned(pod["pod_clevis"][0], a)
                ),
                _cyl_iface("pods", (*pin, pz), radial, 3.1, 9.0),
            )
        )
        base = turned(pod["pod_post_base"][0], a)
        parts.append(
            _with(
                Part(f"pod_post_base_{int(a)}", "base", "al6061", base, "cnc"),
                _cyl_iface("pods", (*post, (zf + zc) / 2), (0, 0, 1), 3.25, (zc - zf) / 2),
            )
        )
    tip = 1.5 * w * RC * math.cos(math.radians(45))  # N·mm: weight x the edge distance
    foot = np.array([0, 0, zf + FOOT_T])
    out.append(
        (
            "base",
            parts,
            [
                (
                    "kerb drop (2 g)",
                    ["pods"],
                    ["column"],
                    np.array([0, 0, -2 * w_up]),
                    np.zeros(3),
                    foot,
                ),
                (
                    "tipping, fore-aft",
                    ["pods"],
                    ["column"],
                    np.array([0, 0, -w_up]),
                    np.array([0, tip, 0]),
                    foot,
                ),
                (
                    "tipping, sideways",
                    ["pods"],
                    ["column"],
                    np.array([0, 0, -w_up]),
                    np.array([tip, 0, 0]),
                    foot,
                ),
            ],
        )
    )

    # floor plate alone: in the base group above it did not mesh into one solid with the
    # parts bolted to it at 6 mm, and a finer mesh of that whole group outgrew the machine. Held
    # where the clevis blocks and post bases bolt on, loaded at the column foot's eight bolts
    # (the 10 mm foot as one rigid body). The deck and collar take part of the column's moment
    # in the base; here the foot takes all of it, which is conservative.
    ifs = [
        _face_iface("column", (x, y, zf), (0, 0, 1), CLEAR["M5"] / 2, 8.0)
        for x in (-80, -27, 27, 80)
        for y in (-60, 60)
    ]
    for a in POD_ANG:
        ifs.append(_face_iface("pods", (*pod_world(a, pu, xi - 10), zf), (0, 0, 1), 0.0, 9.0))
        ifs.append(_face_iface("pods", (*pod_world(a, su, xp), zf), (0, 0, 1), 3.25, 7.0))
    floor = _with(
        made("floor_plate", "floor", *sk["floor_plate"][1::-1], sk["floor_plate"][2]), *ifs
    )
    out.append(
        (
            "floor",
            [floor],
            [(n, h, ld, f, m, foot - [0, 0, FOOT_T]) for n, h, ld, f, m, _ in out[-1][2]],
        )
    )

    # column head
    heads = {p.name: p for p in lc.parts()}
    plate = heads["column_head_plate"]
    # it bears on the motor housing's top round the column's axis, and the cheeks bolt to the
    # housing's sides
    plate.extra["ifaces"] = [_face_iface("column", (0, 0, 0), (0, 0, 1), 0.0, 40.0)]
    group = [plate, heads["column_head_waist_ring"], heads["column_head_waist_cage"]]
    for side, sgn in (("l", 1), ("r", -1)):
        ck = heads[f"column_head_cheek_{side}"]
        y = sgn * (lc.CHEEK_Y + lc.CHEEK_T / 2)
        ck.extra["ifaces"] = [
            _cyl_iface("column", (x, y, lc.SIDE_M6_Z), (0, 1, 0), 3.3, lc.CHEEK_T / 2)
            for x in lc.SIDE_M6_X
        ]
        group.append(ck)
    for p in group:
        p.link = "column_head"
    out.append(
        (
            "column_head",
            group,
            [
                (
                    "hand",
                    ["column"],
                    ["waist_yaw"],
                    np.array([0.25, 0.15, 0.5]) * w,
                    np.zeros(3),
                    np.asarray(palm),
                ),
                (
                    "torque waist_yaw",
                    ["column"],
                    ["waist_yaw"],
                    np.zeros(3),
                    np.array([0, 0, 60.0e3]),
                    np.array(lc.waist_joint()),
                ),
            ],
        )
    )
    return out
