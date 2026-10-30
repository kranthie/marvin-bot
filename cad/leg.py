"""Leg parts: flat aluminium plates on pillar rings, printed shell, ankle linkage, foot.

Every part is built in its link's frame (mm), using the frames of description/skeleton.yaml
(left leg; the right leg is the mirror image). Motors sit where tools/build_urdf.py places
them (motor_mounts), at their STEP-model sizes, and the structure is built around them. The
metal parts are flat plates (cad/b3d.py, build123d); each motor's stator bolts through a plate
and a pillar ring (cad/mounts.py) into the motor, the pillars standing over its driver cap.

- hip roll: the roll RS04 sits in the pelvis behind the hip; a yoke round it (a front plate on
  its output, a rear plate on a far-side bearing on the pelvis's stub axle behind it) carries the
  pitch RS04, whose output faces in and whose stator sits on a web just outboard of the roll
  axis. In front of the front plate the web carries the pitch motor; behind it a strip of the
  same plate runs back over the roll motor to the rear plate. So the hip pitch torque (about y)
  reaches the roll motor's output 9 mm from its axis, in the plates' own planes: in the first
  pass the stator plate sat 60 mm inboard and bent the front plate (cad/fea.py: SF 0.2), and
  a stator plate inboard swept into the pelvis's middle at full hip roll;
- hip pitch: a closed box round the pitch RS04 (flange and inner wall on its output, outer wall
  on a far-side bearing on the web's stub axle, front and back walls below the motor) with the
  hip yaw RS03 under its floor;
- thigh: a flange on the yaw output and an outer side plate carrying the knee RS04, stiffened
  against a sideways fall by two ribs under the flange, under a printed shell;
- shin: two side plates on the shin's centre line with the two ankle RS03 staggered between
  them (one high and forward under the knee, one low behind the ankle), each on one plate; the
  plates' lower ends are the ankle fork cheeks, joined by spacer tubes and a bridge;
- ankle: a cross block, two cranks, two push rods with rod ends;
- foot: a shoe-shaped aluminium plate (a slight V toe) with heel posts for the rod ends and
  lugs for the roll bearings, on a TPU sole.

The shin's plates sit 0.5 mm off the ankle motors' caps, 1 mm further out than the knee's output
face: a 1 mm shim ring goes between the knee output and the inner plate. Plates longer than the
desktop mill's travel are cut by an online laser or water-jet service ("service"); the rest
are milled ("cnc"). Sized by finite elements (cad/fea.py).
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .b3d import cut, disc, hull, lighten, made, plate, prism, rect, rounded, slot, to_manifold
from .b3d import union as union2
from .lib import Part, box, cyl
from .lib import hull as mhull
from .mounts import (
    bore,
    output_face,
    output_holes,
    output_plate,
    pillar_ring,
    ring_band,
    ring_od,
    stator_face,
    stator_holes,
)
from .mounts import face as contact

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"

_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
# the pelvis back plate (cad/torso.py), between the roll motor and its rear bearing
HIP_BACK_T = 8.0
BRG_D, BRG_W = 42.0, 7.0  # 6806 far-side bearings (30 x 42 x 7 mm)
STUB_D, STUB_L = 30.0, 12.0  # their stub axles


def env(model: str) -> tuple[float, float]:
    """(diameter, thickness) mm of a motor housing."""
    d, _, t = _acts[model]["envelope_mm"]
    return d, t


def mm(v) -> list[float]:
    return [x * 1000 for x in v]


def xz(p) -> tuple[float, float]:
    return (p[0], p[2])


def tube(outer: float, wall: float, axis: str, uvc, a0: float, a1: float):
    """A square tube `outer` across with `wall` walls, from a0 to a1 along `axis`."""
    u, v = uvc
    h = outer / 2
    ring = rect(u - h, u + h, v - h, v + h) - rect(
        u - h + wall, u + h - wall, v - h + wall, v + h - wall
    )
    return prism(ring, axis, a0, a1)


def bearing(name: str, link: str, axis: str, centre) -> Part:
    return Part(
        name,
        link,
        "bought",
        cyl(BRG_D, BRG_W, axis, centre) - cyl(30, BRG_W + 2, axis, centre),
        "bought",
        mass_g=25.0,
        notes="6806-2RS far-side bearing",
    )


def stub(axis: str, centre):
    """A stub axle for a far-side bearing, bolted to its plate through the middle (M8)."""
    i = "xyz".index(axis)
    c = [centre[k] for k in range(3) if k != i]
    return plate(cut(disc(STUB_D, c), disc(8.5, c)), axis, centre[i], STUB_L)


def parts() -> list[Part]:
    J = {k: mm(v["xyz"]) for k, v in _skel["joints"].items()}
    mounts = {k: mm(v["xyz"]) for k, v in _skel["motor_mounts"].items()}
    link = _skel["ankle_linkage"]["geometry"]
    d04, t04 = env("RS04")
    d03, t03 = env("RS03")
    out: list[Part] = []

    # --- hip roll link: frame on the roll axis. The roll RS04 is fixed in the pelvis behind the
    # hip (its output face forward); the pitch RS04 is centred on the pitch joint, axis y, its
    # output face inward and its stator outward, on a pillar ring on the yoke's web. The web is a
    # y-normal plate just outboard of the roll axis: in front of the front plate it carries the
    # pitch motor, behind it it runs back over the roll motor to the rear plate, so the hip
    # pitch torque reaches the roll motor's output and the rear bearing in the plates' planes,
    # 9 mm from the roll axis, and nothing swings in towards the pelvis's centre.
    L = "left_hip_roll_link"
    rj = J["left_hip_roll"]
    roll_c = [a - b for a, b in zip(mounts["left_hip_roll"], rj, strict=True)]  # this frame
    pj = J["left_hip_pitch"]
    out_x = roll_c[0] + t04 / 2  # roll output face
    ft = 8.0  # front plate
    fx = out_x + ft / 2
    hp_ring, near = pillar_ring("RS04", pj, "y", -1)
    wt = 8.0  # the web (7075)
    st = 8.0  # the strip behind it, printed: it carries the rear bearing's small share
    w_in = pj[1] + near  # its inner face, on the pitch motor's pillar ring
    w_out = w_in + wt
    top_z = d04 / 2 + 32  # the web's top edge: it runs back 2 mm over the roll motor
    rt = 8.0  # rear plate: the bearing is 7 mm wide
    # the roll motor's pillar ring and the pelvis back plate behind it, then 14 mm for the stub
    back_face = roll_c[0] - pillar_ring("RS04", roll_c, "x", 1)[1] - HIP_BACK_T
    rear_x = back_face - 14 - rt / 2
    web_f = rounded(hull(disc(124, xz(pj)), rect(fx + ft / 2, -30, -40, top_z)))
    web_f = cut(
        web_f,
        stator_holes("RS04", pj, "y"),
        disc(8.5, xz(pj)),
        lighten(web_f, keep=[ring_band("RS04", pj, "y"), disc(20, xz(pj))], pitch=22, rim=6),
    )
    web_r = rect(rear_x + rt / 2, out_x, d04 / 2 + 2, top_z)
    web_r = cut(web_r, lighten(web_r, hole=14, pitch=20, rim=7))
    out.append(made("left_hip_roll_web", L, "al7075", plate(web_f, "y", w_in + wt / 2, wt)))
    out.append(
        made("left_hip_roll_strip", L, "pa_cf", plate(web_r, "y", w_in + st / 2, st), "print")
    )
    out.append(
        made(
            "left_hip_pitch_mount",
            L,
            "al6061",
            hp_ring,
            ifaces=[stator_face("left_hip_pitch", "RS04", pj, "y", -1)],
        )
    )
    stub_c = (pj[0], w_out + STUB_L / 2, pj[2])  # the hip pitch box's far-side bearing
    out.append(
        made(
            "left_hip_pitch_stub",
            L,
            "al6061",
            stub("y", stub_c),
            ifaces=[bore("left_hip_pitch", stub_c, "y", STUB_D, STUB_L)],
        )
    )
    front = rounded(union2(disc(84), rect(w_in - 14, w_out + 14, -40, top_z)))
    front = cut(front, lighten(front, keep=[disc(60)], hole=12, pitch=18, rim=7))
    front, _ = output_plate("RS04", front, roll_c, "x", 1, ft)
    out.append(
        made(
            "left_hip_roll_front",
            L,
            "al7075",
            front,
            ifaces=[output_face("left_hip_roll", "RS04", roll_c, "x", 1)],
        )
    )
    rear = rounded(union2(disc(BRG_D + 16), rect(w_in - 14, w_out + 14, 0, top_z)))
    rear = cut(rear, disc(BRG_D), lighten(rear, keep=[disc(BRG_D + 16)], hole=12, pitch=18))
    out.append(
        made(
            "left_hip_roll_rear",
            L,
            "al6061",
            plate(rear, "x", rear_x, rt),
            ifaces=[bore("left_hip_roll", (rear_x, 0, 0), "x", BRG_D, rt)],
        )
    )
    out.append(bearing("left_hip_roll_bearing", L, "x", (rear_x, 0, 0)))

    # --- hip pitch link: frame on the pitch axis (the pitch RS04 centred here, output face -y).
    # A closed box round the pitch motor: a flange and inner wall on its output, an outer wall on
    # a far-side bearing on the web's stub axle, front and back walls joining them below the
    # motor, and a floor on the yaw RS03's pillar ring.
    L = "left_hip_pitch_link"
    yaw_c = J["left_hip_yaw"]
    flange, _ = output_plate("RS04", disc(80), (0, 0, 0), "y", -1, 6.0)
    out.append(
        made(
            "left_hip_pitch_flange",
            L,
            "al6061",
            flange,
            ifaces=[output_face("left_hip_pitch", "RS04", (0, 0, 0), "y", -1)],
        )
    )
    y_ring, y_near = pillar_ring("RS03", yaw_c, "z", -1, base=2.0)
    floor_t = 8.0
    fz0 = yaw_c[2] + y_near  # the floor's underside, on the yaw motor's pillar ring
    fz1 = fz0 + floor_t
    side_t = 4.0
    iy1 = -t04 / 2 - 6.0  # inner wall, on the flange: its outer face
    oy = stub_c[1] - pj[1]  # outer wall, on the far-side bearing (this frame)
    ot = 8.0
    wz0 = -math.sqrt((d04 / 2) ** 2 - 24.0**2) - 7  # 7 mm under the pitch motor at x = +-24
    walls = hull(disc(48), rect(-24, 24, fz1, fz1 + 1))
    inner_w = rounded(union2(disc(80), walls))
    inner_w = cut(
        inner_w,
        lighten(inner_w, keep=[disc(66)], hole=14, pitch=20),
        output_holes("RS04", (0, 0, 0), "y"),
    )
    out.append(
        made("left_hip_pitch_inner", L, "al6061", plate(inner_w, "y", iy1 - side_t / 2, side_t))
    )
    outer_w = rounded(union2(disc(BRG_D + 16), walls))
    outer_w = cut(
        outer_w, disc(BRG_D), lighten(outer_w, keep=[disc(BRG_D + 12)], hole=12, pitch=18)
    )
    out.append(
        made(
            "left_hip_pitch_outer",
            L,
            "al7075",
            plate(outer_w, "y", oy, ot),
            ifaces=[bore("left_hip_pitch", (0, oy, 0), "y", BRG_D, ot)],
        )
    )
    yaw_xy = (yaw_c[0], yaw_c[1])
    floor = rounded(
        hull(disc(ring_od("RS03") + 4, yaw_xy), rect(-24, 24, iy1 - side_t, oy + ot / 2))
    )
    floor = cut(
        floor,
        stator_holes("RS03", yaw_c, "z"),
        lighten(floor, keep=[ring_band("RS03", yaw_c, "z")], pitch=24, rim=6),
    )
    out.append(made("left_hip_pitch_top", L, "al7075", plate(floor, "z", (fz0 + fz1) / 2, floor_t)))
    out.append(
        made(
            "left_hip_yaw_mount",
            L,
            "al6061",
            y_ring,
            ifaces=[stator_face("left_hip_yaw", "RS03", yaw_c, "z", -1)],
        )
    )
    wall_t = 4.0
    for name, s in (("front", 1), ("back", -1)):
        w = rect(iy1, oy - ot / 2, fz1, wz0)
        out.append(
            made(
                f"left_hip_pitch_{name}", L, "al6061", plate(w, "x", s * (24 - wall_t / 2), wall_t)
            )
        )
    out.append(bearing("left_hip_pitch_bearing", L, "y", (0, oy, 0)))

    # --- thigh: frame on the yaw axis at the yaw motor's centre; yaw output faces down. A flange
    # on the yaw output, an outer side plate on the knee RS04's pillar ring (its stator outward),
    # and two ribs under the flange inside the plate
    L = "left_thigh_link"
    yaw_out = -t03 / 2
    knee = J["left_knee"]
    k_ring, k_near = pillar_ring("RS04", knee, "y", -1, base=2.0)
    tt = 5.0  # 7075
    ty0 = knee[1] + k_near  # the plate's inner face
    tf_sk = rounded(union2(disc(84), rect(-32, 32, 12, ty0 + tt)))
    tflange, _ = output_plate("RS03", tf_sk, (0, 0, 0), "z", -1, 6.0)
    out.append(
        made(
            "left_thigh_flange",
            L,
            "al6061",
            tflange,
            ifaces=[output_face("left_hip_yaw", "RS03", (0, 0, 0), "z", -1)],
        )
    )
    fl_bot = yaw_out - 6.0
    tplate = rounded(hull(rect(-32, 32, -60, fl_bot), disc(124, xz(knee))))
    tplate = cut(
        tplate,
        stator_holes("RS04", knee, "y"),
        disc(8.5, xz(knee)),
        lighten(tplate, keep=[ring_band("RS04", knee, "y"), disc(20, xz(knee))], pitch=24, rim=6),
    )
    out.append(made("left_thigh_plate", L, "al7075", plate(tplate, "y", ty0 + tt / 2, tt)))
    for name, s in (("front", 1), ("back", -1)):
        rib = rect(12, ty0, -96, fl_bot)
        out.append(made(f"left_thigh_rib_{name}", L, "al7075", plate(rib, "x", s * 29.0, 6.0)))
    out.append(
        made(
            "left_thigh_knee_mount",
            L,
            "al6061",
            k_ring,
            ifaces=[stator_face("left_knee", "RS04", knee, "y", -1)],
        )
    )
    kstub_c = (knee[0], ty0 + tt + STUB_L / 2, knee[2])  # the shin's far-side bearing
    out.append(
        made(
            "left_thigh_knee_stub",
            L,
            "al6061",
            stub("y", kstub_c),
            ifaces=[bore("left_knee", kstub_c, "y", STUB_D, STUB_L)],
        )
    )
    shell_top, shell_bot = yaw_out - 12, knee[2] + d04 / 2 + 4
    outer = mhull(
        cyl(d03 + 16, 2, "z", (0, 0, shell_top)), box((d04 + 10, t04 + 30, 2), (0, 2, shell_bot))
    )
    inner = mhull(
        cyl(d03 + 10, 4, "z", (0, 0, shell_top)), box((d04 + 4, t04 + 24, 4), (0, 2, shell_bot))
    )
    out.append(Part("left_thigh_shell", L, "pa_cf", outer - inner, "print"))

    # --- shin: frame on the knee axis; knee output faces inward (-y). Two side plates on the
    # shin's centre line hold the staggered ankle RS03 between them: a (low) bolts its stator to
    # the inner plate and its crank hub passes beside the outer plate; b (high, forward) the
    # other way round. The plates run down to become the ankle fork cheeks.
    L = "left_shin_link"
    ka, kb = mounts["left_ankle_a"], mounts["left_ankle_b"]
    ankle_z = J["left_ankle_pitch"][2]
    shin_t = 5.0  # outer plate
    in_t = 4.0  # inner plate: more margin (cad/fea.py)
    a_ring, a_near = pillar_ring("RS03", ka, "y", 1, clear=0.5)
    b_ring, b_near = pillar_ring("RS03", kb, "y", -1, clear=0.5, base=2.0)
    yi, yo = -a_near, b_near  # the plates' inner faces (toward the motors)
    w = 44.0  # strap width
    cheek = (0.0, ankle_z)
    bridge_c = (70.0, -49.0)  # the bridge, in front of the knee motor
    disc_ab = ring_od("RS03") + 4
    spacers = ((55.0, -200.0), (-45.0, ankle_z + 25))
    # where the bridge and the spacer tubes bolt on: kept solid when the plates are pocketed
    anchors = [rect(bridge_c[0] - 17, bridge_c[0] + 17, bridge_c[1] - 17, bridge_c[1] + 17)]
    anchors += [disc(24, c) for c in spacers]
    inner = rounded(
        union2(
            disc(84),
            disc(disc_ab, xz(ka)),
            slot(w, (0, 0), (-12, -110), xz(ka), cheek),
            slot(30, (0, 0), bridge_c),
        )
    )
    # a slot behind motor a where push rod b passes at full ankle travel
    rod_slot = hull(disc(34, (ka[0] - 36, ka[2] + 35)), disc(34, (ka[0] - 36, ka[2] - 33)))
    inner = cut(
        inner,
        rod_slot,
        disc(22, cheek),
        output_holes("RS04", (0, 0, 0), "y"),
        stator_holes("RS03", ka, "y"),
        lighten(
            inner,
            keep=[disc(48), ring_band("RS03", ka, "y"), disc(40, cheek), *anchors],
            pitch=20,
            rim=6,
        ),
    )
    outer = rounded(
        union2(disc(disc_ab, xz(kb)), slot(w, bridge_c, xz(kb), (kb[0], ka[2] - 20), cheek))
    )
    outer = cut(
        outer,
        disc(136),  # the thigh plate and its stub turn inside r = 68 mm
        disc(22, cheek),
        stator_holes("RS03", kb, "y"),
        lighten(
            outer,
            keep=[ring_band("RS03", kb, "y"), disc(40, cheek), *anchors],
            pitch=20,
            rim=6,
        ),
    )
    out.append(
        made(
            "left_shin_inner",
            L,
            "al7075",
            plate(inner, "y", yi - in_t / 2, in_t),
            "service",
            ifaces=[
                contact("left_knee", (0, yi, 0), "y", 8.0, 24.0),
                bore("left_ankle_pitch", (0, yi - in_t / 2, ankle_z), "y", 22, in_t),
            ],
        )
    )
    out.append(
        made(
            "left_shin_outer",
            L,
            "al7075",
            plate(outer, "y", yo + shin_t / 2, shin_t),
            "service",
            ifaces=[bore("left_ankle_pitch", (0, yo + shin_t / 2, ankle_z), "y", 22, shin_t)],
        )
    )
    for name, c, sgn, ring in (("a", ka, 1, a_ring), ("b", kb, -1, b_ring)):
        out.append(
            made(
                f"left_ankle_mount_{name}",
                L,
                "al6061",
                ring,
                ifaces=[stator_face(f"left_ankle_{name}", "RS03", c, "y", sgn)],
            )
        )
    # the knee's far side: a plate on a bearing on the thigh plate's stub axle, tied to both
    # side plates by a bridge tube in front of the knee motor (and a short spacer)
    ko_y = kstub_c[1]
    ko = rounded(union2(disc(BRG_D + 16), slot(26, (0, 0), bridge_c)))
    ko = cut(ko, disc(BRG_D))
    out.append(
        made(
            "left_shin_knee_outer",
            L,
            "al6061",
            plate(ko, "y", ko_y, 8.0),
            ifaces=[bore("left_knee", (0, ko_y, 0), "y", BRG_D, 8.0)],
        )
    )
    out.append(made("left_shin_bridge", L, "al7075", tube(28, 5, "y", bridge_c, yi, yo)))
    out.append(
        made(
            "left_shin_bridge_spacer",
            L,
            "al6061",
            tube(28, 5, "y", bridge_c, yo + shin_t, ko_y - 4),
        )
    )
    out.append(bearing("left_knee_bearing", L, "y", (0, ko_y, 0)))
    # spacer tubes between the side plates, clear of both motors and the ankle cross
    for k, c in enumerate(spacers):
        sp = prism(cut(disc(16, c), disc(8.5, c)), "y", yi, yo)
        out.append(made(f"left_shin_spacer_{k}", L, "al6061", sp))
    # cranks outside the side plates, pointing back (and slightly down) at neutral
    th = math.pi + link["crank_tilt"]
    r = link["crank_radius"] * 1000
    cy = link["crank_y"] * 1000
    for name, c, s in (("a", ka, 1), ("b", kb, -1)):
        tip = (c[0] + r * math.cos(th), s * cy, c[2] + r * math.sin(th))
        arm = plate(hull(disc(36, xz(c)), disc(18, xz(tip))), "y", s * cy, 5)
        hub = prism(disc(40, xz(c)), "y", s * t03 / 2, s * (cy - 2.5))
        crank = arm + hub
        crank = crank - prism(output_holes("RS03", c, "y"), "y", s * (t03 / 2 - 1), s * (cy + 4))
        crank = crank - prism(disc(6.5, xz(tip)), "y", s * (cy - 4), s * (cy + 4))
        out.append(
            made(
                f"left_ankle_crank_{name}",
                L,
                "al7075",
                crank,
                ifaces=[
                    output_face(f"left_ankle_{name}", "RS03", c, "y", s),
                    bore(f"left_ankle_rod_{name}", tip, "y", 18, 6),
                ],
                extra={"group": f"left_ankle_crank_{name}"},
            )
        )
        # rod from the crank tip to its ball joint on the foot (neutral pose, shin frame)
        foot_end = (
            -link["rod_back"] * 1000,
            s * link["rod_side"] * 1000,
            ankle_z - link["rod_below"] * 1000,
        )
        rod = mhull(cyl(10, 1, "z", tip), cyl(10, 1, "z", foot_end))
        out.append(
            Part(
                f"left_ankle_rod_{name}",
                L,
                "bought",
                rod,
                "bought",
                mass_g=30.0 if name == "a" else 45.0,
                notes="steel tube with two M6 rod ends",
            )
        )

    # --- ankle cross and foot: frames on the ankle axes
    cross_w = t03 - 4  # between the shin's side plates
    cross = prism(rect(-22, 22, -15, 15) - rect(-18, 18, -11, 11), "y", -cross_w / 2, cross_w / 2)
    cross = cross - prism(rect(-16, 16, -cross_w / 2 + 6, cross_w / 2 - 6), "z", -20, 20)
    cx_if = [contact("left_ankle_pitch", (0, s * cross_w / 2, 0), "y", 0, 40) for s in (1, -1)]
    cx_if += [contact("left_ankle_roll", (s * 22, 0, 0), "x", 0, 40) for s in (1, -1)]
    out.append(made("left_ankle_cross", "left_ankle_link", "al7075", cross, ifaces=cx_if))
    L = "left_foot_link"
    foot = _skel["collisions"][[c["link"] for c in _skel["collisions"]].index(L)]
    fl, fw, _ = mm(foot["size"])
    fx = foot["xyz"][0] * 1000
    sole_bot = (foot["xyz"][2] - foot["size"][2] / 2) * 1000
    out.append(Part("left_foot_sole", L, "tpu", shoe(fl, fw, 4, fx, sole_bot + 2), "print"))
    pz = sole_bot + 4 + 2.5  # the base plate's mid-plane
    # the base is longer than the desktop mill's travel: a flat plate cut by an online laser or
    # water-jet service; posts and lugs are machined and bolted on
    base = shoe_outline(fl - 10, fw - 10, fx)
    base = cut(base, lighten(base, hole=24, pitch=30, rim=10))
    out.append(
        made(
            "left_foot_base",
            L,
            "al7075",
            plate(base, "z", pz, 4),
            "service",
            ifaces=[contact("ground", (fx, 0, pz - 2.0), "z", 0, 200, rigid=False)],
        )
    )
    top = pz + 2.0
    # stiffening ribs along the foot either side of the lugs, and one down the middle of the toe
    for k, ry in (("rib_a", 24), ("rib_b", -24)):
        rib = rect(fx - 10 - (fl - 90) / 2, fx - 10 + (fl - 90) / 2, top, top + 16)
        out.append(made(f"left_foot_{k}", L, "al6061", plate(rib, "y", ry, 4)))
    toe_end = fx + (fl - 10) / 2 - 3
    out.append(
        made("left_foot_rib_c", L, "al6061", plate(rect(40, toe_end, top, top + 20), "y", -6, 4))
    )
    rb, rs, rbl = (link[k] * 1000 for k in ("rod_back", "rod_side", "rod_below"))
    for name, s in (("post_a", 1), ("post_b", -1)):
        post = prism(rect(-rb - 8, -rb + 8, s * rs - 8, s * rs + 8), "z", top, -rbl)
        out.append(
            made(
                f"left_foot_{name}",
                L,
                "al6061",
                post,
                ifaces=[contact(f"left_ankle_rod_{name[-1]}", (-rb, s * rs, -rbl), "z", 0, 8)],
            )
        )
    for name, x in (("lug_front", 32), ("lug_back", -32)):
        lug = rounded(union2(rect(-18, 18, top, 0, r=0), disc(26, (0, 0))))
        lug = cut(lug, disc(18, (0, 0)))
        out.append(
            made(
                f"left_foot_{name}",
                L,
                "al7075" if name == "lug_front" else "al6061",
                plate(lug, "x", x, 8),
                ifaces=[bore("left_ankle_roll", (x, 0, 0), "x", 18, 8)],
            )
        )
    return out


def shoe_outline(length: float, width: float, xc: float):
    """A foot outline like a shoe's, `length` along x (centred at xc) and `width` at the ball:
    a heel with rounded corners (full width there, for the rod-end posts), the widest point at
    the ball, and a toe that narrows in a slight V to a small round point set towards the
    inside (-y on the left foot, like the big toe)."""
    back, front = xc - length / 2, xc + length / 2
    heel = [disc(0.4 * width, (back + 0.2 * width, s * 0.3 * width)) for s in (1, -1)]
    ball = [disc(0.3 * width, (back + 0.66 * length, s * 0.35 * width)) for s in (1, -1)]
    toe = disc(0.36 * width, (front - 0.18 * width, -0.06 * width))
    return hull(*heel, *ball, toe)


def shoe(length: float, width: float, t: float, xc: float, z: float):
    """The shoe outline as a solid `t` thick at height z."""
    return to_manifold(plate(shoe_outline(length, width, xc), "z", z, t))


def all_parts() -> list[Part]:
    left = parts()
    return left + [p.mirrored() for p in left]
