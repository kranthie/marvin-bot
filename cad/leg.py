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

from .b3d import (
    clip_spot,
    cut,
    disc,
    hull,
    lighten,
    made,
    plate,
    prism,
    rect,
    rounded,
    slot,
    to_manifold,
)
from .b3d import union as union2
from .clips import edge_clip
from .lib import Part, cyl
from .lib import hull as mhull
from .mounts import (
    BEARINGS,
    CLIP_HOLE,
    CLIP_PAD,
    bore,
    cage,
    ear_face,
    front_ring,
    output_face,
    output_holes,
    output_plate,
    pillar_ring,
    ring_band,
    ring_od,
    spec,
    stator_face,
    stator_holes,
    zero_stop,
)
from .mounts import face as contact
from .mounts import (
    hub as out_hub,
)

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"

_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
# the pelvis back plate (cad/torso.py), between the roll motor and its rear bearing
HIP_BACK_T = 10.0  # the pelvis back plate: each hip's whole load comes in through its roll motor
BRG_D, BRG_W = 42.0, 7.0  # 6806 far-side bearings (30 x 42 x 7 mm)
STUB_D, STUB_L = 30.0, 12.0  # their stub axles
YAW_BRG = "RA6008"  # the hip yaw's crossed-roller bearing (and the hip roll's carrier)
# the hip yaw ring's front face from the yaw motor's centre: 1 mm past the output, the bearing
YAW_RING_FRONT = spec("RS03")["t"] / 2 + 1 + BEARINGS[YAW_BRG]["B"]
ROLL_RING_FRONT = spec("RS04")["t"] / 2 + 1 + BEARINGS[YAW_BRG]["B"]  # the hip roll's carrier
# stop and zero pins (radius, angle) on the bearing rings: between the seat and the screws
YAW_PIN = (41.5, 22.5)
HIP_ROLL_PIN = (42.5, 18.0)


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


def crossed_roller(name: str, link: str, axis: str, centre, kind: str = YAW_BRG) -> Part:
    b = BEARINGS[kind]
    return Part(
        name,
        link,
        "bought",
        cyl(b["D"], b["B"], axis, centre) - cyl(b["d"], b["B"] + 2, axis, centre),
        "bought",
        mass_g=b["mass_g"],
        notes=f"{kind} crossed-roller bearing ({b['d']:.0f} x {b['D']:.0f} x {b['B']:.0f} mm)",
    )


def _along(c, axis: str, d: float) -> tuple:
    """The point `d` along `axis` from c."""
    v = list(c)
    v["xyz".index(axis)] += d
    return tuple(v)


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
    # the roll output drives a hub through the pelvis's bearing carrier (cad/torso.py) on the
    # roll motor's front face; the yoke's front plate bolts on the hub, clamping the inner ring
    rhub, roll_top, roll_relief = out_hub("RS04", roll_c, "x", 1, ROLL_RING_FRONT, bearing=YAW_BRG)
    out.append(
        made(
            "left_hip_roll_hub",
            L,
            "al7075",
            rhub,
            ifaces=[
                output_face("left_hip_roll", "RS04", roll_c, "x", 1),
                bore("left_hip_roll", _along(roll_c, "x", roll_top - 4.5), "x", 60.0, 8.0),
            ],
        )
    )
    out_x = roll_c[0] + roll_top  # the front plate's face, on the hub
    ft = 5.0  # front plate
    fx = out_x + ft / 2
    hp_ring, near = pillar_ring("RS04", pj, "y", -1)
    wt = 6.0  # the web (7075)
    w_in = pj[1] + near  # its inner face, on the pitch motor's pillar ring
    w_out = w_in + wt
    top_z = d04 / 2 + 32  # the web's top edge: it runs back 2 mm over the roll motor
    web_f = rounded(hull(disc(124, xz(pj)), rect(fx + ft / 2, -30, -40, top_z)))
    # the hip pitch's stop and zero pins stand out of the web's outer face into the box's
    # outer wall, 2 mm away across the stub axle
    hp_pins, _, _, hp_pads = zero_stop("RS04", pj, "y", "left_hip_pitch", False, 30.0, 0.0)
    # harness clips (tools/build_harness.py's CLIPS): the hip roll loop's, over the roll
    # motor, and the hip pitch loop's, beside the pitch motor
    web_avoid = [stator_holes("RS04", pj, "y"), disc(8.5, xz(pj)), hp_pads]
    web_clips = [
        clip_spot(web_f, (out_x - 15, 0.0), web_avoid, min_r=64),
        clip_spot(web_f, xz(pj), web_avoid, min_r=64),
    ]
    web_f = cut(
        web_f,
        stator_holes("RS04", pj, "y"),
        disc(8.5, xz(pj)),
        lighten(
            web_f,
            keep=[ring_band("RS04", pj, "y"), disc(20, xz(pj)), hp_pads]
            + [disc(CLIP_PAD, c) for c in web_clips],
            pitch=22,
            rim=6,
        ),
        *[disc(CLIP_HOLE, c) for c in web_clips],
    )
    web = plate(web_f, "y", w_in + wt / 2, wt) - prism(hp_pins, "y", w_out - 5, w_out + 1)
    out.append(made("left_hip_roll_web", L, "al7075", web))
    out.append(
        made(
            "left_hip_pitch_mount",
            L,
            "al7075",
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
    front = rounded(union2(disc(98), rect(w_in - 14, w_out + 14, -40, top_z)))
    front = cut(front, lighten(front, keep=[disc(98)], hole=12, pitch=18, rim=7))
    front, _ = output_plate("RS04", front, roll_c, "x", 1, ft, at=roll_top, d=roll_relief)
    _, groove, zh, _ = zero_stop("RS04", roll_c, "x", "left_hip_roll", True, *HIP_ROLL_PIN)
    front = (
        front - prism(groove, "x", out_x - 1, out_x + 4) - prism(zh, "x", out_x - 1, out_x + ft + 1)
    )
    out.append(made("left_hip_roll_front", L, "al7075", front))

    # --- hip pitch link: frame on the pitch axis (the pitch RS04 centred here, output face -y).
    # A closed box round the pitch motor: a flange and inner wall on its output, an outer wall on
    # a far-side bearing on the web's stub axle, front and back walls joining them below the
    # motor, and a floor on the yaw RS03's pillar ring.
    L = "left_hip_pitch_link"
    yaw_c = J["left_hip_yaw"]
    ht = 4.0  # the flange; the box's inner wall sits on its outer face
    flange, _ = output_plate("RS04", disc(80), (0, 0, 0), "y", -1, ht)
    out.append(
        made(
            "left_hip_pitch_flange",
            L,
            "al6061",
            flange,
            ifaces=[output_face("left_hip_pitch", "RS04", (0, 0, 0), "y", -1)],
        )
    )
    # the yaw RS03 is on the thigh, output up: its hub (here) stands on its output flange through
    # the thigh's bearing ring, and the floor bolts onto the hub, clamping the bearing's inner ring
    yhub, hub_top, hub_relief = out_hub("RS03", yaw_c, "z", 1, YAW_RING_FRONT, bearing=YAW_BRG)
    out.append(
        made(
            "left_hip_yaw_hub",
            L,
            "al7075",
            yhub,
            ifaces=[
                output_face("left_hip_yaw", "RS03", yaw_c, "z", 1),
                bore("left_hip_yaw", _along(yaw_c, "z", hub_top - 4.5), "z", 60.0, 8.0),
            ],
        )
    )
    floor_t = 3.0
    fz0 = yaw_c[2] + hub_top  # the floor's underside, on the hub
    fz1 = fz0 + floor_t
    side_t = 4.0
    iy1 = -t04 / 2 - ht  # inner wall, on the flange: its outer face
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
    outer_w = rounded(union2(disc(76), walls))
    outer_w = cut(outer_w, disc(BRG_D), lighten(outer_w, keep=[disc(60)], hole=12, pitch=18))
    _, hp_groove, hp_zero, _ = zero_stop("RS04", (0, 0, 0), "y", "left_hip_pitch", True, 30.0, 0.0)
    ow0 = oy - ot / 2  # the wall's face toward the web
    out.append(
        made(
            "left_hip_pitch_outer",
            L,
            "al7075",
            plate(outer_w, "y", oy, ot)
            - prism(hp_groove, "y", ow0 - 1, ow0 + 3)
            - prism(hp_zero, "y", ow0 - 1, ow0 + ot + 1),
            ifaces=[bore("left_hip_pitch", (0, oy, 0), "y", BRG_D, ot)],
        )
    )
    # the hip pitch loop's harness clip, on the outer wall's upper edge
    out.append(edge_clip("left_hip_pitch_outer_clip", L, "y", oy, ot, outer_w, (0, 50)))
    yaw_xy = (yaw_c[0], yaw_c[1])
    floor = rounded(hull(disc(96, yaw_xy), rect(-24, 24, iy1 - side_t, oy + ot / 2)))
    floor = cut(floor, disc(12, yaw_xy), lighten(floor, keep=[disc(96, yaw_xy)], pitch=24, rim=6))
    floor, _ = output_plate("RS03", floor, yaw_c, "z", 1, floor_t, at=hub_top, d=hub_relief)
    _, groove, zh, _ = zero_stop("RS03", yaw_c, "z", "left_hip_yaw", False, *YAW_PIN)
    floor = floor - prism(groove, "z", fz0 - 1, fz0 + 2) - prism(zh, "z", fz0 - 1, fz1 + 1)
    out.append(made("left_hip_pitch_top", L, "al7075", floor))
    wall_t = 3.0
    for name, s in (("front", 1), ("back", -1)):
        w = rect(iy1, oy - ot / 2, fz1, wz0)
        # the back wall carries the hip yaw loop's clip, low over the yaw axis
        clips = [clip_spot(w, (yaw_c[1], fz1), min_r=0)] if s == -1 else []
        w = cut(
            w,
            lighten(w, keep=[disc(CLIP_PAD, c) for c in clips], pitch=14, rim=5),
            *[disc(CLIP_HOLE, c) for c in clips],
        )
        out.append(
            made(
                f"left_hip_pitch_{name}", L, "al7075", plate(w, "x", s * (24 - wall_t / 2), wall_t)
            )
        )
    out.append(bearing("left_hip_pitch_bearing", L, "y", (0, oy, 0)))

    # --- thigh: frame on the yaw axis at the yaw motor's centre; the yaw RS03 is the thigh's,
    # output up. A ring on its output-end ears holds it and seats the crossed-roller bearing
    # whose inner ring is on the hip pitch box's hub; four strips round the motor carry the ring
    # down to a floor 1 mm below its driver cap, and under the floor an outer side plate carries
    # the knee RS04's pillar ring (its stator outward), stiffened against a sideways fall by two
    # ribs inside the plate
    L = "left_thigh_link"
    o3 = (0.0, 0.0, 0.0)
    knee = J["left_knee"]
    k_ring, k_near = pillar_ring("RS04", knee, "y", -1, base=2.0)
    tt = 5.0  # 7075
    ty0 = knee[1] + k_near  # the plate's inner face
    cap_end = t03 / 2  # the yaw motor's driver cap end, below its centre
    strips, pads, strips_fea = cage("RS03", o3, "z", 1, to=cap_end + 1.0, width=16.0, t=4.0)
    ring_sk = rounded(union2(disc(ring_od("RS03")), pads))
    y_pins, _, _, y_pads = zero_stop("RS03", o3, "z", "left_hip_yaw", False, *YAW_PIN)
    yring, _ = front_ring("RS03", o3, "z", 1, outline=ring_sk, bearing=YAW_BRG, keep=y_pads)
    yring = yring - prism(y_pins, "z", YAW_RING_FRONT - 6, YAW_RING_FRONT + 1)
    seat_c = (0.0, 0.0, YAW_RING_FRONT - BEARINGS[YAW_BRG]["B"] / 2)
    out.append(
        made(
            "left_thigh_yaw_ring",
            L,
            "al7075",
            yring,
            ifaces=[
                ear_face("left_hip_yaw", "RS03", o3, "z", 1),
                bore("left_hip_yaw", seat_c, "z", BEARINGS[YAW_BRG]["D"], BEARINGS[YAW_BRG]["B"]),
            ],
        )
    )
    out.append(made("left_thigh_cage", L, "al7075", strips, extra={"fea_solid": strips_fea}))
    out.append(crossed_roller("left_hip_yaw_bearing", L, "z", seat_c))
    yaw_out = -cap_end - 1.0  # the floor's top face
    fl_t = 6.0
    tf_sk = rounded(union2(disc(124), rect(-32, 32, 12, ty0 + tt)))  # under the strips
    t_clip = clip_spot(tf_sk, (0.0, 0.0), [disc(30), pads], min_r=40)
    tf_sk = cut(
        tf_sk,
        disc(30),
        lighten(
            tf_sk, keep=[pads, rect(-32, 32, 12, ty0 + tt), disc(CLIP_PAD, t_clip)], pitch=18, rim=5
        ),
        disc(CLIP_HOLE, t_clip),
    )
    out.append(made("left_thigh_floor", L, "al7075", plate(tf_sk, "z", yaw_out - fl_t / 2, fl_t)))
    fl_bot = yaw_out - fl_t
    tplate = rounded(hull(rect(-32, 32, -60, fl_bot), disc(124, xz(knee))))
    k_pins, _, _, k_pads = zero_stop("RS04", knee, "y", "left_knee", False, 30.0, 0.0)
    k_clip = clip_spot(tplate, xz(knee), [stator_holes("RS04", knee, "y"), k_pads], min_r=64)
    tplate = cut(
        tplate,
        stator_holes("RS04", knee, "y"),
        disc(8.5, xz(knee)),
        lighten(
            tplate,
            keep=[ring_band("RS04", knee, "y"), disc(20, xz(knee)), k_pads, disc(CLIP_PAD, k_clip)],
            pitch=24,
            rim=6,
        ),
        disc(CLIP_HOLE, k_clip),
    )
    tplate = plate(tplate, "y", ty0 + tt / 2, tt) - prism(k_pins, "y", ty0, ty0 + tt + 1)
    out.append(
        made(
            "left_thigh_plate",
            L,
            "al7075",
            tplate,
            extra={"clip": (k_clip[0], ty0 + tt / 2, k_clip[1])},
        )
    )
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

    # --- shin: frame on the knee axis; knee output faces inward (-y). Two side plates on the
    # shin's centre line hold the staggered ankle RS03 between them: a (low) bolts its stator to
    # the inner plate and its crank hub passes beside the outer plate; b (high, forward) the
    # other way round. The plates run down to become the ankle fork cheeks.
    L = "left_shin_link"
    ka, kb = mounts["left_ankle_a"], mounts["left_ankle_b"]
    ankle_z = J["left_ankle_pitch"][2]
    shin_t = 5.0  # outer plate
    in_t = 4.0  # inner plate: more margin (cad/fea.py)
    # each ankle motor's output-end ears bolt to the plate its output passes through: a's to the
    # outer plate, b's to the inner; the plates' inner faces (toward the motors)
    yi, yo = kb[1] - spec("RS03")["ear"], ka[1] + spec("RS03")["ear"]
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
            disc(disc_ab, xz(kb)),
            slot(w, (0, 0), (-12, -110), xz(ka), cheek),
            slot(30, (0, 0), bridge_c),
        )
    )
    # a slot behind motor a where push rod b passes at full ankle travel
    rod_slot = hull(disc(34, (ka[0] - 36, ka[2] + 35)), disc(34, (ka[0] - 36, ka[2] - 33)))
    # the knee loop's clip on the shin side
    s_avoid = [
        rod_slot,
        disc(48),
        disc(spec("RS03")["output_d"] + 2, xz(kb)),
        stator_holes("RS03", kb, "y"),
    ]
    s_clip = clip_spot(inner, (0.0, 0.0), s_avoid, min_r=64)
    inner = cut(
        inner,
        rod_slot,
        disc(22, cheek),
        output_holes("RS04", (0, 0, 0), "y"),
        disc(spec("RS03")["output_d"] + 2, xz(kb)),  # b's output and crank hub
        stator_holes("RS03", kb, "y"),
        lighten(
            inner,
            keep=[disc(48), ring_band("RS03", kb, "y"), disc(40, cheek), *anchors]
            + [disc(CLIP_PAD, s_clip)],
            pitch=20,
            rim=6,
        ),
        disc(CLIP_HOLE, s_clip),
    )
    outer = rounded(
        union2(
            disc(disc_ab, xz(ka)),
            slot(w, bridge_c, xz(kb), (kb[0], ka[2] - 20), cheek),
        )
    )
    outer = cut(
        outer,
        disc(136),  # the thigh plate and its stub turn inside r = 68 mm
        disc(22, cheek),
        disc(spec("RS03")["output_d"] + 2, xz(ka)),  # a's output and crank hub
        stator_holes("RS03", ka, "y"),
        lighten(
            outer,
            keep=[ring_band("RS03", ka, "y"), disc(40, cheek), *anchors],
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
            extra={"clip": (s_clip[0], yi - in_t / 2, s_clip[1])},
            ifaces=[
                contact("left_knee", (0, yi, 0), "y", 8.0, 24.0),
                ear_face("left_ankle_b", "RS03", kb, "y", -1),
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
            ifaces=[
                ear_face("left_ankle_a", "RS03", ka, "y", 1),
                bore("left_ankle_pitch", (0, yo + shin_t / 2, ankle_z), "y", 22, shin_t),
            ],
        )
    )
    # the knee's far side: a plate on a bearing on the thigh plate's stub axle, tied to both
    # side plates by a bridge tube in front of the knee motor (and a short spacer)
    ko_y = kstub_c[1]
    ko = rounded(union2(disc(76), slot(26, (0, 0), bridge_c)))
    ko = cut(ko, disc(BRG_D))
    _, k_groove, k_zero, _ = zero_stop("RS04", (0, 0, 0), "y", "left_knee", True, 30.0, 0.0)
    kf = ko_y - 4.0  # its face toward the thigh plate
    out.append(
        made(
            "left_shin_knee_outer",
            L,
            "al6061",
            plate(ko, "y", ko_y, 8.0)
            - prism(k_groove, "y", kf - 1, kf + 3)
            - prism(k_zero, "y", kf - 1, kf + 9),
            ifaces=[bore("left_knee", (0, ko_y, 0), "y", BRG_D, 8.0)],
        )
    )
    out.append(made("left_shin_bridge", L, "al7075", tube(24, 3, "y", bridge_c, yi, yo)))
    out.append(
        made(
            "left_shin_bridge_spacer",
            L,
            "al6061",
            tube(24, 3, "y", bridge_c, yo + shin_t, ko_y - 4),
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
        ca = 3.5  # the arm
        arm = plate(hull(disc(32, xz(c)), disc(16, xz(tip))), "y", s * cy, ca)
        hub = prism(disc(36, xz(c)), "y", c[1] + s * t03 / 2, s * (cy - ca / 2))
        crank = arm + hub
        crank = crank - prism(
            output_holes("RS03", c, "y"), "y", c[1] + s * (t03 / 2 - 1), s * (cy + 4)
        )
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
    out.append(Part("left_foot_sole", L, "tpu", shoe(fl, fw, 3, fx, sole_bot + 1.5), "print"))
    pz = sole_bot + 3 + 2.5  # the base plate's mid-plane, on the 3 mm sole
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
