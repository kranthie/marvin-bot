"""Arm parts, first pass: aluminium motor brackets, printed structural shells, wrist and hand mount.

Built in the arm link frames (mm) of description/skeleton.yaml, left arm (the right arm is the
mirror image), around the motors at their STEP-model sizes:

- shoulder pitch link: an L-bracket from the pitch RS03's output (facing out, +y) to the back
  face of the shoulder roll RS03 below it;
- shoulder roll link: an L-bracket from the roll RS03's output (facing forward, +x) to the top
  of the shoulder yaw RS06;
- upper arm: an aluminium ring on the yaw output, a printed shell down to the elbow, and an
  aluminium ring on the elbow RS06's inner (stator) face; the forearm drives off its outer face;
- forearm: an aluminium flange on the elbow output and a printed shell that wraps under the
  elbow to a cup holding the wrist RS00;
- wrist: a printed cradle on the wrist RS00's output holding the wrist pitch servo, and a
  printed U on the servo's horn and an idler pin carrying a plate for the bought hand;
- hand: the AmazingHand (Pollen Robotics, CC-BY-4.0), from the meshes in
  description/third_party/amazing_hand.

The shells are hollow (2.5 mm walls) and leave a cable path from the shoulder to the hand:
each joint's harness passes beside the motor in a service loop (docs/design/assembly_wiring.md).
No fasteners yet.
"""

from __future__ import annotations

import math
from pathlib import Path

import manifold3d as mf
import numpy as np
import yaml

from .b3d import (
    circle_pts,
    circles,
    clip_spot,
    cut,
    disc,
    hull,
    lighten,
    made,
    rect,
    rounded,
    slot,
    to_manifold,
)
from .b3d import plate as bplate
from .b3d import prism as bprism
from .b3d import union as union2
from .clips import edge_clip
from .lib import Part, cyl
from .mounts import (
    CLIP_HOLE,
    CLIP_PAD,
    cage,
    ear_face,
    front_ring,
    output_face,
    output_plate,
    output_windows,
    ring_band,
    spec,
    zero_stop,
)
from .mounts import face as contact
from .mounts import hub as out_hub
from .torso import SHOULDER_PIN, SHOULDER_RING_T

PIN_06 = (32.0, 22.5)  # stop and zero pins on the RS06 rings: between the bore and the screws
PIN_ROLL = (40.5, 90.0)  # and on the shoulder roll's

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
WALL = 2.5  # printed shells
HAND_MESHES = ROOT / "description" / "third_party" / "amazing_hand"
# the hand's wrist interface (its R-/L-Wrist_Interface part): a 49 mm round flange with six
# M3 clearance holes on a 42 mm circle, at these angles (deg) in the palm link frame
HAND_FLANGE = 49.0
HAND_PCD = 42.0
HAND_HOLES = (-10, -50, -130, -170, 110, 70)


def env(model: str) -> tuple[float, float]:
    d, _, t = _acts[model]["envelope_mm"]
    return d, t


def xz(p) -> tuple[float, float]:
    return (p[0], p[2])


def shell(outer, inner) -> object:
    return outer - inner


def parts() -> list[Part]:
    j = {k: [x * 1000 for x in v["xyz"]] for k, v in _skel["joints"].items()}
    mounts = {k: [x * 1000 for x in v["xyz"]] for k, v in _skel["motor_mounts"].items()}
    d03, t03 = env("RS03")
    d06, t06 = env("RS06")
    d00, t00 = env("RS00")
    sw, sl, st = _acts["STS3250"]["envelope_mm"]  # servo: 45.3 long (x), 35.8 high, 24.8 thick
    t = 6.0
    out: list[Part] = []

    # --- shoulder pitch link: frame on the pitch axis; the pitch RS03 (torso, centred at pm)
    # drives a hub through the torso's ring on its ears, y0 the hub's face; the roll RS03 is
    # centred at `roll`, its axis along x, its output forward. The roll motor's body comes
    # within 0.3 mm of the hub's face, so the flange (and the hub) are cut round it below the
    # axis (they keep the output's upper bolts and pins); the flange reaches forward to a ring
    # on the roll motor's output-end ears. Everything here stays within 56 mm of the roll axis
    # ahead of the flange: the shoulder roll link's top plate swings round outside that.
    L = "left_shoulder_pitch_link"
    sp = j["left_shoulder_pitch"]
    pm = (0.0, mounts["left_shoulder_pitch"][1] - sp[1], 0.0)
    roll = j["left_shoulder_roll"]
    ry, rz = roll[1], roll[2]
    phub, p_top, p_relief = out_hub("RS03", pm, "y", 1, spec("RS03")["ear"] + SHOULDER_RING_T)
    y0 = pm[1] + p_top  # the hub's face
    hub_cut = math.sqrt(51.0**2 - (ry - y0) ** 2)
    phub = phub - bprism(
        rect(-t03 / 2 - 1, t03 / 2 + 1, rz - hub_cut, rz + hub_cut, r=5.0), "y", y0 - 10, y0 + 1
    )
    out.append(
        made(
            "left_shoulder_hub",
            L,
            "al7075",
            phub,
            ifaces=[output_face("left_shoulder_pitch", "RS03", pm, "y", 1)],
        )
    )
    rt = 6.0  # the roll motor's ring
    r_front = roll[0] + spec("RS03")["ear"] + rt
    # the roll motor's body (r = 50 mm, 1 mm clear) where it crosses the flange's plate
    cut_z = math.sqrt(51.0**2 - (ry - (y0 + t)) ** 2)
    fl = rounded(union2(disc(92), rect(-20, r_front, rz - 30, 8)))
    # (to its ears' face: ahead of that only the output spigot, well inside the flange's plane)
    fl = cut(fl, rect(-t03 / 2 - 1, spec("RS03")["ear"], rz - cut_z, rz + cut_z))
    flange, _ = output_plate("RS03", fl, pm, "y", 1, t, at=p_top, d=p_relief)
    _, groove, zh, _ = zero_stop("RS03", pm, "y", "left_shoulder_pitch", True, *SHOULDER_PIN)
    flange = flange - bprism(groove, "y", y0 - 1, y0 + 3.5) - bprism(zh, "y", y0 - 1, y0 + t + 1)
    out.append(made("left_shoulder_flange", L, "al7075", flange))
    # the ring sits on the flange's outer face (one of its eight screws gives way to the flange),
    # and stays 1 mm inside the swing of the shoulder roll link's top plate (r = 53.75 mm)
    r_out = -(j["left_shoulder_yaw"][2] + t06 / 2 + 6.0) - 1.0
    r_sk = cut(disc(2 * r_out, (ry, rz)), rect(y0 + t - 100, y0 + t, rz - 100, rz + 100, r=0))
    r_pins, _, _, r_zp = zero_stop("RS03", roll, "x", "left_shoulder_roll", True, *PIN_ROLL)
    r_ring, _ = front_ring("RS03", roll, "x", 1, t=rt, outline=r_sk, keep=r_zp)
    r_ring = r_ring - bprism(r_pins, "x", r_front - 5, r_front + 1)
    out.append(
        made(
            "left_shoulder_roll_ring",
            L,
            "al7075",
            r_ring,
            ifaces=[ear_face("left_shoulder_roll", "RS03", roll, "x", 1)],
        )
    )

    # --- shoulder roll link: frame on the roll axis (roll RS03 centred here); a hub on its
    # output through the ring, the front plate on the hub; the shoulder yaw RS06 is centred at ym
    # below, axis z, output down: a ring on its output-end ears, four strips round it up to the
    # top plate over its driver cap. The front plate and the top plate make an L (no room for
    # gussets: the roll motor fills the corner).
    L = "left_shoulder_roll_link"
    rhub, face, r_relief = out_hub("RS03", (0, 0, 0), "x", 1, spec("RS03")["ear"] + rt, d=58.0)
    out.append(
        made(
            "left_shoulder_roll_hub",
            L,
            "al7075",
            rhub,
            ifaces=[output_face("left_shoulder_roll", "RS03", (0, 0, 0), "x", 1)],
        )
    )
    ym = j["left_shoulder_yaw"]
    yt = 5.0  # the yaw motor's ring
    tt = 2.5
    top_z0 = ym[2] + t06 / 2  # the top plate's underside, on the yaw motor's driver cap
    # two strips behind the yaw motor; in front, the front plate's two legs come down to its ring
    y_strips, y_pads, y_fea = cage("RS06", ym, "z", -1, to=t06 / 2, angles=(135.0, 225.0))
    ring_z = ym[2] - spec("RS06")["ear"]  # the yaw ring's rear face (the ears' face)
    # each leg widens outward into a foot on the ring
    legs = [rect(s * 30.0, s * 46.0, ring_z, top_z0 + 8) for s in (1, -1)]
    legs += [rect(s * 30.0, s * 56.0, ring_z, ring_z + 14) for s in (1, -1)]
    legs.append(rect(-46.0, 46.0, top_z0, top_z0 + 8))  # joined along the plate's lower edge
    fb = 10.0  # the front plate (7075): its legs stand on the yaw ring (cad/fea.py)
    # (86 mm: clear of the shoulder flange, 44 mm from the roll axis)
    front = rounded(union2(disc(86), rect(-28, 28, top_z0, 0), *legs), inside=8.0)
    front = cut(front, lighten(front, keep=[disc(86)], hole=14, pitch=20, rim=7))
    front = cut(front, output_windows("RS03", (0, 0, 0), "x", 21.0, 33.0))
    front_sk = front
    front, _ = output_plate("RS03", front, (0, 0, 0), "x", 1, fb, at=face, d=r_relief)
    _, groove, zh, _ = zero_stop("RS03", (0, 0, 0), "x", "left_shoulder_roll", True, *PIN_ROLL)
    front = (
        front - bprism(groove, "x", face - 1, face + 4) - bprism(zh, "x", face - 1, face + fb + 1)
    )
    out.append(made("left_shoulder_roll_front", L, "al7075", front))
    # the shoulder roll and yaw loops' harness clip, on the front plate's outer edge
    out.append(
        edge_clip(
            "left_shoulder_roll_front_clip", L, "x", face + fb / 2, fb, front_sk, (-60, top_z0 + 10)
        )
    )
    yxy = (ym[0], ym[1])
    behind = hull(disc(90.0, yxy), y_pads) & rect(-100, ym[0] - 15, -100, 100, r=0)  # the strips
    top = rounded(union2(hull(disc(90.0, yxy), rect(0, face, -28, 28)), behind))
    top = cut(
        top,
        rect(face, face + 100, -100, 100, r=0),  # the front plate's (the strips end under it)
        disc(24, (ym[0], ym[1])),  # the yaw motor's cable
        lighten(top, keep=[y_pads, disc(60, (ym[0], ym[1]))], pitch=20, rim=6),
    )
    out.append(made("left_shoulder_roll_top", L, "al7075", bplate(top, "z", top_z0 + tt / 2, tt)))
    out.append(made("left_shoulder_yaw_cage", L, "al7075", y_strips, extra={"fea_solid": y_fea}))
    leg_pads = [rect(face, face + fb, s * 30.0, s * 56.0) for s in (1, -1)]
    y_sk = rounded(union2(disc(92, (ym[0], ym[1])), y_pads, *leg_pads))
    y_pins, _, _, y_zp = zero_stop("RS06", ym, "z", "left_shoulder_yaw", True, *PIN_06)
    y_ring, y_front = front_ring("RS06", ym, "z", -1, t=yt, outline=y_sk, pocket=False)
    yf = ym[2] - y_front
    y_ring = y_ring - bprism(y_pins, "z", yf - 1, yf + 4)
    # 4 mm deeper under the front plate's legs, outside the upper arm's ring
    y_ring = y_ring + bprism(
        cut(
            union2(*[rect(face, face + fb, s * 26.0, s * 56.0) for s in (1, -1)]),
            disc(90, (ym[0], ym[1])),
        ),
        "z",
        yf - 4,
        yf,
    )
    out.append(
        made(
            "left_shoulder_yaw_ring",
            L,
            "al7075",
            y_ring,
            ifaces=[ear_face("left_shoulder_yaw", "RS06", ym, "z", -1)],
        )
    )

    # --- upper arm: frame on the yaw joint (the yaw RS06 centred at um, output down): a hub on
    # its output through the ring, a ring on the hub, and from the ring down the arm's outer side
    # a strap that is the elbow RS06's ring (its output-end ears, output outward +y); a printed
    # cover round it all. The strap runs down behind the elbow, so the forearm folds clear of it
    L = "left_upper_arm_link"
    el = j["left_elbow"]
    um = (0.0, 0.0, 0.0)
    uhub, u_top, u_relief = out_hub("RS06", um, "z", -1, spec("RS06")["ear"] + yt, d=46.0)
    out.append(
        made(
            "left_upper_arm_hub",
            L,
            "al7075",
            uhub,
            ifaces=[output_face("left_shoulder_yaw", "RS06", um, "z", -1)],
        )
    )
    ut = 4.0  # the ring on the yaw hub (7075)
    ring_top, _ = output_plate(
        "RS06",
        cut(disc(84), disc(16)),
        um,
        "z",
        -1,
        ut,
        at=u_top,
        d=u_relief,
    )
    _, groove, zh, _ = zero_stop("RS06", um, "z", "left_shoulder_yaw", True, *PIN_06)
    uf = um[2] - u_top  # the ring's face toward the yaw ring
    ring_top = ring_top - bprism(groove, "z", uf - 3, uf + 1) - bprism(zh, "z", uf - ut - 1, uf + 1)
    out.append(made("left_upper_arm_ring", L, "al7075", ring_top))
    ring_bot = um[2] - u_top - ut  # the ring's underside
    et = 6.0  # the strap (7075): a hand load bends it out of its plane (cad/fea.py)
    strap = rounded(
        union2(
            disc(96, xz(el)),
            slot(36, (-12, el[2]), (-12, ring_bot - 18)),
            rect(-30, 30, ring_bot - 22, ring_bot),
        )
    )
    e_pins, _, _, e_zp = zero_stop("RS06", el, "y", "left_elbow", True, *PIN_06)
    # harness clips: the shoulder yaw loop's, high on the strap, and the elbow loop's, behind it
    s_avoid = [disc(54, xz(el)), ring_band("RS06", el, "y"), e_zp]
    s_clips = [
        clip_spot(strap, (-12.0, ring_bot - 20), s_avoid, min_r=0),
        clip_spot(strap, xz(el), s_avoid, min_r=50),
    ]
    strap = cut(
        strap,
        lighten(
            strap,
            keep=[ring_band("RS06", el, "y"), rect(-30, 30, ring_bot - 12, ring_bot)]
            + [disc(CLIP_PAD, c) for c in s_clips],
        ),
        *[disc(CLIP_HOLE, c) for c in s_clips],
    )
    strap, s_front = front_ring("RS06", el, "y", 1, t=et, outline=strap, pocket=False)
    strap = strap - bprism(e_pins, "y", el[1] + s_front - 4, el[1] + s_front + 1)
    out.append(
        made(
            "left_upper_arm_strap",
            L,
            "al7075",
            strap,
            ifaces=[ear_face("left_elbow", "RS06", el, "y", 1)],
        )
    )

    # --- forearm: frame on the elbow joint (elbow RS06 centred here, output +y): a hub through
    # the strap and a flange on it; the wrist RS00 is centred on the wrist joint below, axis z,
    # output down
    wr = j["left_wrist_yaw"]
    fhub, out_y, f_relief = out_hub("RS06", (0, 0, 0), "y", 1, spec("RS06")["ear"] + et, d=46.0)
    out.append(
        made(
            "left_forearm_hub",
            "left_forearm_link",
            "al7075",
            fhub,
            ifaces=[output_face("left_elbow", "RS06", (0, 0, 0), "y", 1)],
        )
    )
    # the frame: a 6 mm strap on the hub, down the outside of the forearm (outboard of the upper
    # arm's strap) to a bracket on the wrist RS00's rear face, by its 4 x M3 rear holes
    ft = 5.0
    cup_top = wr[2] + t00 / 2  # the wrist motor's rear face
    bt = 3.5  # the bracket
    fy0, fy1 = out_y, out_y + ft
    foot = rect(-22, 22, cup_top + bt, cup_top + bt + 16)
    frame = rounded(union2(disc(72), slot(30, (0, -10), (0, cup_top + bt + 12)), foot), inside=8.0)
    _, groove, zh, zpads = zero_stop("RS06", (0, 0, 0), "y", "left_elbow", True, *PIN_06)
    # harness clips: the elbow loop's, below the elbow, and the wrist yaw loop's, low down
    f_avoid = [disc(76), zpads, foot]
    fcs = [
        clip_spot(frame, (0.0, 0.0), f_avoid, min_r=46),
        clip_spot(frame, (0.0, cup_top + bt + 16), f_avoid, min_r=0),
    ]
    frame = cut(
        frame,
        lighten(frame, keep=[*f_avoid, *[disc(CLIP_PAD, c) for c in fcs]], pitch=16, rim=5),
        *[disc(CLIP_HOLE, c) for c in fcs],
    )
    frame = cut(frame, rect(-100, 100, cup_top - 100, cup_top + bt, r=0))  # on the bracket
    frame, _ = output_plate("RS06", frame, (0, 0, 0), "y", 1, ft, at=out_y, d=f_relief)
    frame = frame - bprism(groove, "y", fy0 - 1, fy0 + 3) - bprism(zh, "y", fy0 - 1, fy1 + 1)
    out.append(made("left_forearm_frame", "left_forearm_link", "al7075", frame))
    brk = rounded(union2(disc(d00 + 6), rect(-22, 22, 0, fy1)))
    rear_pcd = _acts["RS00"]["holes"]["rear"]["pcd"]
    rear_holes = circles(3.4, circle_pts(4, rear_pcd, phase=np.pi / 4))
    brk = cut(brk, disc(20), rear_holes)
    bracket = bplate(brk, "z", cup_top + bt / 2, bt)
    # for cad/fea.py: 1 mm up into the strap's foot (faces that only touch don't always mesh as
    # one)
    tab = bprism(
        rect(-21.5, 21.5, fy0 + 0.5, fy1 - 0.5, r=0), "z", cup_top + bt - 0.1, cup_top + bt + 1
    )
    out.append(
        made(
            "left_wrist_bracket",
            "left_forearm_link",
            "al7075",
            bracket,
            ifaces=[contact("left_wrist_yaw", (0, 0, cup_top), "z", 10, d00 / 2)],
            extra={"fea_solid": to_manifold(bracket.fuse(tab))},
        )
    )

    # --- wrist: frame on the wrist yaw joint (RS00 centred here, output face down). The pitch
    # servo hangs from the RS00's output in a printed cradle, its case running up from its shaft
    # (the pitch axis, along y); the spline comes out at +y
    wp = j["left_wrist_pitch"]
    w_out = -t00 / 2
    sv = _acts["STS3250"]["shape"]
    s_len, s_wid = sv["box"]
    s_lo, s_hi = sv["axial"]
    horn_t = 3.0
    # the cradle: a top plate on the RS00's output, two side plates along the case's sides down
    # past its upper mounting ears, and a back plate behind the case carrying the shoulder screw
    # the hand mount's idler arm turns on; flat 7075 plates
    hold = wp[2] + 8
    cx = s_wid / 2 + 1.5  # the side plates' mid-planes, against the case
    ct = 3.0
    top_c, _ = output_plate("RS00", disc(d00 - 2), (0, 0, 0), "z", -1, ct)
    out.append(made("left_wrist_cradle", "left_wrist_link", "al7075", top_c))
    top_under = w_out - ct
    for name, g in (("l", 1), ("r", -1)):
        side = bplate(rect(s_lo, 0, hold, top_under), "x", g * cx, ct)
        out.append(made(f"left_wrist_cradle_side_{name}", "left_wrist_link", "al7075", side))
    back_c = rect(-cx - ct / 2, cx + ct / 2, wp[2] - 10, top_under)
    bc = clip_spot(back_c, (0.0, top_under), [disc(4.2, (0, wp[2]))], min_r=0)
    back_c = cut(back_c, disc(4.2, (0, wp[2])), disc(CLIP_HOLE, bc))  # the wrist yaw loop's clip
    out.append(
        made(
            "left_wrist_cradle_back",
            "left_wrist_link",
            "al7075",
            bplate(back_c, "y", s_lo - ct / 2, ct),
        )
    )
    pin_y = s_lo - ct
    out.append(
        Part(
            "left_wrist_idler_pin",
            "left_wrist_link",
            "bought",
            cyl(8, 9, "y", (0, pin_y - 4.5, wp[2])),
            "bought",
            mass_g=4.0,
            notes="M4 shoulder screw, 8 mm shoulder: the hand mount's idler pivot",
        )
    )

    # --- hand: frame on the wrist pitch axis. A printed U round the servo's lower end, on the
    # horn (+y) and the idler pin (-y), with a plate below for the hand's wrist interface: its
    # round flange (49 mm) centred under the forearm, and its six M3 clearance holes
    plate_z = -22.0  # the plate's lower face; the case's lower corners swing within 17 mm
    at = 3.0  # the arms (7075)
    hy = s_hi + horn_t + at / 2  # the horn arm's mid-plane, on the horn
    py = pin_y - 9 + at / 2  # the idler arm's, on the shoulder screw's shoulder
    horn_arm = hull(disc(22), rect(-11, 11, plate_z + 4, plate_z + 5))
    out.append(
        made("left_hand_arm_horn", "left_palm_link", "al7075", bplate(horn_arm, "y", hy, at))
    )
    idle_arm = cut(hull(disc(16), rect(-11, 11, plate_z + 4, plate_z + 5)), disc(8.2))
    out.append(
        made("left_hand_arm_idler", "left_palm_link", "al7075", bplate(idle_arm, "y", py, at))
    )
    plate = union2(rect(-25, 25, py - at / 2, hy + at / 2), disc(HAND_FLANGE + 4))
    holes = [
        (HAND_PCD / 2 * math.cos(math.radians(a)), HAND_PCD / 2 * math.sin(math.radians(a)))
        for a in HAND_HOLES
    ]
    plate = cut(plate, circles(3.2, holes))
    out.append(
        made("left_hand_mount", "left_palm_link", "al7075", bplate(plate, "z", plate_z + 2, 4.0))
    )
    out.append(hand("left", plate_z))
    return out


def hand(side: str, plate_z: float) -> Part:
    """The bought AmazingHand (0.40 kg as published; the solid is tools/build_hand_mesh.py's
    output, in the hand's frame): turned fingers down on the plate's lower face, palm forward
    (+x), so the thumb points outwards."""
    import trimesh

    mesh = trimesh.load(HAND_MESHES / f"{side}_hand.stl")
    solid = mf.Manifold(
        mf.Mesh(
            vert_properties=np.asarray(mesh.vertices * 1000, np.float32),
            tri_verts=np.asarray(mesh.faces, np.uint32),
        )
    )
    # half a turn about x: fingers down, y reversed; the interface face on the plate
    solid = solid.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, plate_z]])
    return Part(
        f"{side}_hand",
        f"{side}_palm_link",
        "bought",
        solid,
        "bought",
        mass_g=400.0,
        notes="AmazingHand by Pollen Robotics (CC-BY-4.0), description/third_party/amazing_hand",
    )


def all_parts() -> list[Part]:
    """Both arms: the right is the left's mirror image, except the right hand, which is the
    real right hand (its fingers are the same parts as the left's, not mirrored)."""
    left = parts()
    right = [p.mirrored() for p in left if p.name != "left_hand"]
    plate_z = next(p for p in left if p.name == "left_hand_mount").solid.bounding_box()[2]
    return left + right + [hand("right", plate_z)]
