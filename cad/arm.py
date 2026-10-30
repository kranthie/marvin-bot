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

from .b3d import cut, disc, hull, lighten, made, rect, rounded
from .b3d import plate as bplate
from .b3d import union as union2
from .lib import Part, box, cyl, union
from .lib import hull as mhull
from .mounts import face as contact
from .mounts import (
    output_face,
    output_plate,
    pillar_ring,
    ring_band,
    spec,
    stator_face,
    stator_holes,
)

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

    # --- shoulder pitch link: frame on the pitch axis; the pitch RS03 (torso) ends at its output
    # face, y0; the roll RS03 is centred at `roll`, its axis along x. The roll motor's body comes
    # within 0.3 mm of the pitch output's face, so the flange is cut round it below the axis
    # (it keeps the output's upper bolts and pins) and reaches back behind the roll motor to a
    # back plate on the roll motor's pillar ring. Everything here stays within 54 mm of the roll
    # axis behind the motor: the shoulder roll link's top plate swings round outside that.
    L = "left_shoulder_pitch_link"
    sp = j["left_shoulder_pitch"]
    y0 = mounts["left_shoulder_pitch"][1] + t03 / 2 - sp[1]  # output face (this frame)
    roll = j["left_shoulder_roll"]
    tb = 10.0  # the back plate
    r_ring, r_near = pillar_ring("RS03", roll, "x", 1, od=108.0)
    back_x = roll[0] - r_near - tb / 2
    ry, rz = roll[1], roll[2]
    # the roll motor's body (r = 50 mm, 1 mm clear) where it crosses the flange's plate
    cut_z = math.sqrt(51.0**2 - (ry - (y0 + t)) ** 2)
    fl = rounded(union2(disc(76), rect(back_x - tb / 2, -20, rz - 30, 8)))
    fl = cut(fl, rect(-t03 / 2 - 1, t03 / 2 + 1, rz - cut_z, rz + cut_z))
    flange, _ = output_plate("RS03", fl, (0, y0 - t03 / 2, 0), "y", 1, t)
    out.append(
        made(
            "left_shoulder_flange",
            L,
            "al7075",
            flange,
            ifaces=[output_face("left_shoulder_pitch", "RS03", (0, y0 - t03 / 2, 0), "y", 1)],
        )
    )
    back = rounded(hull(disc(108, (ry, rz)), rect(y0 + t, y0 + t + 10, rz - 27, rz + 25)))
    back = cut(
        back,
        stator_holes("RS03", roll, "x"),
        lighten(back, keep=[ring_band("RS03", roll, "x")], pitch=20, rim=6),
    )
    out.append(made("left_shoulder_back", L, "al7075", bplate(back, "x", back_x, tb)))
    out.append(
        made(
            "left_shoulder_roll_mount",
            L,
            "al7075",
            r_ring,
            ifaces=[stator_face("left_shoulder_roll", "RS03", roll, "x", 1)],
        )
    )

    # --- shoulder roll link: frame on the roll axis (roll RS03 centred here); output face at
    # +x; the shoulder yaw RS06 is centred on the yaw joint below, axis z, output down, on a
    # pillar ring under the top plate. A front plate on the roll output and the top plate make
    # an L (no room for gussets: the roll motor fills the corner).
    L = "left_shoulder_roll_link"
    face = t03 / 2
    yaw = j["left_shoulder_yaw"]
    y_ring, y_near = pillar_ring("RS06", yaw, "z", -1, clear=0.0, od=90.0)
    tt = 6.0
    top_z0 = yaw[2] + y_near  # the top plate's underside
    fb = 8.0  # the front plate (7075)
    front = rounded(union2(disc(72), rect(-28, 28, top_z0, 0)))
    front = cut(front, lighten(front, keep=[disc(56)], hole=14, pitch=20, rim=7))
    front, _ = output_plate("RS03", front, (0, 0, 0), "x", 1, fb)
    out.append(
        made(
            "left_shoulder_roll_front",
            L,
            "al7075",
            front,
            ifaces=[output_face("left_shoulder_roll", "RS03", (0, 0, 0), "x", 1)],
        )
    )
    top = rounded(hull(disc(90.0, (yaw[0], yaw[1])), rect(0, face, -28, 28)))
    top = cut(
        top,
        stator_holes("RS06", yaw, "z"),
        lighten(top, keep=[ring_band("RS06", yaw, "z")], pitch=20, rim=6),
    )
    out.append(made("left_shoulder_roll_top", L, "al7075", bplate(top, "z", top_z0 + tt / 2, tt)))
    out.append(
        made(
            "left_shoulder_yaw_mount",
            L,
            "al6061",
            y_ring,
            ifaces=[stator_face("left_shoulder_yaw", "RS06", yaw, "z", -1)],
        )
    )

    # --- upper arm: frame on the yaw joint (yaw RS06 centred here, output face at -t06/2); the
    # elbow RS06 is centred on the elbow joint, axis y, its stator face inward (-y). The ring on
    # the yaw output is wider than the shell (84 mm) so the shell bears on it
    L = "left_upper_arm_link"
    el = j["left_elbow"]
    yaw_out = -t06 / 2
    ring_top, _ = output_plate("RS06", cut(disc(84), disc(16)), (0, 0, 0), "z", -1, t)
    out.append(
        made(
            "left_upper_arm_ring",
            L,
            "al6061",
            ring_top,
            ifaces=[output_face("left_shoulder_yaw", "RS06", (0, 0, 0), "z", -1)],
        )
    )
    et = 4.0  # the elbow ring
    stator_y = el[1] - spec("RS06")["rear"] - et / 2  # on the elbow motor's body rear face
    ring_el = cut(
        disc(d06 + 11, xz(el)),
        disc(spec("RS06")["cap_d"] + 2, xz(el)),
        stator_holes("RS06", el, "y"),
    )
    out.append(
        made(
            "left_upper_arm_elbow_ring",
            L,
            "al6061",
            bplate(ring_el, "y", stator_y, et),
            ifaces=[stator_face("left_elbow", "RS06", el, "y", 1)],
        )
    )
    # shell: a rounded tube from the top ring to around the elbow motor's inner half; it
    # carries the forearm and hand, so 3.5 mm walls (cad/fea.py)
    UW = 3.5
    top_z = yaw_out - t
    o = mhull(
        cyl(78, 2, "z", (0, 0, top_z - 1)),
        cyl(d06 + 2 * UW + 2, t06 / 2 + 8, "y", (el[0], el[1] - t06 / 4 - 4, el[2])),
    )
    i = mhull(
        cyl(78 - 2 * UW, 4, "z", (0, 0, top_z)),
        cyl(d06 + 2, t06 / 2 + 8 - 2 * UW, "y", (el[0], el[1] - t06 / 4 - 4 + UW, el[2])),
    )
    upper = o - i - cyl(d06 + 3, t06 + 2, "y", el) - box((200, 200, 40), (0, 0, el[2] - 70))
    upper = upper - box((40, 20, 20), (-30, -10, el[2] + 30))  # harness exit to the elbow loop
    # a relief at the front above the elbow, where the forearm folds against it
    upper = upper - box((100, 200, 140), (15 + 50, 0, el[2] + 5))
    out.append(Part("left_upper_arm_shell", "left_upper_arm_link", "pa_cf", upper, "print"))

    # --- forearm: frame on the elbow joint (elbow RS06 centred here, output face +y); the
    # wrist RS00 is centred on the wrist joint below, axis z, output down
    wr = j["left_wrist_yaw"]
    out_y = t06 / 2
    ft = 5.0  # the forearm flange; the shell starts on its outer face
    flange_f, _ = output_plate("RS06", disc(72), (0, 0, 0), "y", 1, ft)
    out.append(
        made(
            "left_forearm_flange",
            "left_forearm_link",
            "al6061",
            flange_f,
            ifaces=[output_face("left_elbow", "RS06", (0, 0, 0), "y", 1)],
        )
    )
    cup_top = wr[2] + t00 / 2
    o = mhull(
        cyl(68, 6, "y", (0, out_y + ft + 3, -6)),
        cyl(d00 + 2 * WALL, t00 * 0.6, "z", (0, 0, cup_top - t00 * 0.3)),
        cyl(d00 + 2 * WALL + 4, 4, "z", (0, 0, cup_top + 20)),
    )
    i = mhull(
        cyl(68 - 2 * WALL, 6 - 2 * WALL, "y", (0, out_y + ft + 3, -6)),
        cyl(d00 + 0.6, t00 * 0.6 + 2, "z", (0, 0, cup_top - t00 * 0.3 - WALL)),
        cyl(d00 + 4, 4, "z", (0, 0, cup_top + 20)),
    )
    fore = o - i - cyl(d06 + 4, t06 + 4, "y", (0, 0, 0)) - cyl(d00 + 0.6, t00, "z", wr)
    fore = union(
        fore,
        cyl(d00 + 2 * WALL, WALL, "z", (0, 0, cup_top + WALL / 2))
        - cyl(20, 10, "z", (0, 0, cup_top)),
    )
    fs_if = [contact("left_wrist_yaw", (0, 0, cup_top), "z", 10, d00 / 2)]  # the wrist motor's seat
    out.append(
        Part(
            "left_forearm_shell",
            "left_forearm_link",
            "pa_cf",
            fore,
            "print",
            extra={"ifaces": fs_if},
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
    s_off = sv["shaft_offset"]
    horn_t = 3.0
    c_top, c_bot = wp[2] + s_off + s_len / 2, wp[2] + s_off - s_len / 2
    ymid = s_lo / 2  # the case runs from s_lo to 0 along y; the spline from 0 to s_hi
    case = box((s_wid, -s_lo, s_len), (0, ymid, (c_top + c_bot) / 2))
    # the cradle holds the case's upper half (its upper mounting ears), leaving the lower end
    # free for the hand mount to swing round
    hold = wp[2] + 8
    cradle_o = mhull(
        cyl(d00 - 2, 3, "z", (0, 0, w_out - 1.5)),
        box((s_wid + 2 * WALL, -s_lo + 2 * WALL, c_top - hold), (0, ymid, (c_top + hold) / 2)),
    )
    # an idler pin on the case's back (-y) face, on the pitch axis, for the mount's second arm
    pin = cyl(8, 6, "y", (0, s_lo - WALL - 3, wp[2]))
    rib = box((8, WALL, hold - wp[2] + 4), (0, s_lo - WALL / 2, (hold + wp[2]) / 2))
    cradle = union(cradle_o - case - box((s_wid - 6, -s_lo, 20), (0, ymid, hold)), pin, rib)
    out.append(Part("left_wrist_cradle", "left_wrist_link", "pa_cf", cradle, "print"))

    # --- hand: frame on the wrist pitch axis. A printed U round the servo's lower end, on the
    # horn (+y) and the idler pin (-y), with a plate below for the hand's wrist interface: its
    # round flange (49 mm) centred under the forearm, and its six M3 clearance holes
    plate_z = -22.0  # the plate's lower face; the case's lower corners swing within 17 mm
    hy = s_hi + horn_t
    py = s_lo - WALL - 6.5
    arm_horn = mhull(cyl(22, 4, "y", (0, hy + 2, 0)), box((22, 4, 4), (0, hy + 2, plate_z + 2)))
    arm_idle = mhull(cyl(16, 4, "y", (0, py, 0)), box((22, 4, 4), (0, py, plate_z + 2)))
    arm_idle = arm_idle - cyl(8.4, 6, "y", (0, py, 0))
    plate = union(
        box((50, hy + 4 - py + 4, 4), (0, (hy + 4 + py - 2) / 2, plate_z + 2)),
        cyl(HAND_FLANGE + 4, 4, "z", (0, 0, plate_z + 2)),
    )
    for a in HAND_HOLES:
        c = (HAND_PCD / 2 * math.cos(math.radians(a)), HAND_PCD / 2 * math.sin(math.radians(a)))
        plate = plate - cyl(3.2, 10, "z", (*c, plate_z + 2))
    mount = union(arm_horn, arm_idle, plate)
    out.append(Part("left_hand_mount", "left_palm_link", "pa_cf", mount, "print"))
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
