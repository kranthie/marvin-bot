"""Arm parts, first pass: aluminium motor brackets, printed structural shells, 1-DOF gripper.

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
- wrist and gripper: a printed palm on the wrist output, carrying the gripper servo and a fixed
  jaw; a moving jaw on the servo horn.

The shells are hollow (2.5 mm walls) and leave a cable path from the shoulder to the gripper:
each joint's harness passes beside the motor in a service loop (docs/design/assembly_wiring.md).
No fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .lib import Part, box, cyl, hull, lighten, strap, union
from .mounts import bolt_holes, output_flange, rear_ring, spec
from .mounts import posts as mount_posts

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
WALL = 2.5  # printed shells


def env(model: str) -> tuple[float, float]:
    d, _, t = _acts[model]["envelope_mm"]
    return d, t


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
    # face, y0; the roll RS03 is centred at `roll`, its axis along x
    sp = j["left_shoulder_pitch"]
    y0 = mounts["left_shoulder_pitch"][1] + t03 / 2 - sp[1]  # output face (this frame)
    roll = j["left_shoulder_roll"]
    roll_clear = cyl(d03 + 3, t03 + 2, "x", roll)
    tb = 7.0  # shoulder back and roll front plates: SF 2 in the hand case (cad/strength.py)
    # the roll motor's stator bolts through its ears (Ø98) to a ring on its body's rear face,
    # its driver cap through the middle; the two ears next to the pitch flange stay unused
    back_x = roll[0] - t03 / 2 - tb / 2  # over the cap: a solid plate carries the hand load
    flange = output_flange("RS03", (0, y0 - t03 / 2, 0), "y", 1, 76, t)[0] - roll_clear
    back = union(
        cyl(108, tb, "x", (back_x, roll[1], roll[2])),
        strap(40, tb, "x", (back_x, roll[1], roll[2]), (back_x, y0 + 20, -12)),
        box((tb, 34, 40), (back_x, y0 + 17, -12)),
    )
    back = lighten(
        back,
        "x",
        keep=[cyl(30, 40, "x", (back_x, roll[1], roll[2])), box((40, 300, 30), (back_x, 0, -15))],
        hole=14,
        pitch=22,
        rim=10,
    )  # solid along the top, where the hand load's moment peaks
    sr_ring, _ = rear_ring("RS03", roll, "x", 1, outer=108.0)
    sr_ring = sr_ring - box((40, 200, 200), (roll[0] - 20, y0 + t + 1 - 100, roll[2]))
    sr_mount = union(sr_ring, mount_posts("RS03", roll, "x", 1, 4.0, t03 / 2))
    out.append(
        Part("left_shoulder_roll_mount", "left_shoulder_pitch_link", "al6061", sr_mount, "cnc")
    )
    flange = lighten(flange, "y", keep=[cyl(56, 40, "y", (0, y0, 0))], hole=12, pitch=20, rim=6)
    # two plates bolted at the corner (an L in one piece is too tall for 60 mm stock)
    out.append(Part("left_shoulder_flange", "left_shoulder_pitch_link", "al6061", flange, "cnc"))
    out.append(Part("left_shoulder_back", "left_shoulder_pitch_link", "al6061", back, "cnc"))

    # --- shoulder roll link: frame on the roll axis (roll RS03 centred here); output face at
    # +x; the shoulder yaw RS06 is centred on the yaw joint below, axis z, output down
    face = t03 / 2
    yaw = j["left_shoulder_yaw"]
    yaw_top = yaw[2] + t06 / 2  # the yaw motor's cap top; posts down to a ring on its body
    front = union(
        output_flange("RS03", (0, 0, 0), "x", 1, 72, tb)[0],
        strap(56, tb, "x", (face + tb / 2, 0, 0), (face + tb / 2, 0, yaw_top + 30)),
        box((tb, 72, t), (face + tb / 2, 0, yaw_top + t / 2)),
    )
    front = lighten(front, "x", keep=[cyl(56, 40, "x", (face, 0, 0))], hole=16, pitch=24, rim=7)
    top = hull(
        cyl(d06 - 6, t, "z", (0, 0, yaw_top + t / 2)),
        box((tb, 72, t), (face + tb / 2, 0, yaw_top + t / 2)),
    )
    top = lighten(top, "z", keep=[cyl(d06 + 4, 40, "z", (0, 0, yaw_top))], hole=16, rim=8)
    yr, _ = rear_ring("RS06", yaw, "z", -1)
    y_mount = union(yr, mount_posts("RS06", yaw, "z", -1, 4.0, t06 / 2))
    out.append(Part("left_shoulder_yaw_mount", "left_shoulder_roll_link", "al6061", y_mount, "cnc"))
    top = top - box(
        (tb + 0.2, 80, t + 2), (face + tb / 2, 0, yaw_top + t / 2)
    )  # bolts to the front
    out.append(Part("left_shoulder_roll_front", "left_shoulder_roll_link", "al6061", front, "cnc"))
    out.append(Part("left_shoulder_roll_top", "left_shoulder_roll_link", "al6061", top, "cnc"))

    # --- upper arm: frame on the yaw joint (yaw RS06 centred here, output face at -t06/2); the
    # elbow RS06 is centred on the elbow joint, axis y, its stator face inward (-y)
    el = j["left_elbow"]
    yaw_out = -t06 / 2
    ring_top = output_flange("RS06", (0, 0, 0), "z", -1, 70, t)[0] - cyl(
        16, 20, "z", (0, 0, yaw_out)
    )
    out.append(Part("left_upper_arm_ring", "left_upper_arm_link", "al6061", ring_top, "cnc"))
    stator_y = el[1] - spec("RS06")["rear"] - t / 2  # on the elbow motor's body rear face
    ring_el = cyl(d06 + 11, t, "y", (el[0], stator_y, el[2]))
    eh = spec("RS06")["holes"]["stator"]
    ring_el = ring_el - cyl(spec("RS06")["cap_d"] + 2, t + 2, "y", (el[0], stator_y, el[2]))
    ring_el = ring_el - bolt_holes(
        eh["n"], eh["thread"], eh["pcd"], "y", (el[0], stator_y, el[2]), t + 2
    )
    out.append(Part("left_upper_arm_elbow_ring", "left_upper_arm_link", "al6061", ring_el, "cnc"))
    # shell: a rounded tube from the top ring to around the elbow motor's inner half; it
    # carries the forearm and hand, so 3.5 mm walls (cad/strength.py)
    UW = 3.5
    top_z = yaw_out - t
    o = hull(
        cyl(78, 2, "z", (0, 0, top_z - 1)),
        cyl(d06 + 2 * UW + 2, t06 / 2 + 8, "y", (el[0], el[1] - t06 / 4 - 4, el[2])),
    )
    i = hull(
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
    flange_f = output_flange("RS06", (0, 0, 0), "y", 1, 72, t)[0]
    out.append(Part("left_forearm_flange", "left_forearm_link", "al6061", flange_f, "cnc"))
    cup_top = wr[2] + t00 / 2
    o = hull(
        cyl(68, 6, "y", (0, out_y + t + 3, -6)),
        cyl(d00 + 2 * WALL, t00 * 0.6, "z", (0, 0, cup_top - t00 * 0.3)),
        cyl(d00 + 2 * WALL + 4, 4, "z", (0, 0, cup_top + 20)),
    )
    i = hull(
        cyl(68 - 2 * WALL, 6 - 2 * WALL, "y", (0, out_y + t + 3, -6)),
        cyl(d00 + 0.6, t00 * 0.6 + 2, "z", (0, 0, cup_top - t00 * 0.3 - WALL)),
        cyl(d00 + 4, 4, "z", (0, 0, cup_top + 20)),
    )
    fore = o - i - cyl(d06 + 4, t06 + 4, "y", (0, 0, 0)) - cyl(d00 + 0.6, t00, "z", wr)
    fore = union(
        fore,
        cyl(d00 + 2 * WALL, WALL, "z", (0, 0, cup_top + WALL / 2))
        - cyl(20, 10, "z", (0, 0, cup_top)),
    )
    out.append(Part("left_forearm_shell", "left_forearm_link", "pa_cf", fore, "print"))

    # --- wrist: frame on the wrist joint (RS00 centred here, output face down); the gripper
    # servo sits on the gripper joint, its axis along y
    gr = j["left_gripper"]
    w_out = -t00 / 2
    sv = _acts["STS3250"]["shape"]
    s_len, s_wid = sv["box"]
    s_lo, s_hi = sv["axial"]
    s_off = sv["shaft_offset"]
    horn_t = 3.0
    # the servo's case runs down from its shaft (robot.yaml: length -z), inside the palm
    c_top, c_bot = gr[2] - s_off + s_len / 2, gr[2] - s_off - s_len / 2
    ymid = (s_hi + s_lo) / 2
    palm_o = hull(
        cyl(d00 - 2, 4, "z", (0, 0, w_out - 2)),
        box(
            (s_wid + 2 * WALL + 2, s_hi - s_lo + 2 * WALL + 2, s_len + 2 * WALL),
            (0, ymid, (c_top + c_bot) / 2),
        ),
    )
    palm_i = hull(
        cyl(d00 - 2 - 2 * WALL, 2, "z", (0, 0, w_out - 4 - WALL)),
        box((s_wid + 2, s_hi - s_lo + 2, s_len), (0, ymid, (c_top + c_bot) / 2)),
    )
    palm = palm_o - palm_i
    palm = palm - cyl(14, 40, "y", (0, s_hi, gr[2]))  # the spline and horn come out of the side
    jaw_fixed = box((8, 40, 60), (-s_wid / 2 - WALL - 5, ymid, c_bot - 25)) - box(
        (10, 30, 40), (-s_wid / 2 - WALL - 5, ymid, c_bot - 20)
    )
    out.append(
        Part("left_gripper_palm", "left_wrist_link", "pa_cf", union(palm, jaw_fixed), "print")
    )
    # moving jaw on the servo horn (+y, beside the palm), swinging about the servo axis
    hy = s_hi + horn_t + 2.5
    drop = c_bot - gr[2]  # the palm's bottom, below the shaft
    jx = s_wid / 2 + 8
    jaw = union(
        hull(cyl(24, 5, "y", (0, hy, 0)), box((10, 5, 10), (jx, hy, -20))),
        box((10, 5, -drop + 30), (jx, hy, (drop - 30) / 2)),
        box((10, 36, 40), (jx, hy + 2.5 - 18, drop - 50)),
    )
    out.append(Part("left_gripper_jaw", "left_gripper_link", "pa_cf", jaw, "print"))
    return out


def all_parts() -> list[Part]:
    left = parts()
    return left + [p.mirrored() for p in left]
