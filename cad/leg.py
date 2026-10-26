"""Leg parts, first pass: aluminium brackets and cores, printed shell, ankle linkage, foot.

Every part is built in its link's frame (mm), using the frames of description/skeleton.yaml
(left leg; the right leg is the mirror image). Motors sit where tools/build_urdf.py places
them (motor_mounts), at their STEP-model sizes, and the structure is built around them:

- hip: a flange on the hip pitch RS04's output, a side plate and a back plate holding the hip
  roll RS04 behind it;
- hip roll: an L-bracket from the roll output to the top of the hip yaw RS03;
- thigh: a flange on the yaw output and an outer side plate carrying the knee RS04, under a
  printed shell;
- shin: a flange on the knee output (inner side), a central web between the two ankle RS03
  (both bolt to it), and a fork carrying the ankle pitch bearings;
- ankle: a cross block, two cranks, two push rods with rod ends;
- foot: an aluminium plate with heel posts for the rod ends and lugs for the roll bearings, on a
  TPU sole.

A first pass for mass, inertia, part sizes and clearances: plates are pocketed with
lightening holes, but there are no fastener holes, fillets or bearing details yet, and no
strength check.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .lib import Part, box, cyl, hull, lighten, union

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"

_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())


def env(model: str) -> tuple[float, float]:
    """(diameter, thickness) mm of a motor housing."""
    d, _, t = _acts[model]["envelope_mm"]
    return d, t


def mm(v) -> list[float]:
    return [x * 1000 for x in v]


def parts() -> list[Part]:
    J = {k: mm(v["xyz"]) for k, v in _skel["joints"].items()}
    mounts = {k: mm(v["xyz"]) for k, v in _skel["motor_mounts"].items()}
    link = _skel["ankle_linkage"]["geometry"]
    d04, t04 = env("RS04")
    d03, t03 = env("RS03")
    plate = 6.0  # motor flanges (strength pass to confirm)
    web_t = 6.0  # side plates and webs, pocketed
    out: list[Part] = []

    # --- hip pitch link: frame on the hip pitch axis; the pitch RS04 is centred here (on the
    # pelvis), output face outward (+y)
    face = t04 / 2
    roll_c = mounts["left_hip_roll"]  # roll RS04 centre, behind the pitch motor
    roll_back = roll_c[0] - t04 / 2  # its back face (x)
    flange = cyl(80, plate, "y", (0, face + plate / 2, 0))
    out.append(Part("left_hip_pitch_flange", "left_hip_pitch_link", "al6061", flange, "cnc"))
    side_y = face + plate + web_t / 2
    roll_clear = cyl(d04 + 6, t04 + 8, "x", roll_c)
    side = (
        hull(
            cyl(80, web_t, "y", (0, side_y, 0)),
            box((16, web_t, 50), (roll_back - plate - 8, side_y, 5)),
        )
        - roll_clear
    )
    side = lighten(side, "y", keep=[cyl(90, 40, "y", (0, side_y, 0))])
    out.append(Part("left_hip_yoke_side", "left_hip_pitch_link", "al6061", side, "cnc"))
    back_x = roll_back - plate / 2
    back = hull(
        cyl(d04 - 10, web_t, "x", (back_x, roll_c[1], roll_c[2])),
        box((web_t, 14, 10), (back_x, side_y, 25)),
    )
    back = lighten(back, "x", keep=[cyl(60, 40, "x", (back_x, roll_c[1], roll_c[2]))])
    out.append(Part("left_hip_yoke_back", "left_hip_pitch_link", "al6061", back, "cnc"))

    # --- hip roll link: frame on the roll axis; roll output faces forward (+x) at
    # roll_c[0] + t04/2 relative to the roll joint; yaw RS03 centred on the yaw joint below
    rj = J["left_hip_roll"]
    out_x = roll_c[0] - rj[0] + t04 / 2  # roll output face, in this frame
    yaw_c = J["left_hip_yaw"]
    yaw_top = yaw_c[2] + t03 / 2
    front = hull(
        cyl(80, plate, "x", (out_x + plate / 2, 0, 0)),
        box((plate, 80, plate), (out_x + plate / 2, 0, yaw_top + plate / 2)),
    )
    front = lighten(front, "x", keep=[cyl(60, 40, "x", (out_x, 0, 0))], hole=16, pitch=24)
    top = hull(
        cyl(d03 + 4, web_t, "z", (0, 0, yaw_top + web_t / 2)),
        box((14, 80, web_t), (out_x + 7, 0, yaw_top + web_t / 2)),
    )
    top = lighten(top, "z", keep=[cyl(44, 40, "z", (0, 0, yaw_top))])
    # two plates bolted at the corner: an L machined from one block would need 80 mm stock
    out.append(Part("left_hip_roll_front", "left_hip_roll_link", "al6061", front, "cnc"))
    out.append(Part("left_hip_roll_top", "left_hip_roll_link", "al6061", top, "cnc"))

    # --- thigh: frame on the yaw axis at the yaw motor's centre; yaw output faces down
    yaw_out = -t03 / 2
    knee = J["left_knee"]
    tflange = hull(
        cyl(84, plate, "z", (0, 0, yaw_out - plate / 2)),
        box((60, 16, plate), (0, t04 / 2 + 2, yaw_out - plate / 2)),
    )
    out.append(Part("left_thigh_flange", "left_thigh_link", "al6061", tflange, "cnc"))
    py = t04 / 2 + web_t / 2  # outer side plate, against the knee RS04's outer face
    tplate = hull(
        box((60, web_t, 16), (0, py, yaw_out - 8)),
        cyl(d04 + 4, web_t, "y", (0, py, knee[2])),
    )
    tplate = lighten(tplate, "y", keep=[cyl(70, 40, "y", (0, py, knee[2]))])
    out.append(Part("left_thigh_plate", "left_thigh_link", "al6061", tplate, "cnc"))
    shell_top, shell_bot = yaw_out - 12, knee[2] + d04 / 2 + 4
    outer = hull(
        cyl(d03 + 16, 2, "z", (0, 0, shell_top)),
        box((d04 + 10, t04 + 26, 2), (0, 2, shell_bot)),
    )
    inner = hull(
        cyl(d03 + 10, 4, "z", (0, 0, shell_top)),
        box((d04 + 4, t04 + 20, 4), (0, 2, shell_bot)),
    )
    out.append(Part("left_thigh_shell", "left_thigh_link", "pa_cf", outer - inner, "print"))

    # --- shin: frame on the knee axis; knee output faces inward (-y)
    ka, kb = mounts["left_ankle_a"], mounts["left_ankle_b"]
    ankle_z = J["left_ankle_pitch"][2]
    fy = -t04 / 2 - plate / 2
    sflange = hull(
        cyl(84, plate, "y", (0, fy, 0)),
        box((50, plate, 15), (0, fy, -d04 / 2 - 30)),
    )
    sflange = lighten(sflange, "y", keep=[cyl(64, 40, "y", (0, fy, 0))], hole=16, pitch=22)
    tab = box((50, -fy + 3, 15), (0, fy / 2 + 1.5, -d04 / 2 - 30)) - box(
        (38, -fy - 9, 17), (0, fy / 2 + 1.5, -d04 / 2 - 30)
    )  # a U-channel, not a block
    out.append(
        Part("left_shin_knee_flange", "left_shin_link", "al6061", union(sflange, tab), "cnc")
    )
    web_top = -d04 / 2 - 30
    web = hull(
        box((50, 6, 15), (0, 0, web_top)),
        cyl(d03 - 10, 6, "y", (ka[0], 0, ka[2])),
        box((60, 6, 8), (0, 0, ankle_z + 45)),
    )
    web = lighten(web, "y", keep=[cyl(56, 40, "y", (ka[0], 0, ka[2]))])
    out.append(Part("left_shin_web", "left_shin_link", "al6061", web, "cnc"))
    fork_top = ankle_z + 45

    def cheek(s):  # a fork cheek with the ankle pitch bearing bore
        return hull(
            box((44, web_t, 10), (0, s * 46, fork_top - 5)),
            cyl(40, web_t, "y", (0, s * 46, ankle_z)),
        ) - cyl(22, 20, "y", (0, s * 46, ankle_z))

    top = lighten(
        box((60, 100, web_t), (0, 0, fork_top)), "z", keep=[box((60, 12, 20))], hole=18, pitch=24
    )
    fork = union(top, cheek(1), cheek(-1))
    out.append(Part("left_ankle_fork", "left_shin_link", "al6061", fork, "cnc"))
    # cranks on the motors' outer faces, pointing back (and slightly down) at neutral
    th = math.pi + link["crank_tilt"]
    r = link["crank_radius"] * 1000
    for name, c, s in (("a", ka, 1), ("b", kb, -1)):
        cy = c[1] + s * (t03 / 2 + 3)
        tip = (c[0] + r * math.cos(th), cy, c[2] + r * math.sin(th))
        crank = hull(cyl(36, 6, "y", (c[0], cy, c[2])), cyl(18, 6, "y", tip))
        out.append(Part(f"left_ankle_crank_{name}", "left_shin_link", "al7075", crank, "cnc"))
        # rod from the crank tip to its ball joint on the foot (neutral pose, shin frame)
        foot_end = (
            -link["rod_back"] * 1000,
            s * link["rod_side"] * 1000,
            ankle_z - link["rod_below"] * 1000,
        )
        rod = hull(cyl(10, 1, "z", tip), cyl(10, 1, "z", foot_end))
        out.append(
            Part(
                f"left_ankle_rod_{name}",
                "left_shin_link",
                "bought",
                rod,
                "bought",
                mass_g=45.0,
                notes="steel tube with two M6 rod ends",
            )
        )

    # --- ankle cross and foot: frames on the ankle axes
    cross = box((44, 76, 30)) - box((32, 64, 32))  # hollow frame
    out.append(Part("left_ankle_cross", "left_ankle_link", "al7075", cross, "cnc"))
    foot = _skel["collisions"][[c["link"] for c in _skel["collisions"]].index("left_foot_link")]
    fl, fw, _ = mm(foot["size"])
    fx = foot["xyz"][0] * 1000
    sole_bot = (foot["xyz"][2] - foot["size"][2] / 2) * 1000
    sole = box((fl, fw, 4), (fx, 0, sole_bot + 2))
    out.append(Part("left_foot_sole", "left_foot_link", "tpu", sole, "print"))
    pz = sole_bot + 4 + 2.5
    posts = [
        box(
            (18, 18, -link["rod_below"] * 1000 - pz),
            (
                -link["rod_back"] * 1000,
                s * link["rod_side"] * 1000,
                (pz - link["rod_below"] * 1000) / 2,
            ),
        )
        for s in (1, -1)
    ]
    lugs = [
        box((8, 36, -pz + 12), (s * 32, 0, (pz + 12) / 2)) - cyl(18, 10, "x", (s * 32, 0, 0))
        for s in (1, -1)
    ]
    # the base is longer than the desktop mill's travel: a flat plate cut by an online
    # laser or water-jet service; posts and lugs are machined and bolted on
    fbase = lighten(box((fl - 10, fw - 10, 5), (fx, 0, pz)), "z", hole=28, pitch=32, rim=8)
    out.append(Part("left_foot_base", "left_foot_link", "al6061", fbase, "service"))
    for name, solid in (
        ("post_a", posts[0]),
        ("post_b", posts[1]),
        ("lug_front", lugs[0]),
        ("lug_back", lugs[1]),
    ):
        out.append(Part(f"left_foot_{name}", "left_foot_link", "al6061", solid, "cnc"))
    return out


def all_parts() -> list[Part]:
    left = parts()
    return left + [p.mirrored() for p in left]
