"""Leg parts, first pass: aluminium brackets and cores, printed shell, ankle linkage, foot.

Every part is built in its link's frame (mm), using the frames of description/skeleton.yaml
(left leg; the right leg is the mirror image). Motors sit where tools/build_urdf.py places
them (motor_mounts), at their STEP-model sizes, and the structure is built around them:

- hip: a flange on the hip pitch RS04's output, a side plate and a back plate holding the hip
  roll RS04 behind it;
- hip roll: an L-bracket from the roll output to the top of the hip yaw RS03;
- thigh: a flange on the yaw output and an outer side plate carrying the knee RS04, under a
  printed shell;
- shin: two side plates on the shin's centre line with the two ankle RS03 staggered between
  them (one high and forward under the knee, one low behind the ankle), each bolted to one
  plate; the plates' lower ends are the ankle fork cheeks;
- ankle: a cross block, two cranks, two push rods with rod ends;
- foot: an aluminium plate with heel posts for the rod ends and lugs for the roll bearings, on a
  TPU sole.

Plates are rings on the motors' bolt circles joined by straps, not full-width discs, sized by
cad/strength.py. A first pass for mass, inertia, part sizes and clearances: plates are pocketed with
lightening holes, but there are no fastener holes, fillets or bearing details yet.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .lib import Part, box, cyl, hull, lighten, strap, union

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
    plate = 6.0  # motor flanges
    web_t = 6.0  # side plates and webs, pocketed
    hip_t = 8.0  # hip back plate, with edge flanges: it carries the leg's landing moment
    brg_d, brg_w = 42.0, 7.0  # 6806 far-side bearing (30 x 42 x 7 mm)
    out: list[Part] = []

    # --- hip pitch link: frame on the hip pitch axis; the pitch RS04 is centred here (on the
    # pelvis), output face outward (+y)
    face = t04 / 2
    roll_c = mounts["left_hip_roll"]  # roll RS04 centre, behind the pitch motor
    roll_back = roll_c[0] - t04 / 2  # its back face (x)
    flange = cyl(80, plate, "y", (0, face + plate / 2, 0))
    out.append(Part("left_hip_pitch_flange", "left_hip_pitch_link", "al6061", flange, "cnc"))
    side_t = 8.0  # SF 2.7 in the landing case at 8 mm (cad/strength.py)
    side_y = face + plate + side_t / 2
    roll_clear = cyl(d04 + 6, t04 + 8, "x", roll_c)
    side = (
        hull(
            cyl(80, side_t, "y", (0, side_y, 0)),
            box((16, side_t, 50), (roll_back - hip_t - 8, side_y, 5)),
        )
        - roll_clear
    )
    side = lighten(side, "y", keep=[cyl(90, 40, "y", (0, side_y, 0))], hole=20, rim=12)
    out.append(Part("left_hip_yoke_side", "left_hip_pitch_link", "al6061", side, "cnc"))
    back_x = roll_back - hip_t / 2
    # the back plate also carries the roll joint's far-side stub axle (behind it, on the roll axis)
    stub = cyl(30, 14, "x", (back_x - hip_t / 2 - 7, roll_c[1], roll_c[2]))
    # full width (110 mm) up to the top: a narrow neck to the side plate was the weak point
    back = hull(
        cyl(100, hip_t, "x", (back_x, roll_c[1], roll_c[2])),
        box((hip_t, 100, 12), (back_x, roll_c[1], 24)),
    )
    # flanges along both vertical edges, pointing back: a channel section, which carries the
    # out-of-plane landing moment far better than the flat plate alone
    flanges = [
        box(
            (22, 6, 30 - roll_c[2]),
            (back_x - hip_t / 2 - 11, roll_c[1] + s * (d04 / 2 - 8), (30 + roll_c[2]) / 2),
        )
        for s in (1, -1)
    ]
    back = union(
        lighten(
            back, "x", keep=[cyl(70, 40, "x", (back_x, roll_c[1], roll_c[2]))], hole=18, rim=14
        ),
        stub,
        *flanges,
    )
    out.append(Part("left_hip_yoke_back", "left_hip_pitch_link", "al6061", back, "cnc"))

    # --- hip roll link: frame on the roll axis; roll output faces forward (+x) at
    # roll_c[0] + t04/2 relative to the roll joint; yaw RS03 centred on the yaw joint below
    rj = J["left_hip_roll"]
    out_x = roll_c[0] - rj[0] + t04 / 2  # roll output face, in this frame
    yaw_c = J["left_hip_yaw"]
    yaw_top = yaw_c[2] + t03 / 2
    # a U-yoke around the roll RS04: front plate on its output, rear plate on a far-side bearing
    # on the hip block's stub axle, a bar under the motor joining them
    under = -d04 / 2 - 8  # bar top, below the roll motor
    # a ring on the motor's output bolt circle and a strap down to the bar, not a full disc
    front = union(
        cyl(80, plate, "x", (out_x + plate / 2, 0, 0)),
        strap(48, plate, "x", (out_x + plate / 2, 0, 0), (out_x + plate / 2, 0, under + 10)),
        box((plate, 80, plate), (out_x + plate / 2, 0, under - plate / 2)),
    )
    front = lighten(front, "x", keep=[cyl(60, 40, "x", (out_x, 0, 0))], hole=14, pitch=20, rim=7)
    rear_x = (roll_c[0] - rj[0]) - t04 / 2 - hip_t - 14 - plate / 2
    rear = hull(
        cyl(brg_d + 16, plate, "x", (rear_x, 0, 0)),
        box((plate, 60, plate), (rear_x, 0, under - plate / 2)),
    ) - cyl(brg_d, plate + 2, "x", (rear_x, 0, 0))
    bar = box((out_x - rear_x + plate, 60, 10), ((out_x + rear_x) / 2, 0, under - 5))
    bar = bar - box(
        (out_x - rear_x - 10, 44, 12), ((out_x + rear_x) / 2, 0, under - 5 - 3)
    )  # channel
    out.append(Part("left_hip_roll_rear", "left_hip_roll_link", "al6061", rear, "cnc"))
    out.append(Part("left_hip_roll_bar", "left_hip_roll_link", "al6061", bar, "cnc"))
    out.append(
        Part(
            "left_hip_roll_bearing",
            "left_hip_roll_link",
            "bought",
            cyl(brg_d, brg_w, "x", (rear_x, 0, 0)) - cyl(30, brg_w + 2, "x", (rear_x, 0, 0)),
            "bought",
            mass_g=25.0,
            notes="6806-2RS far-side bearing",
        )
    )
    top = hull(
        cyl(d03 + 4, web_t, "z", (0, 0, yaw_top + web_t / 2)),
        box((14, 80, web_t), (out_x + 7, 0, yaw_top + web_t / 2)),
    )
    top = lighten(top, "z", keep=[cyl(44, 40, "z", (0, 0, yaw_top))], hole=24, pitch=28, rim=8)
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
    # a tapered strap from the yaw flange to a ring on the knee motor's stator bolt circle
    # (~86 mm, STEP model), instead of a plate as wide as the motor
    tplate = union(
        hull(box((60, web_t, 16), (0, py, yaw_out - 8)), cyl(46, web_t, "y", (0, py, knee[2]))),
        cyl(100, web_t, "y", (0, py, knee[2])),
    )
    tplate = lighten(tplate, "y", keep=[cyl(70, 40, "y", (0, py, knee[2]))], hole=16, pitch=22)
    # stub axle for the shin's far-side bearing, on the knee axis outside the plate
    tplate = union(tplate, cyl(30, 12, "y", (0, py + web_t / 2 + 6, knee[2])))
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

    # --- shin: frame on the knee axis; knee output faces inward (-y). Two side plates on the
    # shin's centre line hold the staggered ankle RS03 between them: a (low) bolts its stator to
    # the inner plate and its crank hub passes beside the outer plate (the plates are straps
    # routed clear of the other motor's hub); b (high, forward) the other way round. The plates
    # run down to become the ankle fork cheeks.
    ka, kb = mounts["left_ankle_a"], mounts["left_ankle_b"]
    ankle_z = J["left_ankle_pitch"][2]
    yi, yo = -(t03 / 2 + plate / 2), t03 / 2 + plate / 2  # inner and outer side plates
    ring03 = 86.0  # RS03 stator ring: its ~72 mm mounting-hole circle (STEP model) + margin
    hub = 40.0  # crank hub, spanning the gap from the motor's output face to the crank
    w = 30.0  # strap width (cad/strength.py sizes it)

    def cheek_at(y):  # the fork cheek: the plate's lower end, round the ankle bearing
        return (0.0, y, ankle_z)

    knee_clear = cyl(2 * 68, 200, "y")  # the thigh plate and its stub turn inside r = 68 mm
    bridge_c = (65.0, -45.0)  # bridge in front of the knee motor, joining the knee yoke
    inner = union(
        cyl(84, plate, "y", (0, yi, 0)),
        cyl(ring03, plate, "y", (ka[0], yi, ka[2])),
        strap(w, plate, "y", (0, yi, 0), (-20, yi, -110), (ka[0], yi, ka[2]), cheek_at(yi)),
        strap(30, plate, "y", (0, yi, 0), (bridge_c[0], yi, bridge_c[1])),
    ) - cyl(22, 20, "y", (0, yi, ankle_z))
    inner = lighten(
        inner,
        "y",
        keep=[cyl(66, 40, "y", (0, yi, 0)), cyl(64, 40, "y", (ka[0], yi, ka[2]))],
        hole=14,
        pitch=20,
        rim=8,
    )
    outer = (
        union(
            cyl(ring03, plate, "y", (kb[0], yo, kb[2])),
            strap(
                w,
                plate,
                "y",
                (bridge_c[0], yo, bridge_c[1]),
                (kb[0], yo, kb[2]),
                (kb[0], yo, ka[2] - 20),
                cheek_at(yo),
            ),
        )
        - knee_clear
        - cyl(22, 20, "y", (0, yo, ankle_z))
    )
    outer = lighten(
        outer, "y", keep=[cyl(64, 40, "y", (kb[0], yo, kb[2]))], hole=14, pitch=20, rim=8
    )
    out.append(Part("left_shin_inner", "left_shin_link", "al6061", inner, "service"))
    out.append(Part("left_shin_outer", "left_shin_link", "al6061", outer, "service"))
    # the knee's far side: a plate on a bearing on the thigh plate's stub axle, tied to both
    # side plates by a bridge in front of the knee motor
    oy = t04 / 2 + web_t + 2 + 6 + plate / 2  # outside the thigh plate and its stub
    outer_pl = union(
        cyl(brg_d + 16, plate, "y", (0, oy, 0)),
        strap(26, plate, "y", (0, oy, 0), (bridge_c[0], oy, bridge_c[1])),
    ) - cyl(brg_d, plate + 2, "y", (0, oy, 0))
    bridge = box((20, oy - yi + plate, 20), (bridge_c[0], (oy + yi) / 2, bridge_c[1])) - box(
        (14, oy - yi + plate + 2, 14), (bridge_c[0], (oy + yi) / 2, bridge_c[1])
    )  # a square tube
    out.append(Part("left_shin_knee_outer", "left_shin_link", "al6061", outer_pl, "cnc"))
    out.append(Part("left_shin_bridge", "left_shin_link", "al6061", bridge, "cnc"))
    out.append(
        Part(
            "left_knee_bearing",
            "left_shin_link",
            "bought",
            cyl(brg_d, brg_w, "y", (0, oy, 0)) - cyl(30, brg_w + 2, "y", (0, oy, 0)),
            "bought",
            mass_g=25.0,
            notes="6806-2RS far-side bearing",
        )
    )
    # spacer tubes between the side plates, clear of both motors and the ankle cross
    gap = yo - yi - plate
    for k, (sx, sz) in enumerate(((-50.0, -130.0), (55.0, -200.0), (-40.0, ankle_z + 35))):
        tube = cyl(16, gap, "y", (sx, 0, sz)) - cyl(8, gap + 2, "y", (sx, 0, sz))
        out.append(Part(f"left_shin_spacer_{k}", "left_shin_link", "al6061", tube, "cnc"))
    # cranks outside the side plates, pointing back (and slightly down) at neutral
    th = math.pi + link["crank_tilt"]
    r = link["crank_radius"] * 1000
    cy = link["crank_y"] * 1000
    for name, c, s in (("a", ka, 1), ("b", kb, -1)):
        tip = (c[0] + r * math.cos(th), s * cy, c[2] + r * math.sin(th))
        crank = union(
            hull(cyl(36, 6, "y", (c[0], s * cy, c[2])), cyl(18, 6, "y", tip)),
            cyl(hub, cy - t03 / 2 - 3, "y", (c[0], s * (t03 / 2 + cy - 3) / 2, c[2])),
        )
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
                mass_g=30.0 if name == "a" else 45.0,
                notes="steel tube with two M6 rod ends",
            )
        )
    # --- ankle cross and foot: frames on the ankle axes
    cross_w = t03 - 4  # between the shin's side plates
    cross = box((44, cross_w, 30)) - box((32, cross_w - 12, 32))  # hollow frame
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
    fbase = lighten(box((fl - 10, fw - 10, 5), (fx, 0, pz)), "z", hole=24, pitch=30, rim=10)
    out.append(Part("left_foot_base", "left_foot_link", "al6061", fbase, "service"))
    # stiffening ribs along the foot either side of the lugs (bolted on; the base is a flat plate)
    for k, ry in (("rib_a", 24), ("rib_b", -24)):
        rib = box((fl - 50, 6, 16), (fx + 10, ry, pz + 2.5 + 8))  # 190 mm: fits the mill
        out.append(Part(f"left_foot_{k}", "left_foot_link", "al6061", rib, "cnc"))
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
