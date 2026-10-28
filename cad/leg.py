"""Leg parts, first pass: aluminium brackets and cores, printed shell, ankle linkage, foot.

Every part is built in its link's frame (mm), using the frames of description/skeleton.yaml
(left leg; the right leg is the mirror image). Motors sit where tools/build_urdf.py places
them (motor_mounts), at their STEP-model sizes, and the structure is built around them:

- hip roll: the roll RS04 sits in the pelvis behind the hip; a U round it (front plate on its
  output, rear plate on a far-side bearing, a bar over it) reaches forward to the pitch RS04's
  stator face, so the roll motor never swings with the leg;
- hip pitch: a closed box round the pitch RS04 (flange and outer wall on its output, inner wall
  on a far-side bearing, front and back walls below the motor) with the hip yaw RS03 under it;
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
lightening holes, and the motor mounts carry the stator and output hole circles, but there are
no other fastener holes, fillets or bearing details yet.
"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .lib import Part, box, cyl, hull, lighten, strap, union
from .mounts import bolt_holes, output_flange, rear_ring, spec, spider
from .mounts import posts as mount_posts

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
    hip_t = 6.0  # pelvis back plate (cad/torso.py), which the roll bracket's rear plate sits behind
    brg_d, brg_w = 42.0, 7.0  # 6806 far-side bearing (30 x 42 x 7 mm)
    out: list[Part] = []

    # --- hip roll link: frame on the roll axis. The roll RS04 is fixed in the pelvis behind the
    # hip (its output face forward); the pitch RS04 is centred on the pitch joint, axis y, its
    # stator face inward. The roll bracket is a U round the roll motor (front plate on its
    # output, rear plate on a far-side bearing on the pelvis's stub axle behind it, a bar over it
    # joining them) and a plate reaching forward from the front plate to the
    # pitch motor's stator face, with a stub axle for the thigh's far-side bearing.
    rj = J["left_hip_roll"]
    roll_c = [a - b for a, b in zip(mounts["left_hip_roll"], rj, strict=True)]  # this frame
    pj = J["left_hip_pitch"]
    out_x = roll_c[0] + t04 / 2  # roll output face
    fx = out_x + plate / 2  # front plate
    st = 8.0  # stator plate: SF 2 in the landing case needs 8 mm (cad/strength.py)
    sy = pj[1] - t04 / 2 - st / 2  # stator plate, against the pitch motor's inner face
    over = d04 / 2 + 6  # bar's underside, over the roll motor (under it, the thigh would hit it
    # when the hip extends); the pelvis keeps clear of its swing (cad/torso.py)
    rear_x = roll_c[0] - t04 / 2 - hip_t - 14 - plate / 2
    front = union(
        output_flange("RS04", roll_c, "x", 1, 80, plate)[0],
        box((plate, -sy + st / 2, 56), (fx, (sy - st / 2) / 2, 0)),  # in, to the stator
        box((plate, 40, over + 10), (fx, 0, (over + 10) / 2)),  # up, to the bar
    )
    front = lighten(front, "x", keep=[cyl(60, 40, "x", (out_x, 0, 0))], hole=12, pitch=18, rim=7)
    stator = union(
        spider("RS04", pj, "y", 1, t04 / 2, st, width=20),
        hull(cyl(56, st, "y", (pj[0], sy, pj[2])), box((plate, st, 56), (fx, sy, 0))),
    )
    stator = lighten(stator, "y", keep=[cyl(76, 40, "y", (pj[0], sy, pj[2]))], hole=14, pitch=20)
    stub = cyl(30, 12, "y", (pj[0], sy - st / 2 - 6, pj[2]))  # thigh far-side bearing
    stator = union(stator, stub)
    # the pitch motor's stator bolts to a ring on its body's rear (inner) face; posts carry
    # the stator plate, with the thigh's far-side stub, over its driver cap
    hp_ring, _ = rear_ring("RS04", pj, "y", 1)
    hp_mount = union(hp_ring, mount_posts("RS04", pj, "y", 1, 4.0, t04 / 2))
    out.append(Part("left_hip_pitch_mount", "left_hip_roll_link", "al6061", hp_mount, "cnc"))
    rear = union(
        cyl(brg_d + 16, plate, "x", (rear_x, 0, 0)),
        box((plate, 30, over + 10), (rear_x, 0, (over + 10) / 2)),
    ) - cyl(brg_d, plate + 2, "x", (rear_x, 0, 0))
    bar = box((fx - rear_x + plate, 60, 10), ((fx + rear_x) / 2, 0, over + 5))
    bar = bar - box((fx - rear_x - 10, 44, 12), ((fx + rear_x) / 2, 0, over + 5 + 3))  # channel
    out.append(Part("left_hip_roll_front", "left_hip_roll_link", "al6061", front, "cnc"))
    out.append(Part("left_hip_roll_stator", "left_hip_roll_link", "al6061", stator, "cnc"))
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

    # --- hip pitch link: frame on the pitch axis (the pitch RS04 centred here, output face +y).
    # A closed box round the pitch motor: a flange and outer wall on its output, an inner wall on
    # a far-side bearing on the roll bracket's stub axle, front and back walls joining them
    # below the motor, and the yaw RS03 under its floor. A box, not a U of two plates: much
    # stiffer in twist, and one solid piece at the top of the thigh.
    face = t04 / 2
    yaw_c = J["left_hip_yaw"]
    yaw_top = yaw_c[2] + t03 / 2  # the yaw motor's driver cap top
    tz = yaw_top + web_t / 2  # the box floor, over the cap; posts down to a ring on the body
    flange = output_flange("RS04", (0, 0, 0), "y", 1, 80, plate)[0]
    out.append(Part("left_hip_pitch_flange", "left_hip_pitch_link", "al6061", flange, "cnc"))
    side_t = 5.0  # the box walls carry the twist (SF > 4, cad/strength.py)
    oy = face + plate + side_t / 2
    outer_p = union(
        cyl(80, side_t, "y", (0, oy, 0)),
        hull(cyl(48, side_t, "y", (0, oy, 0)), box((48, side_t, 2), (0, oy, tz + 1))),
    )
    outer_p = lighten(outer_p, "y", keep=[cyl(66, 40, "y", (0, oy, 0))], hole=14, pitch=20)
    iy = sy - pj[1] - st / 2 - 3 - plate / 2  # inner plate, on the bearing (this frame)
    inner_p = union(
        cyl(brg_d + 16, plate, "y", (0, iy, 0)),
        hull(cyl(48, plate, "y", (0, iy, 0)), box((48, plate, 2), (0, iy, tz + 1))),
    ) - cyl(brg_d, plate + 2, "y", (0, iy, 0))
    inner_p = lighten(inner_p, "y", keep=[cyl(brg_d + 12, 40, "y", (0, iy, 0))], hole=12, pitch=18)
    top = hull(
        cyl(d03 + 4, web_t, "z", (yaw_c[0], yaw_c[1], tz)),
        box((48, oy - iy + side_t, web_t), (0, (oy + iy) / 2, tz)),
    )
    top = lighten(
        top, "z", keep=[cyl(d03 + 4, 40, "z", (yaw_c[0], yaw_c[1], yaw_top))], hole=20, pitch=26
    )
    y_ring, _ = rear_ring("RS03", yaw_c, "z", -1)
    y_mount = union(y_ring, mount_posts("RS03", yaw_c, "z", -1, 4.0, t03 / 2))
    out.append(Part("left_hip_yaw_mount", "left_hip_pitch_link", "al6061", y_mount, "cnc"))
    out.append(Part("left_hip_pitch_outer", "left_hip_pitch_link", "al6061", outer_p, "cnc"))
    out.append(Part("left_hip_pitch_inner", "left_hip_pitch_link", "al6061", inner_p, "cnc"))
    out.append(Part("left_hip_pitch_top", "left_hip_pitch_link", "al6061", top, "cnc"))
    wall_t = 4.0
    wz0 = -math.sqrt((d04 / 2) ** 2 - 24.0**2) - 7  # 7 mm under the pitch motor at x = +-24
    wz1 = tz + web_t / 2
    for name, s in (("front", 1), ("back", -1)):
        wall = box(
            (wall_t, oy - iy - side_t / 2 - plate / 2, wz0 - wz1),
            (s * (24 - wall_t / 2), (oy + iy) / 2 + (plate - side_t) / 4, (wz0 + wz1) / 2),
        )
        out.append(Part(f"left_hip_pitch_{name}", "left_hip_pitch_link", "al6061", wall, "cnc"))
    out.append(
        Part(
            "left_hip_pitch_bearing",
            "left_hip_pitch_link",
            "bought",
            cyl(brg_d, brg_w, "y", (0, iy, 0)) - cyl(30, brg_w + 2, "y", (0, iy, 0)),
            "bought",
            mass_g=25.0,
            notes="6806-2RS far-side bearing",
        )
    )

    # --- thigh: frame on the yaw axis at the yaw motor's centre; yaw output faces down
    yaw_out = -t03 / 2
    knee = J["left_knee"]
    tflange = union(
        output_flange("RS03", (0, 0, 0), "z", -1, 84, plate)[0],
        box((60, 16, plate), (0, t04 / 2 + 2, yaw_out - plate / 2)),
    )
    out.append(Part("left_thigh_flange", "left_thigh_link", "al6061", tflange, "cnc"))
    # The thigh plate is a single plate beside the leg's line, so a sideways fall onto the knee
    # bends it out of plane: from 9 mm stock, pocketed from the outside to a 2 mm floor, it takes
    # that case at SF 2.3 (cad/strength.py, side fall); the drilled 6 mm plate it replaces was
    # at SF 0.5 there (finite elements, est. load). A topology-optimised outline could save
    # about 35 g of the 61 g it adds
    thigh_t = 9.0
    py = t04 / 2 + thigh_t / 2  # outer side plate, against the knee RS04's outer face
    # a tapered strap from the yaw flange to a ring on the knee motor's stator bolt circle
    # (~86 mm, STEP model), instead of a plate as wide as the motor
    tplate = union(
        hull(box((60, thigh_t, 16), (0, py, yaw_out - 8)), cyl(46, thigh_t, "y", (0, py, knee[2]))),
        spider("RS04", knee, "y", -1, t04 / 2, thigh_t, width=20),
    )
    tplate = lighten(
        tplate, "y", keep=[cyl(70, 40, "y", (0, py, knee[2]))], hole=20, pitch=24, rim=7, floor=2.0
    )
    # stub axle for the shin's far-side bearing, on the knee axis outside the plate
    tplate = union(tplate, cyl(30, 12, "y", (0, py + thigh_t / 2 + 6, knee[2])))
    out.append(Part("left_thigh_plate", "left_thigh_link", "al6061", tplate, "cnc"))
    # the knee motor's stator bolts to a ring on its body's rear (outer) face; posts join it to
    # the thigh plate over the driver cap
    kn_ring, _ = rear_ring("RS04", knee, "y", -1)
    kn_mount = union(kn_ring, mount_posts("RS04", knee, "y", -1, 4.0, t04 / 2))
    out.append(Part("left_thigh_knee_mount", "left_thigh_link", "al6061", kn_mount, "cnc"))
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
    shin_t = 4.0  # side plates: SF 2.1 at 4 mm (cad/strength.py)
    yi, yo = -(t03 / 2 + shin_t / 2), t03 / 2 + shin_t / 2  # inner and outer side plates
    hub = 40.0  # crank hub, spanning the gap from the motor's output face to the crank
    w = 38.0  # strap width (cad/strength.py sizes it)

    def cheek_at(y):  # the fork cheek: the plate's lower end, round the ankle bearing
        return (0.0, y, ankle_z)

    knee_clear = cyl(2 * 68, 200, "y")  # the thigh plate and its stub turn inside r = 68 mm
    bridge_c = (65.0, -45.0)  # bridge in front of the knee motor, joining the knee yoke
    inner = union(
        cyl(84, shin_t, "y", (0, yi, 0)),
        spider("RS03", ka, "y", 1, t03 / 2, shin_t, width=24),
        strap(w, shin_t, "y", (0, yi, 0), (-20, yi, -110), (ka[0], yi, ka[2]), cheek_at(yi)),
        strap(30, shin_t, "y", (0, yi, 0), (bridge_c[0], yi, bridge_c[1])),
    ) - cyl(22, 20, "y", (0, yi, ankle_z))
    kh = spec("RS04")["holes"]["output"]
    inner = inner - bolt_holes(kh["n"], kh["thread"], kh["pcd"], "y", (0, yi, 0), shin_t + 2)
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
            spider("RS03", kb, "y", -1, t03 / 2, shin_t, width=24),
            strap(
                w,
                shin_t,
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
    # each ankle motor's stator bolts to a ring on its body's rear face (its ears), joined by
    # posts to the side plate over its driver cap: a's to the inner plate, b's to the outer
    for name, c, sgn in (("a", ka, 1), ("b", kb, -1)):
        ring, _ = rear_ring("RS03", c, "y", sgn, outer=112.0)
        mount = union(ring, mount_posts("RS03", c, "y", sgn, 4.0, t03 / 2))
        out.append(Part(f"left_ankle_mount_{name}", "left_shin_link", "al6061", mount, "cnc"))
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
    for k, (sx, sz) in enumerate(((-50.0, -130.0), (55.0, -200.0), (-45.0, ankle_z + 25))):
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
