"""Arm parts, first pass: aluminium motor brackets, printed structural shells, wrist and hand mount.

Built in the arm link frames (mm) of description/skeleton.yaml, left arm (the right arm is the
mirror image), around the motors at their STEP-model sizes:

- shoulder pitch link: an L-bracket from the pitch RS03's output (facing out, +y) to the back
  face of the shoulder roll motor below it;
- shoulder roll link: an L-bracket from the roll motor's output (facing forward, +x) to the top
  of the shoulder yaw motor;
- upper arm: an aluminium ring on the yaw output, a printed shell down to the elbow, and an
  aluminium ring on the elbow motor's inner (stator) face; the forearm drives off its outer face;
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
    bore,
    cage,
    ear_face,
    front_ring,
    output_face,
    output_plate,
    output_windows,
    ring_band,
    ring_od,
    spec,
    zero_stop,
)
from .mounts import face as contact
from .mounts import hub as out_hub
from .torso import SHOULDER_PIN, SHOULDER_RING_T

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
_models = {
    m["name"].replace("{side}", "left"): m["model"]
    for m in yaml.safe_load((DESC / "robot.yaml").read_text())["motors"]
}
# the arm's motors (description/robot.yaml); the shoulder pitch is the torso's RS03
ROLL_M, YAW_M, ELBOW_M = (_models[f"left_{n}"] for n in ("shoulder_roll", "shoulder_yaw", "elbow"))
# stop and zero pins on a front ring (radius, angle; the zero pin half a turn round): between
# the bore and the screws; the roll's near the top. On an RS10P the pins sit 19 deg from a
# screw, clear of its counterbore (2.6 mm wall) and of the motor's dowels (2.3 mm), which take
# every other gap between screws (actuators.yaml)
PINS = {"RS06": (32.0, 22.5), "RS10P": (21.8, 19.0)}
PIN_ROLL = {"RS03": (40.5, 90.0), "RS10P": (21.8, 79.0)}[ROLL_M]
PIN_YAW, PIN_ELBOW = PINS[YAW_M], PINS[ELBOW_M]


# the idler bearing of a yoke on a motor's rear end (a thin-section deep-groove ball bearing)
IDLER = dict(name="6805-2RS", d=25.0, D=37.0, B=7.0, shoulder=29.0, mass_g=22.0)
YOKE_T = 6.0  # the yoke plate (7075), round the bearing and down to its bridge
# the elbow yoke's bridge under the motor (forearm frame, x and z): outside the strap's 48 mm
# disc, and behind the forearm's centre line so it stays below the upper arm cover with the
# elbow folded to 135 deg
YOKE_BRIDGE = (-34.0, -16.0, -63.0, -53.0)


def idler_stub(model: str, centre, axis: str, sign: int, joint: str, link: str, name: str):
    """A stub on a motor's rear face (its rear screws), carrying the idler bearing; returns the
    part and the bearing's (near, far) faces along the axis, from the motor centre outward."""
    s = spec(model)
    i = "xyz".index(axis)
    rear = s["rear"]  # the rear face, from the centre
    r = s["holes"]["rear"]
    flange_t = 3.0
    c = centre[i] - sign * rear  # on the rear face
    uv = [centre[k] for k in range(3) if k != i]
    flange = bplate(
        cut(disc(r["pcd"] + 12, uv), circles(3.4, circle_pts(r["n"], r["pcd"], uv, np.pi / 4))),
        axis,
        c - sign * flange_t / 2,
        flange_t,
    )
    near, far = c - sign * (flange_t + 1.0), c - sign * (flange_t + 1.0 + IDLER["B"])
    shoulder = bprism(disc(IDLER["shoulder"], uv), axis, c - sign * flange_t, near)
    shaft = bprism(disc(IDLER["d"], uv), axis, near, far)
    # hollow: the rear face is plain in the middle (the cable leaves the side)
    stub = flange.fuse(shoulder).fuse(shaft) - bprism(disc(14.0, uv), axis, c + sign, far - sign)
    rear_face = contact(joint, tuple(centre[k] if k != i else c for k in range(3)), axis, 10, 25)
    # (left out of cad/fea.py: it hangs on the motor body, which the check does not mesh)
    part = made(name, link, "al7075", stub, ifaces=[rear_face], extra={"fea": False})
    return part, near, far


def elbow_yoke(em, fm, fy0: float) -> list[Part]:
    """The elbow's idler stub (upper arm), its bearing and the forearm's yoke plate."""
    out = []
    stub, near, far = idler_stub(
        ELBOW_M, em, "y", 1, "left_elbow", "left_upper_arm_link", "left_elbow_idler_stub"
    )
    out.append(stub)
    dy = fm[1] - em[1]  # upper arm frame to forearm frame, along y (both on the elbow axis)
    near_f, far_f = near + dy, far + dy
    y_mid = (near_f + far_f) / 2
    out.append(
        Part(
            "left_elbow_idler",
            "left_forearm_link",
            "bought",
            cyl(IDLER["D"], IDLER["B"], "y", (0, y_mid, 0))
            - cyl(IDLER["d"], IDLER["B"] + 1, "y", (0, y_mid, 0)),
            "bought",
            mass_g=IDLER["mass_g"],
            notes=f"{IDLER['name']} ball bearing: the elbow yoke's idler on the motor's rear stub",
        )
    )
    # the plate: the bearing in a bore with a 1 mm lip on its outer face, then down to a bridge
    # across under the motor to the frame
    y0, y1 = far_f - 1.0, near_f  # the plate spans the bearing and its lip (y0 < y1)
    b_sk = rect(*YOKE_BRIDGE)
    plate_sk = rounded(hull(disc(IDLER["D"] + 2 * YOKE_T), b_sk))
    plate_sk = cut(plate_sk, lighten(plate_sk, keep=[disc(IDLER["D"] + 4), b_sk], pitch=14, rim=5))
    plate = bplate(cut(plate_sk, disc(IDLER["D"] - 6)), "y", (y0 + y1) / 2, y1 - y0)
    plate = plate - bprism(disc(IDLER["D"]), "y", far_f, y1 + 1)  # the bearing's seat
    bridge = bprism(b_sk, "y", y0, fy0)
    # a box section: hollow along y but for its ends (2.5 mm walls)
    x0, x1, z0, z1 = YOKE_BRIDGE
    bridge = bridge - bprism(
        rect(x0 + 2.5, x1 - 2.5, z0 + 2.5, z1 - 2.5, r=1.0), "y", y1 + 4, fy0 - 4
    )
    yoke = plate.fuse(bridge)
    out.append(
        made(
            "left_forearm_yoke",
            "left_forearm_link",
            "al7075",
            yoke,
            ifaces=[bore("left_elbow", (0, y_mid, 0), "y", IDLER["D"], IDLER["B"])],
            # for cad/fea.py: 1 mm into the frame it is screwed to
            extra={"fea_solid": to_manifold(plate.fuse(bprism(b_sk, "y", y0, fy0 + 1)))},
        )
    )
    return out


def roll_yoke(roll, top_z0: float, tt: float) -> list[Part]:
    """The shoulder roll's idler stub (shoulder pitch link), its bearing and the roll link's
    yoke plate, which stands on the top plate behind the roll motor."""
    out = []
    stub, near, far = idler_stub(
        ROLL_M,
        roll,
        "x",
        1,
        "left_shoulder_roll",
        "left_shoulder_pitch_link",
        "left_shoulder_roll_idler_stub",
    )
    out.append(stub)
    x_mid = (near + far) / 2  # the roll link's frame is on the roll motor's centre: x as is
    out.append(
        Part(
            "left_shoulder_roll_idler",
            "left_shoulder_roll_link",
            "bought",
            cyl(IDLER["D"], IDLER["B"], "x", (x_mid, 0, 0))
            - cyl(IDLER["d"], IDLER["B"] + 1, "x", (x_mid, 0, 0)),
            "bought",
            mass_g=IDLER["mass_g"],
            notes=f"{IDLER['name']} ball bearing: the shoulder roll yoke's idler",
        )
    )
    x0, x1 = far - 1.0, near  # the plate spans the bearing and its lip (x0 < x1)
    foot = rect(-20.0, 20.0, top_z0, top_z0 + tt + 6)  # (y, z): down onto the top plate
    plate_sk = rounded(hull(disc(IDLER["D"] + 2 * YOKE_T), foot))
    plate_sk = cut(plate_sk, lighten(plate_sk, keep=[disc(IDLER["D"] + 4), foot], pitch=14, rim=5))
    plate = bplate(cut(plate_sk, disc(IDLER["D"] - 6)), "x", (x0 + x1) / 2, x1 - x0)
    plate = plate - bprism(disc(IDLER["D"]), "x", far, x1 + 1)  # the bearing's seat
    out.append(
        made(
            "left_shoulder_roll_yoke",
            "left_shoulder_roll_link",
            "al7075",
            plate,
            ifaces=[bore("left_shoulder_roll", (x_mid, 0, 0), "x", IDLER["D"], IDLER["B"])],
        )
    )
    return out


def hub_d(model: str, d: float) -> float:
    """A hub `d` across, no wider than the output it stands on (it passes the ring's bore)."""
    return min(d, spec(model)["output_d"])


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
    _, t_roll = env(ROLL_M)
    _, t_yaw = env(YAW_M)
    d00, t00 = env("RS00")
    r_roll = spec(ROLL_M)["body_d"] / 2 + 1  # the roll motor's body, 1 mm clear
    sw, sl, st = _acts["STS3250"]["envelope_mm"]  # servo: 45.3 long (x), 35.8 high, 24.8 thick
    t = 6.0
    out: list[Part] = []

    # --- shoulder pitch link: frame on the pitch axis; the pitch RS03 (torso, centred at pm)
    # drives a hub through the torso's ring on its ears, y0 the hub's face; the roll motor is
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
    if abs(ry - y0) < r_roll:
        hub_cut = math.sqrt(r_roll**2 - (ry - y0) ** 2)
        phub = phub - bprism(
            rect(-t_roll / 2 - 1, t_roll / 2 + 1, rz - hub_cut, rz + hub_cut, r=5.0),
            "y",
            y0 - 10,
            y0 + 1,
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
    r_front = roll[0] + spec(ROLL_M)["ear"] + rt
    fl = rounded(union2(disc(92), rect(-20, r_front, rz - 30, 8)))
    # the roll motor's body where it crosses the flange's plate, 1 mm clear, to its ears' face
    # (ahead of that only the output spigot, well inside the flange's plane)
    r_cut, t_cut = r_roll, t_roll
    if abs(ry - (y0 + t)) < r_cut:
        cut_z = math.sqrt(r_cut**2 - (ry - (y0 + t)) ** 2)
        fl = cut(fl, rect(-t_cut / 2 - 1, spec(ROLL_M)["ear"], rz - cut_z, rz + cut_z))
    flange, _ = output_plate("RS03", fl, pm, "y", 1, t, at=p_top, d=p_relief)
    _, groove, zh, _ = zero_stop("RS03", pm, "y", "left_shoulder_pitch", True, *SHOULDER_PIN)
    flange = flange - bprism(groove, "y", y0 - 1, y0 + 3.5) - bprism(zh, "y", y0 - 1, y0 + t + 1)
    out.append(made("left_shoulder_flange", L, "al7075", flange))
    # the ring sits on the flange's outer face (one of its eight screws gives way to the flange),
    # and stays 1 mm inside the swing of the shoulder roll link's top plate (r = 53.75 mm)
    yaw_c = mounts.get("left_shoulder_yaw", j["left_shoulder_yaw"])  # the yaw motor's centre
    top_plate_z = yaw_c[2] + t_yaw / 2  # the roll link's top plate's underside (roll frame)
    r_out = -(yaw_c[2] + t_yaw / 2 + 6.0) - 1.0
    # (no wider than the motor's own ring plus a rim: a small roll motor leaves room round it)
    r_d = ring_od(ROLL_M) + 8.0
    # and a tab down to the flange's face where the ring stops short of it: 38 mm wide, so its
    # corners stay inside the top plate's nearest approach (its underside, at any roll angle)
    tab_w = min(r_d / 2, math.sqrt(max((-top_plate_z) ** 2 - (ry - y0 - t) ** 2, 0)) - 1.0)
    r_sk = union2(
        disc(min(r_d, 2 * r_out), (ry, rz)), rect(y0 + t - 1, ry, rz - tab_w, rz + tab_w, r=0)
    )
    r_sk = cut(r_sk, rect(y0 + t - 100, y0 + t, rz - 100, rz + 100, r=0))
    r_pins, _, _, r_zp = zero_stop(ROLL_M, roll, "x", "left_shoulder_roll", True, *PIN_ROLL)
    r_ring, _ = front_ring(ROLL_M, roll, "x", 1, t=rt, outline=r_sk, keep=r_zp)
    r_ring = r_ring - bprism(r_pins, "x", r_front - 5, r_front + 1)
    out.append(
        made(
            "left_shoulder_roll_ring",
            L,
            "al7075",
            r_ring,
            ifaces=[ear_face("left_shoulder_roll", ROLL_M, roll, "x", 1)],
        )
    )

    # --- shoulder roll link: frame on the roll axis (roll motor centred here); a hub on its
    # output through the ring, the front plate on the hub; the shoulder yaw motor is centred at ym
    # below, axis z, output down: a ring on its output-end ears, four strips round it up to the
    # top plate over its driver cap. The front plate and the top plate make an L (no room for
    # gussets: the roll motor fills the corner).
    L = "left_shoulder_roll_link"
    rhub, face, r_relief = out_hub(
        ROLL_M, (0, 0, 0), "x", 1, spec(ROLL_M)["ear"] + rt, d=hub_d(ROLL_M, 58.0)
    )
    out.append(
        made(
            "left_shoulder_roll_hub",
            L,
            "al7075",
            rhub,
            ifaces=[output_face("left_shoulder_roll", ROLL_M, (0, 0, 0), "x", 1)],
        )
    )
    ym = yaw_c
    rear = spec(YAW_M)["holes"].get("rear")  # an RS10P is tapped on its rear face too
    # the yaw motor's ring; with the motor's rear bolted up to the top plate, its body joins the
    # two and the cage strips behind it go (they read SF 68 with it)
    yt = 3.0 if rear else 5.0
    tt = 2.5  # (4 mm meets the torso with the arm raised forward and rolled in)
    top_z0 = ym[2] + t_yaw / 2  # the top plate's underside, on the yaw motor's rear end
    # two strips behind the yaw motor; in front, the front plate's two legs come down to its ring
    y_strips, y_pads, y_fea = cage(YAW_M, ym, "z", -1, to=t_yaw / 2, angles=(135.0, 225.0))
    ring_z = ym[2] - spec(YAW_M)["ear"]  # the yaw ring's rear face (the ears' face)
    # each leg widens outward into a foot on the ring
    legs = [rect(s * 30.0, s * 46.0, ring_z, top_z0 + 8) for s in (1, -1)]
    legs += [rect(s * 30.0, s * 56.0, ring_z, ring_z + 14) for s in (1, -1)]
    legs.append(rect(-46.0, 46.0, top_z0, top_z0 + 8))  # joined along the plate's lower edge
    # the front plate (7075): its legs stand on the yaw ring (cad/fea.py); 7 mm with the roll
    # yoke carrying the arm's back end (10 without; SF 1.2 at 6 mm if the idler carried nothing)
    fb = 7.0 if spec(ROLL_M)["holes"].get("rear") else 10.0
    # (86 mm: clear of the shoulder flange, 44 mm from the roll axis)
    front = rounded(union2(disc(86), rect(-28, 28, top_z0, 0), *legs), inside=8.0)
    front = cut(front, lighten(front, keep=[disc(86)], hole=14, pitch=20, rim=7))
    front = cut(front, output_windows(ROLL_M, (0, 0, 0), "x", 21.0, 33.0))
    front_sk = front
    front, _ = output_plate(ROLL_M, front, (0, 0, 0), "x", 1, fb, at=face, d=r_relief)
    _, groove, zh, _ = zero_stop(ROLL_M, (0, 0, 0), "x", "left_shoulder_roll", True, *PIN_ROLL)
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
    if rear:
        y_pads = []
    top = rounded(
        union2(hull(disc(90.0, yxy), rect(0, face, -28, 28)), *([] if rear else [behind]))
    )
    top = cut(
        top,
        rect(face, face + 100, -100, 100, r=0),  # the front plate's (the strips end under it)
        *([] if rear else [disc(24, (ym[0], ym[1]))]),  # a capped yaw motor's cable
        lighten(top, keep=[*([] if rear else [y_pads]), disc(60, (ym[0], ym[1]))], pitch=20, rim=6),
    )
    top_ifaces = []
    if rear:
        # the yaw motor's rear face bolts up to the top plate too, so its body ties the plate
        # and its front ring together (its front screws sit on a small circle)
        top = cut(top, circles(3.4, circle_pts(rear["n"], rear["pcd"], yxy, phase=np.pi / 4)))
        top_ifaces = [contact("left_shoulder_yaw", (ym[0], ym[1], top_z0), "z", 13.0, 26.0)]
    out.append(
        made(
            "left_shoulder_roll_top",
            L,
            "al7075",
            bplate(top, "z", top_z0 + tt / 2, tt),
            ifaces=top_ifaces or None,
            # for cad/fea.py: 1 mm into the front plate it is screwed to (faces that only touch
            # don't mesh as one)
            extra={
                "fea_solid": to_manifold(
                    bplate(
                        union2(top, rect(face - 1, face + 1, -27.5, 27.5, r=0)),
                        "z",
                        top_z0 + tt / 2,
                        tt,
                    )
                )
            },
        )
    )
    # the shoulder roll yoke: the roll link also rides on the roll motor's rear end, an idler
    # on a stub there, the yoke plate standing on the top plate behind the motor
    if spec(ROLL_M)["holes"].get("rear"):
        out += roll_yoke(roll, top_z0, tt)
    if not rear:
        out.append(
            made("left_shoulder_yaw_cage", L, "al7075", y_strips, extra={"fea_solid": y_fea})
        )
    leg_pads = [rect(face, face + fb, s * 30.0, s * 56.0) for s in (1, -1)]
    # the legs' pads joined to the ring across their full width (with a small motor's screw
    # circle the load crosses there; a narrow neck between them read SF 0.2)
    y_sk = rounded(union2(hull(disc(92, (ym[0], ym[1])), *leg_pads), *([] if rear else [y_pads])))
    y_pins, _, _, y_zp = zero_stop(YAW_M, ym, "z", "left_shoulder_yaw", True, *PIN_YAW)
    y_ring, y_front = front_ring(YAW_M, ym, "z", -1, t=yt, outline=y_sk, pocket=False)
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
            ifaces=[ear_face("left_shoulder_yaw", YAW_M, ym, "z", -1)],
        )
    )

    # --- upper arm: frame on the yaw joint (the yaw motor centred at um, output down): a hub on
    # its output through the ring, a ring on the hub, and from the ring down the arm's outer side
    # a strap that is the elbow motor's ring (on its front face, output outward +y); a printed
    # cover round it all. The strap runs down behind the elbow, so the forearm folds clear of it
    L = "left_upper_arm_link"
    el = j["left_elbow"]
    em = mounts.get("left_elbow", el)  # the elbow motor's centre (skeleton.yaml motor_mounts)
    fm = (0.0, em[1] - el[1], 0.0)  # and in the forearm's frame
    um = (0.0, 0.0, ym[2] - j["left_shoulder_yaw"][2])  # skeleton.yaml motor_mounts
    uhub, u_top, u_relief = out_hub(
        YAW_M, um, "z", -1, spec(YAW_M)["ear"] + yt, d=hub_d(YAW_M, 46.0)
    )
    out.append(
        made(
            "left_upper_arm_hub",
            L,
            "al7075",
            uhub,
            ifaces=[output_face("left_shoulder_yaw", YAW_M, um, "z", -1)],
        )
    )
    # the ring on the yaw hub (7075); 6 mm round a small hub (an RS10P's Ø34 output), which it
    # bends round under a hand load (4 mm: SF 1.2; 5 mm: 2.2; 6 mm: 4.0)
    ut = 6.0 if spec(YAW_M)["output_d"] < 40 else 4.0
    ring_top, _ = output_plate(
        YAW_M,
        cut(disc(84), disc(16)),
        um,
        "z",
        -1,
        ut,
        at=u_top,
        d=u_relief,
    )
    _, groove, zh, _ = zero_stop(YAW_M, um, "z", "left_shoulder_yaw", True, *PIN_YAW)
    uf = um[2] - u_top  # the ring's face toward the yaw ring
    ring_top = ring_top - bprism(groove, "z", uf - 3, uf + 1) - bprism(zh, "z", uf - ut - 1, uf + 1)
    out.append(made("left_upper_arm_ring", L, "al7075", ring_top))
    ring_bot = um[2] - u_top - ut  # the ring's underside
    # the strap (7075): a hand load bends it out of its plane (cad/fea.py); 5 mm with a small
    # elbow motor, whose stiffer upper arm ring above it carries more of that
    et = 5.0 if spec(ELBOW_M)["output_d"] < 40 else 6.0
    strap = rounded(
        union2(
            disc(96, xz(el)),
            slot(36, (-12, el[2]), (-12, ring_bot - 18)),
            rect(-30, 30, ring_bot - 22, ring_bot),
        )
    )
    e_pins, _, _, e_zp = zero_stop(ELBOW_M, em, "y", "left_elbow", True, *PIN_ELBOW)
    # harness clips: the shoulder yaw loop's, high on the strap, and the elbow loop's, behind it
    s_avoid = [disc(spec(ELBOW_M)["output_d"] + 2, xz(el)), ring_band(ELBOW_M, em, "y"), e_zp]
    s_clips = [
        clip_spot(strap, (-12.0, ring_bot - 20), s_avoid, min_r=0),
        clip_spot(strap, xz(el), s_avoid, min_r=50),
    ]
    strap = cut(
        strap,
        lighten(
            strap,
            keep=[ring_band(ELBOW_M, em, "y"), rect(-30, 30, ring_bot - 12, ring_bot)]
            + [disc(CLIP_PAD, c) for c in s_clips],
        ),
        *[disc(CLIP_HOLE, c) for c in s_clips],
    )
    strap, s_front = front_ring(ELBOW_M, em, "y", 1, t=et, outline=strap, pocket=False)
    strap = strap - bprism(e_pins, "y", em[1] + s_front - 4, em[1] + s_front + 1)
    out.append(
        made(
            "left_upper_arm_strap",
            L,
            "al7075",
            strap,
            ifaces=[ear_face("left_elbow", ELBOW_M, em, "y", 1)],
        )
    )

    # --- forearm: frame on the elbow joint (elbow motor centred at fm, output +y): a hub through
    # the strap and a flange on it; the wrist RS00 is centred on the wrist joint below, axis z,
    # output down
    wr = j["left_wrist_yaw"]
    fhub, out_y, f_relief = out_hub(
        ELBOW_M, fm, "y", 1, spec(ELBOW_M)["ear"] + et, d=hub_d(ELBOW_M, 46.0)
    )
    out.append(
        made(
            "left_forearm_hub",
            "left_forearm_link",
            "al7075",
            fhub,
            ifaces=[output_face("left_elbow", ELBOW_M, fm, "y", 1)],
        )
    )
    # the frame: a 6 mm strap on the hub, down the outside of the forearm (outboard of the upper
    # arm's strap) to a bracket on the wrist RS00's rear face, by its 4 x M3 rear holes
    ft = 5.0  # (4 mm with the elbow yoke: SF 1.9 under the wrist yaw's torque)
    cup_top = wr[2] + t00 / 2  # the wrist motor's rear face
    bt = 3.5  # the bracket
    fy0, fy1 = fm[1] + out_y, fm[1] + out_y + ft
    foot = rect(-22, 22, cup_top + bt, cup_top + bt + 16)
    # (and a lobe under the elbow motor where the yoke's bridge lands, with a yoke)
    lobe = (
        [hull(rect(*YOKE_BRIDGE), rect(-15.0, 15.0, YOKE_BRIDGE[2] - 25, YOKE_BRIDGE[2] - 15))]
        if spec(ELBOW_M)["holes"].get("rear")
        else []
    )
    frame = rounded(
        union2(disc(72), slot(30, (0, -10), (0, cup_top + bt + 12)), foot, *lobe), inside=8.0
    )
    _, groove, zh, zpads = zero_stop(ELBOW_M, fm, "y", "left_elbow", True, *PIN_ELBOW)
    # harness clips: the elbow loop's, below the elbow, and the wrist yaw loop's, low down
    f_avoid = [disc(76), zpads, foot, *lobe]
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
    frame, _ = output_plate(ELBOW_M, frame, fm, "y", 1, ft, at=out_y, d=f_relief)
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

    # --- the elbow yoke: the forearm also rides on the elbow motor's rear end, so a hand load
    # reaches the upper arm through two bearings a motor length apart instead of bending the
    # motor's own output bearing. A stub on the motor's rear face (its 4 x M3) carries an idler
    # ball bearing; a plate on the forearm holds the bearing and joins the frame by a bridge
    # under the motor, outside the strap's disc (the strap stays behind the folding forearm).
    if spec(ELBOW_M)["holes"].get("rear"):
        out += elbow_yoke(em, fm, fy0)

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
