"""Pelvis and torso structure, first pass.

Built in the pelvis and torso link frames (mm) of description/skeleton.yaml, around the motors
at their STEP-model sizes:

- pelvis: a back plate carrying the two hip roll RS04s behind the hips (with stub axles for
  the roll brackets' far-side bearings), a top plate carrying the waist RS03, and two gussets
  under it making a channel; the hip pitch motors swing round the roll axes clear of all of
  it. The pelvis electronics sit between the roll motors.
- torso: a flange on the waist output and a base plate; two side frames (each a lower frame
  that leans out from the computer's width at the base to the battery's under the shoulders,
  and an upper plate) carrying the shoulder pitch RS03s (flat profiles, longer than the desktop
  mill's travel, so cut by an online laser or water-jet service); a shelf carrying the battery
  high, under the shoulder motors, so the torso narrows to the waist; a top bridge carrying the
  neck yaw servo. The clip-on covers (cad/covers.py) close the torso.

The computer and electronics are laid out by cad/electrical.py; the battery and keep-alive pack
are skeleton.yaml estimate boxes. Plates are pocketed; no fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .b3d import clip_spot, cut, disc, hull, lighten, made, rect, rounded, slot
from .b3d import plate as bplate
from .b3d import prism as bprism
from .b3d import union as union2
from .clips import edge_clip
from .leg import (
    HIP_BACK_T,
    HIP_ROLL_PIN,
    ROLL_RING_FRONT,
    YAW_BRG,
    YAW_PIN,
    YAW_RING_FRONT,
    crossed_roller,
)
from .lib import Part
from .mounts import (
    BEARINGS,
    CLIP_HOLE,
    CLIP_PAD,
    bore,
    cage,
    ear_face,
    front_ring,
    output_face,
    output_plate,
    pillar_ring,
    ring_band,
    ring_od,
    spec,
    stator_face,
    stator_holes,
    zero_stop,
)
from .mounts import hub as out_hub

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())


ROLL_RING_BASE = 4.0  # the hip roll motors' pillar rings: all the hip's load comes through them
SHOULDER_RING_T = 5.0  # the shoulder pitch motors' front rings (cad/arm.py's hub stands past it)
SHOULDER_PIN = (40.5, 150.0)  # the shoulder pitch's stop and zero pins: clear of the roll motor


def env(model: str) -> tuple[float, float]:
    d, _, t = _acts[model]["envelope_mm"]
    return d, t


def parts() -> list[Part]:
    j = {k: [x * 1000 for x in v["xyz"]] for k, v in _skel["joints"].items()}
    mounts = {k: [x * 1000 for x in v["xyz"]] for k, v in _skel["motor_mounts"].items()}
    d04, t04 = env("RS04")
    d03, t03 = env("RS03")
    t = 6.0
    out: list[Part] = []

    # --- pelvis: frame on the hip pitch axes. The two hip roll RS04s are fixed behind the hips
    # (axes along x, output faces forward) on pillar rings on a back plate, each with a bearing
    # carrier on its front face that the hip roll yoke (cad/leg.py) turns in. Each hip's moment
    # about y reaches the middle along the back plate's lower band, and two deep gussets
    # (y-normal, 23 mm either side of the centre line) carry it up and forward to the waist;
    # its moment about x goes up the back plate's centre spine into a closed box along x under
    # the waist motor (the top plate, the gussets' upper parts and a floor). The thighs swing to
    # the centre line under the front of the gussets (hip roll -30 deg), so they stop short
    # there. The pelvis electronics sit on the top plate behind the waist motor
    # (cad/electrical.py).
    rc = mounts["left_hip_roll"]  # roll motor centre (pelvis frame)
    hip_t = HIP_BACK_T
    r_near = pillar_ring("RS04", rc, "x", 1, base=ROLL_RING_BASE)[1]
    bf = rc[0] - r_near  # back plate's front face
    bb = bf - hip_t  # and its back face
    # the waist RS03 is tapped only at its output end: a ring on its ears (seating a crossed-roller
    # bearing under the torso's hub) and four strips round it down to the top plate
    waist_c = tuple(mounts["waist_yaw"])
    w_strips, w_pads, w_fea = cage("RS03", waist_c, "z", 1, to=t03 / 2 + 1.0)
    top_t = 4.0
    top_z0 = waist_c[2] - t03 / 2 - 1.0 - top_t  # the top plate's underside, under the cap
    top_z1 = top_z0 + top_t
    bz0 = -62.0  # the back plate's lower band
    yb = 150.0  # the band's ends
    gy, gt = 23.0, 8.0  # the gussets
    back = rounded(  # big fillets where the band meets the motor discs and the spine
        union2(
            *[disc(124, (s * rc[1], rc[2])) for s in (1, -1)],
            rect(-yb, yb, bz0, -20.0),
            rect(-gy - gt / 2, gy + gt / 2, bz0, top_z0),
        ),
        inside=16.0,
    )
    back = cut(
        back,
        *[stator_holes("RS04", (rc[0], s * rc[1], rc[2]), "x") for s in (1, -1)],
        *[disc(8.5, (s * rc[1], rc[2])) for s in (1, -1)],
        lighten(
            back,
            keep=[ring_band("RS04", (rc[0], s * rc[1], rc[2]), "x") for s in (1, -1)]
            + [disc(24, (s * rc[1], rc[2])) for s in (1, -1)],
            pitch=24,
            rim=6,
        ),
    )
    out.append(
        made("pelvis_back", "pelvis", "al7075", bplate(back, "x", bf - hip_t / 2, hip_t), "service")
    )
    for side, s in (("l", 1), ("r", -1)):  # the hip roll loops' harness clips, over each motor
        tgt = (s * 40.0, 40.0)
        out.append(
            edge_clip(f"pelvis_back_clip_{side}", "pelvis", "x", bf - hip_t / 2, hip_t, back, tgt)
        )
    for side, s in (("l", 1), ("r", -1)):
        c = (rc[0], s * rc[1], rc[2])
        # the bearing carrier on the roll motor's front face: the yoke's hub turns in it, so the
        # hip's tilting moment reaches the motor housing (and the back plate) without passing
        # through the motor's own output bearing
        pins, _, _, pads = zero_stop("RS04", c, "x", "left_hip_roll", True, *HIP_ROLL_PIN)
        carrier, _ = front_ring("RS04", c, "x", 1, bearing=YAW_BRG, keep=pads)
        cf = c[0] + ROLL_RING_FRONT
        carrier = carrier - bprism(pins, "x", cf - 6, cf + 1)
        seat = (c[0] + ROLL_RING_FRONT - BEARINGS[YAW_BRG]["B"] / 2, c[1], c[2])
        ci = [
            ear_face("left_hip_roll_housing", "RS04", c, "x", 1),
            bore("left_hip_roll", seat, "x", BEARINGS[YAW_BRG]["D"], BEARINGS[YAW_BRG]["B"]),
        ]
        out.append(
            made(
                f"pelvis_roll_carrier_{side}",
                "pelvis",
                "al7075",
                carrier,
                ifaces=ci if s == 1 else None,
                extra={"group": "pelvis_roll_carrier"} if s == 1 else {"fea": False},
            )
        )
        out.append(crossed_roller(f"pelvis_roll_bearing_{side}", "pelvis", "x", seat))
        out.append(
            made(
                f"pelvis_roll_mount_{side}",
                "pelvis",
                "al7075",
                pillar_ring("RS04", c, "x", 1, base=ROLL_RING_BASE)[0],
                ifaces=[stator_face("left_hip_roll", "RS04", c, "x", 1)] if s == 1 else None,
            )
        )
    # the gussets: in front of the back plate, clear of the thighs (|x| < 40 mm below z = 40 mm)
    kz = 45.0  # the top box's floor
    for side, s in (("l", 1), ("r", -1)):
        front_g = rounded(
            rect(bf, waist_c[0] + 50, bz0, top_z0, r=0) - rect(-40.0, 200.0, bz0 - 1, 40.0, r=0)
        )
        for name, sk in (("front", front_g),):
            # solid along the top box's floor, and under the top plate where the waist loads come in
            keep = [rect(-200, 200, kz - 4, kz + 9), rect(-60, 90, top_z0 - 16, top_z0)]
            sk = cut(sk, lighten(sk, keep=keep, pitch=26))
            out.append(
                made(
                    f"pelvis_gusset_{name}_{side}", "pelvis", "al7075", bplate(sk, "y", s * gy, gt)
                )
            )
    out.append(
        made(
            "pelvis_keel_floor",
            "pelvis",
            "al7075",
            bplate(rect(bf, -40.0, -gy + gt / 2, gy - gt / 2), "z", kz + 1.5, 3.0),
        )
    )
    top = rounded(hull(disc(124, (waist_c[0], 0)), rect(bb, -60.0, -28.0, 28.0)))
    # the waist loop's harness clip, clear of the cage and the power board standing behind it
    board = rect(-135, -51, -27, 27)
    wc = clip_spot(top, (waist_c[0], 0), [disc(30, (waist_c[0], 0)), w_pads, board], min_r=62)
    top = cut(
        top,
        disc(30, (waist_c[0], 0)),  # the waist motor's cable, out of its cap
        lighten(
            top,
            keep=[w_pads, disc(CLIP_PAD, wc)]
            + [rect(-70, 90, s * gy - 12, s * gy + 12) for s in (1, -1)],  # over the gussets
            pitch=24,
            rim=6,
        ),
        disc(CLIP_HOLE, wc),
    )
    out.append(
        made(
            "pelvis_top",
            "pelvis",
            "al7075",
            bplate(top, "z", (top_z0 + top_z1) / 2, top_t),
        )
    )
    w_sk = rounded(union2(disc(ring_od("RS03"), (waist_c[0], 0)), w_pads))
    w_pins, _, _, w_zpads = zero_stop("RS03", waist_c, "z", "waist_yaw", True, *YAW_PIN)
    w_ring, _ = front_ring("RS03", waist_c, "z", 1, outline=w_sk, bearing=YAW_BRG, keep=w_zpads)
    wf = waist_c[2] + YAW_RING_FRONT
    w_ring = w_ring - bprism(w_pins, "z", wf - 6, wf + 1)
    seat = (waist_c[0], 0.0, waist_c[2] + YAW_RING_FRONT - BEARINGS[YAW_BRG]["B"] / 2)
    out.append(
        made(
            "pelvis_waist_ring",
            "pelvis",
            "al7075",
            w_ring,
            ifaces=[
                ear_face("waist_yaw", "RS03", waist_c, "z", 1),
                bore("waist_yaw", seat, "z", BEARINGS[YAW_BRG]["D"], BEARINGS[YAW_BRG]["B"]),
            ],
        )
    )
    out.append(made("pelvis_waist_cage", "pelvis", "al7075", w_strips, extra={"fea_solid": w_fea}))
    out.append(crossed_roller("waist_bearing", "pelvis", "z", seat))

    # --- torso: frame on the waist axis, 9 mm above the waist motor's centre (wm); a hub on its
    # output turns in the pelvis's bearing ring, and the flange bolts on the hub
    wm = tuple(m - jv for m, jv in zip(waist_c, j["waist_yaw"], strict=True))
    whub, w_top, w_relief = out_hub("RS03", wm, "z", 1, YAW_RING_FRONT, bearing=YAW_BRG)
    out.append(
        made(
            "torso_waist_hub",
            "torso_link",
            "al7075",
            whub,
            ifaces=[
                output_face("waist_yaw", "RS03", wm, "z", 1),
                bore("waist_yaw", (0, 0, wm[2] + w_top - 4.5), "z", 60.0, 8.0),
            ],
        )
    )
    out_z = wm[2] + w_top  # the flange's face
    sh = mounts["left_shoulder_pitch"]  # shoulder pitch RS03 centre (torso frame)
    # The shoulder motors sit deep in the torso (narrow shoulders), deeper than the battery is
    # wide, so each side frame is two plates: a lower frame outside the battery, and an upper
    # plate against the shoulder motor's inner face, above the battery and computer, joined by
    # two spacers clear of the motor. The battery, the widest part, sits high on a shelf under
    # the shoulder motors, and the lower frames lean in to the computer's width at the base:
    # the torso narrows to the waist
    bat = next(b for b in _skel["bodies"] if b["budget"] == "battery")
    y_low = bat["size"][1] * 1000 / 2 + 6 + t / 2
    comp = next(b for b in _skel["bodies"] if b["budget"] == "compute")
    y_bot = comp["size"][1] * 1000 / 2 + 6 + t / 2  # the frame narrows to the computer at the base
    bat_floor = (bat["xyz"][2] - bat["size"][2] / 2) * 1000  # the battery's shelf
    tp = 6.0  # the shoulder plates (7075)
    # the shoulder plates on the motors' body rear faces (caps inboard through them); the
    # motors are tapped only at their output ends, so each bolts by its ears into a ring there,
    # carried back to the plate by four strips round the motor
    up_y = sh[1] - spec("RS03")["rear"] - tp / 2
    # the frame runs from behind the battery to just ahead of the computer standing on edge
    fx0, fx1 = -95.0, 55.0
    tops = ((-70.0, sh[2] - 18), (42.0, sh[2] - 58))  # back and front spacers (x, z)
    flange, _ = output_plate("RS03", disc(96), wm, "z", 1, t, at=w_top, d=w_relief)
    _, groove, zh, _ = zero_stop("RS03", wm, "z", "waist_yaw", True, *YAW_PIN)
    flange = (
        flange
        - bprism(groove, "z", out_z - 1, out_z + 4)
        - bprism(zh, "z", out_z - 1, out_z + t + 1)
    )
    bt0 = 2.5  # the base plate (7075)
    base_z = out_z + t + bt0 / 2
    out.append(made("torso_waist_flange", "torso_link", "al6061", flange))
    base = rect(fx0, fx1, -y_bot + t / 2, y_bot - t / 2)  # between the side frames
    bc = clip_spot(base, (0.0, 0.0), [disc(100)], min_r=58)  # the waist loop's clip
    base = cut(
        base,
        lighten(base, keep=[disc(100), disc(CLIP_PAD, bc)], pitch=28, rim=8),
        disc(CLIP_HOLE, bc),
    )
    out.append(made("torso_base", "torso_link", "al7075", bplate(base, "z", base_z, bt0)))
    # the battery shelf, clear of the computer plate in front
    sx0, sx1 = fx0 + 5, (bat["xyz"][0] + bat["size"][0] / 2) * 1000 + 2
    ut = 3.0  # the lower side frames (7075)
    shelf = rect(sx0, sx1, -y_low + ut / 2, y_low - ut / 2)  # between the side frames
    shelf = cut(shelf, lighten(shelf, hole=18, pitch=24, rim=7))
    out.append(
        made(
            "torso_battery_shelf",
            "torso_link",
            "al7075",
            bplate(shelf, "z", bat_floor - 0.75, 1.5),
        )
    )
    (bx, bz), (fx, fz) = tops
    for side, s in (("l", 1), ("r", -1)):
        # the lower frame leans out from the base to the battery: drawn at y_low; its foot is a
        # second plate at the computer's width (two flat plates; the lean is in the straps)
        low = rounded(
            union2(
                slot(30, (fx0 + 20, bat_floor), (bx, bz)),
                slot(30, (fx1 - 20, bat_floor), (fx, fz)),
                slot(24, (fx0 + 15, bat_floor), (fx1 - 10, bat_floor)),
                slot(30, (fx0 + 20, base_z + 12), (fx0 + 20, bat_floor)),
                slot(30, (fx1 - 20, base_z + 12), (fx1 - 20, bat_floor)),
            )
        )
        low = cut(low, lighten(low, hole=12, pitch=18, rim=7))
        out.append(
            made(
                f"torso_upright_{side}",
                "torso_link",
                "al7075",
                bplate(low, "y", s * y_low, ut),
                "service",
            )
        )
        foot = rect(fx0 + 5, fx1 - 5, base_z - t / 2, base_z + t / 2 + 12)
        out.append(
            made(f"torso_foot_{side}", "torso_link", "al6061", bplate(foot, "y", s * y_bot, t))
        )
        # the foot and the lower frame are joined by two struts across the lean
        for k, x in enumerate((fx0 + 20, fx1 - 20)):
            strut = rect(x - 12, x + 12, base_z + t / 2, base_z + t / 2 + 12)
            y0, y1 = sorted((s * (y_bot + t / 2), s * (y_low - ut / 2)))
            out.append(
                made(
                    f"torso_strut_{side}{k}",
                    "torso_link",
                    "al6061",
                    bprism(strut, "y", y0, y1),
                )
            )
        ring = (sh[0], sh[2])
        shc = (sh[0], s * sh[1], sh[2])
        strips, pads, s_fea = cage(
            "RS03", shc, "y", s, to=spec("RS03")["rear"], width=20, t=5.0, phase=70
        )
        up = rounded(
            union2(disc(112, ring), pads, slot(34, ring, (bx, bz)), slot(34, ring, (fx, fz)))
        )
        # the shoulder pitch loop's harness clip, beside the motor
        uc = clip_spot(up, ring, [disc(spec("RS03")["cap_d"] + 2, ring), pads], min_r=64)
        up = cut(
            up,
            disc(spec("RS03")["cap_d"] + 2, ring),
            lighten(up, keep=[disc(100, ring), pads, disc(CLIP_PAD, uc)], pitch=20, rim=6),
            disc(CLIP_HOLE, uc),
        )
        s_pins, _, _, s_zp = zero_stop("RS03", shc, "y", "left_shoulder_pitch", True, *SHOULDER_PIN)
        s_ring, sf = front_ring(
            "RS03",
            shc,
            "y",
            s,
            t=SHOULDER_RING_T,
            outline=rounded(union2(disc(112, ring), pads)),
            keep=union2(s_zp, pads),
        )
        s_ring = s_ring - bprism(s_pins, "y", *sorted((s * (sh[1] + sf - 5), s * (sh[1] + sf + 1))))
        sp_if = [ear_face("left_shoulder_pitch", "RS03", sh, "y", 1)] if s == 1 else None
        out.append(
            made(f"torso_shoulder_ring_{side}", "torso_link", "al7075", s_ring, ifaces=sp_if)
        )
        out.append(
            made(
                f"torso_shoulder_cage_{side}",
                "torso_link",
                "al7075",
                strips,
                extra={"fea_solid": s_fea},
            )
        )
        out.append(
            made(
                f"torso_shoulder_plate_{side}",
                "torso_link",
                "al7075",
                bplate(up, "y", s * up_y, tp),
            )
        )
        for k, (x, z) in enumerate(tops):
            y0, y1 = sorted((s * (up_y + tp / 2), s * (y_low - ut / 2)))
            sp = cut(rect(x - 11, x + 11, z - 11, z + 11), disc(8.5, (x, z)))
            out.append(
                made(f"torso_spacer_{side}{k}", "torso_link", "al7075", bprism(sp, "y", y0, y1))
            )
    # the neck yaw servo hangs under the bridge by its ears, its spline up through it. The bridge
    # is an H: two cross bars in front of and behind the shoulder motors' caps, joined by the
    # servo plate between them
    neck = mounts["neck_yaw"]
    bt = 8.0  # 7075
    bridge_z = neck[2] + 5.8 + bt / 2  # on top of the servo (its spline up through the bridge)
    yb_ = up_y - tp / 2
    bridge = rounded(
        union2(rect(40.0, 56.0, -yb_, yb_), rect(-56.0, -40.0, -yb_, yb_), rect(-56, 56, -35, 35))
    )
    bridge = cut(bridge, disc(26), lighten(bridge, keep=[rect(-56, 56, -35, 35)], pitch=16, rim=5))
    out.append(made("torso_bridge", "torso_link", "al7075", bplate(bridge, "z", bridge_z, bt)))
    # the neck yaw loop's harness clip, on the bridge's back edge
    out.append(edge_clip("torso_bridge_clip", "torso_link", "z", bridge_z, bt, bridge, (-60, 0)))
    # the computer stands on edge, fan forward, its I/O ports toward the sides (cable room to
    # the frames) and its buttons up; it bolts by the feet on its underside to a plate behind
    # it, standing on the base plate
    cx0 = (comp["xyz"][0] - comp["size"][0] / 2) * 1000  # its back face (the underside)
    cz = comp["xyz"][2] * 1000
    oplate = rounded(union2(rect(-52, 52, cz - 52, cz + 52), rect(-30, 30, base_z + bt0 / 2, cz)))
    oplate = cut(oplate, lighten(oplate, pitch=20, rim=8))
    out.append(
        made(
            "torso_compute_plate",
            "torso_link",
            "al7075",
            bplate(oplate, "x", cx0 - 0.75, 1.5),
        )
    )
    return out
