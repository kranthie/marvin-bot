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
  neck yaw servo; and a printed chest shell (front and sides, open at the back for the battery).

The computer and electronics are laid out by cad/electrical.py; the battery and keep-alive pack
are skeleton.yaml estimate boxes. Plates are pocketed; no fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .b3d import cut, disc, hull, lighten, made, rect, rounded, slot, uv
from .b3d import plate as bplate
from .b3d import prism as bprism
from .b3d import union as union2
from .leg import HIP_BACK_T
from .lib import Part, box, cyl
from .lib import hull as mhull
from .mounts import (
    bore,
    output_face,
    output_plate,
    pillar_ring,
    ring_band,
    ring_od,
    spec,
    stator_face,
    stator_holes,
)

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())


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
    # (axes along x, output faces forward) on pillar rings on a back plate; the hip roll yokes
    # (cad/leg.py) swing round the roll axes and their rear bearings ride on stub axles behind
    # it. Each hip's moment about y reaches the middle in a closed box behind the back plate
    # (low, under the rear bearings), and two deep gussets (y-normal, 23 mm either side of the
    # centre line) carry it up and forward to the waist; its moment about x goes up the back
    # plate's centre spine into a closed box along x under the waist motor (the top plate, the
    # gussets' upper parts and a floor). The thighs swing to the centre line under the front of
    # the gussets (hip roll -30 deg), so they stop short there. The pelvis electronics sit on
    # the top plate behind the waist motor (cad/electrical.py).
    rc = mounts["left_hip_roll"]  # roll motor centre (pelvis frame)
    hip_t = HIP_BACK_T
    r_near = pillar_ring("RS04", rc, "x", 1)[1]
    bf = rc[0] - r_near  # back plate's front face
    bb = bf - hip_t  # and its back face
    waist_c = (j["waist_yaw"][0], 0.0, j["waist_yaw"][2])
    w_ring, w_near = pillar_ring("RS03", waist_c, "z", 1, base=2.0)
    top_t = 6.0
    top_z0 = waist_c[2] - w_near - top_t  # the top plate's underside
    top_z1 = top_z0 + top_t
    rx = bb - 30.0  # the box's back: under the yokes' rear plates
    bz0, bz1, bw = -62.0, -32.0, 3.0  # the box: below the rear bearings (r = 29 mm)
    yb = 150.0  # its ends
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
    stub_l = 22.0  # to the roll yokes' far-side bearings
    for side, s in (("l", 1), ("r", -1)):
        c = (rc[0], s * rc[1], rc[2])
        sc = (bb - stub_l / 2, s * rc[1], rc[2])
        stub = bplate(cut(disc(30, uv("x", sc)), disc(8.5, uv("x", sc))), "x", sc[0], stub_l)
        out.append(
            made(
                f"pelvis_stub_{side}",
                "pelvis",
                "al6061",
                stub,
                ifaces=[bore("left_hip_roll", sc, "x", 30, stub_l)] if s == 1 else None,
            )
        )
        out.append(
            made(
                f"pelvis_roll_mount_{side}",
                "pelvis",
                "al7075",
                pillar_ring("RS04", c, "x", 1)[0],
                ifaces=[stator_face("left_hip_roll", "RS04", c, "x", 1)] if s == 1 else None,
            )
        )
    # the box behind the back plate: floor, roof and back wall, each in two halves (the mill's
    # travel) meeting at the centre, closed at the ends and at the gussets
    for side, s in (("l", 1), ("r", -1)):
        y0, y1 = sorted((0.0, s * yb))
        for name, z in (("floor", bz0 + bw / 2), ("roof", bz1 - bw / 2)):
            sk = rect(rx, bb, y0, y1, r=0)
            out.append(
                made(f"pelvis_box_{name}_{side}", "pelvis", "al7075", bplate(sk, "z", z, bw))
            )
        wall = rect(y0, y1, bz0 + bw, bz1 - bw, r=0)
        out.append(
            made(f"pelvis_box_back_{side}", "pelvis", "al7075", bplate(wall, "x", rx + bw / 2, bw))
        )
        for name, y in (("end", s * (yb - 2.0)), ("mid", s * gy)):
            sk = rect(rx + bw, bb, bz0 + bw, bz1 - bw, r=0)
            out.append(
                made(f"pelvis_box_{name}_{side}", "pelvis", "al7075", bplate(sk, "y", y, 4.0))
            )
    # the gussets: behind the back plate (over the box) and in front of it, clear of the thighs
    # in front (|x| < 40 mm below z = 40 mm)
    kz = 45.0  # the top box's floor
    for side, s in (("l", 1), ("r", -1)):
        rear_g = rounded(rect(rx, bb, bz1, top_z0, r=0))
        front_g = rounded(
            rect(bf, waist_c[0] + 50, bz0, top_z0, r=0) - rect(-40.0, 200.0, bz0 - 1, 40.0, r=0)
        )
        for name, sk in (("rear", rear_g), ("front", front_g)):
            # solid along the top box's floor, and under the top plate where the waist loads come in
            keep = [rect(-200, 200, kz - 4, kz + 9), rect(-60, 90, top_z0 - 16, top_z0)]
            sk = cut(sk, lighten(sk, keep=keep, pitch=22))
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
            bplate(rect(bf, -40.0, -gy + gt / 2, gy - gt / 2), "z", kz + 2.5, 5.0),
        )
    )
    top = rounded(hull(disc(ring_od("RS03") + 4, (waist_c[0], 0)), rect(bb, -60.0, -28.0, 28.0)))
    top = cut(
        top,
        stator_holes("RS03", waist_c, "z"),
        lighten(
            top,
            keep=[ring_band("RS03", waist_c, "z")]
            + [rect(-70, 90, s * gy - 12, s * gy + 12) for s in (1, -1)],  # over the gussets
            pitch=24,
            rim=6,
        ),
    )
    out.append(
        made(
            "pelvis_top",
            "pelvis",
            "al7075",
            bplate(top, "z", (top_z0 + top_z1) / 2, top_t),
        )
    )
    out.append(
        made(
            "pelvis_waist_mount",
            "pelvis",
            "al7075",
            w_ring,
            ifaces=[stator_face("waist_yaw", "RS03", waist_c, "z", 1)],
        )
    )

    # --- torso: frame on the waist axis at the waist motor's centre; output face up
    out_z = t03 / 2
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
    up_y = sh[1] - spec("RS03")["rear"] - tp / 2  # on the motors' body rear faces (caps inboard)
    # the frame runs from behind the battery to just ahead of the computer standing on edge
    fx0, fx1 = -95.0, 55.0
    tops = ((-70.0, sh[2] - 18), (42.0, sh[2] - 58))  # back and front spacers (x, z)
    flange, _ = output_plate("RS03", disc(84), (0, 0, 0), "z", 1, t)
    base_z = out_z + t + t / 2
    out.append(
        made(
            "torso_waist_flange",
            "torso_link",
            "al6061",
            flange,
            ifaces=[output_face("waist_yaw", "RS03", (0, 0, 0), "z", 1)],
        )
    )
    base = rect(fx0, fx1, -y_bot + t / 2, y_bot - t / 2)  # between the side frames
    base = cut(base, lighten(base, keep=[disc(100)], pitch=28, rim=8))
    out.append(made("torso_base", "torso_link", "al7075", bplate(base, "z", base_z, t)))
    # the battery shelf, clear of the computer plate in front
    sx0, sx1 = fx0 + 5, (bat["xyz"][0] + bat["size"][0] / 2) * 1000 + 2
    ut = 5.0  # the lower side frames
    shelf = rect(sx0, sx1, -y_low + ut / 2, y_low - ut / 2)  # between the side frames
    shelf = cut(shelf, lighten(shelf, hole=18, pitch=24, rim=7))
    out.append(
        made(
            "torso_battery_shelf",
            "torso_link",
            "pa_cf",
            bplate(shelf, "z", bat_floor - 2.5, 5.0),
            "print",
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
                "al6061",
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
        up = rounded(union2(disc(112, ring), slot(34, ring, (bx, bz)), slot(34, ring, (fx, fz))))
        up = cut(
            up,
            disc(spec("RS03")["cap_d"] + 2, ring),
            stator_holes("RS03", (sh[0], s * sh[1], sh[2]), "y"),
            lighten(up, keep=[ring_band("RS03", (sh[0], s * sh[1], sh[2]), "y")], pitch=20, rim=6),
        )
        sp_if = [stator_face("left_shoulder_pitch", "RS03", sh, "y", 1)] if s == 1 else None
        out.append(
            made(
                f"torso_shoulder_plate_{side}",
                "torso_link",
                "al7075",
                bplate(up, "y", s * up_y, tp),
                ifaces=sp_if,
            )
        )
        for k, (x, z) in enumerate(tops):
            y0, y1 = sorted((s * (up_y + tp / 2), s * (y_low - ut / 2)))
            sp = cut(rect(x - 11, x + 11, z - 11, z + 11), disc(8.5, (x, z)))
            out.append(
                made(f"torso_spacer_{side}{k}", "torso_link", "al6061", bprism(sp, "y", y0, y1))
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
    bridge = cut(bridge, disc(26))
    out.append(made("torso_bridge", "torso_link", "al7075", bplate(bridge, "z", bridge_z, bt)))
    # the computer stands on edge, fan forward, its I/O ports toward the sides (cable room to
    # the frames) and its buttons up; it bolts by the feet on its underside to a plate behind
    # it, standing on the base plate
    cx0 = (comp["xyz"][0] - comp["size"][0] / 2) * 1000  # its back face (the underside)
    cz = comp["xyz"][2] * 1000
    oplate = rounded(union2(rect(-52, 52, cz - 52, cz + 52), rect(-30, 30, base_z + t / 2, cz)))
    oplate = cut(oplate, lighten(oplate, pitch=20, rim=8))
    out.append(
        made(
            "torso_compute_plate",
            "torso_link",
            "pa_cf",
            bplate(oplate, "x", cx0 - 2.5, 5.0),
            "print",
            notes="printed: it carries only the computer",
        )
    )
    # chest shell: front and sides, 2 mm, open at the back (battery) and top and bottom
    z0, z1 = base_z + t, sh[2] + 20
    outer = mhull(
        box((118, 2 * y_bot + 2 * t + 6, 2), (-1, 0, z0)),
        box((124, 2 * y_low + 2 * t + 6, 2), (0, 0, bat_floor)),
        box((124, 2 * y_low + 2 * t + 6, 2), (0, 0, z1)),
    )
    inner_s = mhull(
        box((114, 2 * y_bot + 2 * t + 2, 4), (-1, 0, z0)),
        box((120, 2 * y_low + 2 * t + 2, 4), (0, 0, bat_floor)),
        box((120, 2 * y_low + 2 * t + 2, 4), (0, 0, z1)),
    )
    shell = outer - inner_s - box((80, 400, 1000), (-45 - 40, 0, 0))  # open back
    shell = shell - cyl(d03 + 10, 400, "y", (sh[0], 0, sh[2]))  # clear the shoulder motors
    # intake vents over the computer's fan
    for k in range(-3, 4):
        shell = shell - box((20, 7, 80), (60, k * 13.0, cz))
    # in two pieces, upper and lower, each inside the printer's 250 mm
    split = box((400, 400, 400), (0, 0, (z0 + z1) / 2 + 200))
    out.append(Part("torso_chest_shell_upper", "torso_link", "pa_cf", shell ^ split, "print"))
    out.append(Part("torso_chest_shell_lower", "torso_link", "pa_cf", shell - split, "print"))
    return out
