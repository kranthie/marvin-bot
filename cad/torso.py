"""Pelvis and torso structure, first pass.

Built in the pelvis and torso link frames (mm) of description/skeleton.yaml, around the motors
at their STEP-model sizes:

- pelvis: a back plate carrying the two hip roll RS04s behind the hips (with stub axles for
  the roll brackets' far-side bearings), a top plate carrying the waist RS03, and two gussets
  under it making a channel; the hip pitch motors swing round the roll axes clear of all of
  it. The pelvis electronics sit between the roll motors.
- torso: a flange on the waist output and a base plate; two uprights carrying the shoulder
  pitch RS03s on their outer faces (flat profiles, longer than the desktop mill's travel, so
  cut by an online laser or water-jet service); a top bridge carrying the neck yaw servo; and a
  printed chest shell (front and sides, open at the back for the battery).

The computer, battery and electronics stay as skeleton.yaml estimate boxes, placed inside this
structure. Plates are pocketed; no fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .lib import Part, box, cyl, hull, lighten, strap, union

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
    # (axes along x, output faces forward); the hip pitch motors ride on the roll brackets and
    # swing round the roll axes, so nothing of the pelvis sits where they go: |y| < 35 mm below
    # z = 45 mm, and x > -66 mm (in front of their back edge) only above that.
    rc = mounts["left_hip_roll"]  # roll motor centre (pelvis frame)
    hip_t = 8.0
    bx = rc[0] - t04 / 2 - hip_t / 2  # back plate, on the roll motors' back faces
    waist_z = j["waist_yaw"][2]
    top_z = waist_z - t03 / 2 - t / 2  # under the waist RS03
    back = union(
        *[cyl(100, hip_t, "x", (bx, s * rc[1], rc[2])) for s in (1, -1)],
        box((hip_t, 2 * rc[1], 60), (bx, 0, rc[2])),
        box((hip_t, 80, top_z - rc[2] + t / 2), (bx, 0, (top_z + t / 2 + rc[2]) / 2)),
    )
    back = lighten(
        back,
        "x",
        keep=[cyl(70, 40, "x", (bx, s * rc[1], rc[2])) for s in (1, -1)],
        hole=16,
        pitch=22,
        rim=9,
    )
    # stub axles behind the roll motors for the roll brackets' far-side bearings
    stubs = [cyl(30, 14, "x", (bx - hip_t / 2 - 7, s * rc[1], rc[2])) for s in (1, -1)]
    out.append(Part("pelvis_back", "pelvis", "al6061", union(back, *stubs), "service"))
    wx = j["waist_yaw"][0]  # the waist motor sits a little ahead of the hips
    front_x = wx + 50.0
    bf = bx + hip_t / 2  # back plate's front face
    top = box((front_x - bf, 90, t), ((front_x + bf) / 2, 0, top_z))
    top = union(top, cyl(100, t, "z", (wx, 0, top_z)))  # under the waist motor
    top = lighten(top, "z", keep=[cyl(56, 40, "z", (wx, 0, top_z))], hole=18, pitch=24, rim=9)
    out.append(Part("pelvis_top", "pelvis", "al6061", top, "service"))  # 210 mm: laser cut
    gy = 23.0  # gussets: vertical plates under the top plate, a channel with it
    for side, s in (("l", 1), ("r", -1)):
        g = union(
            box(
                (-66 - bf, t, top_z - t / 2 + 40), ((bf - 66) / 2, s * gy, (top_z - t / 2 - 40) / 2)
            ),
            box(
                (front_x + 66, t, top_z - t / 2 - 45),
                ((front_x - 66) / 2, s * gy, (top_z - t / 2 + 45) / 2),
            ),
        )
        g = lighten(g, "y", hole=14, pitch=20, rim=8)
        out.append(Part(f"pelvis_gusset_{side}", "pelvis", "al6061", g, "service"))

    # --- torso: frame on the waist axis at the waist motor's centre; output face up
    out_z = t03 / 2
    flange = cyl(84, t, "z", (0, 0, out_z + t / 2))
    base_z = out_z + t + t / 2
    sh = mounts["left_shoulder_pitch"]  # shoulder pitch RS03 centre (torso frame)
    # The shoulder motors sit deep in the torso (narrow shoulders), deeper than the battery is
    # wide, so each side frame is two plates: a lower A-frame outside the battery, and an
    # upper plate against the shoulder motor's inner face, above the battery and computer,
    # joined by two spacers clear of the motor
    bat = next(b for b in _skel["bodies"] if b["budget"] == "battery")
    y_low = bat["size"][1] * 1000 / 2 + 6 + t / 2
    up_y = sh[1] - t03 / 2 - t / 2
    # the frame runs from behind the battery to just ahead of the computer standing on edge
    fx0, fx1 = -95.0, 55.0
    tops = ((-70.0, sh[2] - 18), (42.0, sh[2] - 58))  # back and front spacers (x, z)
    base = box((fx1 - fx0, 2 * y_low - t, t), ((fx0 + fx1) / 2, 0, base_z))  # between the plates
    base = lighten(base, "z", keep=[cyl(60, 40, "z", (0, 0, base_z))], hole=22, rim=10)
    out.append(Part("torso_waist_flange", "torso_link", "al6061", flange, "cnc"))
    out.append(Part("torso_base", "torso_link", "al6061", base, "cnc"))
    gap = y_low - up_y - t
    for side, s in (("l", 1), ("r", -1)):
        (bx, bz), (fx, fz) = tops
        low = union(
            box((fx1 - fx0 - 10, t, 12), ((fx0 + fx1) / 2, s * y_low, base_z + 6)),
            strap(30, t, "y", (fx0 + 20, s * y_low, base_z + 12), (bx, s * y_low, bz)),
            strap(30, t, "y", (fx1 - 20, s * y_low, base_z + 12), (fx, s * y_low, fz)),
            strap(24, t, "y", (fx0 + 15, s * y_low, 140), (fx1 - 10, s * y_low, 140)),
        )
        low = lighten(low, "y", hole=12, pitch=18, rim=7)
        out.append(Part(f"torso_upright_{side}", "torso_link", "al6061", low, "service"))
        ring = (sh[0], s * up_y, sh[2])
        up = union(
            cyl(86, t, "y", ring),
            strap(30, t, "y", ring, (bx, s * up_y, bz)),
            strap(30, t, "y", ring, (fx, s * up_y, fz)),
        )
        up = lighten(up, "y", keep=[cyl(70, 40, "y", ring)], hole=12, pitch=18, rim=7)
        out.append(Part(f"torso_shoulder_plate_{side}", "torso_link", "al6061", up, "cnc"))
        for k, (x, z) in enumerate(tops):
            yc = s * (up_y + t / 2 + gap / 2)
            spacer = box((20, gap, 20), (x, yc, z)) - cyl(7, gap + 2, "y", (x, yc, z))
            out.append(Part(f"torso_spacer_{side}{k}", "torso_link", "al6061", spacer, "cnc"))
    neck = mounts["neck_yaw"]
    bridge_z = neck[2] + 12.4 + t / 2  # on top of the neck yaw servo
    bridge = box((70, 2 * up_y - t, t), (0, 0, bridge_z)) - cyl(30, 20, "z", (0, 0, bridge_z))
    out.append(Part("torso_bridge", "torso_link", "al6061", lighten(bridge, "z", hole=16), "cnc"))
    # chest shell: front and sides, 2.5 mm, open at the back (battery) and top and bottom
    z0, z1 = base_z + t, sh[2] + 20
    outer = hull(
        box((118, 2 * y_low + 2 * t + 6, 2), (-1, 0, z0)),
        box((124, 2 * y_low + 2 * t + 6, 2), (0, 0, z1)),
    )
    inner_s = hull(
        box((113, 2 * y_low + 2 * t + 1, 4), (-1, 0, z0)),
        box((119, 2 * y_low + 2 * t + 1, 4), (0, 0, z1)),
    )
    shell = outer - inner_s - box((80, 400, 1000), (-45 - 40, 0, 0))  # open back
    shell = shell - cyl(d03 + 10, 400, "y", (sh[0], 0, sh[2]))  # clear the shoulder motors
    out.append(Part("torso_chest_shell", "torso_link", "pa_cf", shell, "print"))
    return out
