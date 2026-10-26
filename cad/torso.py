"""Pelvis and torso structure, first pass.

Built in the pelvis and torso link frames (mm) of description/skeleton.yaml, around the motors
at their STEP-model sizes:

- pelvis: a box frame. Two side plates carry the hip pitch RS04s on their inner faces; front
  and back cross plates join them; a top plate carries the waist RS03. The space inside the
  frame holds the pelvis electronics.
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

    # --- pelvis: frame on the hip pitch axes; hip pitch RS04s centred at y = +-75
    hip_y = j["left_hip_pitch"][1]
    side_y = hip_y - t04 / 2 - t / 2  # inner faces of the hip motors
    waist_z = j["waist_yaw"][2]
    top_z = waist_z - t03 / 2 - t / 2  # under the waist RS03
    for name, s in (("pelvis_side_l", 1), ("pelvis_side_r", -1)):
        # a ring on the hip motor and a strap up to a bar under the top plate
        plate = union(
            cyl(100, t, "y", (0, s * side_y, 0)),
            strap(54, t, "y", (0, s * side_y, 0), (0, s * side_y, top_z - 20)),
            box((110, t, 12), (0, s * side_y, top_z - 3)),
        )
        plate = lighten(plate, "y", keep=[cyl(80, 40, "y", (0, s * side_y, 0))], hole=18, rim=10)
        out.append(Part(name, "pelvis", "al6061", plate, "cnc"))
    inner = 2 * side_y - t
    for name, s in (("pelvis_front", 1), ("pelvis_back", -1)):
        plate = box((t, inner, top_z + t / 2 + 40), (s * 55, 0, (top_z + t / 2 - 40) / 2))
        out.append(Part(name, "pelvis", "al6061", lighten(plate, "x", hole=20, rim=9), "cnc"))
    top = box((116, inner + 2 * t, t), (0, 0, top_z))
    top = lighten(top, "z", keep=[cyl(56, 40, "z", (0, 0, top_z))], hole=20, rim=9)
    out.append(Part("pelvis_top", "pelvis", "al6061", top, "cnc"))

    # --- torso: frame on the waist axis at the waist motor's centre; output face up
    out_z = t03 / 2
    flange = cyl(84, t, "z", (0, 0, out_z + t / 2))
    base_z = out_z + t + t / 2
    sh = mounts["left_shoulder_pitch"]  # shoulder pitch RS03 centre (torso frame)
    up_y = sh[1] - t03 / 2 - t / 2  # uprights against the shoulder motors' inner faces
    # the frame runs from behind the battery to just ahead of the computer standing on edge
    fx0, fx1 = -95.0, 55.0
    base = box((fx1 - fx0, 2 * up_y + t, t), ((fx0 + fx1) / 2, 0, base_z))
    base = lighten(base, "z", keep=[cyl(60, 40, "z", (0, 0, base_z))], hole=22, rim=10)
    out.append(Part("torso_waist_flange", "torso_link", "al6061", flange, "cnc"))
    out.append(Part("torso_base", "torso_link", "al6061", base, "cnc"))
    for name, s in (("torso_upright_l", 1), ("torso_upright_r", -1)):
        # an A-frame: two straps from the base to a ring on the shoulder motor's stator
        ring = (sh[0], s * up_y, sh[2])
        up = union(
            box((fx1 - fx0 - 10, t, 12), ((fx0 + fx1) / 2, s * up_y, base_z + 6)),
            strap(30, t, "y", (fx0 + 20, s * up_y, base_z + 12), ring),
            strap(30, t, "y", (fx1 - 20, s * up_y, base_z + 12), ring),
            cyl(86, t, "y", ring),
        )
        up = lighten(up, "y", keep=[cyl(70, 40, "y", ring)], hole=12, pitch=18, rim=7)
        out.append(Part(name, "torso_link", "al6061", up, "service"))
    neck = mounts["neck_yaw"]
    bridge_z = neck[2] + 12.4 + t / 2  # on top of the neck yaw servo
    bridge = box((70, 2 * up_y - t, t), (0, 0, bridge_z)) - cyl(30, 20, "z", (0, 0, bridge_z))
    out.append(Part("torso_bridge", "torso_link", "al6061", lighten(bridge, "z", hole=16), "cnc"))
    # chest shell: front and sides, 2.5 mm, open at the back (battery) and top and bottom
    z0, z1 = base_z + t, sh[2] + 20
    outer = hull(
        box((118, 2 * up_y + 2 * t + 6, 2), (-1, 0, z0)),
        box((124, 2 * up_y + 2 * t + 6, 2), (0, 0, z1)),
    )
    inner_s = hull(
        box((113, 2 * up_y + 2 * t + 1, 4), (-1, 0, z0)),
        box((119, 2 * up_y + 2 * t + 1, 4), (0, 0, z1)),
    )
    shell = outer - inner_s - box((80, 400, 1000), (-45 - 40, 0, 0))  # open back
    shell = shell - cyl(d03 + 10, 400, "y", (sh[0], 0, sh[2]))  # clear the shoulder motors
    out.append(Part("torso_chest_shell", "torso_link", "pa_cf", shell, "print"))
    return out
