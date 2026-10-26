"""Head and neck parts, first pass.

Built in the neck and head link frames (mm) of description/skeleton.yaml around the two neck
servos (STS3250, 45.3 x 35.8 x 24.8 mm) and the depth camera (Orbbec Gemini 335L, 124 x 29 x
27 mm, 133 g):

- neck bracket: a plate on the neck yaw servo's horn (the servo sits in the torso bridge) and
  two cheeks cradling the neck pitch servo;
- head frame: two plates on the pitch servo's horn and idler, joined by a bridge that carries
  the camera and a top bar that carries the shell;
- head shell: a helmet, taller than it is wide and narrowing to the chin (the hull of two
  ellipsoids), 2.5 mm wall, open at the bottom for the neck;
- visor: the whole face, a separate glossy part, with a narrow slot in front of the camera's
  lenses and projector (no material in front of the optics);
- the camera, as a bought part.

The helmet sits on a dark neck sleeve (cad/covers.py). No fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .lib import Part, box, ellipsoid, hull, union

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
WALL = 2.5
# helmet: the hull of a crown ellipsoid and a smaller chin ellipsoid (centre, semi-axes; head frame)
CROWN = (2.0, 0.0, 76.0), (72.0, 80.0, 92.0)
CHIN = (12.0, 0.0, 30.0), (54.0, 50.0, 40.0)
CAMERA = (124.0, 29.0, 27.0, 133.0)  # width, height, depth (mm), mass (g)
CAMERA_AT = (19.0, 0.0, 55.0)  # its centre: the front corners just inside the visor
VISOR_X = 25.0  # the visor is the whole face ahead of this plane
NECK_CUT_Z = 6.0  # the shell's open bottom; the neck sleeve (cad/covers.py) runs up inside it


def parts() -> list[Part]:
    sw, sl, st = _acts["STS3250"]["envelope_mm"]  # servo length (x), height (z), thickness (y)
    neck_mount = [x * 1000 for x in _skel["motor_mounts"]["neck_yaw"]["xyz"]]
    neck = [x * 1000 for x in _skel["joints"]["neck_yaw"]["xyz"]]
    pitch = [x * 1000 for x in _skel["joints"]["neck_pitch"]["xyz"]]
    horn_top = neck_mount[2] + sl / 2 - neck[2] + 2  # above the torso bridge (neck frame)
    out: list[Part] = []

    # --- neck link: plate on the yaw horn, cheeks cradling the pitch servo at its ends
    base = box((60, 34, 4), (0, 0, horn_top + 2))
    cheek_h = pitch[2] - horn_top
    cheeks = [box((4, 30, cheek_h), (s * (sw / 2 + 2), 0, horn_top + cheek_h / 2)) for s in (1, -1)]
    out.append(Part("neck_bracket", "neck_link", "al6061", union(base, *cheeks), "cnc"))

    # --- head frame: side plates on the pitch servo's horn and idler (y = +-(st/2 + 1.5)),
    # a camera bridge and a top bar
    cx, cy, cz = CAMERA_AT
    cw, ch, cd, cmass = CAMERA
    back = cx - cd / 2  # the camera's back face
    # side plates kept inside r = 22 mm of the pitch axis below it, clear of the neck cheeks
    plates = [box((back + 20, 3, 110), ((back - 20) / 2, s * (st / 2 + 3), 50)) for s in (1, -1)]
    bridge = box((4, st + 9, 22), (back - 2, 0, cz))  # the camera bolts to this
    top = box((back + 20, st + 9, 4), ((back - 20) / 2, 0, 103))
    out.append(Part("head_frame", "head_link", "pa_cf", union(*plates, bridge, top), "print"))

    # --- shell and visor
    outer = hull(ellipsoid(CROWN[1], CROWN[0]), ellipsoid(CHIN[1], CHIN[0]))
    inner = hull(
        ellipsoid([x - WALL for x in CROWN[1]], CROWN[0]),
        ellipsoid([x - WALL for x in CHIN[1]], CHIN[0]),
    )
    big = 400.0
    skin = outer - inner
    shell = skin - box((big, big, big), (VISOR_X + big / 2, 0, 0))
    shell = shell - box((big, big, big), (0, 0, NECK_CUT_Z - big / 2))
    out.append(Part("head_shell", "head_link", "pa_cf", shell, "print"))
    visor = skin ^ box((big, big, big), (VISOR_X + big / 2, 0, 0))
    visor = visor - box((big, big, big), (0, 0, NECK_CUT_Z - big / 2))
    visor = visor - box((80, cw - 4, ch - 9), (cx + 40, 0, cz))  # optics slot
    out.append(Part("head_visor", "head_link", "pa_cf", visor, "print"))
    cam = box((cd, cw, ch), CAMERA_AT)
    out.append(Part("head_camera", "head_link", "bought", cam, "bought", mass_g=cmass))
    return out
