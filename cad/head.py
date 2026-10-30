"""Head and neck parts, first pass.

Built in the neck and head link frames (mm) of description/skeleton.yaml around the two neck
servos (STS3250, 45.3 x 35.8 x 24.8 mm) and the depth camera (Orbbec Gemini 335L, 124 x 29 x
27 mm, 133 g):

- neck bracket: a plate on the neck yaw servo's horn (the servo sits in the torso bridge) and
  two cheeks cradling the neck pitch servo;
- head frame: two plates on the pitch servo's horn and idler, joined by a bridge with an arm
  forward under the camera, and a top bar that carries the shell;
- head shell: a helmet, taller than it is wide and narrowing to the chin (the hull of two
  ellipsoids), 2.5 mm wall, open at the bottom for the neck; light, like the neck and body
  shells, behind the visor;
- visor: the whole face, a separate glossy dark part, ahead of a plane that runs from the
  back of the crown down to the chin (so the light shell behind it reads as jaw and neck);
- camera frame: a light printed sleeve round the camera, like a spectacle frame across the
  face. The camera's front is level with the face's front on the centre line and nothing is
  ahead of its lenses (a window behind the curved face let through only about a third of its
  view); being straight, its ends stand clear of the round face. The sleeve is only the part
  outside the helmet, glued into its opening in the visor; the camera slides in from the front
  and screws to an arm from the head frame by its tripod thread underneath;
- the camera, as a bought part, with the outline of Orbbec's model of it (OrbbecSDK_ROS2,
  Apache-2.0, fetched by tools/fetch_vendor_cad.py): a 124 x 29 mm stadium (fully rounded
  ends), 27 mm deep, its lenses 4 mm behind the front.

The helmet sits on a light neck sleeve (cad/covers.py). No fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .lib import Part, box, cyl, ellipsoid, hull, union
from .mounts import face as contact

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
WALL = 2.5
# helmet: the hull of a crown ellipsoid and a smaller chin ellipsoid (centre, semi-axes; head frame)
# deeper than it is wide, sitting forward of the neck, the chin forward and low, the crown
# back and high
CROWN = (4.0, 0.0, 94.0), (82.0, 71.0, 84.0)
CHIN = (28.0, 0.0, 42.0), (50.0, 54.0, 40.0)
CAMERA = (124.0, 29.0, 27.0, 133.0)  # width, height, depth (mm), mass (g)
CAMERA_Z = 92.0  # at eye height: half way up the head
# its front level with the helmet's front on the centre line
CAMERA_FRONT = CROWN[0][0] + CROWN[1][0] * np.sqrt(
    1 - ((CAMERA_Z - CROWN[0][2]) / CROWN[1][2]) ** 2
)
CAMERA_AT = (CAMERA_FRONT - CAMERA[2] / 2, 0.0, CAMERA_Z)  # its centre
FRAME_RIM = 3.0  # the camera frame's wall
FRAME_GAP = 0.3  # round the camera, and round the frame where it passes through the shell
TRIPOD_BACK = 15.8  # its tripod thread behind its front (Orbbec's URDF: 11.7 behind the lenses)
HEAD_FRAME_FRONT = 7.5  # the head frame's bridge (front face); the camera arm runs forward from it
VISOR_X = 14.0  # the visor is the whole face ahead of this plane (at eye height) ...
VISOR_TILT = 28.0  # ... leaning back this far (degrees): from the back of the crown to the chin
NECK_CUT_Z = 20.0  # the shell's open bottom over the neck axis; the neck sleeve
NECK_CUT_SLOPE = 0.35  # (cad/covers.py) runs up inside it. The underside rises towards the
# back of the head (mm up per mm back), from the chin up to where the neck enters


def parts() -> list[Part]:
    sv = _acts["STS3250"]["shape"]  # case length x width, axial extent about the shaft
    s_len, s_wid = sv["box"]
    s_lo, s_hi = sv["axial"]  # case bottom and spline top, along the shaft from its datum
    s_off = sv["shaft_offset"]  # case centre from the shaft, along its length
    horn_t = 3.0  # the servo horn, on the spline
    neck_mount = [x * 1000 for x in _skel["motor_mounts"]["neck_yaw"]["xyz"]]
    neck = [x * 1000 for x in _skel["joints"]["neck_yaw"]["xyz"]]
    pitch = [x * 1000 for x in _skel["joints"]["neck_pitch"]["xyz"]]
    out: list[Part] = []

    # --- neck link: the yaw servo hangs under the torso bridge, its horn just above the bridge;
    # a hub on the horn carries the base plate clear of the bridge, and two cheeks hold the
    # pitch servo, which stands case-down below its shaft, by its mounting ears
    horn_top = neck_mount[2] + s_hi + horn_t - neck[2]  # neck frame
    bridge_top = neck_mount[2] + s_hi + 6.0 - neck[2]  # cad/torso.py
    base_z = bridge_top + 2.0 + 2.0  # 4 mm base, 2 mm over the bridge
    hub = cyl(16, base_z - 2.0 - horn_top, "z", (0, 0, (horn_top + base_z - 2.0) / 2))
    base = box((40, 50, 4), (0, -12, base_z))
    case_bot = pitch[2] - s_off - s_len / 2  # the pitch servo's case, along -z below the shaft
    ymid = (s_hi + s_lo) / 2
    cradle = box(
        (s_wid + 6, s_hi - s_lo + 6, case_bot - base_z), (0, ymid, (case_bot + base_z) / 2)
    )
    cheeks = [
        box((3, s_hi - s_lo, 25), (sgn * (s_wid / 2 + 1.5), ymid, case_bot + 12.5))
        for sgn in (1, -1)
    ]
    cradle = cradle - box(
        (s_wid - 4, s_hi - s_lo + 8, case_bot - base_z - 8), (0, ymid, (case_bot + base_z) / 2)
    )  # an open frame, not a block
    bracket = union(hub, base, cradle, *cheeks)
    nb_if = [contact("neck_yaw", (0, 0, horn_top), "z", 0, 8)]
    nb_if += [
        contact("neck_pitch", (g * s_wid / 2, ymid, case_bot + 12.5), "x", 0, 20) for g in (1, -1)
    ]
    out.append(Part("neck_bracket", "neck_link", "al6061", bracket, "cnc", extra={"ifaces": nb_if}))

    # --- head frame: side plates on the pitch servo's horn (+y) and idler (-y), a bridge with an
    # arm forward to the camera's tripod thread, and a top bar joining them
    cx, cy, cz = CAMERA_AT
    cw, ch, cd, cmass = CAMERA
    back = HEAD_FRAME_FRONT
    y_horn = s_hi + horn_t + 1.5
    y_idler = s_lo - 2.5
    plates = [box((back + 10, 3, 110), ((back - 10) / 2, y, 50)) for y in (y_horn, y_idler)]
    span = y_horn - y_idler + 3
    bridge = box((4, span, 22), (back - 2, (y_horn + y_idler) / 2, cz))
    arm_top = cz - ch / 2 - FRAME_GAP  # under the camera, to just past its tripod thread
    arm_end = CAMERA_FRONT - TRIPOD_BACK + 8
    arm = box((arm_end - (back - 4), 16, 9), ((arm_end + back - 4) / 2, 0, arm_top - 4.5))
    top = box((back + 10, span, 4), ((back - 10) / 2, (y_horn + y_idler) / 2, 103))
    out.append(Part("head_frame", "head_link", "pa_cf", union(*plates, bridge, arm, top), "print"))

    # --- shell and visor
    outer = hull(ellipsoid(CROWN[1], CROWN[0]), ellipsoid(CHIN[1], CHIN[0]))
    inner = hull(
        ellipsoid([x - WALL for x in CROWN[1]], CROWN[0]),
        ellipsoid([x - WALL for x in CHIN[1]], CHIN[0]),
    )
    big = 400.0
    under = (
        box((big, big, big), (0, 0, -big / 2))
        .rotate([0, np.degrees(np.arctan(NECK_CUT_SLOPE)), 0])
        .translate([0, 0, NECK_CUT_Z])
    )
    # the camera frame: a stadium sleeve from the head frame's bridge to the camera's front
    length = CAMERA_FRONT - HEAD_FRAME_FRONT
    mid = (CAMERA_FRONT + HEAD_FRAME_FRONT) / 2
    rim = FRAME_GAP + FRAME_RIM
    sleeve_out = stadium(cw + 2 * rim, ch + 2 * rim, length, mid, cz)
    socket = stadium(cw + 2 * FRAME_GAP, ch + 2 * FRAME_GAP, length + 2, mid, cz)
    through = stadium(
        cw + 2 * (2 * FRAME_GAP + FRAME_RIM), ch + 2 * (2 * FRAME_GAP + FRAME_RIM), length, mid, cz
    )
    skin = ((outer - inner) - under) - through
    face = (
        box((big, big, big), (big / 2, 0, 0))
        .rotate([0, -VISOR_TILT, 0])
        .translate([VISOR_X, 0, cz])
    )
    out.append(Part("head_shell", "head_link", "pa_cf", skin - face, "print"))
    out.append(Part("head_visor", "head_link", "pa_cf", skin ^ face, "print"))
    frame = (sleeve_out - socket) - inner  # only the part outside the helmet
    out.append(Part("head_camera_frame", "head_link", "pa_cf", frame, "print"))
    cam = stadium(cw, ch, cd, cx, cz)
    out.append(Part("head_camera", "head_link", "bought", cam, "bought", mass_g=cmass))
    return out


def stadium(length: float, height: float, depth: float, x_mid: float, z: float):
    """A stadium (a slot with fully rounded ends) facing +x: length along y, centred on y = 0."""
    r = height / 2
    return hull(*[cyl(height, depth, "x", (x_mid, s * (length / 2 - r), z)) for s in (1, -1)])
