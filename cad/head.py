"""Head and neck parts, first pass.

Built in the neck and head link frames (mm) of description/skeleton.yaml around the two neck
servos (STS3250, 45.3 x 35.8 x 24.8 mm) and the depth camera (Orbbec Gemini 335L, 124 x 29 x
27 mm, 133 g):

- neck bracket: a plate on the neck yaw servo's horn (the servo sits in the torso bridge) and
  two cheeks cradling the neck pitch servo;
- head frame: two plates on the pitch servo's horn and idler, joined by a bridge with an arm
  forward under the camera, and a top bar that carries the shell;
- head shell: a wide, deep rounded box (156 x 203 x 121 mm), the shape of the concept head
  the body shells are being brought towards: where its plan, side and front outlines (`PLAN`,
  `SIDE`, `FRONT`) extruded across it overlap, 1.8 mm wall, open underneath for the neck. It is
  printed as five light panels split where the concept's are (crown, chin, two cheeks, a rear
  cap) and a dark visor band across the face, at eye height;
- camera frame: a dark printed sleeve round the camera, in an opening in the visor band, so
  the camera reads as part of the band. The camera's front is level with the face's front on
  the centre line and nothing is ahead of its lenses (a window behind the curved face let
  through only about a third of its view). The sleeve is only the part outside the shell,
  glued into its opening; the camera slides in from the front and screws to an arm from the
  head frame by its tripod thread underneath;
- the camera, as a bought part, with the outline of Orbbec's model of it (OrbbecSDK_ROS2,
  Apache-2.0, fetched by tools/fetch_vendor_cad.py): a 124 x 29 mm stadium (fully rounded
  ends), 27 mm deep, its lenses 4 mm behind the front.

The head sits on a light neck sleeve (cad/covers.py) that runs up inside its open underside.
No fasteners yet.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .b3d import clip_spot, cut, disc, made, plate, prism, rect
from .lib import Part, box, cyl, hull
from .mounts import face as contact

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
WALL = 1.8
# The head's outlines (mm), measured from the concept's head panels: PLAN half-width by x;
# SIDE rear and front x by height; FRONT half-width by height. Heights are from the bottom of
# the shell's rim plus 43 mm; HEAD_RISE puts them in the head frame (over the pitch axis), with
# the visor band's middle (outline height 20) at eye height.
PLAN = [
    (-97.6, 0.0),
    (-96.0, 48.8),
    (-90.0, 59.3),
    (-84.0, 65.3),
    (-78.0, 69.2),
    (-72.0, 71.9),
    (-66.0, 73.8),
    (-60.0, 75.2),
    (-54.0, 76.1),
    (-48.0, 76.8),
    (-36.0, 77.6),
    (-18.0, 78.0),
    (24.0, 78.0),
    (42.0, 77.7),
    (54.0, 77.1),
    (66.0, 75.7),
    (78.0, 75.0),
    (84.0, 74.3),
    (90.0, 73.0),
    (96.0, 70.4),
    (102.0, 64.9),
    (105.8, 0.0),
]
SIDE = [
    (-43.0, -48.0, 64.0),
    (-42.0, -51.4, 71.1),
    (-36.0, -60.0, 87.7),
    (-30.0, -67.8, 93.8),
    (-24.0, -74.4, 96.1),
    (-18.0, -79.8, 98.1),
    (-12.0, -84.0, 100.0),
    (-6.0, -87.9, 102.0),
    (0.0, -91.4, 103.8),
    (6.0, -94.1, 105.2),
    (12.0, -95.8, 105.8),
    (18.0, -96.5, 105.8),
    (30.0, -96.9, 105.1),
    (42.0, -97.1, 104.1),
    (48.0, -97.4, 103.4),
    (54.0, -97.6, 100.3),
    (60.0, -97.5, 95.4),
    (66.0, -96.0, 85.1),
    (72.0, -92.2, 73.8),
    (78.0, -76.8, 52.4),
]
FRONT = [
    (-43.0, 48.0),
    (-42.0, 50.7),
    (-36.0, 57.3),
    (-30.0, 63.2),
    (-24.0, 67.9),
    (-18.0, 71.2),
    (-12.0, 73.2),
    (-6.0, 74.9),
    (0.0, 76.3),
    (6.0, 77.4),
    (12.0, 77.9),
    (24.0, 78.0),
    (36.0, 77.7),
    (48.0, 77.2),
    (54.0, 76.4),
    (60.0, 75.0),
    (66.0, 71.9),
    (72.0, 67.8),
    (78.0, 56.6),
]
HEAD_RISE = 72.0
EDGE_R = 12.0  # every edge rounded off by this much (where the outlines meet they would crease)
EYE = 20.0  # the visor band's middle, outline height
# the panels' seams, outline frame (the concept's): the crown above CROWN_Z, the visor band
# ahead of VISOR_X between CHIN_Z and CROWN_Z, the chin ahead of VISOR_X below it, the rear cap
# behind REAR_X, and the cheeks between, split on the centre line
CROWN_Z, CHIN_Z, VISOR_X, REAR_X = 52.0, -11.0, 60.0, -40.0
CAMERA = (124.0, 29.0, 27.0, 133.0)  # width, height, depth (mm), mass (g)
CAMERA_Z = EYE + HEAD_RISE  # at eye height, in the middle of the visor band
# its front level with the head's front on the centre line
CAMERA_FRONT = float(np.interp(EYE, [z for z, _, _ in SIDE], [x for _, _, x in SIDE]))
CAMERA_AT = (CAMERA_FRONT - CAMERA[2] / 2, 0.0, CAMERA_Z)  # its centre
FRAME_RIM = 3.0  # the camera frame's wall
FRAME_GAP = 0.3  # round the camera, and round the frame where it passes through the shell
TRIPOD_BACK = 15.8  # its tripod thread behind its front (Orbbec's URDF: 11.7 behind the lenses)
HEAD_FRAME_FRONT = 7.5  # the head frame's bridge (front face); the camera arm runs forward from it


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
    bridge_top = neck_mount[2] + s_hi + 8.0 - neck[2]  # cad/torso.py: an 8 mm bridge
    base_z = bridge_top + 2.0 + 2.0  # 4 mm base, 2 mm over the bridge
    case_bot = pitch[2] - s_off - s_len / 2  # the pitch servo's case, along -z below the shaft
    ymid = (s_hi + s_lo) / 2
    sy = s_hi - s_lo  # the case's extent along the shaft
    cw = s_wid + 6  # the cradle, 3 mm round the case
    # flat 7075 plates: a hub on the horn, a base plate, and two cheeks down the pitch servo's
    # sides that hold it by its mounting ears
    hub = prism(disc(20), "z", horn_top, base_z - 2.0)
    base_sk = rect(-20, 20, -37, 13)
    cheek_fp = [
        rect(g * (s_wid / 2) - 1, g * (s_wid / 2 + 3.5), ymid - sy / 2 - 4, ymid + sy / 2 + 4)
        for g in (1, -1)
    ]
    nc = clip_spot(base_sk, (0.0, 0.0), [disc(20), *cheek_fp], min_r=18, d=12.0)  # neck yaw loop
    base = prism(cut(base_sk, disc(6.5, nc)), "z", base_z - 2.0, base_z + 0.5)
    cheeks = [
        prism(
            rect(g * (s_wid / 2), g * (s_wid / 2 + 2.5), ymid - sy / 2 - 3, ymid + sy / 2 + 3),
            "z",
            base_z + 0.5,
            case_bot + 25,
        )
        for g in (1, -1)
    ]
    nb_if = [contact("neck_yaw", (0, 0, horn_top), "z", 0, 8)]
    nb_if += [
        contact("neck_pitch", (g * s_wid / 2, ymid, case_bot + 12.5), "x", 0, 20) for g in (1, -1)
    ]
    out.append(made("neck_hub", "neck_link", "al7075", hub, ifaces=nb_if[:1]))
    out.append(made("neck_bracket", "neck_link", "al7075", base))
    for name, ch_ in zip(("l", "r"), cheeks, strict=True):
        out.append(made(f"neck_cheek_{name}", "neck_link", "al7075", ch_, ifaces=nb_if[1:]))

    # --- head frame: side plates on the pitch servo's horn (+y) and idler (-y), a bridge with an
    # arm forward to the camera's tripod thread, and a top bar joining them
    cx, cy, cz = CAMERA_AT
    cw, ch, cd, cmass = CAMERA
    back = HEAD_FRAME_FRONT
    y_horn = s_hi + horn_t + 1.5
    y_idler = s_lo - 2.5
    # flat 7075 plates: side plates (3 mm), the bridge and top bar (4 mm), the camera arm (6 mm,
    # for the tripod thread)
    hf = [
        made(
            f"head_frame_side_{n}",
            "head_link",
            "al7075",
            plate(rect(-10, back, -5, 105), "y", y, 3.0),
        )
        for n, y in (("horn", y_horn), ("idler", y_idler))
    ]
    hf.append(
        made(
            "head_frame_bridge",
            "head_link",
            "al7075",
            plate(
                rect(y_idler + 1.5, y_horn - 1.5, cz - 11, min(cz + 11, 101.0)), "x", back - 2, 4.0
            ),
        )
    )
    arm_top = cz - ch / 2 - FRAME_GAP  # under the camera, to just past its tripod thread
    arm_end = CAMERA_FRONT - TRIPOD_BACK + 8
    hf.append(
        made(
            "head_frame",
            "head_link",
            "al7075",
            plate(rect(back - 4, arm_end, -8, 8), "z", arm_top - 3, 6.0),
        )
    )
    hf.append(
        made(
            "head_frame_top",
            "head_link",
            "al7075",
            plate(rect(-10, back, y_idler + 1.5, y_horn - 1.5), "z", 103, 4.0),
        )
    )
    out.extend(hf)

    # --- shell: the outlines extruded across each other, less the same inset by the wall;
    # open underneath (the inside runs out through the bottom)
    outer = head_solid(0.0)
    inner = head_solid(WALL, open_below=True)
    big = 400.0
    # the camera frame: a stadium sleeve from the head frame's bridge to the camera's front
    length = CAMERA_FRONT - HEAD_FRAME_FRONT
    mid = (CAMERA_FRONT + HEAD_FRAME_FRONT) / 2
    rim = FRAME_GAP + FRAME_RIM
    sleeve_out = stadium(cw + 2 * rim, ch + 2 * rim, length, mid, cz)
    socket = stadium(cw + 2 * FRAME_GAP, ch + 2 * FRAME_GAP, length + 2, mid, cz)
    through = stadium(
        cw + 2 * (2 * FRAME_GAP + FRAME_RIM), ch + 2 * (2 * FRAME_GAP + FRAME_RIM), length, mid, cz
    )
    skin = (outer - inner) - through

    def zone(x0, x1, z0, z1, y0=-big, y1=big):  # outline frame
        return box(
            (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2 + HEAD_RISE)
        )

    lo, hi = -100.0, 200.0
    panels = {
        "head_visor": zone(VISOR_X, hi, CHIN_Z, CROWN_Z),
        "head_shell_crown": zone(-hi, hi, CROWN_Z, hi),
        "head_shell_chin": zone(VISOR_X, hi, lo, CHIN_Z),
        "head_shell_rear": zone(-hi, REAR_X, lo, CROWN_Z),
        "head_shell_cheek_l": zone(REAR_X, VISOR_X, lo, CROWN_Z, 0.0, big),
        "head_shell_cheek_r": zone(REAR_X, VISOR_X, lo, CROWN_Z, -big, 0.0),
    }
    for name, z in panels.items():
        out.append(Part(name, "head_link", "pa_cf", skin ^ z, "print"))
    frame = (sleeve_out - socket) - inner  # only the part outside the shell
    out.append(Part("head_camera_frame", "head_link", "pa_cf", frame, "print"))
    cam = stadium(cw, ch, cd, cx, cz)
    out.append(Part("head_camera", "head_link", "bought", cam, "bought", mass_g=cmass))
    return out


def _outline(pts, rx: float):
    """A closed 2D outline, rounded off by `rx` both ways (removes the sampling's facets)."""
    import manifold3d as mf  # noqa: PLC0415

    cs = mf.CrossSection([pts], mf.FillRule.NonZero)
    return cs.offset(rx, mf.JoinType.Round, circular_segments=48).offset(
        -rx, mf.JoinType.Round, circular_segments=48
    )


def head_solid(inset: float = 0.0, open_below: bool = False):
    """The head's solid (head frame): its plan, side and front outlines extruded across one
    another and intersected, each outline inset by `inset`, and every edge rounded by EDGE_R
    (built EDGE_R smaller, then grown by a sphere); with `open_below`, run out 30 mm through
    the bottom (so the shell is open underneath)."""
    import manifold3d as mf  # noqa: PLC0415

    big = 400.0
    drop = 30.0 if open_below else 0.0
    plan = [(x, y) for x, y in PLAN] + [(x, -y) for x, y in reversed(PLAN)]
    side = [(x0, z) for z, x0, _ in SIDE] + [(x1, z) for z, _, x1 in reversed(SIDE)]
    front = [(y, z) for z, y in FRONT] + [(-y, z) for z, y in reversed(FRONT)]
    zb, zt = SIDE[0][0], SIDE[-1][0]
    if drop:  # extend the side and front outlines straight down
        side = [(SIDE[0][1], zb - drop)] + side + [(SIDE[0][2], zb - drop)]
        front = [(FRONT[0][1], zb - drop)] + front + [(-FRONT[0][1], zb - drop)]

    def ext(pts, how):
        cs = _outline(pts, 3.0)
        cs = cs.offset(-(inset + EDGE_R), mf.JoinType.Round, circular_segments=48)
        m = cs.extrude(big).translate([0, 0, -big / 2])
        if how == "plan":  # (x, y), along z
            return m
        if how == "side":  # (x, z), along y
            return m.rotate([90, 0, 0])
        return m.rotate([90, 0, 90])  # front: (y, z), along x

    solid = ext(plan, "plan") ^ ext(side, "side") ^ ext(front, "front")
    if not drop:
        solid = solid ^ box((big, big, zt - zb - 2 * EDGE_R), (0, 0, (zb + zt) / 2))
    solid = solid.minkowski_sum(mf.Manifold.sphere(EDGE_R, 32))
    return solid.translate([0, 0, HEAD_RISE])


def stadium(length: float, height: float, depth: float, x_mid: float, z: float):
    """A stadium (a slot with fully rounded ends) facing +x: length along y, centred on y = 0."""
    r = height / 2
    return hull(*[cyl(height, depth, "x", (x_mid, s * (length / 2 - r), z)) for s in (1, -1)])
