"""Motor mounting features from the actuators' real outlines (description/actuators.yaml).

A RobStride actuator is a body with its output on the front face and a smaller driver cap (with
the cable exit) standing proud of the rear face. Its stator bolts through its body's front or
rear face on the stator hole circle; whatever it drives bolts to the output flange's hole
circle. Plates stay flat (cad/b3d.py), so:

- `pillar_ring`: on the body's rear face, a pillar round each stator bolt, standing past the
  driver cap and joined by a thin ring at the far end, where a flat plate bolts on; the stator
  bolts run through the plate, the ring and the pillars into the motor. (The first pass used a
  ring on the rear face and three posts to the plate, which carried a joint's whole load
  through three 8 mm posts: finite elements put those at SF 0.2-1.3.)
- `output_plate`: a plate bolted to the output flange's hole circle, relieved 1 mm on the motor
  side outside the rotating flange so it can't rub the fixed face.

That suits the RS04, tapped on both faces. The RS03 and RS06 (and the RS00) are tapped only in
ears at their output end, a little behind the output face (actuators.yaml: `face: front`,
`setback`), so they mount from the front:

- `front_ring`: a ring (or the plate itself) on the ears, round the output, bolted from the
  front by low-head screws in counterbores. With `bearing`, its bore is the seat of a thin
  crossed-roller bearing whose inner ring sits on the driven side's hub, so the bearing takes the
  joint's axial load and tilting moment and the motor's own bearing mostly just the torque;
- `hub`: a disc on the output flange that stands the driven part off past the ring's front face
  (and carries the bearing's inner ring);
- `cage`: four flat strips round the motor, from a front ring back past the driver cap to a
  plate, where the structure is on the motor's rear side.

Positions are along the motor's axis in its link frame (mm): the motor centre is the middle of
its overall length (envelope), so the output face is at +t/2 along the output direction, the
cap's end at -t/2, and the body's rear face at -(t/2 - cap length).
"""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
_acts = yaml.safe_load((ROOT / "description" / "actuators.yaml").read_text())
_joints = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())["joints"]
_ranges = {
    j["name"].replace("{side}", "left"): j["range"]
    for j in yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())["joints"]
}
CLEAR = {"M3": 3.4, "M4": 4.5, "M5": 5.5}  # bolt clearance holes


def spec(model: str) -> dict:
    a = _acts[model]
    d, _, t = a["envelope_mm"]
    sh = a.get("shape") or {}
    cap = sh.get("cap_len") or 0.0
    return dict(
        d=d,
        t=t,
        body_d=sh.get("body_d", d),
        cap_d=sh.get("cap_d"),
        cap=cap,
        output_d=sh.get("output_d", d / 2),
        ears_d=sh.get("ears_d"),
        rear=t / 2 - cap,  # body rear face, from the centre
        holes=a.get("holes", {}),
        # the front mounting face (ears), from the centre along the output
        ear=t / 2 - (a.get("holes", {}).get("stator", {}).get("setback") or 0.0),
    )


# thin crossed-roller bearings (inner d, outer D, width B, mm; mass g; basic ratings kN). THK RA
# series figures as listed by distributors: confirm against the maker's sheet before ordering
BEARINGS = {"RA6008": dict(d=60.0, D=76.0, B=8.0, mass_g=90.0, C=5.68, C0=8.68)}
HUB_BASE = 66.0  # the hub's shoulder under the bearing's inner ring
LIP = 72.0  # the ring's bore under the bearing's outer ring (clears the shoulder)
HEAD = {"M3": (6.5, 2.3), "M4": (8.0, 3.1)}  # low-head (DIN 7984) counterbores: diameter, depth


def _vec(axis: str, sign: int, dist: float):
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = sign * dist
    return v


def _at(centre, axis, sign, dist):
    return [c + o for c, o in zip(centre, _vec(axis, sign, dist), strict=True)]


# --- interfaces: where a part is driven or carries the next link, for cad/fea.py. Each is a dict:
# the joint it serves, and a patch of the part's surface (cad.fea.Patch's fields): "face", an
# annulus r0-r1 on the plane through `centre` normal to `axis`; "cyl", a cylinder of radius r1
# about `axis` through `centre`, `span` along it.
_AX = {"x": (1.0, 0.0, 0.0), "y": (0.0, 1.0, 0.0), "z": (0.0, 0.0, 1.0)}


def face(joint: str, centre, axis: str, r0: float, r1: float, rigid: bool = True) -> dict:
    """`rigid` False: a load spread over the face (the ground), not a bolted part."""
    out = dict(joint=joint, kind="face", centre=tuple(centre), axis=_AX[axis], r0=r0, r1=r1)
    return out if rigid else {**out, "rigid": False}


def bore(joint: str, centre, axis: str, d: float, length: float) -> dict:
    """A bearing bore or pin hole of diameter d through a plate `length` thick."""
    span = (-length / 2, length / 2)
    return dict(joint=joint, kind="cyl", centre=tuple(centre), axis=_AX[axis], r1=d / 2, span=span)


def output_face(joint: str, model: str, centre, axis: str, sign: int) -> dict:
    """The contact face of a part bolted to a motor's output flange."""
    s = spec(model)
    return face(joint, _at(centre, axis, sign, s["t"] / 2), axis, 0.0, s["output_d"] / 2)


def stator_face(joint: str, model: str, centre, axis: str, sign: int) -> dict:
    """The contact face of a rear ring on a motor's body (its stator bolt circle)."""
    s = spec(model)
    r0 = (s["cap_d"] + 2) / 2 if s["cap_d"] else s["body_d"] / 2 - 10
    r1 = s["holes"]["stator"]["pcd"] / 2 + 7 if "stator" in s["holes"] else s["body_d"] / 2
    return face(joint, _at(centre, axis, -sign, s["rear"]), axis, r0, r1)


def ear_face(joint: str, model: str, centre, axis: str, sign: int) -> dict:
    """The contact face of a front ring on a motor's mounting ears (its stator bolt circle)."""
    s = spec(model)
    h = s["holes"]["stator"]
    r0 = h["pcd"] / 2 - CLEAR[h["thread"]] / 2 - 2
    return face(joint, _at(centre, axis, sign, s["ear"]), axis, r0, h["pcd"] / 2 + 4)


__all__ = [
    "bore",
    "face",
    "output_face",
    "output_holes",
    "output_plate",
    "pillar_ring",
    "BEARINGS",
    "cage",
    "ear_face",
    "front_ring",
    "hub",
    "ring_band",
    "ring_od",
    "spec",
    "stator_face",
    "stator_holes",
    "output_windows",
    "zero_stop",
]


# --- build123d mounts (cad/b3d.py): plates stay flat; motors stand off them on pillar rings --


def _uvc(axis: str, centre):
    from . import b3d  # noqa: PLC0415

    return b3d.uv(axis, centre)


def stator_holes(model: str, centre, axis: str):
    """2D clearance holes for a motor's stator bolts, in the plane normal to `axis`."""
    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["stator"]
    return b3d.circles(CLEAR[h["thread"]], b3d.circle_pts(h["n"], h["pcd"], _uvc(axis, centre)))


def output_holes(model: str, centre, axis: str):
    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["output"]
    return b3d.circles(CLEAR[h["thread"]], b3d.circle_pts(h["n"], h["pcd"], _uvc(axis, centre)))


def ring_od(model: str) -> float:
    """The outside diameter of a stator mounting: the body, or round the bolt ears."""
    s = spec(model)
    return max(s["body_d"], s["holes"]["stator"]["pcd"] + 14)


def pillar_ring(
    model: str,
    centre,
    axis: str,
    sign: int,
    base: float = 2.5,
    clear: float = 1.0,
    od: float | None = None,
):
    """A motor's stator mounting: a pillar round each stator bolt, from the body's rear face
    out past the driver cap (`clear` mm), joined by a ring `base` thick at the far end, where a
    flat plate bolts on. The stator bolts run through the plate, the ring and the pillars into
    the motor, clamping them together: (B-rep, the plate's near face from the motor centre,
    along -sign); `od` trims the ring's outside diameter where space is short. Between the
    pillars, room for the cable exit and air round the cap."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    h = s["holes"]["stator"]
    if h.get("face", "rear") == "front":
        raise ValueError(f"{model} is tapped only at its output end: use front_ring")
    c = _uvc(axis, centre)
    r_in = (s["cap_d"] + 2) / 2 if s["cap_d"] else s["body_d"] / 2 - 10
    r_out = (od or ring_od(model)) / 2
    w = CLEAR[h["thread"]] + 7  # 3.5 mm of wall either side of the bolt
    # each pillar a radial strip `w` wide across the ring; the windows between them run right
    # through the ring's width (straight sides, no corners to round, nothing tangent)
    pillars = []
    for k in range(h["n"]):
        a = 360.0 * k / h["n"]
        strip = b3d.rect(r_in - 3, r_out + 3, -w / 2, w / 2, r=0)
        pillars.append(b3d.bd.Pos(*c) * (b3d.bd.Rot(0, 0, a) * strip))
    holes = stator_holes(model, centre, axis)
    ring = b3d.cut(b3d.disc(2 * r_out, c), b3d.disc(2 * r_in, c), holes)
    # milled from one piece: the ring's full height, with windows cut between the pillars
    windows = b3d.cut(b3d.disc(2 * r_out + 2, c), b3d.disc(2 * r_in - 2, c), *pillars)
    i = "xyz".index(axis)
    rear = centre[i] - sign * s["rear"]
    near = s["rear"] + s["cap"] + clear  # the plate's near face, from the motor centre
    far = centre[i] - sign * near
    body = b3d.prism(ring, axis, rear, far)
    body = body - b3d.prism(windows, axis, rear + sign, far + sign * base)
    return body, near


def output_plate(
    model: str, outline, centre, axis: str, sign: int, t: float, at: float | None = None, d=None
):
    """A plate of outline `outline` bolted to a motor's output flange: bolt holes on the output
    circle, and relieved 1 mm on the motor side outside the rotating flange so it can't rub the
    fixed face. On a hub, `at` is the hub's face (from the motor centre) and `d` the relief's
    diameter (hub()'s third value).
    (B-rep, the plate's mid-plane along `axis`)."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    i = "xyz".index(axis)
    face = centre[i] + sign * (s["t"] / 2 if at is None else at)
    mid = face + sign * t / 2
    body = b3d.plate(b3d.cut(outline, output_holes(model, centre, axis)), axis, mid, t)
    c = _uvc(axis, centre)
    relief = b3d.cut(b3d.offset_box(outline, c), b3d.disc(d or s["output_d"] + 2, c))
    body = body - b3d.prism(relief, axis, face - sign, face + sign)
    return body, mid


def ring_band(model: str, centre, axis: str):
    """Where a plate under a motor's pillar ring must stay solid when it is pocketed: a pad
    round each stator bolt (the pillar ring's own base spreads the load between them)."""
    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["stator"]
    pts = b3d.circle_pts(h["n"], h["pcd"], _uvc(axis, centre))
    return b3d.circles(CLEAR[h["thread"]] + 6, pts)


def front_ring(
    model: str,
    centre,
    axis: str,
    sign: int,
    t: float | None = None,
    outline=None,
    bearing: str | None = None,
    pocket: bool = True,
    keep=None,
):
    """A ring on a motor's front mounting ears, round its output: `outline` (a disc round the
    ears by default) less the bore and the stator holes, counterbored from the front for low-head
    screws. `t` is its thickness from the ear face; with `bearing` it runs to 1 mm past the
    output face plus the bearing's width, the bearing's outer ring in a seat from the front on a
    lip. A ring is pocketed between the bolts (from the rear if thick, through if thin), except
    at `keep` (the zero and stop pins, strips' and legs' pads). (B-rep, its front face from the
    motor centre along sign)."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    h = s["holes"]["stator"]
    c = _uvc(axis, centre)
    i = "xyz".index(axis)
    out_f = s["t"] / 2
    if bearing:
        front = out_f + 1 + BEARINGS[bearing]["B"]
        bore = LIP
    else:
        front = s["ear"] + (t or 6.0)
        bore = s["output_d"] + 2
    sk = outline if outline is not None else b3d.disc(ring_od(model), c)
    sk = b3d.cut(sk, b3d.disc(bore, c), stator_holes(model, centre, axis))
    body = b3d.prism(sk, axis, centre[i] + sign * s["ear"], centre[i] + sign * front)
    cb_d, cb_h = HEAD[h["thread"]]
    pts = b3d.circle_pts(h["n"], h["pcd"], c)
    body = body - b3d.prism(
        b3d.circles(cb_d, pts), axis, centre[i] + sign * (front - cb_h), centre[i] + sign * front
    )
    if pocket:
        # lightening pockets from the rear between the bolt pads, leaving a 4 mm web at the front
        r_in = (BEARINGS[bearing]["D"] if bearing else bore) / 2 + 3
        # a thin ring only inside its screw circle, so its outer half stays whole
        r_out = ring_od(model) / 2 - 2.5 if front - s["ear"] > 7 else h["pcd"] / 2 + 1
        pads = b3d.circles(CLEAR[h["thread"]] + 7, pts)
        pk = b3d.cut(b3d.disc(2 * r_out, c), b3d.disc(2 * r_in, c), pads, keep)
        pk = b3d.opened(pk)
        if pk is not None:
            # a thick ring: pockets from the rear, a 2 mm web left at the front; a thin one:
            # through-pockets between the screws
            a1 = front - 2 if front - s["ear"] > 7 else front + 1
            body = body - b3d.prism(
                pk, axis, centre[i] + sign * (s["ear"] - 1), centre[i] + sign * a1
            )
    if bearing:
        seat = b3d.disc(BEARINGS[bearing]["D"], c)
        body = body - b3d.prism(
            seat, axis, centre[i] + sign * (out_f + 1), centre[i] + sign * (front + 1)
        )
    return body, front


def hub(model: str, centre, axis: str, sign: int, front: float, bearing: str | None = None):
    """A hub on a motor's output flange standing the driven part past a front ring's face
    `front` (from the motor centre), bolted through with it: a disc the output's size, 0.5 mm
    proud of the ring; with `bearing`, flush with it (the driven part clamps the bearing's
    inner ring), a shoulder and a spigot for the inner ring, bored out from the driven side to a
    3 mm web round the output's screws. (B-rep, its face from the motor centre, the diameter outside
    which the driven part is relieved)."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    c = _uvc(axis, centre)
    i = "xyz".index(axis)
    out_f = s["t"] / 2
    holes = b3d.union(output_holes(model, centre, axis), b3d.disc(12.0, c))
    if not bearing:
        top = front + 0.5
        d = s["output_d"]
        body = b3d.prism(b3d.cut(b3d.disc(d, c), holes), axis, *_span(centre[i], sign, out_f, top))
        return body, top, d + 2
    top = front
    base = b3d.cut(b3d.disc(HUB_BASE, c), holes)
    body = b3d.prism(base, axis, *_span(centre[i], sign, out_f, out_f + 1))
    spigot = b3d.cut(b3d.disc(BEARINGS[bearing]["d"], c), holes)
    body = body + b3d.prism(spigot, axis, *_span(centre[i], sign, out_f + 0.5, top))
    body = body - b3d.prism(b3d.disc(50.0, c), axis, *_span(centre[i], sign, out_f + 3, top + 1))
    return body, top, HUB_BASE


def _span(c0: float, sign: int, a: float, b: float) -> tuple[float, float]:
    return c0 + sign * a, c0 + sign * b


def cage(
    model: str,
    centre,
    axis: str,
    sign: int,
    to: float,
    width: float = 30.0,
    t: float = 6.0,
    phase: float = 45.0,
    angles=None,
):
    """Flat strips `t` thick round a front-mounted motor (four, at `phase` + k x 90 deg, or at
    `angles`), clear of its body and ears by 1 mm, from its front ring's rear face (the ear
    face) back to `to` (from the motor centre, against sign: a plate's face beyond the driver
    cap); each is bolted to the ring and the plate through its ends. (B-rep, the strips'
    footprint for the ring's and plate's outlines)."""
    from . import b3d  # noqa: PLC0415

    s = spec(model)
    i = "xyz".index(axis)
    r = max(s["body_d"], s["ears_d"] or 0) / 2 + 1 + t / 2
    c = _uvc(axis, centre)
    a0, a1 = centre[i] + sign * s["ear"], centre[i] - sign * to
    pads = []
    for a in angles if angles is not None else [phase + k * 90.0 for k in range(4)]:
        strip = b3d.rect(r - t / 2, r + t / 2, -width / 2, width / 2, r=0)
        pads.append(b3d.bd.Pos(*c) * (b3d.bd.Rot(0, 0, a) * strip))
    foot = b3d.union(*pads)
    # for cad/fea.py: 1 mm into the ring and the plate, its sides 0.5 mm inside theirs (no
    # coincident faces), so the check bonds the strips to both
    core = b3d.union(
        *[
            b3d.bd.Pos(*c)
            * (
                b3d.bd.Rot(0, 0, a)
                * b3d.rect(r - t / 2 + 0.5, r + t / 2 - 0.5, -width / 2 + 0.5, width / 2 - 0.5, r=0)
            )
            for a in (angles if angles is not None else [phase + k * 90.0 for k in range(4)])
        ]
    )
    fea = b3d.prism(core, axis, a0 + sign, a1 - sign)
    return b3d.prism(foot, axis, a0, a1), foot, b3d.to_manifold(fea)


# --- zeroing and hard stops on a front-ringed joint (Roboto Origin sets its joints' zeros with
# pins and printed jigs; this is the pin half of that)
_N_UV = {"x": (1, 0, 0), "y": (0, -1, 0), "z": (0, 0, 1)}  # the (u, v) plane's right-hand normal
PIN = {"M3": 3.0, "M4": 4.0}  # dowel diameter, by the motor's stator thread


def zero_stop(
    model: str,
    centre,
    axis: str,
    joint: str,
    groove_on_child: bool,
    r: float,
    alpha: float,
    margin: float = 3.0,
):
    """A joint's hard stops and zero pin on a motor's front ring: a stop dowel pressed into
    the ring's front face at radius `r`, angle `alpha` (deg, in the plate plane's (u, v)),
    standing proud into an arc groove in the driven part's face that spans the joint's range
    (robot.yaml, left side) plus `margin` degrees either way, so the groove's ends are the hard
    stops; and a zero hole half a turn round in both parts, lined up at zero, for a second
    dowel (or a printed jig's peg) when the joint's zero offset is set. `groove_on_child`: the
    driven part is the joint's child link. Returns sketches: (the ring's two pin holes, the
    groove, the driven part's zero hole, pads round the pin holes to keep solid when the ring is
    pocketed)."""
    import numpy as np  # noqa: PLC0415

    from . import b3d  # noqa: PLC0415

    d = PIN[spec(model)["holes"]["stator"]["thread"]]
    c = _uvc(axis, centre)
    s = 1 if np.dot(_joints[joint]["axis"], _N_UV[axis]) > 0 else -1
    lo, hi = _ranges[joint][0] - margin, _ranges[joint][1] + margin
    if groove_on_child:  # the ring's pin, seen from the child, turns by -q
        a0, a1 = alpha - s * hi, alpha - s * lo
    else:
        a0, a1 = alpha + s * lo, alpha + s * hi
    beta = alpha + 180.0

    def at(a):
        return (c[0] + r * np.cos(np.radians(a)), c[1] + r * np.sin(np.radians(a)))

    ring = b3d.circles(d, [at(alpha), at(beta)])
    groove = b3d.arc_slot(c, r, d + 0.4, min(a0, a1), max(a0, a1))
    pads = b3d.circles(d + 8, [at(alpha), at(beta)])
    return ring, groove, b3d.disc(d, at(beta)), pads


def output_windows(model: str, centre, axis: str, r0: float, r1: float):
    """Lightening windows in a plate on a motor's output, between radii r0 and r1, with a rib
    5 mm wide at each output screw (and pin): sector windows, their corners rounded to the
    cutter's. A 2D sketch (or None)."""
    import numpy as np  # noqa: PLC0415

    from . import b3d  # noqa: PLC0415

    h = spec(model)["holes"]["output"]
    c = _uvc(axis, centre)
    win = b3d.cut(b3d.disc(2 * r1, c), b3d.disc(2 * r0, c))
    ribs = []
    for a in np.degrees(2 * np.pi * np.arange(h["n"]) / h["n"]):
        strip = b3d.rect(0, r1 + 2, -2.5, 2.5, r=0)
        ribs.append(b3d.bd.Pos(*c) * (b3d.bd.Rot(0, 0, a) * strip))
    return b3d.opened(b3d.cut(win, *ribs))
