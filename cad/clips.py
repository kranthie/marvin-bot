"""Printed harness edge clips, for the plates too thick for a push-in cable-tie mount.

A service loop is tied to a plate either side of its joint (tools/build_harness.py's CLIPS). On
plates up to 6 mm thick that is a push-in "fir-tree" cable-tie mount in a round hole on a solid
pad (cad/mounts.py: CLIP_HOLE, CLIP_PAD, placed by cad/b3d.py's clip_spot). On thicker plates it
is this clip: a U that slides over the plate's edge and grips it (two jaws and a bridge, sized to
the plate's thickness), with a ring outside the edge that a cable tie or the harness passes
through. Printed in TPU, which takes the snap strain that stiff PA-CF does not.
"""

from __future__ import annotations

import numpy as np

from . import b3d
from .lib import Part, box, cyl, union

JAW, REACH, WIDTH = 2.0, 10.0, 10.0  # jaw thickness, how far onto the plate, clip width (mm)


def edge_point(sk, target):
    """The point on a plate outline's outer edge nearest `target` (its (u, v)), and the edge's
    outward normal there."""
    best = None
    for f in sk.faces():
        w = f.outer_wire()
        n = max(64, int(w.length / 1.0))
        pts = np.array([(p.X, p.Y) for p in w.positions(np.linspace(0, 1, n, endpoint=False))])
        k = int(np.argmin(np.hypot(pts[:, 0] - target[0], pts[:, 1] - target[1])))
        d = float(np.hypot(*(pts[k] - target)))
        if best is None or d < best[0]:
            tan = pts[(k + 1) % n] - pts[k - 1]
            best = (d, pts[k], tan / np.linalg.norm(tan))
    _, p, tan = best
    normal = np.array([tan[1], -tan[0]])
    # outward: a step along it leaves the outline
    probe = b3d._cs(getattr(sk, "raw", sk)) ^ b3d.mf.CrossSection.circle(0.3, 8).translate(
        (p + 2 * normal).tolist()
    )
    if probe.area() > 0.1:
        normal = -normal
    return p, normal


def edge_clip(name: str, link: str, axis: str, mid: float, t: float, sk, target) -> Part:
    """A clip on the edge of a plate `t` thick (its mid-plane at `mid` along `axis`, outline
    `sk`), at the edge point nearest `target`."""
    p, nrm = edge_point(sk, target)
    i = "xyz".index(axis)
    a_w = np.zeros(3)
    a_w[i] = 1.0
    n_w = np.insert(nrm, i, 0.0)
    s_w = np.cross(a_w, n_w)
    origin = np.insert(p, i, mid)
    # in the clip's own frame: x outward from the edge, y along it, z through the plate
    jaws = [box((REACH, WIDTH, JAW), (-REACH / 2, 0, s * (t / 2 + JAW / 2))) for s in (1, -1)]
    bridge = box((JAW, WIDTH, t + 2 * JAW), (JAW / 2, 0, 0))
    ring = cyl(12, WIDTH, "y", (JAW + 6, 0, 0)) - cyl(7, WIDTH + 2, "y", (JAW + 6, 0, 0))
    clip = union(*jaws, bridge, ring)
    m = np.column_stack([n_w, s_w, a_w, origin])
    return Part(
        name,
        link,
        "tpu",
        clip.transform(m.tolist()),
        "print",
        notes=f"harness edge clip for a {t:g} mm plate (cad/clips.py)",
    )
