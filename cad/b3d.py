"""Made metal parts in build123d (OpenCascade B-rep): flat plates from 2D outlines.

Every machined metal part is a plate of one thickness cut to a 2D outline (2.5D at most: a
pocket or a relief from one face), so the desktop mill cuts it with 2D toolpaths from plate
stock, or a laser or water-jet service cuts it from the DXF. Outlines are drawn in the plane
normal to the plate's thickness axis, in that plane's (u, v) coordinates:

  axis "x": (u, v) = (y, z);  axis "y": (u, v) = (x, z);  axis "z": (u, v) = (x, y)

Inside corners are rounded to the cutter's radius (CUTTER, a 6 mm end mill), by offsetting
the outline out and back; rectangles' outside corners to EDGE. Lightening holes
are whole circles kept `rim` inside the outline and clear of the `keep` regions (no part-holes
with sharp corners where they meet the edge).

Parts carry their B-rep (Part.brep) for STEP files (cad/export.py) and a manifold3d mesh of it
(Part.solid) for mass properties, the clearance check, FEA and STL files.
"""

from __future__ import annotations

import build123d as bd
import manifold3d as mf
import numpy as np

from .lib import Part

CUTTER = 6.0  # mm: the end mill for outlines and pockets; inside corners get its radius
EDGE = 1.5  # mm: outside corners
TOL = 0.05  # mm: tessellation chord tolerance
# the plane each plate's outline is drawn in: local x, y = (u, v), local z along the axis
PLANE = {
    "x": bd.Plane(origin=(0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0)),
    "y": bd.Plane(origin=(0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)),
    "z": bd.Plane.XY,
}


def uv(axis: str, p) -> tuple[float, float]:
    """A 3D point's in-plane coordinates for a plate normal to `axis`."""
    i = "xyz".index(axis)
    return tuple(float(p[k]) for k in range(3) if k != i)


def disc(d: float, at=(0.0, 0.0)) -> bd.Sketch:
    return bd.Pos(*at[:2]) * bd.Circle(d / 2)


def rect(u0: float, u1: float, v0: float, v1: float, r: float = EDGE) -> bd.Sketch:
    """The rectangle u0..u1 by v0..v1, its corners rounded to r."""
    w, h = abs(u1 - u0), abs(v1 - v0)
    r = min(r, 0.49 * min(w, h))
    shape = bd.RectangleRounded(w, h, r) if r > 0.05 else bd.Rectangle(w, h)
    return bd.Pos((u0 + u1) / 2, (v0 + v1) / 2) * shape


def union(*sk) -> bd.Sketch:
    out = sk[0]
    for s in sk[1:]:
        out = out + s
    return out


def hull(*sk) -> bd.Sketch:
    """The convex hull of 2D shapes."""
    edges = [e for s in sk for e in s.edges()]
    return bd.make_hull(edges)


def slot(width: float, *pts) -> bd.Sketch:
    """A strap `width` wide through `pts`, with round ends."""
    return union(
        *[hull(disc(width, a), disc(width, b)) for a, b in zip(pts[:-1], pts[1:], strict=False)]
    )


def rounded(sk: bd.Sketch, inside: float = CUTTER / 2) -> bd.Sketch:
    """Inside corners rounded to `inside` (outside corners are rounded where they're drawn:
    rect(); offsetting an offset outline again crashes OpenCascade on some outlines, so the
    result keeps the outline it came from, for lighten())."""
    out = bd.offset(bd.offset(sk, inside), -inside)
    out.raw = getattr(sk, "raw", sk)
    return out


def circles(d: float, pts) -> bd.Sketch:
    return union(*[disc(d, p) for p in pts])


def circle_pts(n: int, pcd: float, at=(0.0, 0.0), phase: float = 0.0) -> list[tuple]:
    a = phase + 2 * np.pi * np.arange(n) / n
    return [(at[0] + pcd / 2 * np.cos(x), at[1] + pcd / 2 * np.sin(x)) for x in a]


def _cs(sk, step: float = 0.5) -> mf.CrossSection:
    """A sketch as a Clipper cross-section (its edges sampled every `step` mm)."""
    loops = []
    for f in sk.faces():
        for w in [f.outer_wire(), *f.inner_wires()]:
            n = max(8, int(w.length / step))
            loops.append([(p.X, p.Y) for p in w.positions(np.linspace(0, 1, n, endpoint=False))])
    return mf.CrossSection(loops, mf.FillRule.EvenOdd)


def _sketch(cs: mf.CrossSection) -> bd.Sketch | None:
    """A Clipper cross-section as a sketch of polygonal faces."""
    faces = []
    for part in cs.decompose():
        polys = part.to_polygons()
        if not polys:
            continue
        # the outer contour is the one with the largest absolute area
        areas = [abs(_area(p)) for p in polys]
        k = int(np.argmax(areas))
        wires = [bd.Wire.make_polygon(_spaced(p), close=True) for p in polys]
        faces.append(bd.Face(wires[k], [w for i, w in enumerate(wires) if i != k]))
    return bd.Sketch(bd.Compound(faces).wrapped) if faces else None


def _spaced(p, gap: float = 0.25) -> list[tuple]:
    """A polygon's points with any closer than `gap` to the last kept one dropped (Clipper
    leaves near-duplicates that make faces OpenCascade meshes badly)."""
    out = [tuple(p[0])]
    for q in p[1:]:
        if np.hypot(q[0] - out[-1][0], q[1] - out[-1][1]) >= gap:
            out.append(tuple(q))
    if len(out) > 3 and np.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < gap:
        out.pop()
    return out


def _area(p) -> float:
    x, y = np.asarray(p).T
    return 0.5 * float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))


def lighten(
    sk: bd.Sketch,
    keep=(),
    hole: float | None = None,
    pitch: float = 20.0,
    rim: float = 6.0,
    rib: float = 4.0,
    cell: float | None = None,
):
    """Lightening pockets for an outline: an isogrid (triangles of side `cell`, 2 x `pitch` by
    default) of ribs `rib` wide, inside a rim `rim` wide round the outline and round the `keep`
    shapes (motor faces, bolt circles, bearing seats). The pockets' corners are rounded to the
    cutter's radius, and pockets too small to cut are dropped. `hole` is accepted for older
    calls and unused. Through-pockets: the holes to cut, as a sketch (or None)."""
    del hole
    cell = cell or 2 * pitch
    r = CUTTER / 2
    out = _cs(getattr(sk, "raw", sk)).offset(-rim, mf.JoinType.Round)
    for k in keep:
        out = out - _cs(k).offset(rim, mf.JoinType.Round)
    if out.is_empty():
        return None
    x0, y0, x1, y1 = out.bounds()
    c = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
    span = float(np.hypot(x1 - x0, y1 - y0)) + cell
    h = cell * np.sqrt(3) / 2  # spacing of each family of rib lines
    ribs = []
    for ang in (0.0, 60.0, 120.0):
        for k in np.arange(-span / 2 // h - 1, span / 2 // h + 2):
            strip = mf.CrossSection.square([span * 2, rib], center=True).translate([0, k * h])
            ribs.append(strip.rotate(ang).translate(c.tolist()))
    pk = out - mf.CrossSection.batch_boolean(ribs, mf.OpType.Add)
    pk = pk.offset(-r, mf.JoinType.Round).offset(r, mf.JoinType.Round)
    keepers = [p for p in pk.decompose() if p.area() >= 2.5 * np.pi * r**2]
    if not keepers:
        return None
    return _sketch(mf.CrossSection.batch_boolean(keepers, mf.OpType.Add).simplify(0.05))


def cut(sk: bd.Sketch, *holes) -> bd.Sketch:
    for h in holes:
        if h is not None:
            sk = sk - h
    return sk


def place(shape, axis: str, at: float):
    """Move a shape built about local z = 0 to its plane normal to `axis`, at `at` along it."""
    v = [0.0, 0.0, 0.0]
    v["xyz".index(axis)] = at
    return bd.Pos(*v) * (PLANE[axis].location * shape)


def plate(sk: bd.Sketch, axis: str, at: float, t: float):
    """A plate of outline `sk` and thickness `t`, its mid-plane at `at` along `axis`."""
    return place(bd.extrude(sk, amount=t / 2, both=True), axis, at)


def prism(sk: bd.Sketch, axis: str, a0: float, a1: float):
    """`sk` extruded from a0 to a1 along `axis`."""
    return plate(sk, axis, (a0 + a1) / 2, abs(a1 - a0))


def to_manifold(shape, tol: float = TOL) -> mf.Manifold:
    """The B-rep as a closed mesh. OpenCascade's mesher occasionally leaves a seam open between
    two faces at one chord tolerance; a slightly different one closes it."""
    for t in (tol, tol * 0.7, tol * 1.5, tol * 0.4):
        try:
            return _to_manifold(shape, t)
        except (ValueError, AttributeError):
            continue
    return _to_manifold(shape, tol)


def _to_manifold(shape, tol: float) -> mf.Manifold:
    v, f = shape.tessellate(tol, 0.2)
    v = np.array([(p.X, p.Y, p.Z) for p in v])
    f = np.asarray(f, dtype=np.int64)
    # faces share their edges' points but not their indices: merge them
    key = np.round(v / 1e-5).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    v, f = v[first], inv.ravel()[f]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    mesh = mf.Mesh(vert_properties=np.asarray(v, np.float32), tri_verts=np.asarray(f, np.uint32))
    m = mf.Manifold(mesh)
    if m.status() != mf.Error.NoError:
        mesh.merge()  # close what is left open within float precision
        m = mf.Manifold(mesh)
    if m.status() != mf.Error.NoError or m.is_empty():
        raise ValueError(f"tessellation is not a closed solid: {m.status()}")
    return m


def made(name: str, link: str, material: str, brep, process: str = "cnc", ifaces=None, **kw):
    """A made part from its B-rep."""
    extra = kw.pop("extra", {})
    if ifaces:
        extra["ifaces"] = ifaces
    return Part(name, link, material, to_manifold(brep), process, brep=brep, extra=extra, **kw)


def offset_box(sk: bd.Sketch, at=(0.0, 0.0)) -> bd.Sketch:
    """A rectangle covering `sk` with 2 mm to spare."""
    bb = sk.bounding_box()
    return rect(bb.min.X - 2, bb.max.X + 2, bb.min.Y - 2, bb.max.Y + 2)
