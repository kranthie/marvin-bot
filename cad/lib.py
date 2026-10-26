"""Small code-CAD toolkit on manifold3d: primitives, hulls, placement, meshing, mass properties.

Units are millimetres in the part generators; mass properties are returned in SI. Every part
is a manifold solid, so volume, centre of mass and inertia come straight from the mesh.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import manifold3d as mf
import numpy as np

# material densities, g/cm³ (printed parts: effective, walls plus infill)
DENSITY = {"al6061": 2.70, "al7075": 2.81, "pa_cf": 1.10, "tpu": 1.15, "steel": 7.85}
SEGMENTS = 64


def box(size, center=(0, 0, 0)) -> mf.Manifold:
    return mf.Manifold.cube(list(size), center=True).translate(list(center))


def cyl(d: float, h: float, axis: str = "z", center=(0, 0, 0)) -> mf.Manifold:
    """Cylinder of diameter d and length h along x, y or z, centred at `center`."""
    m = mf.Manifold.cylinder(h, d / 2, d / 2, SEGMENTS, center=True)
    if axis == "x":
        m = m.rotate([0, 90, 0])
    elif axis == "y":
        m = m.rotate([90, 0, 0])
    return m.translate(list(center))


def hull(*parts: mf.Manifold) -> mf.Manifold:
    return mf.Manifold.batch_hull(list(parts))


def union(*parts: mf.Manifold) -> mf.Manifold:
    return mf.Manifold.batch_boolean(list(parts), mf.OpType.Add)


def mirror_y(m: mf.Manifold) -> mf.Manifold:
    return m.mirror([0, 1, 0])


def lighten(
    plate: mf.Manifold,
    normal: str,
    keep: list[mf.Manifold] = (),
    hole: float = 22.0,
    pitch: float = 30.0,
    rim: float = 9.0,
) -> mf.Manifold:
    """Drill a grid of lightening holes through a flat plate whose thickness runs along
    `normal`, staying `rim` mm inside its outline and out of the `keep` regions (motor mounting
    faces, bearing bosses)."""
    lo, hi = np.array(plate.bounding_box()[:3]), np.array(plate.bounding_box()[3:])
    ax = "xyz".index(normal)
    inplane = [i for i in range(3) if i != ax]
    centre = (lo + hi) / 2
    size = hi - lo
    # the plate shrunk by `rim` in its plane approximates the inner region
    factor = [1.0, 1.0, 1.0]
    for i in inplane:
        factor[i] = max(0.0, 1 - 2 * rim / size[i])
    inner = plate.translate(list(-centre)).scale(factor).translate(list(centre))
    holes = []
    us = np.arange(lo[inplane[0]] + rim + hole / 2, hi[inplane[0]] - rim, pitch)
    vs = np.arange(lo[inplane[1]] + rim + hole / 2, hi[inplane[1]] - rim, pitch)
    for k, u in enumerate(us):
        for v in vs + (pitch / 2 if k % 2 else 0):
            c = centre.copy()
            c[inplane[0]], c[inplane[1]] = u, v
            holes.append(cyl(hole, size[ax] + 2, normal, tuple(c)))
    if not holes:
        return plate
    drill = mf.Manifold.batch_boolean(holes, mf.OpType.Add) ^ inner
    for k in keep:
        drill = drill - k
    return plate - drill


@dataclass
class Part:
    """One manufactured or bought part, in its link's frame (mm)."""

    name: str
    link: str
    material: str  # key of DENSITY, or "bought" (mass given)
    solid: mf.Manifold
    process: str  # "cnc", "print", "bought"
    mass_g: float | None = None  # for bought parts
    notes: str = ""
    extra: dict = field(default_factory=dict)

    def mass(self) -> float:
        """kg"""
        if self.mass_g is not None:
            return self.mass_g / 1000
        return self.solid.volume() / 1000 * DENSITY[self.material] / 1000

    def mesh(self) -> tuple[np.ndarray, np.ndarray]:
        g = self.solid.to_mesh()
        return np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts)

    def bounds(self) -> np.ndarray:
        b = self.solid.bounding_box()
        return np.array(b[3:]) - np.array(b[:3])

    def mass_properties(self) -> tuple[float, np.ndarray, np.ndarray]:
        """(mass kg, centre of mass m, inertia about it kg·m²) of a uniform solid."""
        import trimesh

        v, f = self.mesh()
        props = trimesh.Trimesh(v / 1000, f).mass_properties
        mass = self.mass()
        return (
            mass,
            np.asarray(props["center_mass"]),
            np.asarray(props["inertia"]) * (mass / props["mass"]),
        )

    def mirrored(self) -> Part:
        name = self.name.replace("left_", "right_")
        link = self.link.replace("left_", "right_")
        return Part(
            name, link, self.material, mirror_y(self.solid), self.process, self.mass_g, self.notes
        )
