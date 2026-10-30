"""Small code-CAD toolkit on manifold3d: primitives, hulls, placement, meshing, mass properties.

The made metal parts are drawn in build123d (cad/b3d.py) and meshed into this; printed shells,
covers and bought parts are drawn here.

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


def ellipsoid(radii, center=(0, 0, 0)) -> mf.Manifold:
    return mf.Manifold.sphere(1.0, SEGMENTS).scale(list(radii)).translate(list(center))


def hull(*parts: mf.Manifold) -> mf.Manifold:
    return mf.Manifold.batch_hull(list(parts))


def union(*parts: mf.Manifold) -> mf.Manifold:
    return mf.Manifold.batch_boolean(list(parts), mf.OpType.Add)


def mirror_y(m: mf.Manifold) -> mf.Manifold:
    return m.mirror([0, 1, 0])


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
    brep: object = None  # build123d shape (cad/b3d.py), for STEP files; None for mesh-only parts

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
        brep = None
        if self.brep is not None:
            import build123d as bd  # noqa: PLC0415

            brep = bd.mirror(self.brep, bd.Plane.XZ)
        return Part(
            name,
            link,
            self.material,
            mirror_y(self.solid),
            self.process,
            self.mass_g,
            self.notes,
            brep=brep,
        )
