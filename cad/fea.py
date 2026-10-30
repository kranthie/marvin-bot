"""Finite-element stress check of the load-carrying parts: each link's parts as one bonded solid.

A beam check (cuts along a load path) misses torsion and every part whose shape a beam can't
describe (motor rings and pillars, flanges, cranks, the torso frames), so this replaced it. It
meshes the metal parts of a link together, with the printed shells that carry load between
them (each in its own material). The parts are treated as bonded: the plate-to-plate fasteners
are not designed yet, and a part that touches nothing that carries load is reported as not
attached. It holds the link where it is driven (the motor output or stator face that carries it,
and that joint's far-side bearing seat) and loads it where the next link hangs on it, with the
load cases: a landing (3 x body weight up at the ankle, 0.5 x fore-aft, 0.3 x sideways), a hand
catch (0.5 x body weight up at the palm, 0.25 x fore-aft, 0.15 x sideways), a sideways fall onto
the knee (1 x body weight), the head dropping (10 g) and each joint's peak torque,
as a force and moment on the loaded patches, tied together as one rigid body (the next link); a
bearing takes no torque about its own axis. Linear elastic, quadratic tetrahedra (fTetWild meshes,
scikit-fem assembles, PARDISO or SciPy solves).

Reported per part: the peak von Mises stress smoothed along the surface over about a fillet's
size (a sharp re-entrant corner is an unbounded peak in a linear model, which the fillets will
remove; the raw peak is listed as "corner"), ignoring elements on a held patch, where the clamp
makes stress that a bolted face doesn't. Checked against beam theory (tests/test_fea.py): at a
3 mm mesh it reads 5-9% low in bending, so the target safety factor is 2.2 on yield.

The interfaces come from the part generators: each motor ring, output flange, bearing bore and
stub axle records the joint it serves and its contact patch (`Part.extra["ifaces"]`).

Usage:
  python -m cad.fea                  # every link, the worst stress per part and its case
  python -m cad.fea --link left_thigh_link
  python -m cad.fea --check          # exit 1 if any part is below the target safety factor

Needs: pip install -e ".[cad,fea]"
"""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np

E_MPA = {"al6061": 68900.0, "al7075": 71700.0, "pa_cf": 6000.0}
NU = {"al6061": 0.33, "al7075": 0.33, "pa_cf": 0.35}


@dataclass
class Patch:
    """A contact patch on the part's surface (mm, link frame). `kind` "face": an annulus on the
    plane through `centre` normal to `axis`, radii r0-r1 about the axis. "cyl": the cylinder of
    radius r1 about `axis` through `centre`, between `span` (along the axis, from the centre)."""

    kind: str
    centre: tuple
    axis: tuple
    r0: float = 0.0
    r1: float = 0.0
    span: tuple = (-1.0, 1.0)
    rigid: bool = True  # loaded: tied to the next link as a rigid body; False: a spread pressure

    def select(self, pts: np.ndarray, normals: np.ndarray, tol: float = 0.4) -> np.ndarray:
        a = np.asarray(self.axis, float)
        a = a / np.linalg.norm(a)
        d = pts - np.asarray(self.centre, float)
        along = d @ a
        radial = np.linalg.norm(d - np.outer(along, a), axis=1)
        if self.kind == "face":
            return (
                (np.abs(along) < tol)
                & (radial >= self.r0 - tol)
                & (radial <= self.r1 + tol)
                & (np.abs(normals @ a) > 0.9)
            )
        return (
            (np.abs(radial - self.r1) < tol)
            & (along >= self.span[0] - tol)
            & (along <= self.span[1] + tol)
            & (np.abs(normals @ a) < 0.2)
        )


def mesh_solid(solid, size: float):
    """Tetrahedral mesh (nodes mm, tets) of a manifold3d solid: fTetWild (pytetwild), which
    makes well-shaped tetrahedra from any closed triangle surface, within 0.05% of the part's
    size of it."""
    from pytetwild import tetrahedralize  # noqa: PLC0415

    m = solid.to_mesh()
    v = np.asarray(m.vert_properties)[:, :3].astype(np.float64)
    f = np.asarray(m.tri_verts).astype(np.int32)
    here = os.getcwd()
    with tempfile.TemporaryDirectory() as tmp:
        os.chdir(tmp)  # fTetWild writes a scratch file into the working directory
        try:
            nodes, tets = tetrahedralize(v, f, edge_length_abs=size, epsilon=5e-4, quiet=True)
        finally:
            os.chdir(here)
    p = nodes[tets]
    vol = np.einsum("ij,ij->i", np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0]), p[:, 3] - p[:, 0])
    tets = np.where((vol < 0)[:, None], tets[:, [0, 2, 1, 3]], tets)  # positive orientation
    return nodes, tets


class Model:
    """A meshed solid ready for load cases: P2 tetrahedra, linear elastic. `solid` is one
    manifold3d solid of one material, or a list of parts (cad.lib.Part, bonded, each with its own
    material); `owner` then says which part each element lies in."""

    def __init__(self, solid, material: str | None = None, size: float = 3.0):
        import skfem  # noqa: PLC0415
        from skfem.helpers import ddot, sym_grad, trace  # noqa: PLC0415

        parts = solid if isinstance(solid, list) else None
        whole = solid
        if parts:
            whole = parts[0].solid
            for q in parts[1:]:
                whole = whole + q.solid
        nodes, tets = mesh_solid(whole, size)
        self.mesh = skfem.MeshTet(nodes.T, tets.T).with_boundaries({})
        self.element = skfem.ElementVector(skfem.ElementTetP2())
        self.basis = skfem.Basis(self.mesh, self.element, intorder=2)
        n = self.mesh.t.shape[1]
        if parts:
            self.owner = owners(self, parts)
            mats = [q.material for q in parts]
            e = np.array([E_MPA[mats[k]] for k in self.owner])
            nu = np.array([NU[mats[k]] for k in self.owner])
        else:
            self.owner = np.zeros(n, int)
            e, nu = np.full(n, E_MPA[material]), np.full(n, NU[material])
        self.lam_e = e * nu / ((1 + nu) * (1 - 2 * nu))
        self.mu_e = e / (2 * (1 + nu))
        lam_q = self.lam_e[:, None]  # per element, constant over its quadrature points
        mu_q = self.mu_e[:, None]

        @skfem.BilinearForm
        def elasticity(u, v, w):
            eu, ev = sym_grad(u), sym_grad(v)
            return 2 * w.mu * ddot(eu, ev) + w.lam * trace(eu) * trace(ev)

        self.K = skfem.asm(elasticity, self.basis, lam=lam_q, mu=mu_q)
        # boundary facets: centroid, outward normal
        f = self.mesh.boundary_facets()
        tri = self.mesh.p[:, self.mesh.facets[:, f]]  # 3 x 3 x nf
        self.fc = tri.mean(axis=1).T
        nrm = np.cross((tri[:, 1] - tri[:, 0]).T, (tri[:, 2] - tri[:, 0]).T)
        nrm /= np.linalg.norm(nrm, axis=1)[:, None]
        self.fn = nrm
        self.facets = f
        # elements touching each boundary facet (for the stress report's exclusion)
        self.f2t = self.mesh.f2t[0, f]
        # connected pieces of the mesh (parts that touch nothing aren't joined yet)
        from scipy.sparse import coo_matrix  # noqa: PLC0415
        from scipy.sparse.csgraph import connected_components  # noqa: PLC0415

        t = self.mesh.t
        rows = np.repeat(t, 4, axis=0).ravel()
        cols = np.tile(t, (4, 1)).ravel()
        g = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(self.mesh.p.shape[1],) * 2)
        _, self.node_piece = connected_components(g, directed=False)
        self.piece = self.node_piece[t[0]]

    def _patch_dofs(self, patches: list[Patch]):
        mask = np.zeros(len(self.facets), bool)
        for p in patches:
            mask |= p.select(self.fc, self.fn)
        facets = self.facets[mask]
        d = self.basis.get_dofs(facets=facets)
        dofs = np.column_stack([np.unique(d.all(f"u^{i + 1}")) for i in range(3)])
        return dofs, mask

    def solve(self, held: list[Patch], loaded: list[Patch], force, moment, about):
        """Displacement for a force (N) and moment (N·mm, about the point `about`) carried into
        the loaded patches and out through the held ones. The loaded patches are tied together
        as one rigid body (the next link, bolted to them), so the structure's stiffness decides
        how the load shares between them; the held ones are fixed. Returns (von Mises per element
        MPa, mask of elements touching a held patch)."""
        from scipy.sparse import coo_matrix, identity  # noqa: PLC0415

        hd, hmask = self._patch_dofs(held)
        ld, _ = self._patch_dofs(loaded)
        if not len(hd) or not len(ld):
            raise ValueError("a held or loaded patch selects no surface")
        held_nodes = self.mesh.facets[:, self.facets[hmask]].ravel()
        held_pieces = np.unique(self.node_piece[held_nodes])
        self.floating = ~np.isin(self.piece, held_pieces)
        fix = [np.unique(hd.ravel())]
        if self.floating.any():
            fix.append(self.basis.get_dofs(elements=np.nonzero(self.floating)[0]).flatten())
        fixed = np.unique(np.concatenate(fix))
        ld = ld[~np.isin(ld[:, 0], fixed)]
        if not len(ld):
            raise ValueError("the loaded patch is on a piece that isn't attached")
        if not all(p.rigid for p in loaded):
            return self._solve_spread(fixed, ld, hmask, force, moment, about)
        n = self.K.shape[0]
        free = np.setdiff1d(np.arange(n), np.concatenate([fixed, ld.ravel()]))
        # u = P q: the free DOFs themselves, then six rigid-body DOFs (U, theta) of the loaded set
        x = self.basis.doflocs[:, ld[:, 0]].T  # mm
        c = x.mean(axis=0)
        r = x - c
        m = len(free)
        rows, cols, vals = [free], [np.arange(m)], [np.ones(m)]
        z = np.zeros(len(r))
        lever = {  # component k of theta x r, as coefficients of (theta_x, theta_y, theta_z)
            0: (z, r[:, 2], -r[:, 1]),
            1: (-r[:, 2], z, r[:, 0]),
            2: (r[:, 1], -r[:, 0], z),
        }
        for k in range(3):
            rows += [ld[:, k]]
            cols += [np.full(len(r), m + k)]
            vals += [np.ones(len(r))]
            for j in range(3):
                rows += [ld[:, k]]
                cols += [np.full(len(r), m + 3 + j)]
                vals += [lever[k][j]]
        P = coo_matrix(
            (np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(n, m + 6)
        ).tocsr()
        a = (P.T @ self.K @ P).tocsr() + 1e-12 * identity(m + 6, format="csr")
        m_c = np.asarray(moment, float) + np.cross(np.asarray(about, float) - c, force)
        rhs = np.zeros(m + 6)
        rhs[m : m + 3] = force
        rhs[m + 3 :] = m_c
        u = P @ self._direct(a, rhs)
        vm = self.smooth(self._von_mises(u))
        near = np.zeros(self.mesh.t.shape[1], bool)
        near[self.f2t[hmask]] = True  # the clamp makes stress there that a bolted face doesn't
        return vm, near

    def _solve_spread(self, fixed, ld, hmask, force, moment, about):
        """A pressure-like load (the ground under the foot): F/n on each loaded node, plus the
        moment as forces growing linearly with the distance from the patch's centre."""
        x = self.basis.doflocs[:, ld[:, 0]].T
        c = x.mean(axis=0)
        r = x - c
        m_c = np.asarray(moment, float) + np.cross(np.asarray(about, float) - c, force)
        inertia = (r * r).sum() * np.eye(3) - r.T @ r
        w = np.linalg.lstsq(inertia, m_c, rcond=None)[0]
        f = np.asarray(force, float) / len(r) + np.cross(w, r)
        rhs = np.zeros(self.K.shape[0])
        for i in range(3):
            np.add.at(rhs, ld[:, i], f[:, i])
        free = np.setdiff1d(np.arange(self.K.shape[0]), fixed)
        u = np.zeros(self.K.shape[0])
        u[free] = self._direct(self.K[free][:, free].tocsr(), rhs[free])
        vm = self.smooth(self._von_mises(u))
        near = np.zeros(self.mesh.t.shape[1], bool)
        near[self.f2t[hmask]] = True
        return vm, near

    @staticmethod
    def _direct(a, rhs):
        try:
            import pypardiso  # noqa: PLC0415

            return pypardiso.spsolve(a, rhs)
        except ImportError:
            from scipy.sparse.linalg import spsolve  # noqa: PLC0415

            return spsolve(a.tocsc(), rhs)

    def smooth(self, vm: np.ndarray, radius: float = 2.5, depth_tol: float = 0.5) -> np.ndarray:
        """Each element's stress averaged with the elements within `radius` mm at the same depth
        below the surface (within `depth_tol`): smoothing along the surface, about a fillet's
        size, so a sharp re-entrant corner (an unbounded peak in a linear model, which a fillet
        removes) doesn't read as the part's stress, while a bent plate keeps its surface stress.
        The raw element stresses stay in `self.raw`."""
        from scipy.spatial import cKDTree  # noqa: PLC0415

        c = self.centroids()
        if not hasattr(self, "_nbr"):
            depth = cKDTree(self.fc).query(c)[0]
            vol = self.volumes()
            nbr = cKDTree(c).query_ball_point(c, radius)
            self._nbr = [
                (np.asarray(n)[np.abs(depth[n] - depth[i]) < depth_tol]) for i, n in enumerate(nbr)
            ]
            self._vol = vol
        self.raw = vm
        return np.array([np.average(vm[n], weights=self._vol[n]) for n in self._nbr])

    def volumes(self) -> np.ndarray:
        p = self.mesh.p[:, self.mesh.t]  # 3 x 4 x n
        d = np.einsum("ijk->kij", p[:, 1:] - p[:, :1])  # n x 3 x 3
        return np.abs(np.linalg.det(d)) / 6

    def _von_mises(self, u: np.ndarray) -> np.ndarray:
        """Element-max von Mises stress over its quadrature points (MPa)."""
        from skfem.helpers import ddot, eye, sym_grad, trace  # noqa: PLC0415

        w = self.basis.interpolate(u)
        eps = sym_grad(w)
        lam, mu = self.lam_e[:, None], self.mu_e[:, None]
        sig = 2 * mu * eps + lam * eye(trace(eps), 3)
        dev = sig - eye(trace(sig) / 3, 3)
        vm = np.sqrt(1.5 * ddot(dev, dev))  # elements x quadrature points
        return vm.max(axis=1)

    def centroids(self) -> np.ndarray:
        return self.mesh.p[:, self.mesh.t].mean(axis=1).T


# --- the load cases ------------------------------------------------------------------------------

YIELD_MPA = {
    "al6061": 240.0,
    "al7075": 480.0,
    # printed carbon-fibre nylon: the strength between layers (25-40 MPa in published
    # tests), the weak direction for a part printed standing up; conservative
    "pa_cf": 25.0,
}
TARGET = 2.2  # SF on yield: 2, plus the mesh's 5-9% under-read in bending
RS03_PEAK = 60.0  # N·m, the ankle motors (they drive the ankle through the linkage)
# printed parts that carry load between metal ones (checked with them, in their own material)
STRUCTURAL_PRINTS = ("_upper_arm_shell", "_forearm_shell")
HELD = {  # joints whose interfaces hold a group, where it isn't the link's parent joint
    "pelvis": ["waist_yaw"],
    "left_foot_link": ["left_ankle_roll", "left_ankle_rod_a", "left_ankle_rod_b"],
}


def _setup():
    import re  # noqa: PLC0415
    import sys  # noqa: PLC0415

    import yaml  # noqa: PLC0415

    from . import arm, head, leg, torso  # noqa: PLC0415

    root = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(root / "tools"))
    import build_urdf  # noqa: PLC0415

    urdf = build_urdf.URDF_OUT.read_text()
    mass = sum(float(x) for x in re.findall(r'<mass value="([^"]+)"', urdf))
    peak = {
        m.group(1): float(m.group(2))
        for m in re.finditer(r'<joint name="([^"]+)".*?effort="([^"]+)"', urdf, re.S)
    }
    skel = yaml.safe_load((root / "description" / "skeleton.yaml").read_text())
    joints = skel["joints"]
    origin = {"pelvis": np.zeros(3)}
    pending = dict(joints)
    while pending:
        for k, v in list(pending.items()):
            if v["parent"] in origin:
                origin[v["child"]] = origin[v["parent"]] + np.array(v["xyz"]) * 1000
                del pending[k]
    parts = leg.parts() + torso.parts() + arm.parts() + head.parts()
    return mass, peak, joints, origin, parts


def groups(parts) -> dict[str, list]:
    """The metal parts of each link (left side and centre), or of a separate group (a crank)."""
    out: dict[str, list] = {}
    for p in parts:
        structural = p.material.startswith("al") or p.name.endswith(STRUCTURAL_PRINTS)
        if not structural or p.link.startswith("right_"):
            continue
        out.setdefault(p.extra.get("group", p.link), []).append(p)
    return out


def cases(name: str, group: list, mass: float, peak: dict, joints: dict, origin: dict):
    """(case, held joints, loaded joints, force N, moment N·mm, about mm) for a group."""
    w = mass * 9.81
    link = group[0].link
    o = origin[link]
    child = {v["parent"]: k for k, v in joints.items() if not k.startswith("right_")}
    parent = {v["child"]: k for k, v in joints.items()}
    held = HELD.get(name, [parent.get(link)])
    ifaces = {i["joint"] for p in group for i in p.extra.get("ifaces", [])}
    loaded = sorted(ifaces - set(held))
    leg_links = [v["child"] for k, v in joints.items() if k.startswith("left_") and "hip" in k]
    leg_links += ["left_thigh_link", "left_shin_link", "left_ankle_link", "left_foot_link"]
    out = []
    if name.startswith("left_ankle_crank"):
        m = name[-1]
        c = next(i for p in group for i in p.extra["ifaces"] if i["joint"] == f"left_ankle_{m}")
        tip = next(
            i for p in group for i in p.extra["ifaces"] if i["joint"].startswith("left_ankle_rod")
        )
        arm = np.array(tip["centre"]) - np.array(c["centre"])
        arm[1] = 0
        f = np.cross((0, 1, 0), arm)
        f = f / np.linalg.norm(f) * RS03_PEAK * 1000 / np.linalg.norm(arm)
        return [("peak torque", [c["joint"]], [tip["joint"]], f, np.zeros(3), tip["centre"])]
    if link in leg_links or link == "pelvis":
        ankle = origin["left_ankle_link"] - o
        path = "ground" if link == "left_foot_link" else child.get(link)
        if link == "pelvis":
            path = "left_hip_roll"
        landing = np.array([0.5, 0.3, 3.0]) * w
        out.append(("landing", held, [path], landing, np.zeros(3), ankle))
        if link in ("pelvis", "left_hip_roll_link", "left_hip_pitch_link", "left_thigh_link"):
            knee = origin["left_shin_link"] - o
            out.append(("side", held, [path], np.array([0, 1.0, 0]) * w, np.zeros(3), knee))
    elif link == "neck_link":
        head = origin["head_link"] - o
        out.append(
            ("head drop (10 g)", held, ["neck_pitch"], np.array([0, 0, -98.0]), np.zeros(3), head)
        )
    elif link.startswith("left_") or link == "torso_link":
        hand = origin["left_palm_link"] + np.array([0, 0, -60.0]) - o
        path = "left_shoulder_pitch" if link == "torso_link" else child.get(link)
        if path in ifaces:
            out.append(("hand", held, [path], np.array([0.25, 0.15, 0.5]) * w, np.zeros(3), hand))
    for j in loaded:
        if j in peak and j in joints:
            axis = np.array(joints[j]["axis"], float)
            at = origin[joints[j]["child"]] - o
            out.append(
                (
                    f"torque {j.removeprefix('left_')}",
                    held,
                    [j],
                    np.zeros(3),
                    axis * peak[j] * 1000,
                    at,
                )
            )
        elif j.startswith("left_ankle_") and j[-1] in "ab" and "rod" not in j:
            c = next(i for p in group for i in p.extra["ifaces"] if i["joint"] == j)
            out.append(
                (
                    f"torque {j.removeprefix('left_')}",
                    held,
                    [j],
                    np.zeros(3),
                    np.array(c["axis"]) * RS03_PEAK * 1000,
                    c["centre"],
                )
            )
    return out


def owners(model: Model, group: list) -> np.ndarray:
    """Which part each element of a group's mesh lies in (index into `group`)."""
    import trimesh  # noqa: PLC0415

    c = model.centroids()
    dist = np.full((len(group), len(c)), np.inf)
    for k, p in enumerate(group):
        m = p.solid.to_mesh()
        tm = trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3], np.asarray(m.tri_verts))
        inside = tm.contains(c)
        dist[k, inside] = 0.0
        if (~inside).any():
            dist[k, ~inside] = np.abs(trimesh.proximity.signed_distance(tm, c[~inside]))
    return np.argmin(dist, axis=0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--link", help="one link or group")
    ap.add_argument("--size", type=float, default=3.0, help="mesh size, mm")
    ap.add_argument("--where", action="store_true", help="where each part's peak is (link frame)")
    args = ap.parse_args()
    mass, peak, joints, origin, parts = _setup()
    worst_sf = np.inf
    print(f"robot {mass:.1f} kg; target SF {TARGET} on yield (smoothed von Mises)")
    print(f"{'part':30} {'case':22} {'stress':>7} {'yield':>6} {'SF':>5}  {'corner':>7}")
    for name, group in groups(parts).items():
        if args.link and args.link != name:
            continue
        todo = cases(name, group, mass, peak, joints, origin)
        if not todo:
            print(f"{name:30} (no load path: no interfaces recorded)")
            continue
        model = Model(group, size=args.size)
        own = model.owner
        patches = {}
        for p in group:
            for i in p.extra.get("ifaces", []):
                kw = {k: i[k] for k in ("r0", "r1", "span", "rigid") if k in i}
                patches.setdefault(i["joint"], []).append(
                    Patch(i["kind"], i["centre"], i["axis"], **kw)
                )
        worst = {}
        for case, held, loaded, force, moment, about in todo:
            hp = [x for j in held for x in patches.get(j, [])]
            lp = [x for j in loaded for x in patches.get(j, [])]
            if case.startswith("torque"):
                # the joint's own bearings (its "cyl" patches) carry no torque about its axis: its
                # motor faces do. Bearings on the held side still hold the link about other axes
                lp = [x for x in lp if x.kind == "face"]
                if not lp:
                    continue
            try:
                vm, near = model.solve(hp, lp, force, moment, about)
            except ValueError as e:
                print(f"{name:30} {case}: skipped ({e})")
                continue
            for k, p in enumerate(group):
                sel = (own == k) & ~near
                if not sel.any():
                    continue
                v, r = float(vm[sel].max()), float(model.raw[sel].max())
                if v > worst.get(p.name, (0,))[0]:
                    at = model.centroids()[np.flatnonzero(sel)[np.argmax(vm[sel])]]
                    worst[p.name] = (v, case, r, at)
        float_el = getattr(model, "floating", np.zeros(len(own), bool))  # unset if all skipped
        floating = {group[k].name for k in np.unique(own[float_el])}
        for p in group:
            if p.name in floating:
                print(f"{p.name:30} not attached: joins nothing that carries load yet")
                continue
            if p.name not in worst:
                continue
            v, case, r, at = worst[p.name]
            ys = YIELD_MPA[p.material]
            sf = ys / v
            worst_sf = min(worst_sf, sf)
            flag = "  <-- below target" if sf < TARGET else ""
            where = f"  at ({at[0]:.0f}, {at[1]:.0f}, {at[2]:.0f})" if args.where else ""
            print(
                f"{p.name:30} {case:22} {v:7.0f} {ys:6.0f} {sf:5.1f}  {r:7.0f}{flag}{where}",
                flush=True,
            )
    if args.check and worst_sf < TARGET:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
