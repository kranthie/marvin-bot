"""Build Marvin Bot's hand meshes from a local clone of the AmazingHand repository.

The AmazingHand (Pollen Robotics, https://github.com/pollen-robotics/AmazingHand) is an
open-source 8-DOF four-finger hand; its mechanical design is CC-BY-4.0. Its MuJoCo model
(Demo/AHSimulation, one per hand) places every part at the assembled pose, with the fingers
straight; the palm and top shells, which the model leaves out, come from cad/stl and are placed
like the hand plate. This script merges the parts of each hand, without the fasteners, pins and
bushings, into one closed solid (each part simplified, the parts united, the result decimated):

  description/third_party/amazing_hand/{left,right}_hand.stl

in the hand's own frame (metres): the wrist interface's mounting face at z = 0, the fingers along
+z, the palm facing +x. cad/arm.py places the hands on the wrist pitch links; being closed, the
same solid serves the visuals and the joint sweep.

Usage (needs mujoco, trimesh, manifold3d and fast_simplification):
  python tools/build_hand_mesh.py /path/to/AmazingHand
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import fast_simplification
import manifold3d as mf
import mujoco
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "description" / "third_party" / "amazing_hand"
SKIP = ("screw", "washer", "pin", "bushing", "std00", "m2_rod", "spacer")  # small hardware
SHELLS = ("Palm_Shell", "Top_Shell")  # in cad/stl, in the hand plate's frame (mm)
TOL = 0.3e-3  # m: each part is simplified to within this
FACES = 30000  # the united solid is decimated to this many triangles


def quat_mat(q) -> np.ndarray:
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, float))
    return m.reshape(3, 3)


def parts(repo: Path, side: str) -> list[trimesh.Trimesh]:
    """Every visual part of one hand at the model's zero pose, in the hand frame (m)."""
    mjcf = repo / f"Demo/AHSimulation/AHSimulation/AH_{side.title()}/mjcf/robot.xml"
    m = mujoco.MjModel.from_xml_path(str(mjcf))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        if any(k in m.mesh(mid).name for k in SKIP):
            continue
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        v = m.mesh_vert[va : va + vn] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        out.append(trimesh.Trimesh(v, m.mesh_face[fa : fa + fn].copy()))
    # the shells, placed as the model places the hand plate (its STL shares their frame)
    s = side[0]
    plate = re.search(
        rf'<geom[^>]*pos="([^"]+)" quat="([^"]+)" mesh="{s}_hand_plate"', mjcf.read_text()
    )
    pos, quat = (np.array(x.split(), float) for x in plate.groups())
    rot = quat_mat(quat)
    for name in SHELLS:
        mesh = trimesh.load(repo / f"cad/stl/Amazing Hand Parts - {s.upper()}-{name}.stl")
        out.append(trimesh.Trimesh(mesh.vertices / 1000 @ rot.T + pos, mesh.faces))
    return out


def solid(meshes: list[trimesh.Trimesh]) -> trimesh.Trimesh:
    """The parts as one closed solid."""
    pieces = []
    for p in meshes:
        p.merge_vertices()
        m = mf.Manifold(
            mf.Mesh(
                vert_properties=np.asarray(p.vertices, np.float32),
                tri_verts=np.asarray(p.faces, np.uint32),
            )
        )
        if m.status() != mf.Error.NoError or m.is_empty():
            m = mf.Manifold.hull_points(np.asarray(p.vertices, np.float64))  # not closed
        pieces.append(m.simplify(TOL))
    g = mf.Manifold.batch_boolean(pieces, mf.OpType.Add).to_mesh()
    v, f = fast_simplification.simplify(
        np.array(np.asarray(g.vert_properties)[:, :3], np.float32),
        np.array(g.tri_verts, np.int32),
        target_count=FACES,
    )
    check = mf.Manifold(
        mf.Mesh(vert_properties=v.astype(np.float32), tri_verts=f.astype(np.uint32))
    )
    if check.status() != mf.Error.NoError:
        raise SystemExit("the decimated hand is not a closed solid: raise FACES")
    return trimesh.Trimesh(v, f)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("repo", type=Path, help="local clone of the AmazingHand repository (main)")
    repo = ap.parse_args().repo
    OUT.mkdir(parents=True, exist_ok=True)
    for side in ("left", "right"):
        ps = parts(repo, side)
        hand = solid(ps)
        hand.export(OUT / f"{side}_hand.stl")
        print(
            f"{side}: {len(ps)} parts, {sum(len(p.faces) for p in ps)} triangles -> "
            f"{len(hand.faces)}; bounds (mm) {np.round(hand.bounds * 1000, 1).tolist()}"
        )


if __name__ == "__main__":
    main()
