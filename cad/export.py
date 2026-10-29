"""Export the parts for making them, and the build BOM.

Writes:
  outputs/parts/<process>/<part>.stl   each part in mm, sitting on z = 0 (git-ignored)
  outputs/parts/dxf/<part>.dxf         the outline of each flat part (a plate of one thickness),
                                       for the laser or water-jet service and for the mill's
                                       2D toolpaths (git-ignored)
  bom/bom.xlsx, sheet "Made parts (CAD)"
                                       every made part: quantity (mirror pairs counted once,
                                       quantity 2), process, material, stock, mass, size
  bom/bom.xlsx, sheet "Actuator bolts (CAD)"
                                       the actuator bolts: each motor's stator and output hole
                                       circles from description/actuators.yaml
The workbook's other sheets, and the builder's status and notes columns on these two, are kept
(tools/bom_book.py).

The bought parts (the workbook's group sheets) and the cover shells are listed in the made-parts
sheet for
completeness; only made parts get files. A part is flat when its volume is its mid-thickness
outline times its thickness (within 2%), so its DXF is the whole part; other parts are milled
or printed from the STL.

Usage:
  python -m cad.export           # write the files
  python -m cad.export --check   # exit 1 if the workbook's CAD sheets are out of date (CI)
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

from tools import bom_book

from . import arm, covers, electrical, head, leg, torso

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "outputs" / "parts"
STOCK_MM = [3, 4, 5, 6, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60]  # common plate thicknesses
PAIR = re.compile(r"^(left|right)_|_(l|r)$")


def all_parts():
    return (
        leg.all_parts()
        + torso.parts()
        + arm.all_parts()
        + head.parts()
        + electrical.parts()
        + covers.all_parts()
    )


def base_name(name: str) -> str:
    """Name with the side taken out, so mirror pairs share it."""
    return PAIR.sub(lambda m: "{side}_" if m.group(1) else "_{side}", name)


def flat(p):
    """(thickness mm, polygons) if the part is a plate of one thickness, else None."""
    bb = np.array(p.solid.bounding_box())
    lo, hi = bb[:3], bb[3:]
    axis = int(np.argmin(hi - lo))
    solid = p.solid
    if axis == 0:
        solid = solid.rotate([0, 90, 0])
    elif axis == 1:
        solid = solid.rotate([90, 0, 0])
    zlo, zhi = solid.bounding_box()[2], solid.bounding_box()[5]
    t = zhi - zlo
    section = solid.slice((zlo + zhi) / 2)
    if section.area() * t <= 0 or abs(section.area() * t - solid.volume()) > 0.02 * solid.volume():
        return None
    return t, section.to_polygons()


def dxf(polygons) -> str:
    """Closed outlines as an R12 DXF (POLYLINE entities, mm)."""
    out = ["0", "SECTION", "2", "ENTITIES"]
    for poly in polygons:
        out += ["0", "POLYLINE", "8", "0", "66", "1", "70", "1"]
        for x, y in poly:
            out += ["0", "VERTEX", "8", "0", "10", f"{x:.4f}", "20", f"{y:.4f}"]
        out += ["0", "SEQEND"]
    out += ["0", "ENDSEC", "0", "EOF"]
    return "\n".join(out) + "\n"


def stock(t: float) -> int | str:
    return next((s for s in STOCK_MM if s >= t - 0.05), "block")


def build_parts(parts):
    rows: dict[str, dict] = {}
    files = {}
    for p in parts:
        key = base_name(p.name)
        shape = flat(p) if p.process in ("cnc", "service") else None
        files[p.name] = (p, shape)  # both sides: the right part is the left one mirrored
        if key in rows:
            rows[key]["qty"] += 1
            continue
        size = p.bounds()
        if p.process in ("cnc", "service"):
            form = f"flat {shape[0]:.1f} mm" if shape else "3D"
            stk = stock(shape[0] if shape else min(size))
        else:
            form, stk = ("3D", "") if p.process != "bought" else ("", "")
        rows[key] = dict(
            part=key,
            qty=1,
            process=p.process,
            material=p.material,
            form=form,
            stock_mm=stk,
            mass_g=round(p.mass() * 1000, 1),
            size_mm=" x ".join(f"{x:.0f}" for x in sorted(size, reverse=True)),
            notes=p.notes,
        )
    return list(rows.values()), files


def build_fasteners():
    robot = yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())
    acts = yaml.safe_load((ROOT / "description" / "actuators.yaml").read_text())
    count: dict[tuple, int] = {}
    for m in robot["motors"]:
        n_sides = 2 if "{side}" in m["name"] else 1
        holes = acts[m["model"]].get("holes")
        if not holes:
            continue
        for face in ("stator", "output"):
            h = holes[face]
            k = (m["model"], face, h["thread"])
            count[k] = count.get(k, 0) + h["n"] * n_sides
    return [
        dict(
            actuator=model,
            face=face,
            thread=thread,
            qty=n,
            notes="socket head, 12.9; length = plate + thread engagement, per part",
        )
        for (model, face, thread), n in sorted(count.items())
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    rows, files = build_parts(all_parts())
    bolts = build_fasteners()
    data = bom_book.read_book()
    if args.check:
        stale = bom_book.generated_stale(data, rows, bolts)
        if stale:
            print(f"{', '.join(stale)} out of date in bom/bom.xlsx: run python -m cad.export")
            return 1
        print("build BOM up to date")
        return 0
    changed = bom_book.save(bom_book.merge_generated(data, rows, bolts))
    n_flat = 0
    for name, (p, shape) in files.items():
        if p.process in ("bought", "cover"):
            continue
        folder = OUT / p.process
        folder.mkdir(parents=True, exist_ok=True)
        v, f = p.mesh()
        v = v - [(v[:, 0].min() + v[:, 0].max()) / 2, (v[:, 1].min() + v[:, 1].max()) / 2, 0]
        v[:, 2] -= v[:, 2].min()
        trimesh.Trimesh(v, f).export(folder / f"{name}.stl")
        if shape:
            (OUT / "dxf").mkdir(parents=True, exist_ok=True)
            (OUT / "dxf" / f"{name}.dxf").write_text(dxf(shape[1]))
            n_flat += 1
    made = sum(r["qty"] for r in rows if r["process"] not in ("bought", "cover"))
    print(f"{made} made parts ({len(files)} files, {n_flat} flat with a DXF) in {OUT}")
    print(f"bom/bom.xlsx {'updated' if changed else 'unchanged'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
