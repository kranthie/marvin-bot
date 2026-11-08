"""Lite's column head and column: the link that rides up and down on the desk column and carries
the waist. Design pass 1: not built or tested.

Frame (mm), the column link (description/skeleton.yaml, variant lite): origin on the top face of
the column's top plate at the column's centre, x forward, z up; with the column retracted it is
cad/lite_base.py's TOP above the floor.

The head plate (6 mm 7075) bolts to the column's top plate and carries the waist RS03 exactly as
Max's pelvis top plate does (cad/torso.py's waist_mount): four cage strips up to a front ring on
the motor's ears, which seats the crossed-roller bearing under the torso's hub. The torso and
everything above it are Max's parts, unchanged. A printed cone covers the plates and the waist
motor up to the torso.

The bought column (LINAK DL6, decisions 2026-11-07) is split between two links: its outer stage,
foot bracket and motor stay on the base; its middle and inner stages ride with this link (in the
real column the middle stage moves half as far; drawn as one piece, mass split 4.5 / 3.0 kg).

usage: python -m cad.lite_column
"""

from __future__ import annotations

import manifold3d as mf

from . import lite_base as lb
from .b3d import circles, cut, disc, hull, lighten, made, rect, rounded
from .b3d import plate as bplate
from .lib import Part, box, union
from .mounts import cage, spec
from .torso import waist_mount

LINK = "column_link"
PLATE_T = 6.0
COL_PLATE = (
    130.0,
    120.0,
    10.0,
)  # the column's top plate (bought), assumed: confirm on the datasheet
COL_HOLES = [(sx * 50.0, sy * 45.0) for sx in (1, -1) for sy in (1, -1)]  # 4 x M6, assumed
WAIST_JOINT_ABOVE_MOTOR = 9.0  # as on the pelvis (skeleton.yaml): the joint 9 mm above the centre
INNER_LEN = 520.0  # the moving stages' visible length: the 450 mm stroke plus overlap


def waist_centre() -> tuple[float, float, float]:
    """The waist RS03's centre in this link's frame."""
    t03 = spec("RS03")["t"]
    return (0.0, 0.0, PLATE_T + t03 / 2 + 1.0)


def waist_joint() -> tuple[float, float, float]:
    c = waist_centre()
    return (c[0], c[1], c[2] + WAIST_JOINT_ABOVE_MOTOR)


def shroud() -> mf.Manifold:
    """A printed cone, 2.5 mm thick, from over the column's top plate up to just under the
    torso's hub: clear of the plates' corners (r 88), the cage (r 53) and the ring (r 56)."""
    r0, z0, r1, z1, t = 97.0, -12.0, 66.0, 74.0, 2.5
    return lb.revolve([(r0, z0), (r0 + t, z0), (r1 + t, z1), (r1, z1)])


def parts() -> list[Part]:
    wc = waist_centre()
    t03 = 2 * (wc[2] - PLATE_T - 1.0)
    strips, pads, fea = cage("RS03", wc, "z", 1, to=t03 / 2 + 1.0)
    sk = rounded(hull(disc(124), rect(-COL_PLATE[0] / 2, COL_PLATE[0] / 2, -60.0, 60.0)))
    sk = cut(
        sk,
        disc(30),  # the waist motor's cable, out of its cap, down beside the column
        circles(6.6, COL_HOLES),
        lighten(sk, keep=[pads, circles(18, COL_HOLES)], pitch=24, rim=6),
    )
    out = [
        made("column_head_plate", LINK, "al7075", bplate(sk, "z", PLATE_T / 2, PLATE_T)),
    ]
    out += waist_mount(LINK, wc, strips, pads, fea, prefix="column_head")
    out.append(
        Part(
            "column_head_shroud",
            LINK,
            "petg",
            shroud(),
            "print",
            notes="a cone over the column's top plate, the head plate and the waist cage",
        )
    )
    # the column's moving stages and top plate (bought)
    stages = mf.Manifold.extrude(mf.CrossSection([lb._rr(71, 41, 6)]), INNER_LEN).translate(
        [0, 0, -COL_PLATE[2] - INNER_LEN]
    )
    plate = box(COL_PLATE, (0, 0, -COL_PLATE[2] / 2))
    out.append(
        Part(
            "desk_column_inner",
            LINK,
            "bought",
            union(stages, plate),
            "bought",
            mass_g=3000.0,
            notes="the DL6's middle and inner stages and top plate (drawn as one piece)",
        )
    )
    return out


def main() -> None:
    ps = parts()
    print(f"{len(ps)} parts, {sum(p.mass() for p in ps):.3f} kg; waist joint at {waist_joint()}")
    for p in ps:
        print(f"  {p.name:26} {p.process:7} {p.mass() * 1000:7.1f} g")


if __name__ == "__main__":
    main()
