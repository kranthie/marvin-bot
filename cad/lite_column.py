"""Lite's column head and column: the link that rides up and down on the desk column and carries
the waist. Design pass 1: not built or tested.

Frame (mm), the column link (description/skeleton.yaml, variant lite): origin on the top face of
the column's motor housing at the column's axis, x forward, z up; with the column retracted it
is cad/lite_base.py's TOP above the floor.

The DL6's top is its motor housing (data sheet: 177 x 97 x 46 mm, the column's axis 29.4 mm from
one end, six M6 holes in its sides), turned so it overhangs to the back. The head plate (6 mm
7075) sits on it and carries the waist RS03 exactly as Max's pelvis top plate does
(cad/torso.py's waist_mount): four cage strips up to a front ring on the motor's ears, which
seats the crossed-roller bearing under the torso's hub. Two 8 mm cheeks either side of the
housing bolt to its long sides (2 x M6 each) and to the head plate's edges (2 x M5 each, into
the cheeks' top edges), so the plate cannot lift off it. The torso and everything above it are
Max's parts, unchanged. A printed cover goes over the housing, the plates and the waist motor up
to the torso.

The bought column (LINAK DL6, decisions 2026-11-07) is split between two links: its outer
profile stays on the base; its middle and inner profiles, spindle and motor housing ride with
this link (in the real column the middle profile moves half as far; drawn as one piece; the data
sheet's 7.5 kg split 2.5 / 5.0 kg by estimate).

usage: python -m cad.lite_column
"""

from __future__ import annotations

import manifold3d as mf

from . import lite_base as lb
from .b3d import circles, cut, disc, hull, lighten, made, rect, rounded
from .b3d import plate as bplate
from .lib import Part, box, union
from .lib import hull as mhull
from .mounts import cage, spec
from .torso import waist_mount

LINK = "column_link"
PLATE_T = 6.0
HOUSING = (177.0, 97.0, 46.0)  # the DL6's motor housing (data sheet), its length along x
HOUSING_FRONT = 29.4  # the column's axis from the housing's front end (data sheet)
HX0, HX1 = HOUSING_FRONT - HOUSING[0], HOUSING_FRONT  # the housing's x extent
# its long sides' M6 holes (data sheet: 145 apart, 17.4 from the end, 17 below the top)
SIDE_M6_X = (HX1 - 17.4, HX1 - 17.4 - 145.0)
SIDE_M6_Z = -17.0
CHEEK_T, CHEEK_H = 8.0, 34.0
CHEEK_Y = HOUSING[1] / 2 + 0.5  # the cheeks' inner faces
CHEEK_X = (HX0 + 3.0, HX1 - 3.0)
CHEEK_M5_X = (-110.0, -10.0)  # head plate to the cheeks' top edges
CABLE = (46.0, 0.0)  # the waist motor's cable, down in front of the housing
WAIST_JOINT_ABOVE_MOTOR = 9.0  # as on the pelvis (skeleton.yaml): the joint 9 mm above the centre
INNER_LEN = 470.0  # the moving profiles below the housing: the 450 mm stroke plus overlap


def waist_centre() -> tuple[float, float, float]:
    """The waist RS03's centre in this link's frame."""
    t03 = spec("RS03")["t"]
    return (0.0, 0.0, PLATE_T + t03 / 2 + 1.0)


def waist_joint() -> tuple[float, float, float]:
    c = waist_centre()
    return (c[0], c[1], c[2] + WAIST_JOINT_ABOVE_MOTOR)


def shroud() -> mf.Manifold:
    """A printed cover, 2.5 mm thick: straight sides round the housing, its cheeks and the head
    plate, then tapering to a circle just under the torso's hub, clear of the cage (r 53) and
    the ring (r 56). Open at the bottom (the column) and the top (the hub)."""
    t = 2.5
    x0, x1, hy = HX0 - 9.0, 72.0, CHEEK_Y + CHEEK_T + 6.0
    z0, zk, z1, r1 = -HOUSING[2] - 4.0, 12.0, 74.0, 66.0

    def body(grow: float) -> mf.Manifold:
        foot = mf.Manifold.extrude(
            mf.CrossSection([lb._rr(x1 - x0 + 2 * grow, 2 * (hy + grow), 18.0 + grow)]), 1.0
        ).translate([(x0 + x1) / 2, 0, 0])
        top = mf.Manifold.cylinder(1.0, r1 + grow, r1 + grow, 96)
        return mhull(
            foot.translate([0, 0, z0]), foot.translate([0, 0, zk]), top.translate([0, 0, z1 - 1])
        )

    shell = body(t) - body(0.0)
    return shell - box((400, 400, 20), (0, 0, z0 - 9.5)) - box((400, 400, 20), (0, 0, z1 + 9.0))


def parts() -> list[Part]:
    wc = waist_centre()
    t03 = 2 * (wc[2] - PLATE_T - 1.0)
    strips, pads, fea = cage("RS03", wc, "z", 1, to=t03 / 2 + 1.0)
    hy = CHEEK_Y + CHEEK_T
    sk = rounded(hull(disc(124), rect(HX0, HX1, -hy, hy)))
    m5 = [(x, s * (CHEEK_Y + CHEEK_T / 2)) for x in CHEEK_M5_X for s in (1, -1)]
    sk = cut(
        sk,
        disc(18, CABLE),  # the waist motor's cable, out of its cap, down in front of the housing
        circles(5.5, m5),
        lighten(sk, keep=[pads, circles(16, m5), disc(26, CABLE)], pitch=24, rim=6),
    )
    out = [
        made("column_head_plate", LINK, "al7075", bplate(sk, "z", PLATE_T / 2, PLATE_T)),
    ]
    # cheeks either side of the housing: 2 x M6 into its side, 2 x M5 up from the head plate
    for side, s in (("l", 1), ("r", -1)):
        ck = rect(CHEEK_X[0], CHEEK_X[1], -CHEEK_H, 0.0, r=3.0)
        ck = cut(ck, circles(6.6, [(x, SIDE_M6_Z) for x in SIDE_M6_X]))
        y = s * (CHEEK_Y + CHEEK_T / 2)
        out.append(made(f"column_head_cheek_{side}", LINK, "al6061", bplate(ck, "y", y, CHEEK_T)))
    out += waist_mount(LINK, wc, strips, pads, fea, prefix="column_head")
    out.append(
        Part(
            "column_head_shroud",
            LINK,
            "petg",
            shroud(),
            "print",
            notes="a cover over the column's motor housing, the head plate and the waist cage",
        )
    )
    # the column's moving profiles and motor housing (bought); the middle profile, 43.5 x 73.5
    stages = mf.Manifold.extrude(mf.CrossSection([lb._rr(43.5, 73.5, 5)]), INNER_LEN).translate(
        [0, 0, -HOUSING[2] - INNER_LEN]
    )
    plate = mf.Manifold.extrude(mf.CrossSection([lb._rr(HOUSING[0], HOUSING[1], 10)]), HOUSING[2])
    plate = plate.translate([(HX0 + HX1) / 2, 0, -HOUSING[2]])
    out.append(
        Part(
            "desk_column_inner",
            LINK,
            "bought",
            union(stages, plate),
            "bought",
            mass_g=5000.0,
            notes="the DL6's middle and inner profiles, spindle and motor housing (one piece)",
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
