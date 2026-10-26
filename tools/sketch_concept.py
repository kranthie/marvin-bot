"""Draw the front and side concept layout of Marvin Bot to scale (mm).

The parts are hand-placed at reference proportions (ankle axis 70 mm, knee 370 mm, hip
690 mm, overall 1255 mm) and then remapped vertically to the ankle, knee and hip heights and
overall height in description/robot.yaml, so leg changes there redraw the sketch. It is a
concept sketch, not generated geometry. Actuators are drawn as discs or boxes depending on
whether their axis points at the viewer, at their STEP-model sizes.

Usage:
  pip install -e ".[sketch]"
  python tools/sketch_concept.py docs/design/concept_v0
"""

import sys
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse, FancyBboxPatch, Polygon, Rectangle

ROOT = Path(__file__).resolve().parent.parent
REF_Z = [0, 70, 370, 690, 1255]  # sole, ankle, knee, hip, top of head as drawn

# type: (diameter, thickness, colour, label)
ACT = {
    "RS04": (120, 56, "#d9534f", "Robstride RS04 (120 N·m)"),
    "RS03": (100, 57, "#3b7dd8", "Robstride RS03 (60 N·m)"),
    "RS06": (85, 51, "#3aa655", "Robstride RS06 (36 N·m)"),
    "RS00": (57, 51, "#8e5bd0", "Robstride RS00 (14 N·m)"),
    "STS": (45, 25, "#e0a800", "Feetech STS3250 (neck, gripper)"),
}
LINK = "#2f343b"
SHELL = "#e9ebef"
ALU = "#b8bec7"

# (name, type, x lateral (+left), y forward, z up, axis)
J = []
for s, side in ((1, "L"), (-1, "R")):
    J += [
        (f"{side} hip pitch", "RS04", s * 78, 0, 690, "x"),
        (f"{side} hip roll", "RS04", s * 100, -90, 605, "y"),  # behind the hip pitch motor
        (f"{side} hip yaw", "RS03", s * 100, 0, 545, "z"),
        (f"{side} knee", "RS04", s * 100, 0, 370, "x"),
        (f"{side} ankle A", "ANKLE", s * (100 - 32), -18, 245, "x"),
        (f"{side} ankle B", "ANKLE", s * (100 + 32), -18, 245, "x"),
        (f"{side} shoulder pitch", "RS03", s * 110, 0, 1055, "x"),
        (f"{side} shoulder roll", "RS03", s * 188, 0, 1005, "y"),
        (f"{side} shoulder yaw", "RS06", s * 190, 0, 920, "z"),
        (f"{side} elbow", "RS06", s * 190, 0, 805, "x"),
        (f"{side} wrist yaw", "RS00", s * 190, 0, 680, "z"),
        (f"{side} gripper", "STS", s * 190, 0, 590, "x"),
    ]
J += [
    ("waist yaw", "RS03", 0, 0, 805, "z"),
    ("neck yaw", "STS", 0, 0, 1095, "z"),
    ("neck pitch", "STS", 0, 0, 1130, "x"),
]


def draw_act(ax, kind, u, z, axis, view):
    d, t, c, _ = ACT[kind]
    circle = (view == "front" and axis == "y") or (view == "side" and axis == "x")
    if circle:
        ax.add_patch(Circle((u, z), d / 2, fc=ALU, ec="#555", lw=0.8, zorder=5))
        ax.add_patch(Circle((u, z), d / 2 - 6, fc="none", ec=c, lw=2.2, zorder=6))
        ax.add_patch(Circle((u, z), d / 7, fc="#7d848e", ec="none", zorder=6))
    else:
        w, h = (d, t) if axis == "z" else (t, d)
        ax.add_patch(
            FancyBboxPatch(
                (u - w / 2, z - h / 2),
                w,
                h,
                boxstyle="round,pad=0,rounding_size=6",
                fc=ALU,
                ec="#555",
                lw=0.8,
                zorder=5,
            )
        )
        ax.add_patch(
            Rectangle(
                (u - w / 2 + 3, z - h / 2 + 3), w - 6, h - 6, fc="none", ec=c, lw=2.0, zorder=6
            )
        )


def taper(ax, u0, z0, w0, u1, z1, w1, fc=LINK, z=2, ec="#1b1e22"):
    return ax.add_patch(
        Polygon(
            [(u0 - w0 / 2, z0), (u0 + w0 / 2, z0), (u1 + w1 / 2, z1), (u1 - w1 / 2, z1)],
            closed=True,
            fc=fc,
            ec=ec,
            lw=0.8,
            zorder=z,
            joinstyle="round",
        )
    )


def body(ax, view):
    sides = (1, -1) if view == "front" else (1,)
    for s in sides:
        lx = s * 100 if view == "front" else 0
        # thigh, shin, feet
        taper(ax, lx, 650, 118 if view == "front" else 128, lx, 400, 92 if view == "front" else 104)
        taper(ax, lx, 340, 96 if view == "front" else 100, lx, 95, 58 if view == "front" else 62)
        if view == "front":
            ax.add_patch(
                FancyBboxPatch(
                    (lx - 48, 0),
                    96,
                    34,
                    boxstyle="round,pad=0,rounding_size=10",
                    fc=LINK,
                    ec="#1b1e22",
                    zorder=3,
                )
            )
            for dx in (-30, 30):  # push rods
                ax.plot([lx + dx, lx + dx * 0.9], [240, 82], color="#8a9099", lw=3, zorder=4)
        else:
            ax.add_patch(
                Polygon(
                    [(-75, 0), (165, 0), (175, 12), (150, 30), (40, 42), (-60, 40), (-80, 18)],
                    fc=LINK,
                    ec="#1b1e22",
                    zorder=3,
                )
            )
            ax.plot([-18 + 45, -38], [262, 78], color="#8a9099", lw=3, zorder=4)
            ax.plot([-18 - 40, -42], [250, 78], color="#8a9099", lw=3, zorder=4)
        ax.add_patch(
            Circle((lx if view == "front" else 0, 70), 14, fc="#7d848e", ec="#444", zorder=6)
        )
        # arm links
        if view == "front":
            ax_ = s * 190
            taper(ax, ax_, 975, 82, ax_, 830, 70)
            taper(ax, ax_, 785, 66, ax_, 700, 58)
            ax.add_patch(
                FancyBboxPatch(
                    (ax_ - 32, 520),
                    64,
                    90,
                    boxstyle="round,pad=0,rounding_size=14",
                    fc="#3d434b",
                    ec="#1b1e22",
                    zorder=4,
                )
            )
        else:
            for p_ in (
                taper(ax, 0, 975, 84, 0, 830, 72, z=7),
                taper(ax, 0, 785, 68, 0, 700, 58, z=7),
            ):
                p_.set_alpha(0.55)
            ax.add_patch(
                FancyBboxPatch(
                    (-32, 520),
                    64,
                    90,
                    boxstyle="round,pad=0,rounding_size=14",
                    fc="#3d434b",
                    ec="#1b1e22",
                    zorder=8,
                    alpha=0.55,
                )
            )
    if view == "front":
        # pelvis
        ax.add_patch(
            FancyBboxPatch(
                (-62, 640),
                124,
                135,
                boxstyle="round,pad=0,rounding_size=18",
                fc=LINK,
                ec="#1b1e22",
                zorder=3,
            )
        )
        # torso: chest vest (structural shell) over an aluminium spine
        ax.add_patch(
            Polygon(
                [
                    (-92, 835),
                    (92, 835),
                    (125, 960),
                    (122, 1075),
                    (60, 1092),
                    (-60, 1092),
                    (-122, 1075),
                    (-125, 960),
                ],
                fc=SHELL,
                ec="#9aa1ab",
                lw=1.2,
                zorder=4,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (-55, 880),
                110,
                110,
                boxstyle="round,pad=0,rounding_size=8",
                fc="none",
                ec="#9aa1ab",
                lw=1,
                ls="--",
                zorder=4,
            )
        )
        ax.text(
            0,
            935,
            "computer\n(behind)",
            ha="center",
            va="center",
            fontsize=6,
            color="#6b7280",
            zorder=4,
        )
        # head: smooth egg, black visor
        ax.add_patch(Ellipse((0, 1170), 160, 168, fc="#16181c", ec="#000", zorder=7))
        ax.add_patch(
            FancyBboxPatch(
                (-62, 1150),
                124,
                22,
                boxstyle="round,pad=0,rounding_size=8",
                fc="#2c3440",
                ec="#56606e",
                zorder=8,
            )
        )
        for cx in (-47, 47):
            ax.add_patch(Circle((cx, 1161), 6, fc="#8fb3e0", zorder=9))
        ax.add_patch(Rectangle((-18, 1088), 36, 40, fc=LINK, zorder=3))
    else:
        ax.add_patch(
            FancyBboxPatch(
                (-70, 640),
                140,
                135,
                boxstyle="round,pad=0,rounding_size=18",
                fc=LINK,
                ec="#1b1e22",
                zorder=3,
            )
        )
        ax.add_patch(
            Polygon(
                [
                    (-80, 835),
                    (55, 835),
                    (85, 960),
                    (78, 1070),
                    (30, 1092),
                    (-60, 1092),
                    (-95, 1040),
                    (-95, 900),
                ],
                fc=SHELL,
                ec="#9aa1ab",
                lw=1.2,
                zorder=4,
            )
        )
        ax.add_patch(
            FancyBboxPatch(
                (-90, 850),
                50,
                170,
                boxstyle="round,pad=0,rounding_size=6",
                fc="none",
                ec="#9aa1ab",
                lw=1,
                ls="--",
                zorder=4,
            )
        )
        ax.text(
            -65,
            935,
            "battery\n13S3P",
            ha="center",
            va="center",
            fontsize=6,
            color="#6b7280",
            rotation=90,
            zorder=4,
        )
        ax.add_patch(
            FancyBboxPatch(
                (-20, 880),
                72,
                110,
                boxstyle="round,pad=0,rounding_size=6",
                fc="none",
                ec="#9aa1ab",
                lw=1,
                ls="--",
                zorder=4,
            )
        )
        ax.text(
            16, 935, "computer", ha="center", va="center", fontsize=6, color="#6b7280", zorder=4
        )
        ax.add_patch(Ellipse((8, 1170), 150, 168, fc="#16181c", ec="#000", zorder=7))
        ax.add_patch(
            Polygon([(55, 1150), (83, 1152), (83, 1172), (55, 1174)], fc="#2c3440", zorder=8)
        )
        ax.add_patch(Rectangle((-18, 1088), 36, 40, fc=LINK, zorder=3))


def joints(ax, view, ankle_model):
    for _name, kind, x, y, z, axis in J:
        if view == "side" and x < 0:
            continue  # right side hidden behind left
        u = x if view == "front" else y
        draw_act(ax, ankle_model if kind == "ANKLE" else kind, u, z, axis, view)


def remap(ax, new_z):
    """Move everything drawn at REF_Z heights to new_z, piecewise linearly, without
    distorting round parts (circles and ellipses keep their size, only their centres move)."""

    def f(z):
        return np.interp(z, REF_Z, new_z, right=None) if z <= REF_Z[-1] else z + new_z[-1] - 1255

    for p in ax.patches:
        if isinstance(p, (Circle, Ellipse)):
            x, z = p.center
            p.center = (x, f(z))
        elif isinstance(p, FancyBboxPatch):
            z0, z1 = f(p.get_y()), f(p.get_y() + p.get_height())
            p.set_y(z0)
            p.set_height(z1 - z0)
        elif isinstance(p, Rectangle):
            z0, z1 = f(p.get_y()), f(p.get_y() + p.get_height())
            p.set_y(z0)
            p.set_height(z1 - z0)
        elif isinstance(p, Polygon):
            p.set_xy([(x, f(z)) for x, z in p.get_xy()])
    for line in ax.lines:
        line.set_ydata([f(z) for z in line.get_ydata()])
    for t in ax.texts:
        x, z = t.get_position()
        t.set_position((x, f(z)))


def dim(ax, u, z0, z1, text, side=1):
    ax.annotate("", (u, z0), (u, z1), arrowprops=dict(arrowstyle="<->", lw=0.7, color="#555"))
    ax.text(
        u + side * 6,
        (z0 + z1) / 2,
        text,
        fontsize=7,
        color="#333",
        rotation=90,
        va="center",
        ha="left" if side > 0 else "right",
    )


def main(out):
    matplotlib.rcParams["svg.hashsalt"] = "marvin-bot"  # stable element ids: stable diffs
    robot = yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())
    d = {k: v * 1000 for k, v in robot["dimensions"].items()}
    ankle_model = next(m["model"] for m in robot["motors"] if m["name"] == "{side}_ankle_a")
    ankle, hip = d["ankle_height"], d["hip_axis_height"]
    knee = ankle + d["shin"]
    top = robot["robot"]["height"] * 1000
    assert abs(knee + d["thigh"] - hip) < 1, "thigh + shin + ankle height must equal hip height"
    fig, axs = plt.subplots(1, 2, figsize=(12, 9.2), gridspec_kw=dict(width_ratios=[1.15, 1]))
    for ax, view in zip(axs, ("front", "side"), strict=True):
        body(ax, view)
        joints(ax, view, ankle_model)
        remap(ax, [0, ankle, knee, hip, top])
        ax.axhline(0, color="#888", lw=1)
        ax.set_aspect("equal")
        ax.set_xlim(-330, 330) if view == "front" else ax.set_xlim(-280, 300)
        ax.set_ylim(-40, 1300)
        ax.axis("off")
        ax.set_title("Front" if view == "front" else "Side (left)", fontsize=11)
    a = axs[0]
    up = top - 1255  # upper body moves with the overall height
    dim(a, -300, 0, top, f"{top:.0f} mm overall")
    dim(a, 270, 0, hip, f"{hip:.0f} hip axis to sole")
    dim(a, 230, knee, hip, f"{d['thigh']:.0f} thigh")
    dim(a, 230, ankle, knee, f"{d['shin']:.0f} shin")
    a.annotate(
        "",
        (-243, top + 20),
        (243, top + 20),
        arrowprops=dict(arrowstyle="<->", lw=0.7, color="#555"),
    )
    a.text(0, top + 28, "≈ 490 mm shoulder width", fontsize=7, ha="center", color="#333")
    b = axs[1]
    dim(b, 250, 805 + up, 1005 + up, "200 upper arm")
    dim(b, 250, 590 + up, 805 + up, "215 forearm + wrist")
    handles = [Rectangle((0, 0), 1, 1, fc=ALU, ec=v[2], lw=2.5) for v in ACT.values()]
    handles += [Rectangle((0, 0), 1, 1, fc=LINK), Rectangle((0, 0), 1, 1, fc=SHELL, ec="#9aa1ab")]
    labels = [v[3] for v in ACT.values()] + [
        "structural link (Al core + printed skin)",
        "chest shell (structural)",
    ]
    fig.legend(handles, labels, loc="lower center", ncol=4, fontsize=8, frameon=False)
    fig.suptitle(
        f"Marvin Bot: concept layout v{robot['robot']['version']} "
        "(to scale, 25 powered DOF + grippers)",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.97))
    fig.savefig(out + ".svg", metadata={"Date": None})  # no timestamp: stable diffs


if __name__ == "__main__":
    main(sys.argv[1])
