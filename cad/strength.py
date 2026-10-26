"""Hand-calculation strength check of the legs', pelvis's and arms' load-carrying parts.

Each checked part is cut at several stations along its load path. At each cut the real cross
section of the CAD solid (lightening holes included) gives the area and the second moments of
area; the loads give the axial force and the bending moments there; the bending-plus-axial
stress is compared with the material's yield strength. Torsion and stress concentrations at
holes and corners are not included, so the target safety factor is 2 on yield.

Load cases, at the default straight-limb pose (all link frames aligned; the pelvis takes one
leg's landing load through its hip):
  landing: at the ankle axis, 3 x body weight up, 0.5 x fore-aft, 0.3 x sideways, together
  hand: for the arms' aluminium parts, at the gripper: 0.5 x body weight up, 0.25 x
        fore-aft, 0.15 x sideways (catching a fall or pushing up from the floor on one hand)
  peak torque: the joint's peak torque (description/actuators.yaml, the linkage ratios for the
               ankle) as a moment on the part that reacts it

This is a first-pass check; FEA of the final parts, and fatigue, come later.

Usage:
  python -m cad.strength          # table of the worst station per part and load case
  python -m cad.strength --check  # exit 1 if any safety factor is below the target (CI)

Needs: pip install -e ".[cad]"
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from . import arm, leg, torso

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
YIELD_MPA = {"al6061": 240.0, "al7075": 480.0}
TARGET = 2.0
G = 9.81


def section(solid, axis: str, at: float):
    """Cut `solid` (mm) perpendicular to `axis` at `at`: (area mm², centroid (u, v),
    Iuu, Ivv, Iuv mm⁴, vertices) in the cut plane's (u, v) coordinates, where (u, v) are the
    other two axes in x, y, z order."""
    rot = {"z": [0, 0, 0], "x": [0, -90, 0], "y": [90, 0, 0]}[axis]
    m = solid.rotate(rot) if axis != "z" else solid
    polys = m.slice(at).to_polygons()
    a = cu = cv = iuu = ivv = iuv = 0.0
    pts = []
    for poly in polys:
        p = np.asarray(poly, float)
        if axis == "x":  # rotate([0, -90, 0]) maps (x, y, z) -> (z, y, -x)... keep u = y, v = z
            p = np.column_stack([p[:, 1], -p[:, 0]])
        elif axis == "y":  # rotate([90, 0, 0]) maps y -> z, z -> -y: keep u = x, v = z
            p = np.column_stack([p[:, 0], -p[:, 1]])
        x, y = p[:, 0], p[:, 1]
        x1, y1 = np.roll(x, -1), np.roll(y, -1)
        cross = x * y1 - x1 * y
        a += cross.sum() / 2
        cu += ((x + x1) * cross).sum() / 6
        cv += ((y + y1) * cross).sum() / 6
        ivv += ((x * x + x * x1 + x1 * x1) * cross).sum() / 12  # about v axis (x²)
        iuu += ((y * y + y * y1 + y1 * y1) * cross).sum() / 12  # about u axis (y²)
        iuv += ((x * y1 + 2 * x * y + 2 * x1 * y1 + x1 * y) * cross).sum() / 24
        pts.append(p)
    if abs(a) < 1e-9:
        return None
    sign = 1.0 if a > 0 else -1.0
    a, cu, cv, iuu, ivv, iuv = (sign * v for v in (a, cu, cv, iuu, ivv, iuv))
    cu, cv = cu / a, cv / a
    iuu -= a * cv * cv
    ivv -= a * cu * cu
    iuv -= a * cu * cv
    return a, (cu, cv), iuu, ivv, iuv, np.vstack(pts)


def bending_stress(sec, n_force: float, m_u: float, m_v: float) -> float:
    """Peak |normal stress| (MPa) over the section's outline for axial force n (N) and bending
    moments about the section's u and v axes (N·mm), unsymmetric bending formula."""
    a, (cu, cv), iuu, ivv, iuv, pts = sec
    den = iuu * ivv - iuv * iuv
    u, v = pts[:, 0] - cu, pts[:, 1] - cv
    # sigma = N/A + [(Mu Ivv + Mv Iuv) v - (Mv Iuu + Mu Iuv) u] / den
    sig = n_force / a + ((m_u * ivv + m_v * iuv) * v - (m_v * iuu + m_u * iuv) * u) / den
    return float(np.max(np.abs(sig)))


@dataclass
class Check:
    part: str  # one part, or several joined by "+" and checked as one section (a yoke)
    axis: str  # cut perpendicular to this axis
    stations: list[float]  # mm along the axis
    point: tuple  # where the landing (or hand) force acts, in the part's link frame (mm)
    moment: tuple = (0.0, 0.0, 0.0)  # peak-torque case: moment vector (N·mm)
    load: str = "landing"  # or "hand"


def checks(peak: dict, mass: float) -> list[Check]:
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
    j = {k: np.array(v["xyz"]) * 1000 for k, v in skel["joints"].items()}
    ankle_shin = j["left_ankle_pitch"]  # ankle axis in the shin frame
    knee_thigh = j["left_knee"]
    ankle_thigh = knee_thigh + ankle_shin
    # the ankle in each frame up the leg: thigh, hip pitch link, hip roll link, pelvis
    ankle_pitch_frame = j["left_hip_yaw"] + ankle_thigh
    ankle_roll_frame = j["left_hip_pitch"] + ankle_pitch_frame
    ankle_pelvis = j["left_hip_roll"] + ankle_roll_frame
    ankle_pitch = 1000 * peak["left_ankle_pitch"]
    hand_roll = sum(
        j[k] for k in ("left_shoulder_yaw", "left_elbow", "left_wrist_yaw", "left_gripper")
    )
    hand_pitch = j["left_shoulder_roll"] + hand_roll
    return [
        # the two side plates act together as the shin; the ankle fork is their lower end
        Check(
            "left_shin_inner+left_shin_outer",
            "z",
            list(np.linspace(-80, -270, 12)),
            tuple(ankle_shin),
            (0.0, ankle_pitch, 0.0),
        ),
        Check(
            "left_shin_inner+left_shin_outer+left_shin_knee_outer+left_shin_bridge",
            "z",
            list(np.linspace(-30, -80, 6)),
            tuple(ankle_shin),
            (0.0, 1000 * peak["left_knee"], 0.0),
        ),
        Check(
            "left_thigh_plate",
            "z",
            list(np.linspace(-40, -150, 8)),
            tuple(ankle_thigh),
            (0.0, 1000 * peak["left_knee"], 0.0),
        ),
        # hip pitch link: the U-yoke's two plates round the pitch motor, down to the yaw motor
        Check(
            "left_hip_pitch_outer+left_hip_pitch_inner",
            "z",
            list(np.linspace(-40, -105, 6)),
            tuple(ankle_pitch_frame),
            (0.0, 1000 * peak["left_hip_pitch"], 0.0),
        ),
        # roll bracket: the plate reaching forward to the pitch motor, and the front plate
        # carrying it from the roll motor's output
        Check(
            "left_hip_roll_stator",
            "x",
            list(np.linspace(-62, -30, 5)),
            tuple(ankle_roll_frame),
            (0.0, 1000 * peak["left_hip_pitch"], 0.0),
        ),
        Check(
            "left_hip_roll_front",
            "y",
            list(np.linspace(-50, -25, 4)),
            tuple(ankle_roll_frame),
            (1000 * peak["left_hip_roll"], 0.0, 0.0),
        ),
        # pelvis: the back plate between a roll motor and the middle, then the top plate and
        # gussets as a channel carrying one leg's landing to the waist
        Check(
            "pelvis_back",
            "y",
            list(np.linspace(30, 45, 4)),
            tuple(ankle_pelvis),
        ),
        Check(
            "pelvis_top+pelvis_gusset_l+pelvis_gusset_r",
            "x",
            list(np.linspace(-120, -50, 8)),
            tuple(ankle_pelvis),
        ),
        Check(
            "left_foot_base+left_foot_rib_a+left_foot_rib_b",
            "x",
            list(np.linspace(40, 140, 6)),
            (150.0, 0.0, -60.0),
        ),
        # arms: the shoulder pitch bracket's back plate carries the roll motor; the roll
        # bracket's front and top plates carry the yaw motor and everything below
        Check(
            "left_shoulder_back",
            "z",
            list(np.linspace(-85, -20, 6)),
            tuple(hand_pitch),
            (1000 * peak["left_shoulder_roll"], 0.0, 0.0),
            "hand",
        ),
        Check(
            "left_shoulder_roll_front",
            "z",
            list(np.linspace(-50, -10, 5)),
            tuple(hand_roll),
            (0.0, 0.0, 1000 * peak["left_shoulder_yaw"]),
            "hand",
        ),
    ]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    sys.path.insert(0, str(ROOT / "tools"))
    import build_urdf  # noqa: E402

    urdf = build_urdf.URDF_OUT.read_text()
    import re

    mass = sum(float(x) for x in re.findall(r'<mass value="([^"]+)"', urdf))
    peak = {}
    for m in re.finditer(r'<joint name="([^"]+)".*?effort="([^"]+)"', urdf, re.S):
        peak[m.group(1)] = float(m.group(2))
    w = mass * G
    landing = np.array([0.5 * w, 0.3 * w, 3.0 * w])  # N: fore-aft, sideways, up
    hand = np.array([0.25 * w, 0.15 * w, 0.5 * w])
    parts = {p.name: p for p in leg.parts() + torso.parts() + arm.parts()}
    worst_sf = np.inf
    print(
        f"robot {mass:.1f} kg; landing load at the ankle {np.round(landing):} N, hand load"
        f" {np.round(hand):} N; target SF {TARGET}"
    )
    print(f"{'part':34} {'case':8} {'station':>8} {'stress':>8} {'yield':>6} {'SF':>5}")
    for c in checks(peak, mass):
        names = c.part.split("+")
        ys = YIELD_MPA[parts[names[0]].material]
        solid = parts[names[0]].solid
        for n in names[1:]:
            solid = solid + parts[n].solid
        label = names[0] + (" (combined)" if len(names) > 1 else "")
        ax = "xyz".index(c.axis)
        uv = [i for i in range(3) if i != ax]
        for case in (c.load, "peak torque"):
            worst = (0.0, None)
            for s in c.stations:
                sec = section(solid, c.axis, s)
                if sec is None:
                    continue
                centroid = np.zeros(3)
                centroid[ax] = s
                centroid[uv[0]], centroid[uv[1]] = sec[1]
                if case in ("landing", "hand"):
                    force = landing if case == "landing" else hand
                    mom = np.cross(np.array(c.point) - centroid, force)  # N·mm
                else:
                    force = np.zeros(3)
                    mom = np.array(c.moment)
                if not mom.any() and not force.any():
                    continue
                stress = bending_stress(sec, force[ax], mom[uv[0]], mom[uv[1]])
                if stress > worst[0]:
                    worst = (stress, s)
            if worst[1] is None:
                continue
            sf = ys / worst[0] if worst[0] > 0 else np.inf
            worst_sf = min(worst_sf, sf)
            flag = "  <-- below target" if sf < TARGET else ""
            print(f"{label:34} {case:8} {worst[1]:8.0f} {worst[0]:8.0f} {ys:6.0f} {sf:5.1f}{flag}")
    if args.check and worst_sf < TARGET:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
