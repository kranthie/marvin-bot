"""Build the robot's URDF from the description YAML files.

Reads description/robot.yaml (joints, ranges, motors), actuators.yaml (torque, speed, mass,
size), mass_budget.yaml (non-motor masses) and skeleton.yaml (frames, where masses sit,
collision boxes), mirrors the left side to the right, and writes
description/urdf/marvin_bot.urdf.

Model:
- each link's inertia combines its structure boxes (skeleton.yaml "bodies") and the motor
  housings mounted on it, as solid cylinders along the joint axis;
- joint effort and velocity limits are the motor's peak torque and no-load speed; the two
  ankle joints, driven by a pair of motors through a linkage, get the pair's combined torque
  times the linkage ratios in skeleton.yaml;
- collision geometry is the skeleton's boxes; visuals are those boxes plus the motor housings.

Usage:
  python tools/build_urdf.py           # write the URDF and print the mass summary
  python tools/build_urdf.py --check   # exit 1 if the URDF is out of date (used by CI)
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
URDF_OUT = DESC / "urdf" / "marvin_bot.urdf"
SIDES = ("left", "right")
DEFAULT_ENVELOPE_MM = [80, 80, 50]  # for actuators without a measured envelope yet
MOTOR_COLOURS = {
    "RS04": "0.85 0.33 0.31 1",
    "RS03": "0.23 0.49 0.85 1",
    "RS06": "0.23 0.65 0.33 1",
    "RS00": "0.56 0.36 0.82 1",
    "STS3250": "0.88 0.66 0.0 1",
}
BODY_COLOUR = "0.19 0.20 0.23 1"
TOL = 1e-6


# ---------------------------------------------------------------------------------- helpers


def mirror_name(name: str) -> str:
    return "right_" + name[len("left_") :] if name.startswith("left_") else name


def mirror_xyz(v: list[float]) -> list[float]:
    return [v[0], -v[1], v[2]]


def mirror_axis(a: list[float]) -> list[float]:
    """Pitch axes (y) stay; roll (x) and yaw (z) axes flip, so ranges keep their meaning."""
    return [-a[0], a[1], -a[2]]


def fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


def mat_add(a, b):
    return [[a[i][j] + b[i][j] for j in range(3)] for i in range(3)]


def scale(a, s):
    return [[a[i][j] * s for j in range(3)] for i in range(3)]


def eye():
    return [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]


def outer(u, v):
    return [[u[i] * v[j] for j in range(3)] for i in range(3)]


def box_inertia(m: float, size: list[float]):
    x, y, z = size
    return [
        [m * (y * y + z * z) / 12, 0, 0],
        [0, m * (x * x + z * z) / 12, 0],
        [0, 0, m * (x * x + y * y) / 12],
    ]


def cylinder_inertia(m: float, diameter: float, height: float, axis: list[float]):
    r = diameter / 2
    i_axis = m * r * r / 2
    i_perp = m * (3 * r * r + height * height) / 12
    n = math.sqrt(sum(a * a for a in axis))
    a = [x / n for x in axis]
    return mat_add(scale(eye(), i_perp), scale(outer(a, a), i_axis - i_perp))


def combine(parts):
    """Mass, centre of mass and inertia about it of (mass, com, inertia-about-own-com) parts."""
    m = sum(p[0] for p in parts)
    com = [sum(p[0] * p[1][k] for p in parts) / m for k in range(3)]
    inertia = [[0.0] * 3 for _ in range(3)]
    for pm, pc, pi in parts:
        d = [pc[k] - com[k] for k in range(3)]
        dd = sum(x * x for x in d)
        inertia = mat_add(
            inertia, mat_add(pi, scale(mat_add(scale(eye(), dd), scale(outer(d, d), -1)), pm))
        )
    return m, com, inertia


# ------------------------------------------------------------------------------ the model


def expand(entries: list[dict]) -> list[dict]:
    """robot.yaml entries with "{side}" become a left and a right copy."""
    out = []
    for e in entries:
        sides = SIDES if any("{side}" in str(v) for v in e.values()) else (None,)
        for side in sides:

            def sub(v, side=side):
                if isinstance(v, str):
                    return v.replace("{side}", side) if side else v
                if isinstance(v, list):
                    return [sub(x) for x in v]
                return v

            out.append({k: sub(v) for k, v in e.items()})
    return out


def mirrored(items: dict | list, key_is_name: bool):
    """Add right-side copies of left-side skeleton entries."""
    if key_is_name:
        out = dict(items)
        for name, e in items.items():
            if name.startswith("left_"):
                r = dict(e)
                for k in ("parent", "child", "link"):
                    if k in r:
                        r[k] = mirror_name(r[k])
                r["xyz"] = mirror_xyz(e["xyz"])
                if "axis" in r:
                    r["axis"] = mirror_axis(e["axis"])
                out[mirror_name(name)] = r
        return out
    out = list(items)
    for e in items:
        if e["link"].startswith("left_"):
            out.append({**e, "link": mirror_name(e["link"]), "xyz": mirror_xyz(e["xyz"])})
    return out


def motor_placements() -> list[dict]:
    """Every motor housing: name, model, the link carrying it, centre and axis in that link's
    frame, envelope (diameter, -, thickness in m), and the joints it drives.

    A housing sits on the parent link of the (first) joint it drives, centred on the joint origin
    along the joint axis, unless skeleton.yaml's motor_mounts moves it (``xyz``, and ``axis``
    if it is not the joint axis; the ankle motors default to a y axis)."""
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
    joints = mirrored(skel["joints"], key_is_name=True)
    mounts = mirrored(skel["motor_mounts"], key_is_name=True)
    out = []
    for m in expand(robot["motors"]):
        a = acts[m["model"]]
        env = [x / 1000 for x in (a.get("envelope_mm") or DEFAULT_ENVELOPE_MM)]
        j = joints[m["drives"][0]]
        mount = mounts.get(m["name"])
        if mount:
            link, xyz = mount["link"], mount["xyz"]
            axis = mount.get("axis", j["axis"] if len(m["drives"]) == 1 else [0, 1, 0])
        else:
            link, xyz, axis = j["parent"], j["xyz"], j["axis"]
        out.append(
            dict(
                name=m["name"],
                model=m["model"],
                link=link,
                xyz=xyz,
                axis=axis,
                envelope=env,
                drives=m["drives"],
            )
        )
    return out


def build() -> tuple[str, dict]:
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    budget = yaml.safe_load((DESC / "mass_budget.yaml").read_text())["items"]
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())

    ranges = {j["name"]: j["range"] for j in expand(robot["joints"])}
    joints = mirrored(skel["joints"], key_is_name=True)
    bodies = mirrored(skel["bodies"], key_is_name=False)
    collisions = mirrored(skel["collisions"], key_is_name=False)
    errors = []

    # skeleton lengths must match robot.yaml
    dims = robot["dimensions"]
    z = {n: -j["xyz"][2] for n, j in joints.items()}
    checks = {
        "thigh": z["left_hip_roll"] + z["left_hip_yaw"] + z["left_knee"],
        "shin": z["left_ankle_pitch"],
        "upper_arm": z["left_shoulder_yaw"] + z["left_elbow"],
        "forearm": z["left_wrist_yaw"] + z["left_gripper"],
        "shoulder_height": dims["hip_axis_height"]
        + joints["waist_yaw"]["xyz"][2]
        + joints["left_shoulder_pitch"]["xyz"][2],
    }
    for k, v in checks.items():
        if abs(v - dims[k]) > 1e-6:
            errors.append(f"skeleton gives {k} = {v:.3f} m, robot.yaml says {dims[k]:.3f} m")
    if set(joints) != set(ranges):
        errors.append(f"joints differ: {sorted(set(joints) ^ set(ranges))}")

    # every budget item fully placed
    placed: dict[str, float] = {}
    for b in bodies:
        placed[b["budget"]] = placed.get(b["budget"], 0.0) + b["mass"]
    for k, v in budget.items():
        if abs(placed.get(k, 0.0) - v["mass"]) > TOL:
            errors.append(
                f"budget {k}: {v['mass']} kg, skeleton places {placed.get(k, 0.0):.3f} kg"
            )

    links: dict[str, list] = {"pelvis": []}
    for j in joints.values():
        links.setdefault(j["parent"], [])
        links.setdefault(j["child"], [])
    visuals: dict[str, list[str]] = {n: [] for n in links}
    for b in bodies:
        links[b["link"]].append((b["mass"], b["xyz"], box_inertia(b["mass"], b["size"])))
        visuals[b["link"]].append(
            f'<visual><origin xyz="{fmt(b["xyz"])}"/><geometry><box size="{fmt(b["size"])}"/>'
            f'</geometry><material name="body"><color rgba="{BODY_COLOUR}"/></material></visual>'
        )

    # motors: mass on the parent link of the (first) joint they drive
    effort: dict[str, float] = {}
    velocity: dict[str, float] = {}
    pairs: dict[tuple, list] = {}
    for m in motor_placements():
        a = acts[m["model"]]
        env, link, xyz, axis = m["envelope"], m["link"], m["xyz"], m["axis"]
        mass = a["mass"]
        links[link].append((mass, xyz, cylinder_inertia(mass, env[0], env[2], axis)))
        rpy = (
            "1.5708 0 0"
            if abs(axis[1]) > 0.5
            else ("0 1.5708 0" if abs(axis[0]) > 0.5 else "0 0 0")
        )
        visuals[link].append(
            f'<visual><origin xyz="{fmt(xyz)}" rpy="{rpy}"/><geometry>'
            f'<cylinder radius="{env[0] / 2:.6g}" length="{env[2]:.6g}"/></geometry>'
            f'<material name="{m["model"]}"><color rgba="{MOTOR_COLOURS[m["model"]]}"/></material>'
            "</visual>"
        )
        speed = a["no_load_speed_rpm"] * 2 * math.pi / 60
        if len(m["drives"]) == 1:
            effort[m["drives"][0]] = a["peak_torque"]
            velocity[m["drives"][0]] = speed
        else:
            pairs.setdefault(tuple(m["drives"]), []).append((a["peak_torque"], speed))
    ratios = skel["ankle_linkage"]
    for (pitch, roll), ms in pairs.items():
        torque = sum(t for t, _ in ms)
        speed = min(s for _, s in ms)
        effort[pitch] = torque * ratios["pitch_ratio"]
        effort[roll] = torque * ratios["roll_ratio"]
        velocity[pitch] = speed / ratios["pitch_ratio"]
        velocity[roll] = speed / ratios["roll_ratio"]

    if errors:
        for e in errors:
            print(f"ERROR: {e}")
        sys.exit(1)

    # write
    out = [f'<robot name="{robot["robot"]["name"]}">']
    summary = {}
    for name, parts in links.items():
        if not parts:
            parts = [(0.01, [0.0, 0.0, 0.0], box_inertia(0.01, [0.02, 0.02, 0.02]))]
        m, com, i = combine(parts)
        summary[name] = m
        out.append(f'  <link name="{name}">')
        out.append(
            f'    <inertial><origin xyz="{fmt(com)}"/><mass value="{m:.6g}"/>'
            f'<inertia ixx="{i[0][0]:.6g}" ixy="{i[0][1]:.6g}" ixz="{i[0][2]:.6g}" '
            f'iyy="{i[1][1]:.6g}" iyz="{i[1][2]:.6g}" izz="{i[2][2]:.6g}"/></inertial>'
        )
        out += [f"    {v}" for v in visuals[name]]
        for c in collisions:
            if c["link"] == name:
                out.append(
                    f'    <collision><origin xyz="{fmt(c["xyz"])}"/><geometry>'
                    f'<box size="{fmt(c["size"])}"/></geometry></collision>'
                )
        out.append("  </link>")
    for name, j in joints.items():
        lo, hi = (math.radians(x) for x in ranges[name])
        out += [
            f'  <joint name="{name}" type="revolute">',
            f'    <parent link="{j["parent"]}"/><child link="{j["child"]}"/>',
            f'    <origin xyz="{fmt(j["xyz"])}"/><axis xyz="{fmt(j["axis"])}"/>',
            f'    <limit lower="{lo:.6g}" upper="{hi:.6g}" effort="{effort[name]:.6g}" '
            f'velocity="{velocity[name]:.6g}"/>',
            "  </joint>",
        ]
    out.append("</robot>")
    header = (
        "<?xml version='1.0' encoding='utf-8'?>\n"
        "<!-- Generated by tools/build_urdf.py from description/*.yaml. Do not edit. -->\n"
    )
    ET.fromstring("\n".join(out))  # well-formed
    return header + "\n".join(out) + "\n", summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="fail if the URDF is out of date")
    args = ap.parse_args()
    text, summary = build()
    if args.check:
        if not URDF_OUT.exists() or URDF_OUT.read_text() != text:
            print(f"{URDF_OUT.relative_to(ROOT)} is out of date: run python tools/build_urdf.py")
            return 1
        print("URDF up to date")
        return 0
    URDF_OUT.parent.mkdir(parents=True, exist_ok=True)
    URDF_OUT.write_text(text)
    print(
        f"wrote {URDF_OUT.relative_to(ROOT)}: {len(summary)} links, {sum(summary.values()):.2f} kg"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
