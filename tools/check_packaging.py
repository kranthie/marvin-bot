"""Check that the motors and structure don't run into each other anywhere in the joint ranges.

Places every motor housing at its real size (description/actuators.yaml envelopes, from the
STEP models) and every collision box of skeleton.yaml on its link, poses the robot with MuJoCo
forward kinematics of the URDF, and measures the overlap volume (manifold3d) of:

- every pair of motor housings on different links;
- a motor housing and a collision box on links that are not parent and child (the box of the
  link a motor drives sits against that motor by design);
- collision boxes on links that are not parent and child.

Overlaps within one limb, or between a limb and the body it is mounted on (legs on the pelvis,
arms and head on the torso), or between the pelvis and the torso, are packaging errors.
Overlaps between different limbs, or between an arm and the pelvis (legs crossing, a hand
touching a hip), are ordinary self-collision, which the simulators model and the controllers
avoid; they are listed as warnings.

Poses: the default stance; each leg and arm joint swept over its range with the rest at the
stance; and a few combined poses (deep squat, stair step, sitting, legs crossed in roll and yaw).
Fails if any packaging error's overlap exceeds MAX_OVERLAP.

Usage:
  python tools/check_packaging.py
  python tools/check_packaging.py --verbose     # list every overlapping pair and pose

Needs: pip install -e ".[sim,cad]"
"""

import argparse
import math
import sys
from pathlib import Path

import manifold3d as mf
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
import build_urdf as bu  # noqa: E402

MAX_OVERLAP = 1e-6  # m³ (1 cm³)
SWEEP = 9


def cylinder(d: float, h: float, axis) -> mf.Manifold:
    m = mf.Manifold.cylinder(h, d / 2, d / 2, 48, center=True)
    z = np.asarray(axis, float) / np.linalg.norm(axis)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.cross(ref, z)
    x /= np.linalg.norm(x)
    rot = np.column_stack([x, np.cross(z, x), z])
    return m.transform(np.hstack([rot, np.zeros((3, 1))]))


def place(m: mf.Manifold, rot: np.ndarray, pos) -> mf.Manifold:
    return m.transform(np.hstack([rot, np.asarray(pos, float).reshape(3, 1)]))


def shapes() -> dict[str, list[tuple[str, str, mf.Manifold]]]:
    """Per link: (kind, name, solid in the link frame)."""
    skel = yaml.safe_load((bu.DESC / "skeleton.yaml").read_text())
    out: dict[str, list] = {}
    for m in bu.motor_placements():
        parts = []
        for p in bu.motor_primitives(m):
            if p["kind"] == "box":
                solid = mf.Manifold.cube(p["size"], center=True)
            else:
                solid = cylinder(p["d"], p["length"], p["axis"])
            parts.append(place(solid, np.eye(3), p["centre"]))
        s = mf.Manifold.batch_boolean(parts, mf.OpType.Add)
        out.setdefault(m["link"], []).append(("motor", m["name"], s))
    for c in bu.mirrored(skel["collisions"], key_is_name=False):
        s = place(mf.Manifold.cube(c["size"], center=True), np.eye(3), c["xyz"])
        out.setdefault(c["link"], []).append(("box", c["link"], s))
    return out


def poses(model: mujoco.MjModel, robot: dict) -> list[tuple[str, dict]]:
    stance = robot["stance"]
    names = [model.joint(j).name for j in range(model.njnt)]

    def base() -> dict:
        q = {}
        for n in names:
            key = n.split("_", 1)[1] if n.startswith(("left_", "right_")) else n
            q[n] = math.radians(stance.get(key, 0.0))
        return q

    out = [("stance", base())]
    swept = [n for n in names if n.startswith("left_") and "gripper" not in n] + ["waist_yaw"]
    for n in swept:
        lo, hi = model.jnt_range[model.joint(n).id]
        for v in np.linspace(lo, hi, SWEEP):
            q = base()
            q[n] = v
            out.append((f"{n} {math.degrees(v):+.0f}°", q))
    combos = {
        "deep squat": {"hip_pitch": 110, "knee": 125, "ankle_pitch": 40},
        "stair step (left up)": {"left_hip_pitch": 100, "left_knee": 120, "left_ankle_pitch": 30},
        "sitting": {"hip_pitch": 90, "knee": 90},
        "legs out, toes out": {"hip_roll": 45, "hip_yaw": -45},
        "high knee, hip out": {"left_hip_pitch": 110, "left_knee": 125, "left_hip_roll": 45},
    }
    for label, extra in combos.items():
        q = base()
        for n in names:
            key = n.split("_", 1)[1] if n.startswith(("left_", "right_")) else n
            if n in extra:
                q[n] = math.radians(extra[n])
            elif key in extra:
                q[n] = math.radians(extra[key])
        out.append((label, q))
    return out


def limb(link: str) -> str:
    if link.startswith(("left_", "right_")):
        side = link.split("_", 1)[0]
        arm = any(k in link for k in ("shoulder", "arm", "forearm", "wrist", "gripper"))
        return f"{side} {'arm' if arm else 'leg'}"
    return "pelvis" if link == "pelvis" else "upper body"


def same_structure(a: str, b: str) -> bool:
    """Same limb, or a limb and the body it hangs from (legs from the pelvis, arms from the
    upper body), or the pelvis and the upper body. An arm against the pelvis (a hand at the
    hip with the waist turned) is self-collision, like an arm against a leg."""
    la, lb = limb(a), limb(b)
    if la == lb:
        return True
    pair = {la, lb}
    if pair == {"pelvis", "upper body"}:
        return True
    hangs = {"leg": "pelvis", "arm": "upper body"}
    for x, y in ((la, lb), (lb, la)):
        if x.endswith(("leg", "arm")) and hangs[x.split()[-1]] == y:
            return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()
    robot = yaml.safe_load((bu.DESC / "robot.yaml").read_text())
    model = mujoco.MjSpec.from_file(str(bu.URDF_OUT)).compile()
    data = mujoco.MjData(model)

    # MuJoCo's URDF loader merges the root link (pelvis) into the world body
    def body_name(b: int) -> str:
        return "pelvis" if b == 0 else model.body(b).name

    parent = {body_name(b): body_name(model.body_parentid[b]) for b in range(1, model.nbody)}
    per_link = shapes()
    worst: dict[tuple, tuple[float, str]] = {}
    for label, q in poses(model, robot):
        for n, v in q.items():
            lo, hi = model.jnt_range[model.joint(n).id]
            data.qpos[model.jnt_qposadr[model.joint(n).id]] = min(max(v, lo), hi)
        mujoco.mj_kinematics(model, data)
        placed = []
        for link, items in per_link.items():
            b = 0 if link == "pelvis" else model.body(link).id
            rot, pos = data.xmat[b].reshape(3, 3), data.xpos[b]
            for kind, name, s in items:
                placed.append((link, kind, name, place(s, rot, pos)))
        for i in range(len(placed)):
            la, ka, na, sa = placed[i]
            for j in range(i + 1, len(placed)):
                lb, kb, nb, sb = placed[j]
                if la == lb:
                    continue
                adjacent = parent.get(la) == lb or parent.get(lb) == la
                if (ka, kb) != ("motor", "motor") and adjacent:
                    continue
                bounds_a, bounds_b = sa.bounding_box(), sb.bounding_box()
                if any(
                    bounds_a[k] > bounds_b[k + 3] or bounds_b[k] > bounds_a[k + 3] for k in range(3)
                ):
                    continue
                vol = (sa ^ sb).volume()
                if vol > MAX_OVERLAP:
                    key = (f"{ka} {na}", f"{kb} {nb}", same_structure(la, lb))
                    if key not in worst or vol > worst[key][0]:
                        worst[key] = (vol, label)
                    if args.verbose:
                        print(f"{label:28} {key[0]} x {key[1]}: {vol * 1e6:.0f} cm³")
    errors = {k: v for k, v in worst.items() if k[2]}
    warnings = {k: v for k, v in worst.items() if not k[2]}
    for title, group in (("packaging errors", errors), ("self-collision between limbs", warnings)):
        print(f"{title}: {len(group)} pairs" + (" (worst pose each)" if group else ""))
        for (a, b, _), (vol, label) in sorted(group.items(), key=lambda kv: -kv[1][0]):
            print(f"  {a:30} x {b:30} {vol * 1e6:7.0f} cm³  at {label}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
