"""Stand the robot up in MuJoCo and report balance and joint torques against the actuators.

Loads description/urdf/marvin_bot.urdf (run tools/build_urdf.py first), adds a floor, a free
base and a position servo per joint, force-limited to the joint's peak torque, then holds a
pose for a few seconds. Passes if the robot stays up, every joint's average holding torque
after settling is within its rated (continuous) torque, and no joint peaks above 70% of its
peak torque (the phase 1 pass check in docs/plan.md).

Rated torques come from actuators.yaml; the ankle joints get the pair's combined rating times
the linkage ratios in skeleton.yaml, as in the URDF.

Usage:
  python sim/mujoco/check_standing.py                  # crouched stance
  python sim/mujoco/check_standing.py --pose straight
  python sim/mujoco/check_standing.py --render out.png

Needs: pip install -e ".[sim]"
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / "description"
URDF = DESC / "urdf" / "marvin_bot.urdf"
PEAK_LIMIT = 0.70

# Position-servo gains (kp N·m/rad, kd N·m·s/rad) and reflected rotor inertia (kg·m²) by
# motor model. This check holds a pose with no balance controller, so the leg gains are stiffer
# than a learned policy would use (comparable robots train with kp 100-150 on 120 N·m joints);
# softer knees sag, the hips drift back and the robot tips over its heels. Rotor inertias are
# placeholders until the phase 2 bench test.
GAINS = {
    "RS04": (400.0, 8.0, 0.03),
    "RS03": (250.0, 5.0, 0.02),
    "RS06": (40.0, 1.5, 0.01),
    "RS00": (20.0, 1.0, 0.005),
    "STS3250": (10.0, 0.5, 0.002),
}
# Joint space, for the two-motor ankle. Standing still without a balance controller needs the
# two ankles' combined stiffness above m*g*h (~210 N·m/rad for 36 kg at 0.6 m), so this check
# uses stiffer ankles than a learned policy would (comparable robots train with ~40).
ANKLE_GAINS = (250.0, 6.0, 0.02)

# Poses in degrees, robot.yaml's convention (left side; the right side uses the same values).
POSES = {
    "crouch": {
        "hip_pitch": 20,
        "knee": 40,
        "ankle_pitch": 20,
        "shoulder_roll": 8,
        "elbow": 20,
    },
    "straight": {"shoulder_roll": 8},
}


def side_names(names: list[str]) -> dict[str, str]:
    """Map each URDF joint name to its side-less key (left_knee -> knee)."""
    return {n: n.split("_", 1)[1] if n.startswith(("left_", "right_")) else n for n in names}


def joint_models() -> tuple[dict, dict]:
    """Motor model driving each joint (direct), and the ankle pair's model."""
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    direct, pair = {}, {}
    for m in robot["motors"]:
        for side in ("left", "right") if "{side}" in m["name"] else (None,):
            drives = [d.replace("{side}", side) if side else d for d in m["drives"]]
            if len(drives) == 1:
                direct[drives[0]] = m["model"]
            else:
                for d in drives:
                    pair[d] = m["model"]
    return direct, pair


def rated_torques(direct: dict, pair: dict) -> dict[str, float]:
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    ratio = yaml.safe_load((DESC / "skeleton.yaml").read_text())["ankle_linkage"]
    rated = {j: acts[m]["rated_torque"] for j, m in direct.items()}
    for j, m in pair.items():
        r = ratio["pitch_ratio"] if j.endswith("pitch") else ratio["roll_ratio"]
        rated[j] = 2 * acts[m]["rated_torque"] * r
    return rated


def build() -> tuple[mujoco.MjSpec, dict]:
    # keep the URDF's visual geoms (MuJoCo drops them by default) for --render
    urdf = URDF.read_text().replace(
        '<robot name="marvin_bot">',
        '<robot name="marvin_bot"><mujoco><compiler discardvisual="false"/></mujoco>',
        1,
    )
    spec = mujoco.MjSpec.from_string(urdf)
    spec.modelname = "marvin_bot"
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.compiler.autolimits = True
    world = spec.worldbody
    world.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[3, 3, 0.05],
        friction=[1.0, 0.005, 1e-4],
    )
    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    pelvis = spec.body("pelvis")
    pelvis.add_freejoint(name="root")
    pelvis.add_site(name="imu", pos=[0, 0, 0.06])
    for g in spec.geoms:
        if g.contype and g.name != "floor":
            g.group = 3  # collision boxes hidden by default
    direct, pair = joint_models()
    for j in spec.joints:
        if j.name == "root":
            continue
        kp, kd, armature = GAINS[direct[j.name]] if j.name in direct else ANKLE_GAINS
        j.armature = armature
        act = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=kp, kv=kd)
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = j.actfrcrange if j.actfrclimited else [-1e3, 1e3]
        act.ctrlrange = j.range
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec, rated_torques(direct, pair)


def pose(model: mujoco.MjModel, name: str) -> np.ndarray:
    q = np.zeros(model.nu)
    keys = side_names([model.actuator(i).name for i in range(model.nu)])
    for i in range(model.nu):
        q[i] = np.radians(POSES[name].get(keys[model.actuator(i).name], 0.0))
    return q


def place_on_floor(model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray) -> None:
    for i in range(model.nu):
        data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]] = q[i]
    data.qpos[2] = 1.0
    mujoco.mj_forward(model, data)
    lowest = min(
        (data.geom_xpos[g] - np.abs(data.geom_xmat[g].reshape(3, 3)) @ model.geom_size[g])[2]
        for g in range(model.ngeom)
        if model.geom_bodyid[g] != 0 and model.geom_contype[g]
    )
    data.qpos[2] -= lowest
    mujoco.mj_forward(model, data)


def support_margin(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Distance from the CoM's ground projection to the edge of the feet's bounding box."""
    corners = []
    for g in range(model.ngeom):
        body = model.body(model.geom_bodyid[g]).name
        if body.endswith("foot_link") and model.geom_contype[g]:
            r, half = data.geom_xmat[g].reshape(3, 3), model.geom_size[g]
            for sx in (-1, 1):
                for sy in (-1, 1):
                    corners.append((data.geom_xpos[g] + r @ (half * [sx, sy, -1]))[:2])
    c = np.array(corners)
    com = data.subtree_com[model.body("pelvis").id][:2]
    lo, hi = c.min(0), c.max(0)
    return float(min(com[0] - lo[0], hi[0] - com[0], com[1] - lo[1], hi[1] - com[1]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pose", choices=sorted(POSES), default="crouch")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--render", type=Path, help="save a PNG of the settled pose")
    args = ap.parse_args()

    spec, rated = build()
    model = spec.compile()
    data = mujoco.MjData(model)
    q = pose(model, args.pose)
    place_on_floor(model, data, q)
    data.ctrl[:] = q
    mass = model.body_subtreemass[model.body("pelvis").id]
    h0 = data.qpos[2]
    margin = support_margin(model, data)

    steps = int(args.seconds / model.opt.timestep)
    settle = int(1.0 / model.opt.timestep)
    peak = np.zeros(model.nu)
    hold = np.zeros(model.nu)
    for k in range(steps):
        mujoco.mj_step(model, data)
        if k >= settle:
            f = np.abs(data.actuator_force)
            peak = np.maximum(peak, f)
            hold += f
    hold /= steps - settle
    up = data.xmat[model.body("pelvis").id].reshape(3, 3)[2, 2]
    fell = data.qpos[2] < 0.7 * h0 or up < np.cos(np.radians(30))

    print(f"pose {args.pose}: total mass {mass:.2f} kg, pelvis {h0:.3f} m -> {data.qpos[2]:.3f} m")
    print(f"CoM margin inside the feet's bounding box: {margin * 1000:.0f} mm")
    print(f"pelvis tilt at end: {np.degrees(np.arccos(min(1.0, up))):.1f} deg\n")
    print(f"{'joint':24} {'hold':>6} {'rated':>6} {'peak':>6} {'max':>6} {'peak %':>6}")
    worst_hold, worst_peak = 0.0, 0.0
    for i in range(model.nu):
        name = model.actuator(i).name
        if name.startswith("right_"):
            continue  # symmetric pose: the left side stands for both
        cap = model.actuator_forcerange[i, 1]
        frac = peak[i] / cap
        over = hold[i] > rated[name] or frac > PEAK_LIMIT
        worst_hold = max(worst_hold, hold[i] / rated[name])
        worst_peak = max(worst_peak, frac)
        flag = "  <--" if over else ""
        cols = f"{hold[i]:6.2f} {rated[name]:6.1f} {peak[i]:6.2f} {cap:6.1f} {frac:6.0%}"
        print(f"{name:24} {cols}{flag}")
    print(f"\nworst holding torque {worst_hold:.0%} of rated, worst peak {worst_peak:.0%} of peak")

    if args.render:
        renderer = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.6]
        cam.distance, cam.azimuth, cam.elevation = 2.2, 210, -10
        renderer.update_scene(data, cam)
        from PIL import Image

        Image.fromarray(renderer.render()).save(args.render)
        print(f"saved {args.render}")

    if fell:
        print("FAIL: robot fell")
        return 1
    if worst_hold > 1.0 or worst_peak > PEAK_LIMIT:
        print("FAIL: a joint is over its rated or peak limit")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
