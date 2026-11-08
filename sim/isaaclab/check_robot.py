"""Load Marvin Bot in Isaac Lab and check it against the description files and MuJoCo.

Spawns MARVIN_BOT_CFG twice: a fixed-base copy in the air for a kinematics check and a free copy
on the floor for a standing check. Checks:

- joint names and order, position limits (robot.yaml), effort and velocity limits (the motor
  models, as in the URDF);
- total mass against the URDF;
- link positions for random joint angles against MuJoCo forward kinematics of the same URDF,
  which catches axis sign and ordering mistakes;
- the default stance holds for a few seconds with the stiff holding gains of
  sim/mujoco/check_standing.py (no policy balances it, so the training gains are too soft), with
  no joint torque clipped at its limit and none above 70% of its peak after 1 s.

Usage (from an Isaac Lab 3.0 checkout, with this package installed and the USD converted by
convert_urdf.py; see README.md):
  uv run --no-sync python <repo>/sim/isaaclab/check_robot.py                    # Newton
  uv run --no-sync python <repo>/sim/isaaclab/check_robot.py --backend physx
  uv run --no-sync python <repo>/sim/isaaclab/check_robot.py --variant lite

With --variant lite it checks MARVIN_LITE_CFG, whose base is fixed: the same spec, mass and
kinematics checks (limits read from the Lite URDF, which takes them from the description files),
and, in place of standing, that the stance holds with the training gains, the column within 5 mm
of its target and no joint above 70% of its peak, and that the column rises at its rated speed.
"""

import argparse
import math
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

PEAK_LIMIT = 0.70
FK_TOL_M = 0.003

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--backend", choices=["newton", "physx"], default="newton")
parser.add_argument("--seconds", type=float, default=5.0)
parser.add_argument("--fk-samples", type=int, default=8)
parser.add_argument("--variant", choices=["lite"])
add_launcher_args(parser)
args = parser.parse_args()

import isaaclab.sim as sim_utils  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from isaaclab.assets import Articulation, ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg, SimulationContext  # noqa: E402
from isaaclab.utils import configclass, instantiate, replace  # noqa: E402
from isaaclab.utils.math import quat_apply_inverse  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from marvin_bot.robot_cfg import (  # noqa: E402
    DESC,
    JOINT_ORDER,
    LITE_JOINT_ORDER,
    MARVIN_BOT_CFG,
    MARVIN_LITE_CFG,
    URDF_LITE_PATH,
    URDF_PATH,
    joint_limits,
)

LITE = args.variant == "lite"
ROOT_BODY = "lite_base" if LITE else "pelvis"

DT = 0.005  # the tasks step physics at 200 Hz
# holding gains (kp, kd) as in sim/mujoco/check_standing.py, by actuator group of MARVIN_BOT_CFG
HOLD_GAINS = {
    "hip_pitch": (400.0, 8.0),
    "hip_roll": (400.0, 8.0),
    "knee": (400.0, 8.0),
    "hip_yaw": (250.0, 5.0),
    "waist_yaw": (250.0, 5.0),
    "ankle_pitch": (250.0, 6.0),
    "ankle_roll": (250.0, 6.0),
    "shoulder_pitch": (250.0, 5.0),
    "shoulder_roll": (40.0, 1.5),  # an RS10P
}


def _holding(cfg: ArticulationCfg) -> ArticulationCfg:
    acts = {
        k: replace(a, stiffness=HOLD_GAINS[k][0], damping=HOLD_GAINS[k][1])
        if k in HOLD_GAINS
        else a
        for k, a in cfg.actuators.items()
    }
    return replace(cfg, actuators=acts)


def _fixed_base(cfg: ArticulationCfg) -> ArticulationCfg:
    spawn = replace(cfg.spawn, fix_root_link=True)
    init_state = replace(cfg.init_state, pos=(2.0, 0.0, 1.5))
    return replace(cfg, prim_path="{ENV_REGEX_NS}/MarvinFixed", spawn=spawn, init_state=init_state)


@configclass
class CheckSceneCfg(InteractiveSceneCfg):
    """A free robot on the floor and a fixed-base copy in the air beside it."""

    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    robot: ArticulationCfg = replace(_holding(MARVIN_BOT_CFG), prim_path="{ENV_REGEX_NS}/Marvin")
    fixed: ArticulationCfg = _fixed_base(MARVIN_BOT_CFG)


@configclass
class LiteSceneCfg(InteractiveSceneCfg):
    """Lite on its fixed base, with its training gains."""

    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
    robot: ArticulationCfg = replace(
        MARVIN_LITE_CFG,
        prim_path="{ENV_REGEX_NS}/MarvinLite",
        actuators={
            k: replace(a, dt=DT) if k == "column" else a
            for k, a in MARVIN_LITE_CFG.actuators.items()
        },
    )


def _no_self_collision(cfg: ArticulationCfg) -> ArticulationCfg:
    """For the kinematics check: random poses may put an arm inside the torso, and PhysX then
    pushes it out within the step."""
    props = [
        replace(p, enabled_self_collisions=False)
        if hasattr(p, "enabled_self_collisions")
        else replace(p, self_collision_enabled=False)
        for p in cfg.spawn.articulation_props
    ]
    spawn = replace(cfg.spawn, articulation_props=props)
    init_state = replace(cfg.init_state, pos=(2.0, 0.0, 0.0))
    return replace(cfg, prim_path="{ENV_REGEX_NS}/LiteFk", spawn=spawn, init_state=init_state)


@configclass
class LiteFkSceneCfg(LiteSceneCfg):
    fixed: ArticulationCfg = _no_self_collision(MARVIN_LITE_CFG)


def physics_cfg(name: str):
    if name == "physx":
        from isaaclab_physx.physics import PhysxCfg

        return PhysxCfg()
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    return NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=300, nconmax=80, integrator="implicitfast", cone="pyramidal"
        ),
        num_substeps=2,
    )


def mujoco_model() -> mujoco.MjModel:
    """The URDF in MuJoCo with a free base (the loader would weld the root to the world)."""
    spec = mujoco.MjSpec.from_string((URDF_LITE_PATH if LITE else URDF_PATH).read_text())
    spec.body(ROOT_BODY).add_freejoint(name="root")
    return spec.compile()


def urdf_limits() -> dict[str, tuple[float, float, float, float]]:
    """(lower, upper, effort, velocity) per joint from the Lite URDF's <limit> elements."""
    import xml.etree.ElementTree as ET

    out = {}
    for j in ET.fromstring(URDF_LITE_PATH.read_text()).iter("joint"):
        lim = j.find("limit")
        if lim is not None and j.get("type") != "fixed":
            out[j.get("name")] = tuple(
                float(lim.get(k, "nan")) for k in ("lower", "upper", "effort", "velocity")
            )
    return out


def mujoco_fk(qs: dict[str, float]) -> dict[str, np.ndarray]:
    """Link origins in the pelvis frame from MuJoCo."""
    model = mujoco_model()
    data = mujoco.MjData(model)
    for name, q in qs.items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = q
    mujoco.mj_kinematics(model, data)
    base = model.body(ROOT_BODY).id
    rot = data.xmat[base].reshape(3, 3)
    return {
        model.body(b).name: rot.T @ (data.xpos[b] - data.xpos[base]) for b in range(1, model.nbody)
    }


def check_spec_lite(robot: Articulation, failures: list[str]) -> None:
    names = robot.joint_names
    if tuple(names) != LITE_JOINT_ORDER:
        failures.append(f"joint order {names} != {list(LITE_JOINT_ORDER)}")
        return
    want = urdf_limits()
    limits = robot.data.joint_pos_limits.torch[0].cpu().numpy()
    effort = robot.data.joint_effort_limits.torch[0].cpu().numpy()
    vel = robot.data.joint_vel_limits.torch[0].cpu().numpy()
    print(f"{'joint':24} {'range':>18} {'effort':>7} {'vel':>6}")
    for i, name in enumerate(names):
        lo, hi, eff, v = want[name]
        print(
            f"{name:24} [{limits[i, 0]:7.3f}, {limits[i, 1]:7.3f}] {effort[i]:7.1f} {vel[i]:6.2f}"
        )
        if not name.startswith("wheel_") and not np.allclose(limits[i], [lo, hi], atol=1e-3):
            failures.append(f"{name}: range {limits[i].tolist()} != URDF {[lo, hi]}")
        if not np.isclose(effort[i], eff, rtol=1e-3):
            failures.append(f"{name}: effort limit {effort[i]:.2f} != URDF {eff:.2f}")
        if not np.isclose(vel[i], v, rtol=1e-3):
            failures.append(f"{name}: velocity limit {vel[i]:.3f} != URDF {v:.3f}")


def check_hold_lite(sim, robot: Articulation, failures: list[str]) -> None:
    q0 = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q0)
    robot.actuators.target_command.set_position_index(value=q0)
    effort = robot.data.joint_effort_limits.torch[0]
    col = robot.joint_names.index("column_lift")
    steps, settle = int(args.seconds / DT), int(1.0 / DT)
    peak = torch.zeros_like(effort)
    for k in range(steps):
        robot.actuators.target_command.set_position_index(value=q0)
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
        if k >= settle:
            peak = torch.maximum(peak, robot.data.applied_torque.torch[0].abs())
    q = robot.data.joint_pos.torch[0]
    sag = float(q0[0, col] - q[col])
    err = (q - q0[0]).abs()
    err[col] = 0.0
    frac = (peak / effort).cpu().numpy()
    print(f"\nstance, {args.seconds:.0f} s, training gains")
    print(f"column {float(q[col]):.4f} m for {float(q0[0, col]):.4f} m (sag {sag * 1000:.1f} mm)")
    print(f"worst joint error {math.degrees(float(err.max())):.1f} deg")
    for i in np.argsort(-frac)[:6]:
        print(f"  {robot.joint_names[i]:24} peak {float(peak[i]):7.2f}  {frac[i]:4.0%}")
    if abs(sag) > 0.005:
        failures.append(f"column off its target by {sag * 1000:.1f} mm")
    # the column asked to rise 0.1 m: it moves at its rated speed, not the drive's
    q1 = q0.clone()
    q1[0, col] += 0.1
    z0 = float(robot.data.joint_pos.torch[0, col])
    for _ in range(int(1.0 / DT)):
        robot.actuators.target_command.set_position_index(value=q1)
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
    rise = float(robot.data.joint_pos.torch[0, col]) - z0
    print(f"column asked up 100 mm: {rise * 1000:.0f} mm in 1 s (rated 38 mm/s)")
    if not 0.030 < rise < 0.045:
        failures.append(f"column rose {rise * 1000:.0f} mm in 1 s, not about 38")
    if frac.max() > PEAK_LIMIT:
        failures.append("a joint is over 70% of its peak")


def check_spec(robot: Articulation, failures: list[str]) -> None:
    robot_yaml = yaml.safe_load((DESC / "robot.yaml").read_text())
    ranges = {}
    for j in robot_yaml["joints"]:
        for side in ("left", "right") if "{side}" in j["name"] else (None,):
            ranges[j["name"].replace("{side}", side) if side else j["name"]] = j["range"]
    names = robot.joint_names
    if tuple(names) != JOINT_ORDER:
        failures.append(f"joint order {names} != robot.yaml {list(JOINT_ORDER)}")
        return
    want = joint_limits()
    limits = robot.data.joint_pos_limits.torch[0].cpu().numpy()
    effort = robot.data.joint_effort_limits.torch[0].cpu().numpy()
    vel = robot.data.joint_vel_limits.torch[0].cpu().numpy()
    print(f"{'joint':24} {'range (deg)':>16} {'effort':>7} {'vel':>6}")
    for i, name in enumerate(names):
        lo, hi = np.degrees(limits[i])
        print(f"{name:24} [{lo:6.1f}, {hi:6.1f}] {effort[i]:7.1f} {vel[i]:6.1f}")
        if not np.allclose([lo, hi], ranges[name], atol=0.1):
            failures.append(f"{name}: range {[lo, hi]} != robot.yaml {ranges[name]}")
        if not np.isclose(effort[i], want[name][0], rtol=1e-3):
            failures.append(f"{name}: effort limit {effort[i]:.2f} != {want[name][0]:.2f}")
        if not np.isclose(vel[i], want[name][1], rtol=1e-3):
            failures.append(f"{name}: velocity limit {vel[i]:.2f} != {want[name][1]:.2f}")


def check_fk(sim, robot: Articulation, failures: list[str]) -> None:
    rng = np.random.default_rng(0)
    limits = robot.data.joint_pos_limits.torch[0].cpu().numpy()
    worst = 0.0
    for _ in range(args.fk_samples):
        q = rng.uniform(limits[:, 0], limits[:, 1]) * 0.9
        qt = torch.tensor(q, dtype=torch.float32, device=sim.device).unsqueeze(0)
        robot.write_joint_position_to_sim_index(position=qt)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(qt))
        robot.actuators.target_command.set_position_index(value=qt)
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
        root_pos = robot.data.root_link_pos_w.torch[0]
        root_quat = robot.data.root_link_quat_w.torch[0]
        body_pos = robot.data.body_link_pos_w.torch[0]
        rel = quat_apply_inverse(root_quat.expand(len(body_pos), 4), body_pos - root_pos)
        ref = mujoco_fk(dict(zip(robot.joint_names, q, strict=True)))
        for b, name in enumerate(robot.body_names):
            if name not in ref:
                continue
            err = float(np.linalg.norm(rel[b].cpu().numpy() - ref[name]))
            worst = max(worst, err)
            if err > FK_TOL_M:
                failures.append(f"FK: {name} off by {err * 1000:.1f} mm")
    print(
        f"\nlink positions vs MuJoCo, {args.fk_samples} random poses: worst {worst * 1000:.2f} mm"
    )


def check_standing(sim, robot: Articulation, failures: list[str]) -> None:
    q0 = robot.data.default_joint_pos.torch.clone()
    robot.write_joint_position_to_sim_index(position=q0)
    robot.actuators.target_command.set_position_index(value=q0)
    h0 = float(robot.data.root_link_pos_w.torch[0, 2])
    effort = robot.data.joint_effort_limits.torch[0]
    steps, settle = int(args.seconds / DT), int(1.0 / DT)
    peak = torch.zeros_like(effort)
    clipped = torch.zeros_like(effort, dtype=torch.bool)
    for k in range(steps):
        robot.actuators.target_command.set_position_index(value=q0)
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
        if k >= settle:
            applied = robot.data.applied_torque.torch[0].abs()
            peak = torch.maximum(peak, applied)
            clipped |= robot.data.computed_torque.torch[0].abs() > effort * 0.999
    h1 = float(robot.data.root_link_pos_w.torch[0, 2])
    up = float(-robot.data.projected_gravity_b.torch[0, 2])
    tilt = math.degrees(math.acos(min(1.0, up)))
    frac = (peak / effort).cpu().numpy()
    print(f"\ndefault stance, {args.seconds:.0f} s, holding gains")
    print(f"pelvis height start {h0:.3f} m, end {h1:.3f} m, tilt {tilt:.1f} deg")
    for i in np.argsort(-frac)[:6]:
        print(f"  {robot.joint_names[i]:24} peak {float(peak[i]):6.2f} N·m  {frac[i]:4.0%}")
    if h1 < 0.7 * h0 or tilt > 30:
        failures.append("robot fell")
    if frac.max() > PEAK_LIMIT:
        failures.append("a joint is over 70% of its peak torque")
    if bool(clipped.any()):
        names = [n for n, c in zip(robot.joint_names, clipped.tolist(), strict=True) if c]
        failures.append(f"torque clipped at the effort limit: {names}")


def main() -> None:
    sim_cfg = SimulationCfg(dt=DT, device=args.device, physics=physics_cfg(args.backend))
    sim_cfg.use_newton_actuators = False  # the delayed DC motors run in the Python actuator path
    with launch_simulation(sim_cfg, args):
        sim = SimulationContext(sim_cfg)
        scene_cfg = LiteFkSceneCfg if LITE else CheckSceneCfg
        scene = instantiate(scene_cfg(num_envs=1, env_spacing=4.0))
        sim.reset()
        scene.update(DT)
        floating = scene["robot"]

        failures: list[str] = []
        mass = float(floating.data.body_mass.torch[0].sum())
        urdf_mass = mujoco_model().body_subtreemass[1]
        print(f"physics: {args.backend}, total mass {mass:.3f} kg (URDF {urdf_mass:.3f} kg)\n")
        if abs(mass - urdf_mass) > 0.01:
            failures.append(f"mass {mass:.3f} kg != URDF {urdf_mass:.3f} kg")
        if LITE:
            check_spec_lite(floating, failures)
            check_hold_lite(sim, floating, failures)
            check_fk(sim, scene["fixed"], failures)
        else:
            check_spec(floating, failures)
            check_standing(sim, floating, failures)
            check_fk(sim, scene["fixed"], failures)

        if failures:
            print("\nFAIL:\n  " + "\n  ".join(failures[:20]))
            sys.exit(1)
        print("\nPASS")


if __name__ == "__main__":
    main()
