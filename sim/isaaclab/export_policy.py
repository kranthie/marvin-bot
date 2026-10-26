"""Export a trained rsl_rl policy to ONNX with the metadata the MuJoCo check and the robot need.

Writes policy.onnx and policy.json to --out (default: an exported/ folder next to the
checkpoint). The JSON records the joint order the policy was trained with (robot.yaml on
every backend), the default pose, the action scale, the observation layout, the control
period and, for rough-ground policies, the height map layout, so sim/mujoco/check_policy.py
can rebuild the same observations.

Usage (from an Isaac Lab 3.0 checkout, with this package installed; see README.md):
  uv run --no-sync python <repo>/sim/isaaclab/export_policy.py --checkpoint logs/.../model_1499.pt
  ... physics=isaacsim_physx   # for a policy trained on PhysX
"""

import argparse
import json
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--task", default="MarvinBot-Velocity-Flat")
parser.add_argument("--checkpoint", type=Path, required=True)
parser.add_argument("--out", type=Path, default=None)
add_launcher_args(parser)
args, overrides = setup_preset_cli(parser)

import gymnasium as gym  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlVecEnvWrapper,
    check_rsl_rl_version,
    create_rsl_rl_runner,
    handle_deprecated_rsl_rl_cfg,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import marvin_bot.tasks  # noqa: E402, F401  (registers the tasks)


def main() -> None:
    env_cfg, agent_cfg = resolve_task_config(
        args.task, "rsl_rl_cfg_entry_point", play_mode=True, overrides=overrides
    )
    env_cfg.scene.num_envs = 1
    out = args.out or args.checkpoint.resolve().parent / "exported"
    with launch_simulation(env_cfg, args):
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, check_rsl_rl_version())
        env = RslRlVecEnvWrapper(gym.make(args.task, cfg=env_cfg))
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(str(args.checkpoint))
        runner.export_policy_to_onnx(path=str(out), filename="policy.onnx")

        base = env.unwrapped
        robot = base.scene["robot"]
        action = base.action_manager.get_term("joint_pos")
        obs = base.observation_manager
        policy_terms = obs.active_terms["policy"]
        policy_dims = obs.group_obs_term_dim["policy"]
        meta = {
            "task": args.task,
            "physics": type(env_cfg.sim.physics).__name__,
            "control_dt": base.step_dt,
            "joint_names": robot.joint_names,
            "default_joint_pos": robot.data.default_joint_pos.torch[0].tolist(),
            "action_joint_names": action._joint_names,
            "action_scale": float(action.cfg.scale),
            "observations": [
                {"name": n, "dim": int(d[0])}
                for n, d in zip(policy_terms, policy_dims, strict=True)
            ],
        }
        if "height_scan" in policy_terms:
            # the terrain map layout, so the check (and the robot) can build the same input
            scanner = base.scene["height_scanner"].cfg
            term = obs._group_obs_term_cfgs["policy"][policy_terms.index("height_scan")]
            meta["height_scan"] = {
                "size": list(scanner.pattern_cfg.size),
                "resolution": scanner.pattern_cfg.resolution,
                "ordering": scanner.pattern_cfg.ordering,
                "center": list(scanner.offset.pos[:2]),
                "alignment": scanner.ray_alignment,
                "offset": term.params["offset"],
                "update_period": scanner.update_period,
                "clip": list(term.clip),
                "note": "value = pelvis height - ground height - offset, rows of constant y",
            }
        (out / "policy.json").write_text(json.dumps(meta, indent=2) + "\n")
        print(f"wrote {out / 'policy.onnx'} and policy.json")
        env.close()


if __name__ == "__main__":
    main()
