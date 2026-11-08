"""Score a trained MarvinLite-Reach policy: how close the left palm gets to each target.

Runs the policy on many robots with the success termination off, so each target is held for its
full 4 s; records the palm's distance to the target at the end of each window, the centre of
mass's distance from the column's axis, and the joints' peak torque while holding as a share of
their limit.
Prints the error's spread, the share of targets reached within 5 cm, and where the misses are.

Usage (from an Isaac Lab 3.0 checkout, with this package installed; see README.md):
  uv run --no-sync python <repo>/sim/isaaclab/eval_reach.py \
      --checkpoint logs/rsl_rl/marvin_lite_reach/<run>/model_1499.pt
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation
from isaaclab_tasks.utils import resolve_task_config, setup_preset_cli

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--task", default="MarvinLite-Reach")
parser.add_argument("--checkpoint", type=Path, required=True)
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--windows", type=int, default=4, help="targets per robot")
add_launcher_args(parser)
args, overrides = setup_preset_cli(parser)

import gymnasium as gym  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlVecEnvWrapper,
    check_rsl_rl_version,
    create_rsl_rl_runner,
    handle_deprecated_rsl_rl_cfg,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import marvin_bot.tasks  # noqa: E402, F401  (registers the tasks)
from marvin_bot.tasks.reach_env_cfg import HAND  # noqa: E402


def main() -> None:
    env_cfg, agent_cfg = resolve_task_config(
        args.task, "rsl_rl_cfg_entry_point", play_mode=True, overrides=overrides
    )
    env_cfg.scene.num_envs = args.num_envs
    env_cfg.terminations.success = None
    env_cfg.rewards.success = None
    env_cfg.episode_length_s = 4.0 * args.windows + 1.0
    with launch_simulation(env_cfg, args):
        agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, check_rsl_rl_version())
        env = RslRlVecEnvWrapper(gym.make(args.task, cfg=env_cfg))
        runner = create_rsl_rl_runner(env, agent_cfg)
        runner.load(str(args.checkpoint))
        policy = runner.get_inference_policy(device=env.unwrapped.device)

        base = env.unwrapped
        robot = base.scene["robot"]
        cmd = base.command_manager.get_term("ee_pose")
        hand = robot.body_names.index(HAND)
        per_window = round(4.0 / base.step_dt)
        effort = robot.data.joint_effort_limits.torch[0]
        peak = torch.zeros(args.num_envs, len(effort), device=base.device)  # per robot
        errs, targets, com_max = [], [], torch.zeros(args.num_envs, device=base.device)
        trace = np.zeros(per_window)  # mean error by step into the window
        obs = env.get_observations()
        for k in range(per_window * args.windows):
            with torch.inference_mode():
                obs, _, _, _ = env.step(policy(obs))
            if k % per_window >= per_window - round(1.0 / base.step_dt):  # holding, last 1 s
                peak = torch.maximum(peak, robot.data.applied_torque.torch.abs())
            m = robot.data.body_mass.torch.to(base.device)
            c = (m.unsqueeze(-1) * robot.data.body_com_pos_w.torch).sum(1) / m.sum(1, True)
            com = torch.linalg.norm(c[:, :2] - robot.data.root_pos_w.torch[:, :2], dim=1)
            com_max = torch.maximum(com_max, com)
            palm = robot.data.body_link_pos_w.torch[:, hand]
            trace[k % per_window] += (
                float(torch.linalg.norm(palm - cmd.pose_command_w[:, :3], dim=1).mean())
                / args.windows
            )
            if k % per_window == per_window - 2:  # just before the next target
                goal_w = cmd.pose_command_w[:, :3]
                palm = robot.data.body_link_pos_w.torch[:, hand]
                errs.append(torch.linalg.norm(palm - goal_w, dim=1).cpu().numpy())
                targets.append(cmd.pose_command_b[:, :3].cpu().numpy())

        e = np.concatenate(errs) * 100
        t = np.concatenate(targets)
        print(f"\n{len(e)} targets: palm error at the end of each 4 s window (cm)")
        print(
            f"  median {np.median(e):.1f}, 90th percentile {np.percentile(e, 90):.1f}, "
            f"within 5 cm {np.mean(e < 5):.0%}, within 10 cm {np.mean(e < 10):.0%}"
        )
        print("  by target height (base frame):")
        for lo, hi in ((0.75, 1.0), (1.0, 1.2), (1.2, 1.45)):
            s = (t[:, 2] >= lo) & (t[:, 2] < hi)
            print(
                f"    z {lo:.2f}-{hi:.2f} m: median {np.median(e[s]):.1f} cm, "
                f"<5 cm {np.mean(e[s] < 5):.0%}"
            )
        print("  by distance in front:")
        for lo, hi in ((0.25, 0.4), (0.4, 0.5), (0.5, 0.6)):
            s = (t[:, 0] >= lo) & (t[:, 0] < hi)
            print(
                f"    x {lo:.2f}-{hi:.2f} m: median {np.median(e[s]):.1f} cm, "
                f"<5 cm {np.mean(e[s] < 5):.0%}"
            )
        print(
            "  mean error by time into the window: "
            + ", ".join(
                f"{(i + 1) * base.step_dt:.1f} s {trace[i] * 100:.1f} cm"
                for i in (5, 15, 30, 60, 90, per_window - 2)
            )
        )
        cm = com_max.cpu().numpy() * 1000
        print(
            f"centre of mass off the column's axis: worst {cm.max():.0f} mm, "
            f"median {np.median(cm):.0f} mm"
        )
        frac = (peak / effort).cpu().numpy()  # robots x joints
        p50, p95 = np.percentile(frac, 50, axis=0), np.percentile(frac, 95, axis=0)
        print(
            "peak torque or force while holding (each window's last second), share of the "
            "limit, over the robots: median, 95th percentile, robots above 90%"
        )
        for i in np.argsort(-p95)[:6]:
            print(
                f"  {robot.joint_names[i]:24} {p50[i]:4.0%} {p95[i]:4.0%} "
                f"{np.mean(frac[:, i] > 0.9):4.0%}"
            )


if __name__ == "__main__":
    main()
