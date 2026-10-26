"""Curriculum terms for Marvin Bot's tasks."""

import torch


def forward_speed_levels(
    env,
    env_ids,
    reward_term_name: str = "track_lin_vel_xy_exp",
    max_speed: float = 1.0,
    step: float = 0.05,
    threshold: float = 0.75,
) -> torch.Tensor:
    """Raise the top forward-speed command once the robots track the current range.

    Once per episode length, if the resetting robots earned more than ``threshold`` of the
    tracking reward available, the upper end of ``lin_vel_x`` grows by ``step`` up to
    ``max_speed``. Returns the current top speed for logging.
    """
    cfg = env.command_manager.get_term("base_velocity").cfg
    lo, hi = cfg.ranges.lin_vel_x
    if env.common_step_counter % env.max_episode_length == 0:
        weight = env.reward_manager.get_term_cfg(reward_term_name).weight
        earned = env.reward_manager._episode_sums[reward_term_name][env_ids]
        if torch.mean(earned) / env.max_episode_length_s > threshold * weight:
            cfg.ranges.lin_vel_x = (lo, min(hi + step, max_speed))
    return torch.tensor(cfg.ranges.lin_vel_x[1])


def terrain_levels_tracking(
    env,
    env_ids,
    reward_term_name: str = "track_lin_vel_xy_exp",
    up: float = 0.5,
    down: float = 0.4,
    min_distance: float = 1.0,
) -> torch.Tensor:
    """Terrain curriculum judged by command tracking and by the ground covered.

    Isaac Lab's ``terrain_levels_vel`` promotes robots that walk half a tile from their spawn point;
    at slow commanded speeds few get that far even when they walk well (seen on an earlier prototype).
    Tracking alone isn't enough either: on a stair tile a robot can follow its commands on the
    central platform and never take a stair. So a robot moves up if it lasted the episode, earned
    more than ``up`` of the velocity-tracking reward and ended at least ``min_distance`` from its
    spawn point (past the stairs), and down if it fell or earned less than ``down``. Returns the
    mean terrain level.
    """
    terrain = env.scene.terrain
    if isinstance(env_ids, slice):
        env_ids = torch.arange(env.num_envs, device=env.device)[env_ids]
    weight = env.reward_manager.get_term_cfg(reward_term_name).weight
    earned = env.reward_manager._episode_sums[reward_term_name][env_ids] / env.max_episode_length_s
    survived = env.termination_manager.time_outs[env_ids]
    robot = env.scene["robot"]
    moved = robot.data.root_pos_w.torch[env_ids, :2] - env.scene.env_origins[env_ids, :2]
    far = torch.linalg.norm(moved, dim=1) > min_distance
    move_up = survived & (earned > up * weight) & far
    move_down = (~survived | (earned < down * weight)) & ~move_up
    terrain.update_env_origins(env_ids, move_up, move_down)
    return torch.mean(terrain.terrain_levels.float())
