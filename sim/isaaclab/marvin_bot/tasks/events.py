"""Event terms for Marvin Bot's tasks."""

import torch
from isaaclab.managers import SceneEntityCfg


def randomize_joint_targets(
    env, env_ids: torch.Tensor, asset_cfg: SceneEntityCfg, fraction: float = 0.5
) -> None:
    """Send the joints in ``asset_cfg`` to random position targets.

    Targets are drawn uniformly from the soft joint limits and pulled ``fraction`` of the way from
    the default pose, so the servos move the joints there at their own speed and torque limits.
    """
    asset = env.scene[asset_cfg.name]
    ids = asset_cfg.joint_ids
    limits = asset.data.soft_joint_pos_limits.torch[env_ids][:, ids]
    default = asset.data.default_joint_pos.torch[env_ids][:, ids]
    sample = limits[..., 0] + torch.rand_like(default) * (limits[..., 1] - limits[..., 0])
    target = default + fraction * (sample - default)
    asset.actuators.target_command.set_position_index(value=target, joint_ids=ids, env_ids=env_ids)
