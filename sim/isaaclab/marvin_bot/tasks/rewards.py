"""Reward terms for Marvin Bot's tasks."""

import torch
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import quat_apply_inverse

SOLE_BELOW_FOOT_FRAME = (
    0.060  # m, foot link origin (ankle axis) to the sole (description/skeleton.yaml)
)


def foot_clearance(
    env,
    asset_cfg: SceneEntityCfg,
    command_name: str,
    target_height: float,
    std: float,
    tanh_mult: float = 2.0,
    reach: float = 0.16,
) -> torch.Tensor:
    """Reward moving feet for lifting their soles ``target_height`` above the ground around them.

    After Isaac Lab's Spot clearance term. On rough ground the ground is the highest of the pelvis
    height scanner's hits within ``reach`` of each foot, so a swinging foot is pushed over the step
    edge its toe (15 cm ahead of the ankle) is about to meet. On an earlier prototype, measured from the
    ground under the ankle, the policies lifted their feet 3.5 cm, caught 3 cm steps with their
    toes, and learned to stop at stairs instead. On flat ground z = 0. Zero for near-zero commands.
    """
    asset = env.scene[asset_cfg.name]
    feet = asset.data.body_pos_w.torch[:, asset_cfg.body_ids]  # (N, F, 3)
    ground = torch.zeros_like(feet[..., 2])
    if "height_scanner" in env.scene.sensors:
        hits = env.scene.sensors["height_scanner"].data.ray_hits_w.torch  # (N, R, 3)
        z = torch.nan_to_num(hits[..., 2], nan=-1e3, posinf=-1e3, neginf=-1e3)  # (N, R)
        d = torch.cdist(feet[..., :2], hits[..., :2])  # (N, F, R)
        near = torch.where(d < reach, z[:, None, :], torch.full_like(d, -1e3)).amax(dim=2)
        nearest = torch.gather(z, 1, d.argmin(dim=2))  # if no ray is within reach
        ground = torch.where(near > -1e2, near, nearest).clamp(min=-1e2)
    clearance = feet[..., 2] - SOLE_BELOW_FOOT_FRAME - ground
    speed = torch.linalg.norm(asset.data.body_lin_vel_w.torch[:, asset_cfg.body_ids, :2], dim=2)
    error = torch.square(clearance - target_height) * torch.tanh(tanh_mult * speed)
    reward = torch.exp(-torch.sum(error, dim=1) / std)
    return reward * (torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) > 0.1)


def feet_flat(env, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize tilted feet in stance: the squared sine of each touching sole's tilt, summed.

    Without it the symmetric gait stood on the edges of its feet (about 11 degrees of tilt in
    stance in the MuJoCo check), which made pivoting on the spot fragile.
    """
    contact = env.scene.sensors[sensor_cfg.name].data.current_contact_time.torch[
        :, sensor_cfg.body_ids
    ]
    quat = env.scene[asset_cfg.name].data.body_quat_w.torch[:, asset_cfg.body_ids]  # (N, F, 4)
    down = torch.tensor([0.0, 0.0, -1.0], device=quat.device).expand(quat.shape[:-1] + (3,))
    g = quat_apply_inverse(quat.reshape(-1, 4), down.reshape(-1, 3)).reshape(down.shape)
    tilt = torch.sum(torch.square(g[..., :2]), dim=-1)
    return torch.sum(tilt * (contact > 0.0), dim=1)
