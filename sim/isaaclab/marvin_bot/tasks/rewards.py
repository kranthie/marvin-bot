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

    Only swinging feet earn it: each foot's term is weighted by tanh of its speed, so a planted
    foot earns nothing. (Weighting the height error by speed instead, as the Spot term does, gave
    a planted foot the full reward; Marvin Bot's first flat policy learned to stand and shuffle.)
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
    swinging = torch.tanh(tanh_mult * speed)
    reward = torch.mean(swinging * torch.exp(-torch.square(clearance - target_height) / std), dim=1)
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


_heat: dict[int, torch.Tensor] = {}
_rated: dict[int, torch.Tensor] = {}


def sustained_torque(
    env, asset_cfg: SceneEntityCfg, time_constant: float = 1.0, threshold: float = 0.8
) -> torch.Tensor:
    """Penalize joints that hold torque above their continuous rating.

    Each joint keeps a running average (``time_constant`` seconds) of (torque / rated torque)²,
    a stand-in for its winding heat; the penalty is the sum of how far those averages exceed
    ``threshold``. Short peaks cost nothing, holding a bent knee on a stair does. The real
    thermal time constant (minutes) is fitted on the bench; a short one keeps the signal inside
    a training episode.
    """
    from ..robot_cfg import joint_rated

    asset = env.scene[asset_cfg.name]
    ids = asset_cfg.joint_ids
    torque = asset.data.applied_torque.torch[:, ids]
    key = id(env)
    if key not in _rated:
        names = [
            asset.joint_names[i]
            for i in (ids if not isinstance(ids, slice) else range(len(asset.joint_names)))
        ]
        rated = joint_rated()
        _rated[key] = torch.tensor([rated[n] for n in names], device=torque.device)
    if key not in _heat or _heat[key].shape != torque.shape:
        _heat[key] = torch.zeros_like(torque)
    h = _heat[key]
    h[env.episode_length_buf == 0] = 0.0  # episodes that just started
    alpha = env.step_dt / time_constant
    h += alpha * (torch.square(torque / _rated[key]) - h)
    return torch.sum(torch.clamp(h - threshold, min=0.0), dim=1)


def forward_progress(env, command_name: str, asset_name: str = "robot") -> torch.Tensor:
    """Reward ground covered along the commanded direction, linearly, up to the command speed.

    The exponential tracking reward with a sharp width pays almost nothing for a slow climb, so
    at a stair the best-paying choice was to stand still at the bottom (rough v3 stopped in front
    of 10 cm stairs). This term pays a robot climbing at half the commanded speed half the reward.
    Zero for near-zero commands.
    """
    cmd = env.command_manager.get_command(command_name)[:, :2]
    speed = torch.linalg.norm(cmd, dim=1)
    direction = cmd / speed.clamp(min=1e-6)[:, None]
    vel = env.scene[asset_name].data.root_lin_vel_b.torch[:, :2]
    along = torch.sum(vel * direction, dim=1)
    # capped at the commanded speed: uncapped, v4 learned to lunge forward and fall
    progress = torch.clamp(along / speed.clamp(min=1e-6), min=0.0, max=1.0)
    return progress * (speed > 0.1)


def pelvis_height(
    env, asset_cfg: SceneEntityCfg, target: float, tolerance: float, asset_name: str = "robot"
) -> torch.Tensor:
    """Penalize a pelvis carried low: how far it sits below ``target - tolerance`` above the
    lower foot, in metres.

    Measured from the lower foot so it holds on stairs (the trailing foot going up, the leading
    one going down). Nothing else rewarded height, and rough v6 stood with its knees at 95 degrees
    and its pelvis 19 cm below the stance height, and climbed 8 cm low; flat v4 walked 2-3 cm low.
    ``target`` is the stance height of the pelvis above the foot frames; the tolerance leaves the
    knee room to flex in stance and on a step.
    """
    asset = env.scene[asset_name]
    feet = asset.data.body_pos_w.torch[:, asset_cfg.body_ids, 2]
    height = asset.data.root_pos_w.torch[:, 2] - feet.min(dim=1).values
    return torch.clamp(target - tolerance - height, min=0.0)


def stand_pose(env, asset_cfg: SceneEntityCfg, command_name: str) -> torch.Tensor:
    """Penalize leg joints away from the default stance while commanded to stand still (L1, rad)."""
    asset = env.scene[asset_cfg.name]
    q = asset.data.joint_pos.torch[:, asset_cfg.joint_ids]
    q0 = asset.data.default_joint_pos.torch[:, asset_cfg.joint_ids]
    still = torch.linalg.norm(env.command_manager.get_command(command_name), dim=1) < 0.1
    return torch.sum(torch.abs(q - q0), dim=1) * still
