"""Left-right mirror augmentation for rsl_rl PPO (``RslRlSymmetryCfg.data_augmentation_func``).

Every batch is doubled with its mirror image, so the policy learns the same gait on both legs.
Marvin Bot's joint axes are chosen so that a joint angle means the same motion on both sides
(description/skeleton.yaml: pitch axes are shared, roll and yaw axes are mirrored), so a mirrored
pose swaps the left and right joints with the same values. The centre joints whose axes are
vertical (waist yaw, neck yaw) change sign; neck pitch keeps it. sim/mujoco/check_symmetry.py
checks this against MuJoCo forward kinematics.
"""

import torch
from tensordict import TensorDict

FLIP_SIGN = {"waist_yaw", "neck_yaw"}
# per-term sign patterns for the mirror about the x-z plane
VECTOR_SIGNS = {
    "base_ang_vel": (-1.0, 1.0, -1.0),  # roll and yaw rates flip
    "projected_gravity": (1.0, -1.0, 1.0),
    "base_lin_vel": (1.0, -1.0, 1.0),
    "velocity_commands": (1.0, -1.0, -1.0),  # vx, vy, yaw rate
}


def mirror_name(name: str) -> str:
    if name.startswith("left_"):
        return "right_" + name[len("left_") :]
    if name.startswith("right_"):
        return "left_" + name[len("right_") :]
    return name


def joint_map(names: list[str]) -> tuple[list[int], list[float]]:
    perm = [names.index(mirror_name(n)) for n in names]
    sign = [-1.0 if n in FLIP_SIGN else 1.0 for n in names]
    return perm, sign


_cache: dict = {}


def _obs_map(env, group: str, device) -> tuple[torch.Tensor, torch.Tensor]:
    key = (group, str(device))
    if key in _cache:
        return _cache[key]
    base = env.unwrapped
    robot = base.scene["robot"]
    action_names = base.action_manager.get_term("joint_pos")._joint_names
    om = base.observation_manager
    perm, sign, start = [], [], 0
    for name, dims in zip(om.active_terms[group], om.group_obs_term_dim[group], strict=True):
        dim = int(dims[0])
        if name in VECTOR_SIGNS:
            p, s = list(range(dim)), list(VECTOR_SIGNS[name])
        elif name in ("joint_pos", "joint_vel"):
            p, s = joint_map(robot.joint_names)
        elif name == "actions":
            p, s = joint_map(action_names)
        elif name == "height_scan":
            # grid rays in "xy" ordering: rows of constant y, so mirroring flips the row order
            pattern = base.scene["height_scanner"].cfg.pattern_cfg
            nx = round(pattern.size[0] / pattern.resolution) + 1
            ny = dim // nx
            idx = torch.arange(dim).reshape(ny, nx).flip(0).flatten()
            p, s = idx.tolist(), [1.0] * dim
        else:
            raise KeyError(f"no mirror rule for observation term '{name}'")
        if len(p) != dim:
            raise ValueError(f"mirror rule for '{name}' covers {len(p)} of {dim} values")
        perm += [start + i for i in p]
        sign += s
        start += dim
    out = (torch.tensor(perm, device=device), torch.tensor(sign, device=device))
    _cache[key] = out
    return out


@torch.no_grad()
def compute_symmetric_states(
    env, obs: TensorDict | None = None, actions: torch.Tensor | None = None
):
    """Return the batch followed by its left-right mirror image."""
    obs_aug, actions_aug = None, None
    if obs is not None:
        n = obs.batch_size[0]
        obs_aug = obs.repeat(2)
        for group in obs.keys():
            perm, sign = _obs_map(env, group, obs[group].device)
            obs_aug[group][n:] = obs[group][:, perm] * sign
    if actions is not None:
        names = env.unwrapped.action_manager.get_term("joint_pos")._joint_names
        perm, sign = joint_map(names)
        perm = torch.tensor(perm, device=actions.device)
        sign = torch.tensor(sign, device=actions.device)
        actions_aug = torch.cat([actions, actions[:, perm] * sign])
    return obs_aug, actions_aug
