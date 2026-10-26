"""Observation terms for Marvin Bot's tasks."""

import torch
from isaaclab.envs import mdp
from isaaclab.managers import SceneEntityCfg

_bias: dict[int, torch.Tensor] = {}


def mapped_height_scan(
    env,
    sensor_cfg: SceneEntityCfg,
    offset: float,
    bias: float = 0.02,
    dropout: float = 0.1,
) -> torch.Tensor:
    """Height scan as the robot's terrain map would give it, not as the simulator knows it.

    On the robot, the height map comes from a terrain map that the perception stack (Isaac ROS
    on the AGX Orin) fuses from the head camera's estimated depth and odometry, so it also
    covers the ground under the feet, which the camera saw on the way in. Its errors are
    modelled here:

    - a height offset per episode, uniform in +-``bias`` m (map drift against the feet);
    - a fraction ``dropout`` of cells reading 0, the nominal ground level (holes in the map);
    - staleness, from the scanner's update period (``RayCasterCfg.update_period``).

    Cell noise comes from the observation term's ``noise``.
    """
    scan = mdp.height_scan(env, sensor_cfg, offset)
    key = id(env)
    if key not in _bias or _bias[key].shape[0] != scan.shape[0]:
        _bias[key] = torch.zeros(scan.shape[0], 1, device=scan.device)
    new = env.episode_length_buf == 0  # resample the drift for episodes that just started
    if new.any():
        _bias[key][new] = (torch.rand(int(new.sum()), 1, device=scan.device) * 2 - 1) * bias
    holes = torch.rand_like(scan) < dropout
    return torch.where(holes, torch.zeros_like(scan), scan + _bias[key])
