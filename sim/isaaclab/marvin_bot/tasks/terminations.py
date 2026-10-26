"""Termination terms for Marvin Bot's tasks."""

import torch


def base_below_ground(env, minimum_height: float, asset_name: str = "robot") -> torch.Tensor:
    """End the episode when the pelvis is less than ``minimum_height`` above the ground under it.

    The ground is the height scanner's hit nearest the pelvis when there is a scanner (stairs and
    pits), and z = 0 otherwise. Not the scan's mean: in front of a flight of stairs the mean
    includes the steps ahead and ended episodes of robots standing normally at the bottom.
    """
    root = env.scene[asset_name].data.root_pos_w.torch
    z = root[:, 2]
    if "height_scanner" in env.scene.sensors:
        hits = env.scene.sensors["height_scanner"].data.ray_hits_w.torch  # (N, R, 3)
        nearest = torch.linalg.norm(hits[..., :2] - root[:, None, :2], dim=-1).argmin(dim=1)
        ground = hits[torch.arange(len(z), device=z.device), nearest, 2]
        z = z - torch.nan_to_num(ground, nan=0.0, posinf=0.0, neginf=0.0)
    return z < minimum_height
