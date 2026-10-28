"""Compare a ProtoMotions tracking rollout with its reference motions, clip by clip.

ProtoMotions' evaluation counts a clip as tracked when the torso never strays more than
25 cm in height from the reference. A policy that shuffles on the spot while the reference
walks off passes that test, so this also prints how far the robot went against the reference
(travel, path length in the plane), the joint-angle error, and the body-position error in the
pelvis frame (heading-aligned, so a slow turn is not counted as error): averaged over all bodies,
and for the hands and feet alone, since the all-body average dilutes a limb that misses (an arm
left at the side while the reference raises it barely moves the average).

The rollout is the predicted_motion_lib_epoch_*.pt that training writes under results/, or
the one a --full-eval inference run writes; the reference is the packaged motion file trained on.

Usage (in the ProtoMotions environment, from the checkout, which has Marvin Bot's MJCF):
  python sim/motion/check_tracking.py <predicted_motion_lib.pt> <motions.pt>
"""

import argparse
import math
import re
from pathlib import Path

import torch


def yaw(q: torch.Tensor) -> torch.Tensor:
    x, y, z, w = q.unbind(-1)  # xyzw
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def local(gts: torch.Tensor, heading: torch.Tensor) -> torch.Tensor:
    """Body positions relative to the pelvis, rotated so the pelvis faces +x."""
    d = gts - gts[:, :1]
    c, s = torch.cos(-heading)[:, None], torch.sin(-heading)[:, None]
    return torch.stack(
        [c * d[..., 0] - s * d[..., 1], s * d[..., 0] + c * d[..., 1], d[..., 2]], -1
    )


def clip(lib: dict, i: int) -> tuple[dict, float]:
    s, n = int(lib["length_starts"][i]), int(lib["motion_num_frames"][i])
    return {k: lib[k][s : s + n] for k in ("gts", "grs", "dps")}, float(lib["motion_dt"][i])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("rollout")
    ap.add_argument("reference")
    args = ap.parse_args()
    pred = torch.load(args.rollout, weights_only=False, map_location="cpu")
    ref = torch.load(args.reference, weights_only=False, map_location="cpu")
    mjcf = Path("protomotions/data/assets/mjcf/marvin_bot.xml").read_text()
    bodies = re.findall(r'<body name="([^"]+)"', mjcf)
    hands = [bodies.index(f"{s}_palm_link") for s in ("left", "right")]
    feet = [bodies.index(f"{s}_foot_link") for s in ("left", "right")]
    print(
        f"{'clip':32} {'s':>5} {'travel':>7} {'path':>6} {'joint':>6} {'body':>6} {'hands':>6} "
        f"{'feet':>6} {'pelvis':>7}"
    )
    for i, name in enumerate(ref["motion_files"]):
        p, pdt = clip(pred, i)
        r, rdt = clip(ref, i)
        n = p["gts"].shape[0]
        idx = (torch.arange(n) * pdt / rdt).round().long().clamp(max=r["gts"].shape[0] - 1)
        r = {k: v[idx] for k, v in r.items()}

        def travel(g):
            root = g[:, 0, :2]
            return (root[-1] - root[0]).norm(), (root[1:] - root[:-1]).norm(dim=-1).sum()

        (pt, pp), (rt, rp) = travel(p["gts"]), travel(r["gts"])
        joint = (p["dps"] - r["dps"]).pow(2).mean().sqrt()
        body = local(p["gts"], yaw(p["grs"][:, 0])) - local(r["gts"], yaw(r["grs"][:, 0]))
        body = body.norm(dim=-1)
        label = name.split("/")[-1].split("__")[0].strip("_")
        label = re.sub(r"^(soma_)?proto_|_keypoints.*$|_retargeted.*$", "", label)[:32]
        print(
            f"{label:32} {n * pdt:5.1f} {pt / max(rt, 1e-3):6.0%} {pp / max(rp, 1e-3):6.0%} "
            f"{math.degrees(joint):5.1f}° {body.mean() * 100:4.1f}cm "
            f"{body[:, hands].mean() * 100:4.1f}cm {body[:, feet].mean() * 100:4.1f}cm "
            f"{p['gts'][:, 0, 2].min():6.3f}m"
        )
    print("travel, path: the robot's against the reference's; joint: RMS error; body, hands, feet:")
    print(
        "mean position error relative to the pelvis; pelvis: its lowest height (under 0.45 m: fell)"
    )


if __name__ == "__main__":
    main()
