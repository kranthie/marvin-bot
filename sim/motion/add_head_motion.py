"""Give the retargeted clips the human's head motion: neck yaw and pitch from the source.

ProtoMotions' retargeter (built for a robot without a neck) has no head keypoint, so
retarget_marvin_bot.py holds the neck at rest and every reference looked straight ahead: the
look-around clip was a robot standing still. This sets the two neck joints of each retargeted
clip from Kimodo's head orientation: yaw against the chest (the waist's own yaw is retargeted),
pitch against the hips (Marvin Bot's torso has no pitch joint, so it stays as upright as its
pelvis), clipped to the joints' ranges. Run it between the retarget and the conversion to
ProtoMotions' format, which computes the bodies from the joint angles.

Usage (any environment with numpy):
  python sim/motion/add_head_motion.py <Kimodo npz dir> <retargeted dir> <retargeting URDF>
"""

import argparse
import re
from pathlib import Path

import numpy as np
import yaml

DESC = Path(__file__).resolve().parents[2] / "description"
# SOMA joint indices in Kimodo's 77-joint output (SOMASkeleton77.bone_order_names)
HIPS, CHEST, HEAD = 0, 3, 6


def neck_angles(npz: Path) -> tuple[np.ndarray, np.ndarray]:
    """Neck yaw and pitch (rad, left and nose-down positive) per frame. Kimodo's frames are y up,
    z forward, x to the person's left; at rest every joint frame is aligned with them."""
    r = np.load(npz)["global_rot_mats"]

    def facing(parent: int) -> np.ndarray:
        rel = np.einsum("tji,tjk->tik", r[:, parent], r[:, HEAD])
        return rel @ np.array([0.0, 0.0, 1.0])

    f = facing(CHEST)
    yaw = np.arctan2(f[:, 0], f[:, 2])
    f = facing(HIPS)
    pitch = np.arctan2(-f[:, 1], np.hypot(f[:, 0], f[:, 2]))
    return yaw, pitch


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("npz_dir", type=Path)
    ap.add_argument("retargeted", type=Path)
    ap.add_argument("urdf", type=Path)
    a = ap.parse_args()
    text = a.urdf.read_text()
    joints = re.findall(r'<joint name="([^"]+)" type="([^"]+)"', text)
    order = [n for n, t in joints if t != "fixed"]
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    ranges = {j["name"]: np.radians(j["range"]) for j in robot["joints"]}
    for path in sorted(a.retargeted.glob("*_retargeted.npz")):
        clip = re.sub(r"^(soma_)?proto_|_keypoints_retargeted\.npz$", "", path.name)
        src = a.npz_dir / f"{clip}.npz"
        if not src.is_file():
            print(f"{clip}: no source, neck left at rest")
            continue
        d = dict(np.load(path))
        q = d["joint_angles"]
        yaw, pitch = neck_angles(src)
        # the retarget may be at another frame rate: resample by time
        t = np.arange(len(q)) / float(d["fps"])
        ts = np.arange(len(yaw)) / 30.0
        for name, v in (("neck_yaw", yaw), ("neck_pitch", pitch)):
            lo, hi = ranges[name]
            q[:, order.index(name)] = np.clip(np.interp(t, ts, v), lo, hi)
        d["joint_angles"] = q
        np.savez(path, **d)
        print(
            f"{clip:14s} neck yaw {np.degrees(yaw.min()):+4.0f}..{np.degrees(yaw.max()):+4.0f}°, "
            f"pitch {np.degrees(pitch.min()):+4.0f}..{np.degrees(pitch.max()):+4.0f}°"
        )


if __name__ == "__main__":
    main()
