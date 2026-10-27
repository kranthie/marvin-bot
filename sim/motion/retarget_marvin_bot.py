"""Retarget human keypoints to Marvin Bot with ProtoMotions' PyRoki retargeter.

Runs ProtoMotions' G1 retargeter (pyroki/batch_retarget_to_g1_from_keypoints.py) unchanged
except for what is robot-specific:
- which robot frame each human keypoint is matched to (LINKS), on the retargeting URDF that
  setup_protomotions.py writes (it adds the toe, hand and torso frames);
- the scale from the human source to Marvin Bot, applied on top of the retargeter's G1 scale
  (SCALE);
- the joints held near their rest angle (the retargeter's G1 list names joints Marvin Bot has
  not got: a waist roll and pitch, wrist pitch).

It checks that PyRoki's joint order is the MJCF's, since the converter that follows reads the
retargeted joint vector in MJCF order.

Usage (in the PyRoki environment), then continue with ProtoMotions' converter:
  python sim/motion/retarget_marvin_bot.py --protomotions /path/to/ProtoMotions \\
      --keypoints-folder-path keypoints/ --output-dir retargeted/ --source-type soma --no-visualize
"""

import re
import sys
from pathlib import Path

import numpy as onp

# human keypoint -> Marvin Bot frame, in the retargeter's order
LINKS = [
    ("pelvis", "pelvis"),  # the pelvis origin is on the hip axes, where SOMA's pelvis point is
    ("left_hip", "left_hip_pitch_link"),
    ("right_hip", "right_hip_pitch_link"),
    ("left_knee", "left_shin_link"),
    ("right_knee", "right_shin_link"),
    ("left_ankle", "left_foot_link"),
    ("right_ankle", "right_foot_link"),
    ("left_foot", "left_toe_link"),
    ("right_foot", "right_toe_link"),
    ("left_shoulder", "left_shoulder_pitch_link"),
    ("right_shoulder", "right_shoulder_pitch_link"),
    ("left_elbow", "left_forearm_link"),
    ("right_elbow", "right_forearm_link"),
    ("left_wrist", "left_hand_link"),
    ("right_wrist", "right_hand_link"),
]
# Source-to-robot scale (x, y, z) for the SOMA source, and the retargeter's own for G1. Legs:
# horizontal 0.657 is the straight-leg pelvis height ratio (0.65 / 0.99 m) and vertical 0.629
# the pelvis-to-ankle drop (0.59 / 0.938 m), as the retargeter derives G1's. Upper body: shoulder
# width and arm length are ~0.6 of the source's, shoulder height above the hips ~0.75
SCALE = {"lower": (0.657, 0.657, 0.629), "upper": (0.62, 0.62, 0.70)}
G1_SCALE = {"lower": (0.8, 0.8, 0.807), "upper": (0.8, 0.8, 0.7)}
HOLD = ["neck_yaw", "neck_pitch", "left_gripper", "right_gripper"]


def main() -> None:
    argv = sys.argv[1:]
    i = argv.index("--protomotions")
    pm = Path(argv.pop(i + 1))
    argv.pop(i)
    sys.path.insert(0, str(pm / "pyroki"))
    import batch_retarget_to_g1_from_keypoints as r  # noqa: PLC0415
    import pyroki as pk  # noqa: PLC0415
    import yourdfpy  # noqa: PLC0415

    urdf = pm / "protomotions/data/assets/urdf/for_retargeting/marvin_bot.urdf"
    mjcf = pm / "protomotions/data/assets/mjcf/marvin_bot.xml"
    robot = pk.Robot.from_urdf(yourdfpy.URDF.load(urdf))
    mj_order = [n for n in re.findall(r'<joint name="([^"]+)"', mjcf.read_text()) if n != "root"]
    if list(robot.joints.actuated_names) != mj_order:
        sys.exit(f"PyRoki joint order {robot.joints.actuated_names} is not the MJCF's {mj_order}")

    def indices():
        names = [h for h, _ in LINKS]
        return names, r.jnp.array([r.G1_LINK_NAMES.index(link) for _, link in LINKS])

    load = r.load_motion_data

    def load_motion_data(motion_path, source_type, *args, **kwargs):
        out = list(load(motion_path, source_type, *args, **kwargs))
        if source_type == "soma":
            # the retargeter returns [root, root + lower body offsets (8), root + upper body
            # offsets], each part scaled for G1; rescale each part for Marvin Bot
            kp = out[0]
            root = kp[:, :1]
            ratio = {k: onp.array(SCALE[k]) / onp.array(G1_SCALE[k]) for k in SCALE}
            new = root * ratio["lower"] + (kp - root) * ratio["upper"]
            new[:, 1:9] = root * ratio["lower"] + (kp[:, 1:9] - root) * ratio["lower"]
            new[:, 0] = root[:, 0] * ratio["lower"]
            out[0] = new
        return tuple(out)

    r.get_humanoid_retarget_indices = indices
    r.load_motion_data = load_motion_data
    r.JOINTS_TO_MOVE_LESS = {"soma": HOLD, "smpl": HOLD}
    sys.argv = [sys.argv[0], "--urdf-path", str(urdf), *argv]
    r.main()


if __name__ == "__main__":
    main()
