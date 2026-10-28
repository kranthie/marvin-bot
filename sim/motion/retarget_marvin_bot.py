"""Retarget human keypoints to Marvin Bot with ProtoMotions' PyRoki retargeter.

Runs ProtoMotions' G1 retargeter (pyroki/batch_retarget_to_g1_from_keypoints.py) unchanged
except for what is robot-specific:
- which robot frame each human keypoint is matched to (LINKS), on the retargeting URDF that
  setup_protomotions.py writes (it adds the toe, hand and torso frames);
- the scale from the human source to Marvin Bot in place of the retargeter's G1 scale
  (scale_keypoints), with the feet moved apart to Marvin Bot's hip spacing (WIDEN);
- the rest pose the solver starts from and pulls towards, the standing pose instead of the
  middle of each joint's range;
- the joints held near their rest angle (the retargeter's G1 list names joints Marvin Bot has
  not got: a waist roll and pitch, wrist roll; the wrist yaw and pitch are held too, at the
  standing pose's palms-in angle, since no keypoint sets the hand's angle).

It checks that PyRoki's joint order is the MJCF's, since the converter that follows reads the
retargeted joint vector in MJCF order.

Keypoints converted from other motion capture into the same convention (convert_100style.py)
take --converted.

Usage (in the PyRoki environment), then continue with ProtoMotions' converter:
  python sim/motion/retarget_marvin_bot.py --protomotions /path/to/ProtoMotions \\
      --keypoints-folder-path keypoints/ --output-dir retargeted/ --source-type soma --no-visualize
"""

import ast
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
# The retargeter scales its SOMA source for G1 (legs and upper body about the pelvis keypoint,
# by axis); scale_keypoints undoes that and scales for Marvin Bot instead
G1_SCALE = {"lower": (0.8, 0.8, 0.807), "upper": (0.8, 0.8, 0.7)}
# Legs: uniform, the robot's straight hip-to-ankle length over the source's standing one (knee
# 5-7 deg bent; 0.59 / 0.853 m). The knee is sensitive to this near straight: a target leg 1.5
# cm short of the robot's holds the knee at 26 deg
LEG = 0.69
# Shoulders about the hips: width 0.62 of the source's, height 0.75, which puts them 0.34-0.39
# m above the hip axis across the clips (the robot's are 0.365). Arms: uniform about each
# shoulder, the robot's straight-arm reach to the wrist pitch axis over the source's
# (0.443 / 0.56 m)
TORSO = (0.62, 0.62, 0.75)
# and forward (m, along the hips' heading): the source's shoulders are 2.5-4 cm behind its hips,
# Marvin Bot's 2.5 cm ahead of its own. Left there, the solver sat the robot back to reach them
SHOULDER_FORWARD = 0.065
ARM = 0.79
# The source's pelvis keypoint is 8 cm above its hips; Marvin Bot's pelvis origin is on the hip
# axis, so its target is put midway between the hip targets. With either this or the shoulders
# left high, the solver crouches the robot to reach them (knees bent 37-58 deg through a walk)
# Scaled down, the human's feet end up 12-14 cm apart, often less mid-swing, and Marvin Bot's
# feet are 11 cm wide: they would clip each other. The knees (keypoints 3, 4) and feet (5-8)
# move out by this much (m) each, giving 18-20 cm, about the hip spacing (left keypoints are odd)
WIDEN = {3: 0.015, 4: 0.015, 5: 0.03, 6: 0.03, 7: 0.03, 8: 0.03}
HOLD = [
    "neck_yaw",
    "neck_pitch",
    "left_wrist_yaw",
    "right_wrist_yaw",
    "left_wrist_pitch",
    "right_wrist_pitch",
]


def scale_keypoints(kp: onp.ndarray) -> onp.ndarray:
    """The retargeter's SOMA keypoints (T, 18, 3; pelvis, legs 1-8, shoulders, elbows, wrists
    9-14 with left first, hand and torso aids 15-17), rescaled from G1's size to Marvin Bot's."""
    root = kp[:, :1]
    src = kp.copy()  # the unscaled source
    src[:, 0] = root[:, 0] / G1_SCALE["lower"]
    src[:, 1:9] = src[:, :1] + (kp[:, 1:9] - root) / G1_SCALE["lower"]
    src[:, 9:] = src[:, :1] + (kp[:, 9:] - root) / G1_SCALE["upper"]
    new = src * LEG
    hips_src, hips = src[:, 1:3].mean(1, keepdims=True), new[:, 1:3].mean(1, keepdims=True)
    new[:, 0] = hips[:, 0]
    side = new[:, 1, :2] - new[:, 2, :2]  # along the hip axis, to the left
    side /= onp.linalg.norm(side, axis=-1, keepdims=True)
    forward = onp.stack([side[:, 1], -side[:, 0], onp.zeros(len(side))], -1)[:, None]
    new[:, 9:11] = hips + (src[:, 9:11] - hips_src) * TORSO + SHOULDER_FORWARD * forward
    new[:, 17] = hips[:, 0] + (src[:, 17] - hips_src[:, 0]) * TORSO
    for arm, shoulder in ((11, 9), (13, 9), (15, 9), (12, 10), (14, 10), (16, 10)):
        new[:, arm] = new[:, shoulder] + (src[:, arm] - src[:, shoulder]) * ARM
    # move the knees and feet outwards along the hip axis (left is +)
    for k, w in WIDEN.items():
        new[:, k, :2] += onp.sign(k % 2 - 0.5) * w * side
    return new


def main() -> None:
    argv = sys.argv[1:]
    i = argv.index("--protomotions")
    pm = Path(argv.pop(i + 1))
    argv.pop(i)
    # --converted: keypoints from other motion capture, already in the SOMA keypoint convention
    # with the pelvis keypoint on the hips (not extracted from a ProtoMotions SOMA motion)
    converted = "--converted" in argv
    if converted:
        argv.remove("--converted")
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

    # PyRoki's default pose, which the solver starts from and its rest cost pulls every joint
    # towards, is the middle of each joint's range: for Marvin Bot a 65 deg knee and 45 deg hip
    # pitch. Near a straight leg that small pull outweighs a centimetre of keypoint error, and
    # it held the knees at 23 deg or more through a walk. Use the standing pose instead
    config = (pm / "protomotions/robot_configs/marvin_bot.py").read_text()
    stance = ast.literal_eval(re.search(r"DEFAULT_JOINT_POS = (\{.*\})", config).group(1))
    rest = onp.array([stance.get(n, 0.0) for n in mj_order])
    from_urdf = pk.Robot.from_urdf
    pk.Robot.from_urdf = lambda urdf, **kw: from_urdf(urdf, default_joint_cfg=rest, **kw)

    def indices():
        names = [h for h, _ in LINKS]
        return names, r.jnp.array([r.G1_LINK_NAMES.index(link) for _, link in LINKS])

    load = r.load_motion_data

    def load_motion_data(motion_path, source_type, *args, **kwargs):
        out = list(load(motion_path, source_type, *args, **kwargs))
        if source_type == "soma":
            out[0] = scale_keypoints(out[0])
        return tuple(out)

    if converted:
        r.soma_pelvis_keypoint_on_hips = lambda positions, orientations: positions
    r.get_humanoid_retarget_indices = indices
    r.load_motion_data = load_motion_data
    r.JOINTS_TO_MOVE_LESS = {"soma": HOLD, "smpl": HOLD}
    sys.argv = [sys.argv[0], "--urdf-path", str(urdf), *argv]
    r.main()


if __name__ == "__main__":
    main()
