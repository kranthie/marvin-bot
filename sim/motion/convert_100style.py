"""Turn 100STYLE motion capture (BVH) into keypoint files for retarget_marvin_bot.py.

100STYLE (Mason, Starke and Komura, 2022; CC-BY-4.0, https://zenodo.org/records/8127870) is
about 4 million frames of one actor walking, running and turning in 100 styles. This reads the
global joint positions from the BVH files and writes the retargeter's 18 keypoints in
ProtoMotions' SOMA keypoint convention, so retarget_marvin_bot.py's scaling applies unchanged
(run it with --converted): the legs scaled to SOMA's length, the shoulders moved to SOMA's
height and width, 30 fps, z up, metres, the lowest foot point 4.8 cm up (as SOMA's keypoints)
and foot contacts from height and speed. Each clip is an 8 s segment (CLIPS).

Usage (numpy only):
  python sim/motion/convert_100style.py /path/to/100STYLE /path/to/keypoints
"""

import argparse
from pathlib import Path

import numpy as np

CLIPS = [  # (style, file, start in s, name): the clips the trackers so far train on
    ("Neutral", "FW", 10, "walk_straight"),
    ("Neutral", "FW", 40, "walk_turn"),
    ("Neutral", "BW", 10, "walk_back"),
    ("Neutral", "SW", 10, "walk_side"),
    ("Neutral", "FR", 10, "jog"),
    ("Neutral", "ID", 5, "idle"),
    ("Neutral", "TR1", 10, "transition"),
    ("Proud", "FW", 10, "proud_walk"),
]
SECONDS = 8
NAMES = {  # keypoint source joints (left = the actor's left)
    "pelvis": "Hips",
    "left_hip": "LeftHip",
    "right_hip": "RightHip",
    "left_knee": "LeftKnee",
    "right_knee": "RightKnee",
    "left_ankle": "LeftAnkle",
    "right_ankle": "RightAnkle",
    "left_toe": "LeftToe",
    "right_toe": "RightToe",
    "left_shoulder": "LeftShoulder",
    "right_shoulder": "RightShoulder",
    "left_elbow": "LeftElbow",
    "right_elbow": "RightElbow",
    "left_wrist": "LeftWrist",
    "right_wrist": "RightWrist",
    "spine": "Chest2",
}

# SOMA keypoint geometry, measured from ProtoMotions' extracted keypoints (before the
# retargeter's G1 scale): standing hip-to-ankle length, shoulders, toe, hand and torso aids
SOMA_LEG = 0.837
SOMA_SHOULDER_UP = 0.493  # shoulders above the hip midpoint
SOMA_SHOULDER_WIDTH = 0.339
TOE = 0.15
HAND_AUX = 0.20
TORSO_AUX = (0.118, 0.0, 0.098)  # ahead, left, up of the hip midpoint, pelvis frame
PELVIS_KP = (-0.026, 0.0, 0.072)  # SOMA's Hips joint relative to the hip midpoint
FOOT_UP = 0.048  # SOMA keypoints' lowest foot point above the floor
# the retargeter starts the robot root from orientations[:, 0] @ this
SOMA_ROOT_TO_ROBOT = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
FPS = 30.0


def _rot(axis: str, deg: np.ndarray) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    o, z = np.ones_like(a), np.zeros_like(a)
    m = {
        "X": [[o, z, z], [z, c, -s], [z, s, c]],
        "Y": [[c, z, s], [z, o, z], [-s, z, c]],
        "Z": [[c, -s, z], [s, c, z], [z, z, o]],
    }[axis.upper()]
    return np.moveaxis(np.array(m), [0, 1], [-2, -1])


def read_bvh(path: Path) -> tuple[dict[str, np.ndarray], float]:
    """Global joint positions {name: (T, 3)} in the file's units and axes, and the fps."""
    head, motion = path.read_text().split("MOTION", 1)
    tok = head.split()
    joints, stack, i = [], [], 0
    while i < len(tok):
        t = tok[i]
        if t in ("ROOT", "JOINT"):
            joints.append({"name": tok[i + 1], "parent": stack[-1] if stack else None})
            i += 2
        elif t == "End":
            joints.append({"name": f"{joints[stack[-1]]['name']}_end", "parent": stack[-1]})
            i += 2
        elif t == "{":
            stack.append(len(joints) - 1)
            i += 1
        elif t == "}":
            stack.pop()
            i += 1
        elif t == "OFFSET":
            joints[-1]["offset"] = np.array([float(x) for x in tok[i + 1 : i + 4]])
            i += 4
        elif t == "CHANNELS":
            n = int(tok[i + 1])
            joints[-1]["channels"] = tok[i + 2 : i + 2 + n]
            i += 2 + n
        else:
            i += 1
    lines = motion.strip().splitlines()
    frame_time = float(lines[1].split(":")[1])
    data = np.array([[float(x) for x in ln.split()] for ln in lines[2:] if ln.strip()])
    T = len(data)
    col, out_R, out_p = 0, [], []
    for j in joints:
        ch = j.get("channels", [])
        vals = data[:, col : col + len(ch)]
        col += len(ch)
        pos = np.tile(j["offset"], (T, 1))
        R = np.tile(np.eye(3), (T, 1, 1))
        for k, c in enumerate(ch):
            if c.endswith("position"):
                pos[:, "XYZ".index(c[0])] = vals[:, k]
            else:
                R = R @ _rot(c[0], vals[:, k])
        if j["parent"] is None:
            gR, gp = R, pos
        else:
            pR, pp = out_R[j["parent"]], out_p[j["parent"]]
            gR = pR @ R
            gp = pp + np.einsum("tij,tj->ti", pR, pos)
        out_R.append(gR)
        out_p.append(gp)
    return {j["name"]: p for j, p in zip(joints, out_p, strict=True)}, 1.0 / frame_time


def to_keypoints(joints: dict[str, np.ndarray], fps: float) -> dict:
    """The retargeter's keypoint dict from y-up global joint positions."""
    P = {k: np.asarray(joints[v], float) for k, v in NAMES.items()}
    P = {k: np.stack([v[:, 2], v[:, 0], v[:, 1]], 1) for k, v in P.items()}  # y up -> z up
    t_src = np.arange(len(P["pelvis"])) / fps
    t_new = np.arange(0, t_src[-1], 1 / FPS)
    P = {k: np.stack([np.interp(t_new, t_src, v[:, a]) for a in range(3)], 1) for k, v in P.items()}
    hip_mid = (P["left_hip"] + P["right_hip"]) / 2
    leg = np.median(
        np.linalg.norm(P["left_hip"] - P["left_knee"], axis=1)
        + np.linalg.norm(P["left_knee"] - P["left_ankle"], axis=1)
    )
    P = {k: (v - hip_mid[:1] * [1, 1, 0]) * (SOMA_LEG / leg) for k, v in P.items()}
    hip_mid = (P["left_hip"] + P["right_hip"]) / 2
    # shoulders (arms with them) moved to SOMA's height above the hips and width, since
    # retarget_marvin_bot.py's torso scale was set on SOMA
    sh_mid = (P["left_shoulder"] + P["right_shoulder"]) / 2
    up = np.median((sh_mid - hip_mid)[:, 2])
    width = np.median(np.linalg.norm(P["left_shoulder"] - P["right_shoulder"], axis=1))
    for side in ("left", "right"):
        half = (P[f"{side}_shoulder"] - sh_mid) * (SOMA_SHOULDER_WIDTH / width)
        new = hip_mid + (sh_mid - hip_mid) * [1, 1, SOMA_SHOULDER_UP / up] + half
        d = new - P[f"{side}_shoulder"]
        for part in ("shoulder", "elbow", "wrist"):
            P[f"{side}_{part}"] = P[f"{side}_{part}"] + d
    # pelvis frame: y along the hip axis (left), z towards the spine, x forward
    y = P["left_hip"] - P["right_hip"]
    y /= np.linalg.norm(y, axis=1, keepdims=True)
    z = P["spine"] - hip_mid
    z -= (z * y).sum(1, keepdims=True) * y
    z /= np.linalg.norm(z, axis=1, keepdims=True)
    Rp = np.stack([np.cross(y, z), y, z], -1)  # columns: pelvis axes in the world

    def ahead(a, b, dist):  # dist beyond a, along b -> a
        d = P[a] - P[b]
        return P[a] + dist * d / np.linalg.norm(d, axis=1, keepdims=True)

    def toe(side):
        d = P[f"{side}_toe"] - P[f"{side}_ankle"]
        return P[f"{side}_ankle"] + TOE * d / np.linalg.norm(d, axis=1, keepdims=True)

    kp = np.stack(
        [
            hip_mid + np.einsum("tij,j->ti", Rp, PELVIS_KP),
            P["left_hip"],
            P["right_hip"],
            P["left_knee"],
            P["right_knee"],
            P["left_ankle"],
            P["right_ankle"],
            toe("left"),
            toe("right"),
            P["left_shoulder"],
            P["right_shoulder"],
            P["left_elbow"],
            P["right_elbow"],
            P["left_wrist"],
            P["right_wrist"],
            ahead("left_wrist", "left_elbow", HAND_AUX),
            ahead("right_wrist", "right_elbow", HAND_AUX),
            hip_mid + np.einsum("tij,j->ti", Rp, TORSO_AUX),
        ],
        1,
    )
    kp[..., 2] += FOOT_UP - kp[:, 5:9, 2].min()  # lowest ankle or toe point
    rot = np.tile(np.eye(3), (len(kp), 18, 1, 1))
    rot[:, 0] = Rp @ SOMA_ROOT_TO_ROBOT.T  # only the root orientation is used

    def contacts(*idx):  # within 4 cm of the point's lowest and slower than 0.4 m/s
        out = []
        for i in idx:
            h = kp[:, i, 2] - kp[:, i, 2].min()
            v = np.r_[0, np.linalg.norm(np.diff(kp[:, i, :2], axis=0), axis=1) * FPS]
            out.append((h < 0.04) & (v < 0.4))
        return np.stack(out, 1)

    return {
        "positions": kp.astype(np.float32),
        "orientations": rot.astype(np.float32),
        "left_foot_contacts": contacts(5, 7),
        "right_foot_contacts": contacts(6, 8),
        "fps": FPS,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("source", type=Path, help="the unzipped 100STYLE folder (one per style)")
    ap.add_argument("output", type=Path, help="folder for the keypoint files")
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for style, kind, start, name in CLIPS:
        joints, fps = read_bvh(args.source / style / f"{style}_{kind}.bvh")
        a, b = int(start * fps), int((start + SECONDS) * fps)
        kp = to_keypoints({k: v[a:b] for k, v in joints.items()}, fps)
        np.save(args.output / f"100style_{name}_keypoints.npy", kp, allow_pickle=True)
        print(f"{name:15} {len(kp['positions'])} frames")


if __name__ == "__main__":
    main()
