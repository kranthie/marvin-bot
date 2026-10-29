"""Generate the human motions Marvin Bot's trackers learn, with NVIDIA's Kimodo (text to motion).

Each clip is a text prompt and, for the locomotion clips and the in-place skills, a dense root
path: the speed, heading and direction of travel of the hips, frame by frame. The path sets the
pace, which wording alone does not: "a person jogs forward" gave a 2 m/s run and "turns around"
a 400 deg/s spin, both too fast for Marvin Bot once scaled to its size (about 0.7 of a person's
speed). Speeds here are at human scale. Several samples are generated per clip and the one with
the fewest failed skill checks (check_clips.py: the hands meet in a clap, a hand above the shoulder
in a wave...) is kept, and among those the one with the least planted-foot sliding, crossing of the
feet and root jerk (the kick: the highest kick).

Kimodo's prompting advice is followed: each prompt starts with "A person", covers one or two
behaviours at medium detail and stays within what the model was trained on (locomotion,
gestures, everyday actions, dancing and styles).

Kimodo's frame: y up, the hips start at x, z = (0, 0) facing +z; a person facing +z has their
left at +x. The heading angle th is atan2(x, z) of the facing direction (a left turn makes it
grow) and the constraint holds [cos th, sin th].

Output: one Kimodo NPZ per clip (SOMA skeleton, 30 fps), for ProtoMotions'
convert_soma23_npz_to_proto.py (see README.md).

Usage (in the Kimodo environment, with CHECKPOINT_DIR set if the models are stored locally):
  python sim/motion/generate_kimodo.py OUT_DIR [locomotion|skills|CLIP ...] [--samples N]
"""

import argparse
import json
import tempfile
from pathlib import Path

import numpy as np

MODEL = "Kimodo-SOMA-RP-v1.2"
FPS, SECONDS = 30, 8.0
N = int(FPS * SECONDS)
t = np.arange(N) / FPS


def ramp(t0: float, t1: float, v0: float, v1: float) -> np.ndarray:
    """v0 until t0, then linearly to v1 at t1."""
    return v0 + (v1 - v0) * np.clip((t - t0) / (t1 - t0), 0, 1)


ZERO = 0 * t
STILL = (ZERO, ZERO, 0.0)  # stay in place, facing forward
# name: (prompt, speed m/s, heading rate rad/s, direction of travel from the facing, rad)
LOCOMOTION = {
    "walk": (
        "A person walks forward at a relaxed, steady pace.",
        ramp(0.3, 1.5, 0, 0.85),
        ZERO,
        0.0,
    ),
    "walk_slow": ("A person walks forward slowly.", ramp(0.3, 1.5, 0, 0.6), ZERO, 0.0),
    "turn_left": (
        "A person walks forward and gradually curves to the left.",
        ramp(0.3, 1.5, 0, 0.8),
        ramp(1.5, 2.5, 0, 0.35),
        0.0,
    ),
    "turn_right": (
        "A person walks forward and gradually curves to the right.",
        ramp(0.3, 1.5, 0, 0.8),
        ramp(1.5, 2.5, 0, -0.35),
        0.0,
    ),
    "walk_back": (
        "A person walks backwards slowly with short steps.",
        ramp(0.5, 2, 0, 0.5),
        ZERO,
        np.pi,
    ),
    "side_left": (
        "A person takes small side steps to the left, facing forward, without crossing their feet.",
        ramp(0.5, 2, 0, 0.35),
        ZERO,
        np.pi / 2,
    ),
    "side_right": (
        "A person takes small side steps to the right, facing forward, "
        "without crossing their feet.",
        ramp(0.5, 2, 0, 0.35),
        ZERO,
        -np.pi / 2,
    ),
    "jog": ("A person jogs forward slowly with short strides.", ramp(0.3, 2.3, 0, 1.3), ZERO, 0.0),
    "start_stop": (
        "A person walks forward, slows down and stops.",
        np.minimum(ramp(0.3, 1.8, 0, 0.8), ramp(4.5, 6.3, 0.8, 0)),
        ZERO,
        0.0,
    ),
    "turn_in_place": (
        "A person turns in place to the left with small steps.",
        ZERO,
        np.minimum(ramp(1.0, 1.8, 0, 0.6), ramp(5.5, 6.3, 0.6, 0)),
        0.0,
    ),
    # no root path for the idle and the dance: a path pinned at the start holds the hips there too,
    # and both clips barely moved (1-2 cm)
    "idle": (
        "A person stands relaxed and slowly shifts their weight from one foot to the other.",
        None,
        None,
        0.0,
    ),
}
SKILLS = {
    # (the first wording waved for 2 s of 8)
    "wave": (
        "A person stands and waves hello with their right hand raised above their shoulder.",
        *STILL,
    ),
    "wave_both": ("A person stands and waves both hands above their head.", *STILL),
    "point": ("A person stands and points to the right with their right hand.", *STILL),
    "clap": ("A person stands and claps their hands in front of their chest.", *STILL),
    "reach_forward": (
        "A person stands and reaches forward with both hands, then lowers them.",
        *STILL,
    ),
    "arms_raise": (  # (the first wording lifted the arms about 45°)
        "A person stands and slowly raises both arms straight out to the sides to shoulder "
        "height, then lowers them.",
        *STILL,
    ),
    "look_around": ("A person stands and looks around to the left and to the right.", *STILL),
    "bow": ("A person stands and bows politely, then stands up straight.", *STILL),
    "pick_up": (
        "A person squats down, picks up a box from the ground with both hands and stands up.",
        *STILL,
    ),
    "squat": ("A person does slow squats with their arms forward.", *STILL),
    # (the first wording marched for 3 s and stood)
    "march": ("A person marches in place continuously, lifting their knees high.", *STILL),
    "dance_sway": (
        "A happy person dances, swaying their hips from side to side to music.",
        None,
        None,
        0.0,
    ),
    "carry_walk": (
        "A person walks forward slowly carrying a box in front of them with both hands.",
        ramp(0.3, 1.5, 0, 0.6),
        ZERO,
        0.0,
    ),
    # no root path: a kick steps into the ball. "A person kicks a soccer ball forward with their
    # right leg" barely lifted the foot (at most 9 cm in six samples)
    "kick": (
        "A person takes a step and kicks a ball hard with their right foot.",
        None,
        None,
        None,
    ),
}
CLIPS = {**LOCOMOTION, **SKILLS}


def root_path(speed: np.ndarray, rate: np.ndarray, offset: float):
    """Hips path x, z (N, 2) and heading angle (N,) from speed, heading rate and the direction
    of travel relative to the facing."""
    th = np.cumsum(rate) / FPS
    th -= th[0]
    a = th + offset
    step = speed[:, None] * np.stack([np.sin(a), np.cos(a)], 1) / FPS
    return np.cumsum(step, 0) - step[0], th


def badness(joints: np.ndarray, th: np.ndarray, names: list[str]) -> float:
    """Lower is better: planted-foot sliding (m/s), the share of frames with the feet crossed
    or within 2 cm of it, and the hips' jerk."""
    feet = joints[:, [names.index("LeftFoot"), names.index("RightFoot")]]
    toes = joints[:, [names.index("LeftToeBase"), names.index("RightToeBase")]]
    height = feet[..., 1] - feet[..., 1].min()
    speed = np.linalg.norm(np.gradient(feet[..., [0, 2]], 1 / FPS, axis=0), axis=-1)
    planted = height < 0.03
    slide = np.median(speed[planted]) if planted.any() else 1.0
    left = np.stack([np.cos(th), -np.sin(th)], 1)  # the person's left on the ground
    gap = np.minimum(
        ((feet[:, 0, [0, 2]] - feet[:, 1, [0, 2]]) * left).sum(1),
        ((toes[:, 0, [0, 2]] - toes[:, 1, [0, 2]]) * left).sum(1),
    )
    crossed = np.mean(gap < 0.02)
    jerk = np.median(np.abs(np.diff(joints[:, 0], 3, axis=0))) * FPS**3
    return slide * 10 + crossed * 5 + jerk * 0.01


def kick_badness(joints: np.ndarray, names: list[str]) -> float:
    """Lower is better: the right toe's lift, with run-ups over 3 m ruled out."""
    toe = joints[:, names.index("RightToeBase")]
    drift = np.linalg.norm(joints[-1, 0, [0, 2]] - joints[0, 0, [0, 2]])
    return -(toe[:, 1].max() - toe[:, 1].min()) + (drift > 3.0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("out", type=Path)
    ap.add_argument("clips", nargs="*", default=["locomotion"])
    ap.add_argument("--samples", type=int, default=4, help="samples per clip; the best is kept")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()
    groups = {"locomotion": list(LOCOMOTION), "skills": list(SKILLS)}
    names = [c for g in args.clips for c in groups.get(g, [g])]
    args.out.mkdir(parents=True, exist_ok=True)

    import check_clips  # noqa: PLC0415
    import torch  # noqa: PLC0415
    from check_clips import soma_points  # noqa: PLC0415
    from kimodo import load_model  # noqa: PLC0415
    from kimodo.constraints import load_constraints_lst  # noqa: PLC0415
    from kimodo.exports.motion_io import save_kimodo_npz  # noqa: PLC0415
    from kimodo.skeleton import SOMASkeleton77  # noqa: PLC0415
    from kimodo.tools import seed_everything  # noqa: PLC0415

    model = load_model(MODEL, device="cuda:0", default_family="Kimodo")
    joint_names = list(SOMASkeleton77().bone_order_names)  # the outputs use all 77 joints
    for name in names:
        prompt, speed, rate, offset = CLIPS[name]
        constraints = []
        if speed is not None:
            xz, th = root_path(speed, rate, offset)
            spec = [
                {
                    "type": "root2d",
                    "frame_indices": list(range(N)),
                    "smooth_root_2d": xz.tolist(),
                    "global_root_heading": np.stack([np.cos(th), np.sin(th)], 1).tolist(),
                }
            ]
            with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
                json.dump(spec, f)
            constraints = load_constraints_lst(f.name, model.skeleton)
            Path(f.name).unlink()
        seed_everything(args.seed)
        out = model(
            [prompt],
            [N],
            constraint_lst=constraints,
            num_denoising_steps=100,
            num_samples=args.samples,
            multi_prompt=True,
            post_processing=True,  # foot-skate clean-up and exact constraint following
            return_numpy=True,
        )

        def pick(k: int, out=out) -> dict:
            return {
                key: (v[k] if hasattr(v, "shape") and v.ndim and v.shape[0] == args.samples else v)
                for key, v in out.items()
            }

        scores = []
        for k in range(args.samples):
            s = pick(k)
            fails = sum(
                not ok for _, ok, _ in check_clips.tests(name, soma_points(s, joint_names), 1 / FPS)
            )
            j = s["posed_joints"]
            if name == "kick":
                b = kick_badness(j, joint_names)
            else:
                h = s["global_root_heading"]
                b = badness(
                    j, th if speed is not None else np.arctan2(h[:, 1], h[:, 0]), joint_names
                )
            scores.append((fails, b))
        best = min(range(args.samples), key=lambda k: scores[k])
        sample = pick(best)
        save_kimodo_npz(str(args.out / f"{name}.npz"), sample)
        print(
            f"{name:14} sample {best} of {args.samples}: {scores[best][0]} failed checks "
            f"(all samples: {[f for f, _ in scores]})",
            flush=True,
        )
        torch.cuda.empty_cache()


if __name__ == "__main__":
    main()
