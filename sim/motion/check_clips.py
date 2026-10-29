"""Check that each generated reference clip does what its prompt asked for, before training on it.

Four layers, the first three on Kimodo's human clip and the third again on the robot reference
retargeted from it, so a failure can be traced to generation or to retargeting:

  1. text match: NVIDIA's text-motion retrieval model for the SOMA skeleton (TMR-SOMA-RP-v1, the
     model Kimodo's benchmark scores with) embeds every clip and every prompt. Each clip gets its
     similarity to its own prompt and the rank of its own prompt among all the prompts: a clip
     that a different prompt describes better is suspect.
  2. motion quality, with Kimodo's benchmark metrics: planted-foot sliding (from toe height and
     from Kimodo's contact labels), the share of contact frames that skate, contact consistency,
     and how far the hips strayed from the root path the clip was generated on.
  3. skill checks: a geometric test per clip of what the prompt asked for (the hands meet in a
     clap, a hand above the shoulder swinging in a wave, the pelvis dropping in a squat, the
     speed and turn of a walk), with thresholds in units of the standing head height so the same
     test runs on the human and on the robot.
  4. review renders (render subcommand): the human clip as a stick figure beside the robot
     reference, with the layer 3 results on the frame.

Usage:
  (Kimodo environment, with the TMR model downloaded on first use)
  python sim/motion/check_clips.py human <Kimodo npz dir> [--out report.json]
  (this repo's environment; motions.npz from render_tracking.py export)
  python sim/motion/check_clips.py robot <motions.npz> [--out report.json]
  python sim/motion/check_clips.py render <Kimodo npz dir> <motions.npz> <out dir> [clip ...]
      (after the human step, which leaves the SOMA joint names beside the clips)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate_kimodo import CLIPS, FPS, root_path  # noqa: E402

# ------------------------------------------------------------------------------- body points
# Every test reads these, z up, metres, from the human (SOMA joints) or the robot (link origins):
# pelvis, head, l/r shoulder, l/r hand, l/r foot (T, 3), head_fwd (T, 3) a unit vector the face
# points along, and heading (T,) the pelvis yaw.

SOMA = {
    "pelvis": "Hips",
    "head": "Head",
    "lsho": "LeftArm",
    "rsho": "RightArm",
    "lhand": "LeftHand",
    "rhand": "RightHand",
    "lfoot": "LeftFoot",
    "rfoot": "RightFoot",
}
ROBOT = {
    "pelvis": "pelvis",
    "head": "head_link",
    "lsho": "left_shoulder_roll_link",
    "rsho": "right_shoulder_roll_link",
    "lhand": "left_palm_link",
    "rhand": "right_palm_link",
    "lfoot": "left_foot_link",
    "rfoot": "right_foot_link",
}


def soma_points(npz: dict, names: list[str]) -> dict:
    """Body points from a Kimodo NPZ (y up, facing +z, left +x) turned to x forward, z up."""
    j = npz["posed_joints"][..., [2, 0, 1]]
    p = {k: j[:, names.index(v)] for k, v in SOMA.items()}
    eyes = (j[:, names.index("LeftEye")] + j[:, names.index("RightEye")]) / 2
    fwd = eyes - j[:, names.index("Head")]
    p["head_fwd"] = fwd / np.linalg.norm(fwd, axis=1, keepdims=True)
    h = npz["global_root_heading"]  # cos, sin of the heading in Kimodo's frame
    p["heading"] = np.arctan2(h[:, 1], h[:, 0])
    return p


# ------------------------------------------------------------------------------- the tests


def _yaw(v: np.ndarray) -> np.ndarray:
    return np.unwrap(np.arctan2(v[:, 1], v[:, 0]))


def _local(p: dict, k: str) -> np.ndarray:
    """A point relative to the pelvis in the pelvis's heading frame: x forward, y left, z up."""
    d = p[k] - p["pelvis"]
    c, s = np.cos(-p["heading"]), np.sin(-p["heading"])
    return np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1], d[:, 2]], 1)


def _events(mask: np.ndarray, min_gap: int = 5) -> int:
    """Separate runs of True, runs closer than min_gap frames merged."""
    idx = np.flatnonzero(mask)
    return 0 if not len(idx) else 1 + int(np.sum(np.diff(idx) > min_gap))


def _cycles(x: np.ndarray, amp: float) -> int:
    """Swings of x about its mean larger than amp each side, counted as crossings / 2."""
    x = x - x.mean()
    state, n = 0, 0
    for v in x:
        s = 1 if v > amp else -1 if v < -amp else 0
        if s and s != state:
            n += state != 0
            state = s
    return (n + 1) // 2


def standing_height(p: dict) -> float:
    return float(np.percentile(p["head"][:, 2] - np.minimum(p["lfoot"], p["rfoot"])[:, 2], 90))


def tests(name: str, p: dict, dt: float) -> list[tuple[str, bool, str]]:
    """(test, passed, measured) for one clip; H is the standing head height."""
    H = standing_height(p)
    t = len(p["pelvis"]) * dt
    out = []

    def check(label, ok, measured):
        out.append((label, bool(ok), measured))

    lh, rh = _local(p, "lhand"), _local(p, "rhand")
    ls, rs = _local(p, "lsho"), _local(p, "rsho")
    floor = min(p["lfoot"][:, 2].min(), p["rfoot"][:, 2].min())
    pel_h = p["pelvis"][:, 2] - floor
    travel = p["pelvis"][-1, :2] - p["pelvis"][0, :2]
    yaw = np.degrees(p["heading"][-1] - p["heading"][0])
    head_yaw = np.degrees(_yaw(p["head_fwd"]) - p["heading"])

    spec = CLIPS[name]
    if spec[1] is not None and np.any(spec[1]):  # a walk: the root path's speed and turn
        xz, th = root_path(*spec[1:])
        want = np.linalg.norm(xz[-1] - xz[0])
        scale = H / 1.75  # the paths are at human scale (head about 1.75 m)
        got = np.linalg.norm(travel)
        check("distance", 0.75 < got / (want * scale) < 1.25, f"{got:.2f} m of {want * scale:.2f}")
        check(
            "turn",
            abs(yaw - np.degrees(th[-1])) < 25,
            f"{yaw:+.0f}° of {np.degrees(th[-1]):+.0f}°",
        )
        fwd0 = np.array([np.cos(p["heading"][0]), np.sin(p["heading"][0])])
        if spec[3]:  # sideways or backwards: the direction of travel against the facing
            cross = fwd0[0] * travel[1] - fwd0[1] * travel[0]
            ang = np.degrees(np.arctan2(cross, fwd0 @ travel))
            check(
                "direction",
                abs(((ang - np.degrees(spec[3])) + 180) % 360 - 180) < 30,
                f"{ang:+.0f}° of {np.degrees(spec[3]):+.0f}°",
            )
    elif name != "kick":  # in place
        check(
            "stays in place",
            np.linalg.norm(travel) < 0.25 * H,
            f"moved {np.linalg.norm(travel):.2f} m",
        )

    feet_up = np.minimum(p["lfoot"][:, 2], p["rfoot"][:, 2]) - floor
    if name == "jog":
        flight = feet_up > 0.02 * H
        check(
            "both feet leave the ground",
            flight.mean() > 0.05,
            f"{100 * flight.mean():.0f}% of frames",
        )
    if name == "turn_in_place":
        check("turns", abs(yaw) > 90, f"{yaw:+.0f}°")
    if name == "idle":
        sway = np.ptp(_local(p, "head")[:, 1])
        check("shifts weight", sway > 0.02 * H, f"head sways {100 * sway:.0f} cm")
    if name == "dance_sway":
        n = _cycles(p["pelvis"][:, 1] - p["pelvis"][:, 1].mean(), 0.01 * H)
        check("sways side to side", n >= 2, f"{n} swings")
    if name in ("wave", "wave_both"):
        hands = [("right", rh, rs)] + ([("left", lh, ls)] if name == "wave_both" else [])
        for side, h, s in hands:
            up = h[:, 2] > s[:, 2]
            check(
                f"{side} hand above the shoulder",
                up.mean() > 0.4,
                f"{100 * up.mean():.0f}% of the time",
            )
            n = _cycles(h[up, 1], 0.02 * H) if up.any() else 0
            check(f"{side} hand swings", n >= 2, f"{n} swings")
        if name == "wave":
            down = lh[:, 2] < ls[:, 2]
            check(
                "left hand stays down", down.mean() > 0.8, f"{100 * down.mean():.0f}% of the time"
            )
    if name == "wave_both":
        above = np.minimum(lh[:, 2], rh[:, 2]) > _local(p, "head")[:, 2]
        check(
            "both hands above the head",
            above.mean() > 0.3,
            f"{100 * above.mean():.0f}% of the time",
        )
    if name == "clap":
        gap = np.linalg.norm(lh - rh, axis=1)
        meet = gap < 0.07 * H
        front = (lh[:, 0] + rh[:, 0]) / 2 > 0.05 * H
        height = ((lh[:, 2] + rh[:, 2]) / 2 + (p["pelvis"][:, 2] - floor)) / H
        chest = (height > 0.55) & (height < 0.95)
        n = _events(meet & front & chest)
        check(
            "hands meet in front of the chest",
            n >= 3,
            f"{n} claps, closest {100 * gap.min():.0f} cm",
        )
    if name == "point":
        reach = np.linalg.norm(rh - rs, axis=1)
        arm = np.percentile(reach, 99)
        out_right = (rh[:, 1] - rs[:, 1]) < -0.6 * reach
        straight = reach > 0.85 * arm
        held = (out_right & straight).mean() * t
        check("right arm out to the right", held > 1.0, f"held {held:.1f} s")
    if name == "reach_forward":
        fwd = np.minimum(lh[:, 0] - ls[:, 0], rh[:, 0] - rs[:, 0])
        check(
            "both hands reach forward",
            fwd.max() > 0.25 * H,
            f"{100 * fwd.max():.0f} cm ahead of the shoulders",
        )
        check("then lowered", (lh[-1, 2] < ls[-1, 2]) and (rh[-1, 2] < rs[-1, 2]), "at the end")
    if name == "arms_raise":
        side = np.minimum(lh[:, 1] - ls[:, 1], -(rh[:, 1] - rs[:, 1]))
        level = np.maximum(abs(lh[:, 2] - ls[:, 2]), abs(rh[:, 2] - rs[:, 2])) < 0.1 * H
        check("arms out to the sides", (side > 0.2 * H).any(), f"{100 * side.max():.0f} cm out")
        check("at shoulder height", (level & (side > 0.15 * H)).any(), "")
    if name == "look_around":
        rel = head_yaw - np.median(head_yaw)
        check(
            "head turns both ways",
            rel.max() > 25 and rel.min() < -25,
            f"{rel.min():+.0f}°..{rel.max():+.0f}°",
        )
    if name == "bow":
        up = p["head"] - p["pelvis"]
        lean = np.degrees(np.arctan2(np.linalg.norm(up[:, :2], axis=1), up[:, 2]))
        check("bows", lean.max() > 25, f"{lean.max():.0f}° from upright")
        check("stands up again", lean[-int(0.5 / dt) :].mean() < 12, f"{lean[-1]:.0f}° at the end")
    if name in ("squat", "pick_up"):
        drop = 1 - pel_h / np.percentile(pel_h, 95)
        check("pelvis drops", drop.max() > 0.3, f"{100 * drop.max():.0f}% lower")
        if name == "squat":
            n = _events(drop > 0.25, min_gap=int(0.5 / dt))
            check("squats more than once", n >= 2, f"{n} squats")
        else:
            hand_h = np.minimum(p["lhand"][:, 2], p["rhand"][:, 2]) - floor
            check(
                "hands reach the floor",
                hand_h.min() < 0.15 * H,
                f"lowest {100 * hand_h.min():.0f} cm up",
            )
            check("stands up again", drop[-int(0.5 / dt) :].mean() < 0.1, "")
    if name == "march":
        lifts = [p[f][:, 2] - floor > 0.08 * H for f in ("lfoot", "rfoot")]
        n = sum(_events(u, min_gap=int(0.2 / dt)) for u in lifts)
        late = sum(_events(u[len(u) // 2 :], min_gap=int(0.2 / dt)) for u in lifts)
        top = max(p[f][:, 2].max() for f in ("lfoot", "rfoot")) - floor
        check(
            "knees lift, both feet in turn", n >= 6, f"{n} steps, a foot up to {100 * top:.0f} cm"
        )
        check("keeps marching", late >= 3, f"{late} steps in the second half")
    if name == "kick":
        rf = p["rfoot"]
        speed = np.linalg.norm(np.diff(rf, axis=0), axis=1) / dt
        check("right foot kicks", speed.max() > 2.0 * H / 1.75, f"{speed.max():.1f} m/s")
        check(
            "right foot lifts",
            (rf[:, 2] - floor).max() > 0.15 * H,
            f"{100 * (rf[:, 2] - floor).max():.0f} cm",
        )
    if name == "carry_walk":
        gap = np.linalg.norm(lh - rh, axis=1)
        front = (lh[:, 0] + rh[:, 0]) / 2 > 0.1 * H
        held = ((gap > 0.1 * H) & (gap < 0.5 * H) & front).mean()
        check(
            "hands held in front, apart as round a box",
            held > 0.7,
            f"{100 * held:.0f}% of the time",
        )
    return out


# ------------------------------------------------------------------------------- human (Kimodo env)


def human(npz_dir: Path, out: Path | None) -> None:
    import torch  # noqa: PLC0415
    from kimodo.metrics.foot_skate import (  # noqa: PLC0415
        FootContactConsistency,
        FootSkateFromContacts,
        FootSkateFromHeight,
        FootSkateRatio,
    )
    from kimodo.model.load_model import load_model  # noqa: PLC0415
    from kimodo.skeleton import SOMASkeleton77  # noqa: PLC0415

    skel = SOMASkeleton77()
    names = list(skel.bone_order_names)
    clips = sorted(c for c in CLIPS if (npz_dir / f"{c}.npz").is_file())
    # the joint names, for the review renders (which run without Kimodo)
    (npz_dir / "soma_names.json").write_text(json.dumps(names))
    data = {c: dict(np.load(npz_dir / f"{c}.npz")) for c in clips}

    # layer 1: TMR
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    tmr = load_model(modelname="tmr-soma-rp", device=dev, default_family="TMR")
    with torch.inference_mode():
        m = torch.cat(
            [
                tmr.encode_motion(
                    torch.from_numpy(data[c]["posed_joints"]).float().to(dev), unit_vector=True
                )
                for c in clips
            ]
        )
        t = torch.cat([tmr.encode_raw_text([CLIPS[c][0]], unit_vector=True) for c in clips])
    sim = (m.reshape(len(clips), -1) @ t.reshape(len(clips), -1).T).cpu().numpy()
    # TMR's similarity as Kimodo's benchmark reports it: cosine mapped to 0..1
    sim01 = (sim + 1) / 2

    # layer 2: Kimodo's metrics
    kw = {"skeleton": skel, "fps": FPS}
    metrics = [
        FootSkateFromHeight(**kw),
        FootSkateFromContacts(**kw),
        FootSkateRatio(**kw),
        FootContactConsistency(**kw),
    ]
    report = {}
    print(
        f"{'clip':14s} {'TMR':>5s} {'rank':>4s} best other prompt   "
        f"{'skate':>5s} {'cons':>5s} {'path':>6s}  skill checks"
    )
    for i, c in enumerate(clips):
        d = data[c]
        pj = torch.from_numpy(d["posed_joints"]).float()
        q = {}
        for mt in metrics:
            r = mt(
                posed_joints=pj[None],
                foot_contacts=torch.from_numpy(d["foot_contacts"]).bool()[None],
                lengths=torch.tensor([len(pj)]),
            )
            q.update({k: float(v.mean()) for k, v in r.items() if hasattr(v, "mean")})
        spec = CLIPS[c]
        path_err = None
        if spec[1] is not None and np.any(spec[1]):
            xz, _ = root_path(*spec[1:])
            got = d["smooth_root_pos"][:, [0, 2]] - d["smooth_root_pos"][0, [0, 2]]
            path_err = float(np.linalg.norm(got - xz, axis=1).mean())
        rank = int((sim[i] > sim[i, i]).sum()) + 1
        other = int(np.argmax(np.where(np.arange(len(clips)) == i, -9, sim[i])))
        p = soma_points(d, names)
        res = tests(c, p, 1 / FPS)
        report[c] = {
            "prompt": spec[0],
            "tmr_similarity": float(sim01[i, i]),
            "tmr_rank": rank,
            "tmr_best_other": clips[other],
            "tmr_best_other_similarity": float(sim01[i, other]),
            **q,
            "root_path_error_m": path_err,
            "checks": [{"test": a, "passed": b, "measured": m_} for a, b, m_ in res],
        }
        fails = [a for a, b, _ in res if not b]
        print(
            f"{c:14s} {sim01[i, i]:5.2f} {rank:4d} {clips[other]:14s} {sim01[i, other]:4.2f} "
            f"{100 * q.get('foot_skate_from_pred_contacts', float('nan')):5.1f} "
            f"{q.get('foot_contact_consistency', float('nan')):5.2f} "
            f"{'' if path_err is None else f'{path_err:.2f}':>6s}  "
            + ("all pass" if not fails else "FAIL: " + ", ".join(fails))
        )
    print(
        "TMR: similarity to its own prompt (0-1) and that prompt's rank among all; skate: cm/s of"
    )
    print("the feet while Kimodo labels them in contact (its benchmark's Skate: 2-4 cm/s); cons:")
    print("contact consistency; path: mean distance in m of the hips from the root path asked for")
    if out:
        out.write_text(json.dumps(report, indent=1))


# ------------------------------------------------------------------------------- robot (repo env)


def robot_points(model, data, d: dict, clip: str) -> tuple[dict, float]:
    import mujoco  # noqa: PLC0415

    p_ = f"ref|{clip}|"
    pos, quat, q, dt = d[p_ + "pos"], d[p_ + "quat"], d[p_ + "q"], float(d[p_ + "dt"])
    adr = [model.jnt_qposadr[model.joint(str(n)).id] for n in d["joints"]]
    ids = {k: model.body(v).id for k, v in ROBOT.items()}
    pts = {k: [] for k in ids} | {"head_fwd": [], "heading": []}
    for i in range(len(pos)):
        data.qpos[:] = 0
        data.qpos[:3] = pos[i]
        data.qpos[3:7] = [quat[i, 3], *quat[i, :3]]
        for a, v in zip(adr, q[i], strict=True):
            data.qpos[a] = v
        mujoco.mj_kinematics(model, data)
        for k, b in ids.items():
            pts[k].append(data.xpos[b].copy())
        pts["head_fwd"].append(data.xmat[ids["head"]].reshape(3, 3)[:, 0].copy())
        f = data.xmat[ids["pelvis"]].reshape(3, 3)[:, 0]
        pts["heading"].append(np.arctan2(f[1], f[0]))
    pts = {k: np.array(v) for k, v in pts.items()}
    pts["heading"] = np.unwrap(pts["heading"])
    return pts, dt


def robot(npz: str, out: Path | None) -> None:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
    import mujoco  # noqa: PLC0415
    import render_robot as rr  # noqa: PLC0415

    model, data = rr.scene(False)
    d = np.load(npz)
    clips = sorted({k.split("|")[1] for k in d.files if k.startswith("ref|")})
    report = {}
    for c in clips:
        if c not in CLIPS:
            continue
        p, dt = robot_points(model, data, d, c)
        res = tests(c, p, dt)
        report[c] = {"checks": [{"test": a, "passed": b, "measured": m} for a, b, m in res]}
        fails = [f"{a} ({m})" for a, b, m in res if not b]
        print(f"{c:14s} " + ("all pass" if not fails else "FAIL: " + "; ".join(fails)))
    del mujoco
    if out:
        out.write_text(json.dumps(report, indent=1))


# ------------------------------------------------------------------------------- review renders

BONES = [
    ("Hips", "Spine1"), ("Spine1", "Spine2"), ("Spine2", "Chest"), ("Chest", "Neck1"),
    ("Neck1", "Head"),
    *[(f"{s}{a}", f"{s}{b}") for s in ("Left", "Right") for a, b in (
        ("Shoulder", "Arm"), ("Arm", "ForeArm"), ("ForeArm", "Hand"),
        ("Leg", "Shin"), ("Shin", "Foot"), ("Foot", "ToeBase"), ("ToeBase", "ToeEnd"))],
    ("Chest", "LeftShoulder"), ("Chest", "RightShoulder"),
    ("Hips", "LeftLeg"), ("Hips", "RightLeg"),
]  # fmt: skip


# each finger as a chain from the hand: thumb (3 joints) and four fingers (4)
FINGERS = [
    [f"{s}Hand"] + [f"{s}Hand{n}{i}" for i in range(1, k + 1)] + [f"{s}Hand{n}End"]
    for s in ("Left", "Right")
    for n, k in (("Thumb", 3), ("Index", 4), ("Middle", 4), ("Ring", 4), ("Pinky", 4))
]


def render(npz_dir: str, robot_npz: str, out_dir: str, only: list[str]) -> None:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
    import mujoco  # noqa: PLC0415
    import render_robot as rr  # noqa: PLC0415
    from PIL import Image, ImageDraw, ImageFont  # noqa: PLC0415

    W, H, AZ, EL, DIST = 360, 360, 215, -8, 2.7
    rr.W, rr.H = W, H
    model, data = rr.scene(True)
    renderer = mujoco.Renderer(model, H, W)
    font, small = ImageFont.truetype("DejaVuSans.ttf", 18), ImageFont.truetype("DejaVuSans.ttf", 11)
    names = json.loads(Path(npz_dir, "soma_names.json").read_text())
    d = np.load(robot_npz)
    clips = only or sorted({k.split("|")[1] for k in d.files if k.startswith("ref|")})
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    adr = [model.jnt_qposadr[model.joint(str(n)).id] for n in d["joints"]]
    # the camera's right and up vectors, to draw the human from the robot's viewpoint
    az, el = np.radians(AZ), np.radians(EL)
    fwd = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.cross(fwd, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    f = H / (2 * np.tan(np.radians(45 / 2)))  # MuJoCo's default 45° field of view
    for c in clips:
        hum = dict(np.load(Path(npz_dir) / f"{c}.npz"))
        hj = hum["posed_joints"][..., [2, 0, 1]]
        hj = hj - hj[0, names.index("Hips")] * [1, 1, 0]
        p_, ch = soma_points(hum, names), tests(c, soma_points(hum, names), 1 / FPS)
        rp, rdt = robot_points(model, data, d, c)
        rc = tests(c, rp, rdt)
        pos, quat, q = d[f"ref|{c}|pos"], d[f"ref|{c}|quat"], d[f"ref|{c}|q"]
        n = int(len(hj) / FPS * 15)
        frames = []
        for k in range(n):
            hi, ri = min(int(k / 15 * FPS), len(hj) - 1), min(int(k / 15 / rdt), len(pos) - 1)
            data.qpos[:] = 0
            data.qpos[:3] = pos[ri] - pos[0] * [1, 1, 0]
            data.qpos[3:7] = [quat[ri, 3], *quat[ri, :3]]
            for a, v in zip(adr, q[ri], strict=True):
                data.qpos[a] = v
            mujoco.mj_forward(model, data)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.75]
            cam.distance, cam.azimuth, cam.elevation = DIST, AZ, EL
            renderer.update_scene(data, cam)
            rimg = Image.fromarray(renderer.render())
            # the human, scaled to the robot's height and seen from the same camera
            s = standing_height(rp) / standing_height(p_)
            look = np.array(
                [
                    hj[hi, names.index("Hips"), 0] * s,
                    hj[hi, names.index("Hips"), 1] * s,
                    0.75 * 1.75 / 1.35,
                ]
            )
            himg = Image.new("RGB", (W, H), (236, 238, 242))
            dr = ImageDraw.Draw(himg)
            eye = look * [1, 1, 0] + [0, 0, 0.75] - fwd * DIST

            def proj(x, eye=eye, s=s):
                v = x * s - eye
                depth = v @ fwd
                return (W / 2 + f * (v @ right) / depth, H / 2 - f * (v @ up) / depth), depth

            def at(jn, hi=hi, hj=hj):
                return proj(hj[hi, names.index(jn)])[0]

            blue = (60, 110, 200)
            for a, b in BONES:
                dr.line([at(a), at(b)], fill=blue, width=4)
            for chain in FINGERS:
                dr.line([at(jn) for jn in chain], fill=blue, width=2)
            # the head: a disc round the middle of the skull, a dot where the eyes are
            top, base = hj[hi, names.index("HeadEnd")], hj[hi, names.index("Head")]
            (cx, cy), depth = proj((top + base) / 2)
            r = 0.6 * f * s * np.linalg.norm(top - base) / depth
            dr.ellipse([cx - r, cy - r, cx + r, cy + r], outline=blue, width=3)
            eyes = (hj[hi, names.index("LeftEye")] + hj[hi, names.index("RightEye")]) / 2
            ex, ey = proj(eyes)[0]
            dr.ellipse([ex - 3, ey - 3, ex + 3, ey + 3], fill=(200, 80, 60))
            both = Image.new("RGB", (2 * W, H + 120), (250, 250, 250))
            both.paste(himg, (0, 0))
            both.paste(rimg, (W, 0))
            dd = ImageDraw.Draw(both)
            dd.text((10, 8), f"{c.replace('_', ' ')}: Kimodo", fill=(25, 27, 31), font=font)
            dd.text((W + 10, 8), "robot reference", fill=(25, 27, 31), font=font)
            for col, res in ((0, ch), (W, rc)):
                for i, (label, ok, m) in enumerate(res[:7]):
                    dd.text(
                        (col + 10, H + 6 + 16 * i),
                        f"{'pass' if ok else 'FAIL'}  {label}: {m}"[:56],
                        fill=(40, 130, 60) if ok else (200, 50, 40),
                        font=small,
                    )
            frames.append(both)
        path = Path(out_dir) / f"{c}.webp"
        frames[0].save(
            path,
            save_all=True,
            append_images=frames[1:],
            duration=round(1000 / 15),
            loop=0,
            quality=60,
            method=6,
        )
        print(f"wrote {path}", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("human")
    h.add_argument("npz_dir", type=Path)
    h.add_argument("--out", type=Path)
    r = sub.add_parser("robot")
    r.add_argument("npz")
    r.add_argument("--out", type=Path)
    v = sub.add_parser("render")
    v.add_argument("npz_dir")
    v.add_argument("robot_npz")
    v.add_argument("out_dir")
    v.add_argument("clips", nargs="*")
    a = ap.parse_args()
    if a.cmd == "human":
        human(a.npz_dir, a.out)
    elif a.cmd == "robot":
        robot(a.npz, a.out)
    else:
        render(a.npz_dir, a.robot_npz, a.out_dir, a.clips)


if __name__ == "__main__":
    main()
