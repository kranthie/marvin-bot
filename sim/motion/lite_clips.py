"""Lite's skills: upper-body clips from the motion pipeline, with a base path and column height
written for each, played on Lite's wheels in MuJoCo.

Lite has no legs, so a clip's legs and hips are dropped (decisions 2026-11-07): the waist, arms,
wrists and neck come from the retargeted clip (sim/motion/README.md: generated with Kimodo from
standing prompts and retargeted to Marvin Bot, whose upper body Lite shares); the base path and
the column height are keyframes below, joined by minimum-jerk curves, kept inside the base
controller's acceleration limits (sim/mujoco/check_lite.py) and the column's 38 mm/s.

Each clip is played on sim/mujoco/check_lite.py's model (the omni wheels' rollers, the motors'
torque limits): the joints follow the clip with their position servos and the wheels follow the
base path (feed-forward speed plus a correction on the base's pose). No learned policy: Lite
does not balance, so tracking is the servos' job. Per clip it reports how far the hands are from
where the clip puts them, the base's path error, the two lightest wheels' loads (both near zero is
the onset of tipping), and each joint's peak torque against its peak and RMS torque against its
rating.

Usage (the clips: the retargeted *_retargeted.npz files; ProtoMotions' MJCF gives their joint
order):
  python sim/motion/lite_clips.py run WORK/lite.npz --protomotions PM CLIPS_DIR [CLIPS_DIR ...]
  python sim/motion/lite_clips.py render WORK/lite.npz docs/media/skills/lite [CLIP ...]

Needs: pip install -e ".[sim]"
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim" / "mujoco"))
import check_lite as cl  # noqa: E402

STAND = 0.20  # column height (m) for the standing clips: shoulders at 1.21 m
SECONDS = 8.0
FPS = 15  # WebP frame rate
UPPER = ("waist_yaw", "neck_", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow")
UPPER += ("left_wrist", "right_wrist")
K_POS, K_YAW = 2.0, 2.0  # base pose correction, 1/s
STILL = [(0.0, 0.0, 0.0, 0.0)]
# name: (source clip, base keyframes (t s, x m, y m, heading deg), column keyframes (t s, m), kg
# in each hand)
CLIPS = {
    "idle": ("idle", STILL, [(0, STAND)], 0.0),
    "look_around": ("look_around", STILL, [(0, STAND)], 0.0),
    "point": ("point", STILL, [(0, STAND)], 0.0),
    "wave": ("wave", STILL, [(0, STAND)], 0.0),
    "wave_both": ("wave_both", STILL, [(0, STAND)], 0.0),
    "clap": ("clap", STILL, [(0, STAND)], 0.0),
    "arms_raise": ("arms_raise", STILL, [(0, STAND)], 0.0),
    "dance_sway": ("dance_sway", STILL, [(0, STAND)], 0.0),
    "hand_over": ("hand_over", STILL, [(0, STAND)], 0.0),
    # the column rises 0.10 m as the hands reach (5 s: under its 38 mm/s)
    "reach_forward": ("reach_forward", STILL, [(0, STAND), (1, STAND), (6.5, STAND + 0.10)], 0.0),
    # a box of 1 kg carried 1.2 m
    "carry": ("hold_box", [(0, 0, 0, 0), (1.5, 0, 0, 0), (6.5, 1.2, 0, 0)], [(0, STAND)], 0.5),
    "turn": ("idle", [(0, 0, 0, 0), (1, 0, 0, 0), (7, 0, 0, 180)], [(0, STAND)], 0.0),
    "sidestep": (
        "idle",
        [(0, 0, 0, 0), (0.5, 0, 0, 0), (4, 0, 0.6, 0), (7.5, 0, 0, 0)],
        [(0, STAND)],
        0.0,
    ),
    "wave_drive": ("wave", [(0, 0, 0, 0), (1, 0, 0, 0), (7, 1.0, 0, 0)], [(0, STAND)], 0.0),
    # the column from the bottom: 0.13 m in 7 s is what its 38 mm/s allows in a clip's 8 s
    "rise": ("idle", STILL, [(0, 0.0), (0.5, 0.0), (7.5, 0.13)], 0.0),
}


def minjerk(keys: list[tuple], t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Value, speed and acceleration through keyframes (t, v...), minimum jerk between them."""
    keys = [tuple(map(float, k)) for k in keys]
    n = len(keys[0]) - 1
    pos, vel, acc = (np.zeros((len(t), n)) for _ in range(3))
    for i, ti in enumerate(t):
        k = max([j for j, key in enumerate(keys) if key[0] <= ti], default=0)
        if k == len(keys) - 1:
            pos[i] = keys[k][1:]
            continue
        (t0, *a), (t1, *b) = keys[k], keys[k + 1]
        T = t1 - t0
        s = (ti - t0) / T
        d = np.subtract(b, a)
        pos[i] = np.add(a, d * (10 * s**3 - 15 * s**4 + 6 * s**5))
        vel[i] = d * (30 * s**2 - 60 * s**3 + 30 * s**4) / T
        acc[i] = d * (60 * s - 180 * s**2 + 120 * s**3) / T**2
    return pos, vel, acc


def find(dirs: list[Path], clip: str) -> Path:
    for d in dirs:
        for p in d.glob(f"*{clip}_keypoints_retargeted.npz"):
            if re.sub(r"^(soma_)?(proto_)?|_keypoints_retargeted\.npz$", "", p.name) == clip:
                return p
    raise SystemExit(f"no retargeted clip {clip} in {[str(d) for d in dirs]}")


def ref_qpos(model, base, column, q_named, adr) -> np.ndarray:
    """A full qpos: the base at (x, y, heading rad) on the floor, the column, named joints."""
    q = np.zeros(model.nq)
    x, y, h = base
    q[:7] = [x, y, 0.0, math.cos(h / 2), 0, 0, math.sin(h / 2)]
    q[adr["column_lift"]] = column
    for n, v in q_named.items():
        q[adr[n]] = v
    return q


def play(name: str, dirs: list[Path], order: list[str]) -> dict:
    src, base_keys, col_keys, load = CLIPS[name]
    d = np.load(find(dirs, src))
    q_clip, fps = d["joint_angles"], float(d["fps"])
    upper = [i for i, n in enumerate(order) if n.startswith(UPPER)]
    spec = cl.build(load)
    model = spec.compile()
    data = mujoco.MjData(model)
    ref = mujoco.MjData(model)
    adr = {model.joint(i).name: model.jnt_qposadr[i] for i in range(model.njnt)}
    act = {model.actuator(i).name: i for i in range(model.nu)}
    dt = model.opt.timestep
    steps = int(SECONDS / dt)
    t = np.arange(steps) * dt
    bpos, bvel, bacc = minjerk([(k[0], k[1], k[2], math.radians(k[3])) for k in base_keys], t)
    cpos, cvel, _ = minjerk(col_keys, t)

    def upper_at(ti):
        f = min(ti * fps, len(q_clip) - 1.001)
        i = int(f)
        q = q_clip[i] + (q_clip[i + 1] - q_clip[i]) * (f - i)
        return {order[j]: float(q[j]) for j in upper}

    q0 = ref_qpos(model, bpos[0], cpos[0, 0], upper_at(0.0), adr)
    data.qpos[:] = q0
    data.qpos[2] = 0.002
    mujoco.mj_forward(model, data)
    for n, v in upper_at(0.0).items():
        data.ctrl[act[n]] = v
    data.ctrl[act["column_lift"]] = cpos[0, 0]
    cl.run(model, data, 1.0)  # settle on the wheels before the clip starts
    every = round(1 / (30 * dt))
    rec = dict(sim=[], ref=[], loads=[], force=[], hands=[], path=[])
    names = [model.actuator(i).name for i in range(model.nu)]
    palms = [model.body(f"{s}_palm_link").id for s in ("left", "right")]
    origin = data.qpos[:2].copy()
    for k in range(steps):
        ti = t[k]
        qn = upper_at(ti)
        for n, v in qn.items():
            data.ctrl[act[n]] = v
        data.ctrl[act["column_lift"]] = cpos[k, 0]
        # base: the path's velocity plus a correction on the pose, into the base frame
        x, y = data.qpos[0] - origin[0], data.qpos[1] - origin[1]
        w, qz = data.qpos[3], data.qpos[6]
        h = 2 * math.atan2(qz, w)
        ex, ey = bpos[k, 0] - x, bpos[k, 1] - y
        eh = (bpos[k, 2] - h + math.pi) % (2 * math.pi) - math.pi
        vx, vy = bvel[k, 0] + K_POS * ex, bvel[k, 1] + K_POS * ey
        c, s = math.cos(h), math.sin(h)
        cl.drive(model, data, (c * vx + s * vy, -s * vx + c * vy, bvel[k, 2] + K_YAW * eh))
        mujoco.mj_step(model, data)
        if k % every == 0:
            ref.qpos[:] = ref_qpos(
                model,
                (bpos[k, 0] + origin[0], bpos[k, 1] + origin[1], bpos[k, 2]),
                cpos[k, 0],
                qn,
                adr,
            )
            mujoco.mj_kinematics(model, ref)
            rec["sim"].append(data.qpos.copy())
            rec["ref"].append(ref.qpos.copy())
            rec["loads"].append(cl.wheel_loads(model, data))
            rec["force"].append(data.actuator_force.copy())
            rec["hands"].append(
                np.mean([np.linalg.norm(data.xpos[b] - ref.xpos[b]) for b in palms])
            )
            rec["path"].append(math.hypot(ex, ey))
    share = model.body_subtreemass[model.body("lite_base").id] * cl.G / 4
    one, two = cl.lowest(rec["loads"], every * dt)
    force = np.array(rec["force"])
    acts = yaml.safe_load((cl.DESC / "actuators.yaml").read_text())
    models = cl._models()
    peak, heat = {}, {}
    for i, n in enumerate(names):
        if n.startswith("wheel_") or n == "column_lift":  # the column: check_lite.py
            continue
        a = acts[models[n]]
        lim, rated = a["peak_torque"], a["rated_torque"]
        peak[n] = float(np.abs(force[:, i]).max() / lim)
        heat[n] = float(np.sqrt((force[:, i] ** 2).mean()) / rated)
    return dict(
        sim=np.array(rec["sim"]),
        ref=np.array(rec["ref"]),
        hands=float(np.mean(rec["hands"])),
        path=float(np.max(rec["path"])),
        light=(one / share, two / share),
        peak=max(peak.items(), key=lambda kv: kv[1]),
        heat=max(heat.items(), key=lambda kv: kv[1]),
        accel=float(np.abs(bacc[:, :2]).max()),
        col_speed=float(np.abs(cvel).max()),
    )


def run(out: Path, dirs: list[Path], pm: Path, clips: list[str]) -> int:
    mjcf = pm / "protomotions/data/assets/mjcf/marvin_bot.xml"
    order = [n for n in re.findall(r'<joint name="([^"]+)"', mjcf.read_text()) if n != "root"]
    saved, bad = {}, []
    print(
        f"{'clip':14} {'hands':>6} {'path':>6} {'wheels':>10} {'accel':>6} {'column':>7}  "
        "peak torque / RMS"
    )
    for name in clips or CLIPS:
        r = play(name, dirs, order)
        saved[f"sim|{name}"] = r["sim"]
        saved[f"ref|{name}"] = r["ref"]
        one, two = r["light"]
        pk, ht = r["peak"], r["heat"]
        print(
            f"{name:14} {r['hands'] * 100:4.1f}cm {r['path'] * 100:4.1f}cm "
            f"{one:4.0%} {two:4.0%} {r['accel']:4.2f}  {r['col_speed'] * 1000:4.0f}mm/s  "
            f"{pk[0]} {pk[1]:.0%} / {ht[0]} {ht[1]:.0%}",
            flush=True,
        )
        if two < cl.MIN_WHEEL_LOAD:
            bad.append(f"{name}: two wheels unload ({two:.0%})")
        if pk[1] > 1.0 or ht[1] > 1.0:
            bad.append(f"{name}: {pk[0]} at {pk[1]:.0%} of peak or {ht[0]} at {ht[1]:.0%} rated")
        if r["col_speed"] > 0.038:
            bad.append(f"{name}: column at {r['col_speed'] * 1000:.0f} mm/s, over 38")
    np.savez_compressed(out, fps=30.0, **saved)
    print(f"wrote {out}")
    for b in bad:
        print(f"FAIL: {b}")
    return 1 if bad else 0


def render(npz: Path, out_dir: Path, clips: list[str]) -> None:
    sys.path.insert(0, str(ROOT / "tools"))
    import render_lite as rl  # noqa: PLC0415
    from PIL import Image, ImageDraw, ImageFont  # noqa: PLC0415

    W, H = 640, 480
    model, data = rl.scene(True, size=(W, H))
    floor = model.geom("floor").id
    model.geom_size[floor, :2] = 40.0
    model.mat_texrepeat[model.geom_matid[floor]] *= 40.0 / 6
    renderer = mujoco.Renderer(model, H, W)
    font = ImageFont.truetype("DejaVuSans.ttf", 22)
    d = np.load(npz)
    out_dir.mkdir(parents=True, exist_ok=True)
    names = [k.split("|")[1] for k in d.files if k.startswith("sim|")]
    for clip in clips or names:
        sim = d[f"sim|{clip}"]
        frames = []
        for k in range(0, len(sim), round(30 / FPS)):
            data.qpos[:] = sim[k]
            mujoco.mj_forward(model, data)
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.75]
            cam.distance, cam.azimuth, cam.elevation = 3.2, 215, -8
            renderer.update_scene(data, cam)
            if CLIPS[clip][3]:  # the box carried, between the palms
                scn = renderer.scene
                p = [data.xpos[model.body(f"{s}_palm_link").id] for s in ("left", "right")]
                h = 2 * math.atan2(data.qpos[6], data.qpos[3])
                rot = np.array(
                    [[math.cos(h), -math.sin(h), 0], [math.sin(h), math.cos(h), 0], [0, 0, 1]]
                )
                half = [0.11, max(0.05, np.linalg.norm(p[0] - p[1]) / 2 - 0.03), 0.09]
                mujoco.mjv_initGeom(
                    scn.geoms[scn.ngeom],
                    mujoco.mjtGeom.mjGEOM_BOX,
                    half,
                    (p[0] + p[1]) / 2 + [0, 0, -0.06],
                    rot.ravel(),
                    [0.72, 0.56, 0.36, 1],
                )
                scn.ngeom += 1
            im = Image.fromarray(renderer.render())
            ImageDraw.Draw(im).text((14, 10), clip.replace("_", " "), fill=(25, 27, 31), font=font)
            frames.append(im.resize((480, 360), Image.LANCZOS))
        path = out_dir / f"{clip}.webp"
        frames[0].save(
            path,
            save_all=True,
            append_images=frames[1:],
            duration=round(1000 / FPS),
            loop=0,
            quality=60,
            method=6,
        )
        print(f"wrote {path} ({path.stat().st_size // 1024} kB)", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("out", type=Path)
    r.add_argument("dirs", type=Path, nargs="+")
    r.add_argument("--protomotions", type=Path, required=True)
    r.add_argument("--clips", nargs="*", default=[])
    v = sub.add_parser("render")
    v.add_argument("npz", type=Path)
    v.add_argument("out_dir", type=Path)
    v.add_argument("clips", nargs="*")
    a = ap.parse_args()
    if a.cmd == "run":
        return run(a.out, a.dirs, a.protomotions, a.clips)
    render(a.npz, a.out_dir, a.clips)
    return 0


if __name__ == "__main__":
    sys.exit(main())
