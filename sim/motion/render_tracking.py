"""Render motion-tracking clips for the docs: the trained tracker (solid) with its reference
motion as a translucent blue ghost in the same scene, one animated WebP per clip.

Two steps, in two environments:

  export (ProtoMotions environment, from its checkout, which has Marvin Bot's MJCF): reads the
      rollout a training run wrote (results/<run>/results/predicted_motion_lib_epoch_*.pt) and
      the packaged reference motions it was trained on, and writes joint angles and base poses
      to one .npz;
  render (this repo's environment, with the sim extra and the CAD meshes; MUJOCO_GL=egl without
      a display): draws each clip with the covered robot (tools/render_robot.py) and writes
      <out dir>/<clip>.webp.

Usage:
  python sim/motion/render_tracking.py export <predicted_motion_lib.pt> <motions.pt> <out.npz>
  python sim/motion/render_tracking.py render <motions.npz> <out dir> [clip ...]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent
W, H = 640, 480  # rendered; the WebP is scaled to 480 x 360
FPS = 15  # WebP frame rate (the references are 30 fps)


def clip_name(path: str) -> str:
    name = Path(path).name.split("__")[0]
    name = re.sub(r"(_keypoints)?(_retargeted)?(\.motion)?$", "", name)
    return re.sub(r"^(soma_)?proto_", "", name)


def export(pred: str, ref: str, out: str) -> None:
    import torch  # noqa: PLC0415

    xml = Path("protomotions/data/assets/mjcf/marvin_bot.xml").read_text()
    joints = [n for n in re.findall(r'<joint name="([^"]+)"', xml) if n != "root"]
    data = {}
    for kind, path in (("ref", ref), ("track", pred)):
        lib = torch.load(path, weights_only=False, map_location="cpu")
        for i, f in enumerate(lib["motion_files"]):
            s, n = int(lib["length_starts"][i]), int(lib["motion_num_frames"][i])
            key = f"{kind}|{clip_name(f)}|"
            data[key + "pos"] = lib["gts"][s : s + n, 0].numpy()
            data[key + "quat"] = lib["grs"][s : s + n, 0].numpy()  # x, y, z, w
            data[key + "q"] = lib["dps"][s : s + n].numpy()
            data[key + "dt"] = np.array(float(lib["motion_dt"][i]))
    np.savez(out, joints=np.array(joints), **data)
    print(f"wrote {out}: {sorted({k.split('|')[1] for k in data})}")


def render(npz: str, out_dir: str, clips: list[str]) -> None:
    sys.path.insert(0, str(ROOT / "tools"))
    import mujoco  # noqa: PLC0415
    import render_robot as rr  # noqa: PLC0415
    from PIL import Image, ImageDraw, ImageFont  # noqa: PLC0415

    rr.W, rr.H = W, H
    model, data = rr.scene(True)
    floor = model.geom("floor").id
    scale = 40.0 / model.geom_size[floor, 0]  # a large floor: the clips travel several metres
    model.geom_size[floor, :2] = 40.0
    model.mat_texrepeat[model.geom_matid[floor]] *= scale
    ghost = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, H, W)
    opt, pert = mujoco.MjvOption(), mujoco.MjvPerturb()
    font = ImageFont.truetype("DejaVuSans.ttf", 22)

    d = np.load(npz)
    adr = [model.jnt_qposadr[model.joint(str(n)).id] for n in d["joints"]]
    names = sorted({k.split("|")[1] for k in d.files if k.startswith("track|")})
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    def series(kind, clip):
        p = f"{kind}|{clip}|"
        return d[p + "pos"], d[p + "quat"], d[p + "q"], float(d[p + "dt"])

    def pose(dd, pos, quat, q, origin):
        dd.qpos[:] = 0
        dd.qpos[:3] = pos - origin
        dd.qpos[3:7] = [quat[3], quat[0], quat[1], quat[2]]
        for a, v in zip(adr, q, strict=True):
            dd.qpos[a] = v
        mujoco.mj_forward(model, dd)

    for clip in clips or names:
        rp, rq, rj, rdt = series("ref", clip)
        tp, tq, tj, tdt = series("track", clip)
        n = int(len(rp) * rdt * FPS)
        ts = np.arange(n) / FPS
        ri = np.minimum((ts / rdt).round().astype(int), len(rp) - 1)
        ti = np.minimum((ts / tdt).round().astype(int), len(tp) - 1)
        frames = []
        for k in range(n):
            # both start at the origin, so the ghost shows where the robot should be
            pose(data, tp[ti[k]], tq[ti[k]], tj[ti[k]], tp[0] * [1, 1, 0])
            pose(ghost, rp[ri[k]], rq[ri[k]], rj[ri[k]], rp[0] * [1, 1, 0])
            cam = mujoco.MjvCamera()
            mid = (data.qpos[:2] + ghost.qpos[:2]) / 2
            cam.lookat[:] = [mid[0], mid[1], 0.75]
            cam.distance, cam.azimuth, cam.elevation = 2.7, 215, -8
            renderer.update_scene(data, cam)
            scn = renderer.scene
            first = scn.ngeom
            mujoco.mjv_addGeoms(model, ghost, opt, pert, mujoco.mjtCatBit.mjCAT_DYNAMIC, scn)
            for g in range(first, scn.ngeom):
                scn.geoms[g].rgba[:] = [0.35, 0.6, 1.0, 0.28]  # the reference: translucent blue
            im = Image.fromarray(renderer.render())
            ImageDraw.Draw(im).text((14, 10), clip.replace("_", " "), fill=(25, 27, 31), font=font)
            frames.append(im.resize((480, 360), Image.LANCZOS))
        path = Path(out_dir) / f"{clip}.webp"
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


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export")
    e.add_argument("pred")
    e.add_argument("ref")
    e.add_argument("out")
    r = sub.add_parser("render")
    r.add_argument("npz")
    r.add_argument("out_dir")
    r.add_argument("clips", nargs="*")
    args = ap.parse_args()
    if args.cmd == "export":
        export(args.pred, args.ref, args.out)
    else:
        render(args.npz, args.out_dir, args.clips)


if __name__ == "__main__":
    main()
