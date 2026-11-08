"""Render Lite: the robot with its column down and up, its base in detail, and a short animation.

Kinematic only (no physics): the generated Lite URDF (tools/build_urdf.py) with the CAD meshes
(python -m cad.build), loaded through sim/mujoco/check_lite.py (its rollers and collision boxes
hidden), posed and coloured in the finish colours of tools/render_robot.py, with the base's own:
light printed shell, dark bumper and covers, blue light strip, dark wheels.

Usage (needs the sim extra and the meshes; MUJOCO_GL=egl renders without a display):
  python -m cad.build > /dev/null
  python tools/render_lite.py             # docs/media/lite.webp, lite_base.webp, lite_motion.webp
  python tools/render_lite.py --only robot
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim" / "mujoco"))
sys.path.insert(0, str(ROOT / "tools"))
import check_lite as cl  # noqa: E402
import render_robot as rr  # noqa: E402

MEDIA = ROOT / "docs" / "media"
W, H = 960, 1080
POSE = {"shoulder_roll": 8, "elbow": 15, "wrist_yaw": 90}
SHELL_PARTS = ("shell_", "bumper", "light_diffuser", "lidar_window", "deck_cover", "column_trim")
BASE = {  # Lite's own parts, by name prefix
    "shell_": rr.COVER,
    "column_head_shroud": rr.COVER,
    "bumper": [0.08, 0.08, 0.09, 1],
    "deck_cover": [0.14, 0.15, 0.17, 1],
    "column_trim": [0.14, 0.15, 0.17, 1],
    "light_diffuser": [0.25, 0.62, 0.90, 1],
    "lidar_window": rr.GLOSS,
    "pod_wheel": [0.10, 0.10, 0.11, 1],
    "pod_motor": rr.MOTOR,
    "desk_column": [0.74, 0.75, 0.77, 1],
    "estop": [0.80, 0.08, 0.06, 1],
    "battery": [0.20, 0.36, 0.55, 1],
    "elec": [0.10, 0.35, 0.18, 1],
    "lidar": [0.12, 0.12, 0.13, 1],
    "charge": [0.80, 0.62, 0.30, 1],
    "screws": [0.25, 0.25, 0.27, 1],
}


def scene(covers: bool = True, size=(W, H)) -> tuple[mujoco.MjModel, mujoco.MjData]:
    cl.URDF = ROOT / "description" / "urdf" / f"marvin_bot_lite{'_covers' if covers else ''}.urdf"
    spec = cl.build()
    if covers:
        rr.flat_shade(spec, "torso_cover")
    spec.visual.headlight.ambient = [0.25, 0.25, 0.27]
    spec.visual.headlight.diffuse = [0.30, 0.30, 0.30]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth, spec.visual.global_.offheight = size
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.80, 0.82, 0.86],
        rgb2=[0.45, 0.49, 0.55],
        width=512,
        height=3072,
    )
    spec.add_texture(
        name="grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.52, 0.54, 0.57],
        rgb2=[0.44, 0.46, 0.49],
        width=512,
        height=512,
    )
    mat = spec.add_material(name="grid", texrepeat=[6, 6], reflectance=0.0)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    spec.geom("floor").material = "grid"
    spec.worldbody.add_light(
        pos=[2, -2, 3], dir=[-1, 1, -1.5], diffuse=[0.45, 0.45, 0.45], castshadow=True
    )
    spec.worldbody.add_light(
        pos=[-2, 2, 2.5], dir=[1, -1, -1], diffuse=[0.25, 0.25, 0.28], castshadow=False
    )
    model = spec.compile()
    colour(model)
    model.mat_specular[:] = 0.6
    model.mat_shininess[:] = 0.6
    return model, mujoco.MjData(model)


def colour(model: mujoco.MjModel) -> None:
    rr.colour(model)
    cad = yaml.safe_load((ROOT / "description" / "cad_model_lite.yaml").read_text())
    for g in range(model.ngeom):
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        name = model.mesh(model.geom_dataid[g]).name
        if name not in cad:
            continue
        hit = next((c for k, c in BASE.items() if name.startswith(k)), None)
        if hit is not None:
            model.geom_rgba[g] = hit
        elif cad[name]["process"] in ("cnc", "service"):
            model.geom_rgba[g] = rr.ALU
        elif cad[name]["material"] == "steel":
            model.geom_rgba[g] = [0.58, 0.59, 0.62, 1]
        else:
            model.geom_rgba[g] = rr.GRAPHITE


def hide(model: mujoco.MjModel, prefixes) -> None:
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            if model.mesh(model.geom_dataid[g]).name.startswith(tuple(prefixes)):
                model.geom_rgba[g, 3] = 0


def pose(model, data, column: float, deg: dict, base=(0.0, 0.0, 0.0)) -> None:
    """Joint angles in degrees (by name or without the side prefix), the column (m), and the
    base's (x, y, heading in degrees)."""
    mujoco.mj_resetData(model, data)
    keys = cl.side_names([model.joint(i).name for i in range(model.njnt)])
    for i in range(model.njnt):
        j = model.joint(i)
        if j.name == "root":
            continue
        if j.name == "column_lift":
            v = column
        elif j.name in keys and model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            v = math.radians(deg.get(j.name, deg.get(keys[j.name], 0.0)))
        else:
            continue
        data.qpos[model.jnt_qposadr[i]] = v
    x, y, h = base
    data.qpos[:3] = [x, y, 0.0]
    data.qpos[3:7] = [math.cos(math.radians(h) / 2), 0, 0, math.sin(math.radians(h) / 2)]
    mujoco.mj_forward(model, data)


def shot(model, data, az, el, dist, look, size=(W, H)) -> np.ndarray:
    r = mujoco.Renderer(model, size[1], size[0])
    cam = mujoco.MjvCamera()
    cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = look, dist, az, el
    r.update_scene(data, cam)
    img = r.render()
    r.close()
    return img


def robot_sheet() -> np.ndarray:
    model, data = scene()
    row = []
    for column, text in ((0.0, "column down"), (0.45, "column up")):
        pose(model, data, column, POSE)
        for az in (215, 270):
            img = shot(model, data, az, -6, 3.1, [0, 0, 0.82])[40:-60, 200:-200]
            row.append(rr.label(img, text) if az == 215 else img)
    return np.concatenate(row, axis=1)


def base_sheet() -> np.ndarray:
    size = (900, 760)
    tiles = []
    for text, hidden, az, el, dist, look in (
        (
            "the base",
            ("desk_column",),
            215,
            -22,
            1.35,
            [0, 0, 0.12],
        ),
        (
            "shell off",
            SHELL_PARTS + ("desk_column",),
            215,
            -28,
            1.25,
            [0, 0, 0.10],
        ),
        (
            "deck off",
            SHELL_PARTS
            + (
                "deck_plate",
                "column_collar",
                "estop",
                "screws_frame",
                "desk_column",
                "column_",
                "torso",
                "neck",
                "head",
                "left_",
                "right_",
            ),
            160,
            -50,
            1.15,
            [0, 0, 0.07],
        ),
        ("wheel pod", (), 105, -12, 0.55, None),
    ):
        model, data = scene(size=size)
        pose(model, data, 0.0, POSE)
        if text == "wheel pod":
            keep = ("pod_", "screws_pod")
            for g in range(model.ngeom):
                if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                    n = model.mesh(model.geom_dataid[g]).name
                    if not (n.startswith(keep) and n.endswith("_315")):
                        model.geom_rgba[g, 3] = 0
                elif model.geom_bodyid[g]:
                    model.geom_rgba[g, 3] = 0
            a = math.radians(315)
            look = [0.233 * math.cos(a), 0.233 * math.sin(a), 0.09]
        else:
            hide(model, hidden)
            for g in range(model.ngeom):  # motor housings drawn as cylinders on the upper body
                if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    if not model.body(model.geom_bodyid[g]).name.startswith(
                        ("lite_base", "wheel_")
                    ):
                        model.geom_rgba[g, 3] = 0
        tiles.append(rr.label(shot(model, data, az, el, dist, look, size), text))
    top = np.concatenate(tiles[:2], axis=1)
    bottom = np.concatenate(tiles[2:], axis=1)
    return np.concatenate([top, bottom], axis=0)


def motion(frames: int = 72) -> list[Image.Image]:
    """The column rising and falling while the waist turns and the base drives an arc."""
    size = (720, 900)
    model, data = scene(size=size)
    out = []
    for k in range(frames):
        t = k / frames
        s = 0.5 - 0.5 * math.cos(2 * math.pi * t)
        deg = dict(
            POSE, waist_yaw=50 * math.sin(2 * math.pi * t), shoulder_pitch=40 * s, elbow=15 + 50 * s
        )
        heading = 40 * math.sin(2 * math.pi * t)
        pose(model, data, 0.45 * s, deg, (0.4 * math.sin(2 * math.pi * t), 0.0, heading))
        img = shot(model, data, 215, -6, 3.4, [0.0, 0, 0.85], size)
        out.append(Image.fromarray(img).resize((size[0] // 2, size[1] // 2)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--only", choices=["robot", "base", "motion"])
    args = ap.parse_args()
    if args.only in (None, "robot"):
        img = robot_sheet()
        Image.fromarray(img).resize((img.shape[1] // 2, img.shape[0] // 2)).save(
            MEDIA / "lite.webp", quality=88
        )
        print(f"wrote {MEDIA / 'lite.webp'}")
    if args.only in (None, "base"):
        img = base_sheet()
        Image.fromarray(img).resize((img.shape[1] * 2 // 3, img.shape[0] * 2 // 3)).save(
            MEDIA / "lite_base.webp", quality=88
        )
        print(f"wrote {MEDIA / 'lite_base.webp'}")
    if args.only in (None, "motion"):
        fr = motion()
        fr[0].save(
            MEDIA / "lite_motion.webp",
            save_all=True,
            append_images=fr[1:],
            duration=60,
            loop=0,
            quality=80,
        )
        print(f"wrote {MEDIA / 'lite_motion.webp'}")


if __name__ == "__main__":
    main()
