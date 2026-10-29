"""Render the README's robot picture: the bare robot and the robot with its clip-on covers, each
from the front, three-quarter, side and back, standing with the palms towards the thighs.

Kinematic only (no physics): the generated URDFs (tools/build_urdf.py) with the CAD meshes
(python -m cad.build) are loaded through sim/mujoco/check_standing.py, posed and placed on the
floor, and coloured in the finish colours: aluminium machined parts, graphite printed parts,
light covers, a light head shell with a glossy dark visor.

Usage (needs the sim extra and the meshes; MUJOCO_GL=egl renders without a display):
  python -m cad.build > /dev/null
  python tools/render_robot.py                 # writes docs/media/robot.webp
  python tools/render_robot.py --out other.webp
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim" / "mujoco"))
import check_standing as cs  # noqa: E402

W, H = 960, 1080
ALU = [0.78, 0.80, 0.83, 1]
GRAPHITE = [0.16, 0.17, 0.19, 1]
MOTOR = [0.30, 0.31, 0.34, 1]
COVER = [0.92, 0.93, 0.94, 1]
GLOSS = [0.05, 0.05, 0.06, 1]
DARK = [0.09, 0.09, 0.10, 1]
BOUGHT = [0.25, 0.25, 0.27, 1]
# standing, arms relaxed, palms turned in towards the thighs
POSE = {
    "hip_pitch": 8,
    "knee": 16,
    "ankle_pitch": 8,
    "shoulder_roll": 8,
    "elbow": 15,
    "wrist_yaw": 90,
}
VIEWS = {"front": 180, "three-quarter": 215, "side": 270, "back": 20}


def scene(covers: bool) -> tuple[mujoco.MjModel, mujoco.MjData]:
    cs.URDF = ROOT / "description" / "urdf" / f"marvin_bot{'_covers' if covers else ''}.urdf"
    spec, _ = cs.build()
    spec.visual.headlight.ambient = [0.25, 0.25, 0.27]
    spec.visual.headlight.diffuse = [0.30, 0.30, 0.30]
    spec.visual.quality.shadowsize = 4096
    spec.visual.global_.offwidth, spec.visual.global_.offheight = W, H
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
    light = dict(pos=[2, -2, 3], dir=[-1, 1, -1.5], diffuse=[0.45, 0.45, 0.45], castshadow=True)
    spec.worldbody.add_light(**light)
    spec.worldbody.add_light(
        pos=[-2, 2, 2.5], dir=[1, -1, -1], diffuse=[0.25, 0.25, 0.28], castshadow=False
    )
    model = spec.compile()
    colour(model)
    model.mat_specular[:] = 0.6
    model.mat_shininess[:] = 0.6
    return model, mujoco.MjData(model)


def colour(model: mujoco.MjModel) -> None:
    """Finish colours by part, from description/cad_model.yaml (process and material)."""
    cad = yaml.safe_load((ROOT / "description" / "cad_model.yaml").read_text())
    for g in range(model.ngeom):
        t = model.geom_type[g]
        if model.geom_contype[g] and t == mujoco.mjtGeom.mjGEOM_BOX:
            continue  # collision boxes are not drawn
        if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
            model.geom_rgba[g], model.geom_matid[g] = MOTOR, -1
        elif t == mujoco.mjtGeom.mjGEOM_MESH:
            name = model.mesh(model.geom_dataid[g]).name
            p = cad.get(name, {})
            if name == "head_visor":
                model.geom_rgba[g] = GLOSS
            elif name in ("head_shell", "neck_cover"):
                model.geom_rgba[g] = COVER
            elif name == "head_camera_frame":
                model.geom_rgba[g] = COVER
            elif name == "pelvis_cover" or "glute" in name:
                model.geom_rgba[g] = DARK
            elif p.get("process") == "cover":
                model.geom_rgba[g] = COVER
            elif p.get("process") in ("cnc", "service"):
                model.geom_rgba[g] = ALU
            elif p.get("material") == "tpu":
                model.geom_rgba[g] = [0.10, 0.10, 0.11, 1]
            elif p.get("process") == "bought":
                model.geom_rgba[g] = BOUGHT
            else:
                model.geom_rgba[g] = GRAPHITE
        elif t == mujoco.mjtGeom.mjGEOM_BOX:
            model.geom_rgba[g] = [0.22, 0.23, 0.26, 1]  # estimate boxes: computer, battery


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, deg: dict[str, float]) -> None:
    """Joint angles in degrees, by joint name or by name without the side prefix."""
    keys = cs.side_names([model.actuator(i).name for i in range(model.nu)])
    q = np.zeros(model.nu)
    for i in range(model.nu):
        name = model.actuator(i).name
        q[i] = np.radians(deg.get(name, deg.get(keys[name], 0.0)))
    cs.place_on_floor(model, data, q)


def label(img: np.ndarray, text: str) -> np.ndarray:
    im = Image.fromarray(img)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 30)
    except OSError:
        font = ImageFont.load_default()
    ImageDraw.Draw(im).text((24, 20), text, fill=(30, 32, 36), font=font)
    return np.asarray(im)


def sheet(covers: bool) -> np.ndarray:
    model, data = scene(covers)
    renderer = mujoco.Renderer(model, H, W)
    set_pose(model, data, POSE)
    imgs = []
    for az in VIEWS.values():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.62]
        cam.distance, cam.azimuth, cam.elevation = 2.5, az, -8
        renderer.update_scene(data, cam)
        imgs.append(renderer.render()[60:-120, 170:-170])
    renderer.close()
    return np.concatenate(imgs, axis=1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "media" / "robot.webp")
    args = ap.parse_args()
    both = np.concatenate(
        [label(sheet(False), "bare structure"), label(sheet(True), "with clip-on covers")], axis=0
    )
    img = Image.fromarray(both)
    img.resize((both.shape[1] // 2, both.shape[0] // 2)).save(args.out, quality=88)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
