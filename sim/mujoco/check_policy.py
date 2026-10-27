"""Run an exported walking policy in plain MuJoCo: stand, walk, turn and stop, or climb stairs.

Loads policy.onnx and policy.json from sim/isaaclab/export_policy.py, builds the URDF with a free
base and the same DC motor models, gains, armature and friction as
sim/isaaclab/marvin_bot/robot_cfg.py (PD clipped by the torque-speed line), and drives it at the
policy's control rate through a command script. Fails if the robot falls, if it doesn't track the
walk, turn and stop commands, if it slides its feet instead of stepping, or if any joint's RMS
torque over a segment exceeds its continuous rating (the steady-state heating, from
description/actuators.yaml). Prints the step rate,
the peak torque, the posture (pelvis height above the floor, knee bend), the left/right gait
(swing, stance, step length, foot lift) and the side-to-side sway (head and pelvis sway, pelvis
and torso roll, step width) per segment.

Sensor noise at half the training level is added to the observations (--noise, --seed).
--fast V adds a walk at V m/s before the stop. --stairs H replaces the script with a walk at
0.4 m/s over 5 steps up and 5 down, H m each, 28 cm treads, and fails unless the robot gets to
the top and down the other side.

Usage:
  python sim/mujoco/check_policy.py <dir with policy.onnx and policy.json>
  python sim/mujoco/check_policy.py <dir> --fast 1.0
  python sim/mujoco/check_policy.py <dir> --stairs 0.15
  MUJOCO_GL=egl python sim/mujoco/check_policy.py <dir> --render walk.mp4  # + imageio[ffmpeg]

Needs: pip install -e ".[sim]"
"""

import argparse
import json
import re
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_standing import ROOT, URDF, place_on_floor  # noqa: E402
from joint_specs import joint_specs  # noqa: E402

PHYSICS_DT = 0.002
AIR_MIN = 0.04  # s
GROUND_GROUP = 2  # geom group of the floor and stairs, the only geoms the height map sees
# terrain-map errors at full noise, as in sim/isaaclab/marvin_bot/tasks/observations.py
MAP_BIAS, MAP_DROPOUT, MAP_NOISE = 0.02, 0.1, 0.01
# training observation noise (marvin_bot/tasks/velocity_env_cfg.py). A real robot's sensors are
# never noise-free; without any, a mirror-symmetric policy standing in a mirror-symmetric pose
# has nothing to pick the first foot with.
SENSOR_NOISE = {"base_ang_vel": 0.2, "projected_gravity": 0.05, "joint_pos": 0.01, "joint_vel": 0.5}
# (name, seconds, (vx m/s, vy m/s, yaw rate rad/s))
SCRIPT = [
    ("stand", 2.0, (0.0, 0.0, 0.0)),
    ("walk", 6.0, (0.5, 0.0, 0.0)),
    ("turn", 4.0, (0.0, 0.0, 0.8)),
    ("walk and turn", 5.0, (0.5, 0.0, -0.5)),
    ("stop", 4.0, (0.0, 0.0, 0.0)),
]
STAIRS = dict(start=1.2, steps=5, tread=0.28, platform=1.2)  # m
STAIRS_SCRIPT = [
    ("stand", 2.0, (0.0, 0.0, 0.0)),
    ("stairs", 30.0, (0.4, 0.0, 0.0)),
    ("stop", 3.0, (0.0, 0.0, 0.0)),
]
_ROBOT = yaml.safe_load((ROOT / "description" / "robot.yaml").read_text())
STANCE_Z, STANCE_KNEE = _ROBOT["stance_pelvis_height"], _ROBOT["stance"]["knee"]
FALL_HEIGHT = 0.35  # m, pelvis above the ground under it, as the training termination


def sideways(xy: np.ndarray, side: np.ndarray, half: int) -> float:
    """5-95 percentile range of a point's offset across the heading from its moving mean over
    2 * half samples, in metres."""
    mean = np.array([xy[max(0, i - half) : i + half].mean(0) for i in range(len(xy))])
    r = np.sum((xy - mean) * side, axis=1)[half:-half]
    return float(np.ptp(np.percentile(r, [5, 95]))) if len(r) else float("nan")


def build(spec_joints: dict, stairs_height: float | None = None) -> mujoco.MjModel:
    # keep the URDF visuals (the drawn parts) and the floor material, as
    # check_standing.py does; collision boxes go to group 3, hidden in the video
    urdf = re.sub(
        r"(<robot [^>]*>)",
        rf'\1<mujoco><compiler discardvisual="false" meshdir="{URDF.parent}"/></mujoco>',
        URDF.read_text(),
        count=1,
    )
    spec = mujoco.MjSpec.from_string(urdf)
    for g in spec.geoms:
        if g.contype:
            g.group = 3
    spec.option.timestep = PHYSICS_DT
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 960, 540  # --render size
    # 10 cm checker floor, so walking speed is visible in the video
    spec.add_texture(
        name="grid",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        rgb1=[0.82, 0.82, 0.82],
        rgb2=[0.62, 0.62, 0.62],
        width=256,
        height=256,
    )
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.75, 0.85, 0.95],
        rgb2=[0.95, 0.95, 0.95],
        width=256,
        height=256,
    )
    grid = spec.add_material(name="grid", texrepeat=[5, 5], texuniform=True)
    grid.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[20, 20, 0.05],
        friction=[1.0, 0.005, 1e-4],
        material="grid",
        group=GROUND_GROUP,
    )
    spec.worldbody.add_light(
        pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
    )
    if stairs_height:
        # a pyramid: box i reaches the top of step i + 1 and spans to the matching step down
        x0, n, t, p = STAIRS["start"], STAIRS["steps"], STAIRS["tread"], STAIRS["platform"]
        for i in range(n):
            lo, hi, top = x0 + i * t, x0 + (2 * n - i) * t + p, (i + 1) * stairs_height
            spec.worldbody.add_geom(
                name=f"step_{i}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[(hi - lo) / 2, 1.0, top / 2],
                pos=[(hi + lo) / 2, 0.0, top / 2],
                rgba=[0.55, 0.6, 0.7, 1.0],
                friction=[1.0, 0.005, 1e-4],
                group=GROUND_GROUP,
            )
    spec.body("pelvis").add_freejoint(name="root")
    for j in spec.joints:
        if j.name not in spec_joints:
            continue
        s = spec_joints[j.name]
        j.armature = s["armature"]
        j.frictionloss = s["friction"]
        act = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.ctrlrange = [-s["effort"], s["effort"]]
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec.compile()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("policy_dir", type=Path)
    ap.add_argument("--render", type=Path, help="save an MP4 of the run")
    ap.add_argument("--fast", type=float, help="add a walk at this speed, m/s")
    ap.add_argument("--stairs", type=float, help="walk over stairs with this step height, m")
    ap.add_argument(
        "--noise",
        type=float,
        default=0.5,
        help="sensor noise as a fraction of the training noise (0 for none)",
    )
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--wrist-yaw",
        type=float,
        default=0.0,
        help="hold the wrist yaw joints here, deg (90: palms towards the thighs)",
    )
    args = ap.parse_args()
    script = list(STAIRS_SCRIPT if args.stairs else SCRIPT)
    if args.fast and not args.stairs:
        script.insert(-1, ("fast walk", 6.0, (args.fast, 0.0, 0.0)))

    meta = json.loads((args.policy_dir / "policy.json").read_text())
    # policies exported before the hands (2026-10-27) name the wrist pitch joint "gripper"
    for key in ("joint_names", "action_joint_names"):
        meta[key] = [n.replace("_gripper", "_wrist_pitch") for n in meta[key]]
    rng = np.random.default_rng(args.seed)
    session = ort.InferenceSession(str(args.policy_dir / "policy.onnx"))
    in_name = session.get_inputs()[0].name

    spec_joints = joint_specs()
    model = build(spec_joints, args.stairs)
    data = mujoco.MjData(model)

    # policy joint order (from the training backend) -> MuJoCo addresses
    names = meta["joint_names"]
    qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in names])
    vadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in names])
    aid = np.array([model.actuator(n).id for n in names])
    default = np.array(meta["default_joint_pos"])
    act_idx = np.array([names.index(n) for n in meta["action_joint_names"]])
    scale = meta["action_scale"]
    kp = np.array([spec_joints[n]["kp"] for n in names])
    kd = np.array([spec_joints[n]["kd"] for n in names])
    rated = np.array([spec_joints[n]["rated"] for n in names])
    effort = np.array([spec_joints[n]["effort"] for n in names])
    speed = np.array([spec_joints[n]["speed"] for n in names])

    # the joints the policy does not drive are held at the default pose, or at --wrist-yaw
    hold = default.copy()
    hold[[i for i, n in enumerate(names) if n.endswith("_wrist_yaw")]] = np.radians(args.wrist_yaw)
    q_act = np.zeros(model.nu)
    q_act[aid] = hold
    place_on_floor(model, data, q_act)
    substeps = round(meta["control_dt"] / PHYSICS_DT)
    last_action = np.zeros(len(act_idx))
    target = hold.copy()

    # rough-ground policies: the terrain map around the feet, ray-cast like Isaac Lab's height
    # scanner (yaw-aligned grid, rows of constant y), refreshed at the map rate
    hs = meta.get("height_scan")
    scan, scan_every, map_bias = None, 1, 0.0
    if hs:
        xs = hs["center"][0] + np.arange(
            -hs["size"][0] / 2, hs["size"][0] / 2 + 1e-9, hs["resolution"]
        )
        ys = hs["center"][1] + np.arange(
            -hs["size"][1] / 2, hs["size"][1] / 2 + 1e-9, hs["resolution"]
        )
        gx, gy = np.meshgrid(xs, ys, indexing="xy")
        grid = np.stack([gx.ravel(), gy.ravel()], axis=1)
        scan_every = max(1, round(hs["update_period"] / meta["control_dt"]))
        map_bias = args.noise * MAP_BIAS * rng.uniform(-1, 1)
        groups = np.zeros(6, dtype=np.uint8)
        groups[GROUND_GROUP] = 1
        hit = np.zeros(1, dtype=np.int32)

    def height_map() -> np.ndarray:
        rot = data.xmat[model.body("pelvis").id].reshape(3, 3)
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        c, s_ = np.cos(yaw), np.sin(yaw)
        base = data.qpos[:3]
        out = np.zeros(len(grid))
        for i, (gx_, gy_) in enumerate(grid):
            start = np.array(
                [base[0] + c * gx_ - s_ * gy_, base[1] + s_ * gx_ + c * gy_, base[2] + 20]
            )
            d = mujoco.mj_ray(model, data, start, np.array([0.0, 0.0, -1.0]), groups, 1, -1, hit)
            ground = start[2] - d if d >= 0 else 0.0
            out[i] = base[2] - ground - hs["offset"]
        out += map_bias + args.noise * MAP_NOISE * rng.uniform(-1, 1, len(out))
        out[rng.uniform(size=len(out)) < args.noise * MAP_DROPOUT] = 0.0
        return np.clip(out, *hs["clip"])

    def observe(cmd: np.ndarray, step: int) -> np.ndarray:
        nonlocal scan
        rot = data.xmat[model.body("pelvis").id].reshape(3, 3)
        if hs and (scan is None or step % scan_every == 0):
            scan = height_map()
        parts = {
            "base_ang_vel": data.qvel[3:6],  # free joint angular velocity is in the body frame
            "projected_gravity": rot.T @ np.array([0.0, 0.0, -1.0]),
            "velocity_commands": cmd,
            "joint_pos": data.qpos[qadr] - default,
            "joint_vel": data.qvel[vadr],
            "actions": last_action,
        }
        for key, amp in SENSOR_NOISE.items():  # uniform, like the training corruption
            parts[key] = parts[key] + args.noise * amp * rng.uniform(-1, 1, np.shape(parts[key]))
        if hs:
            parts["height_scan"] = scan
        obs = [np.asarray(parts[o["name"]], dtype=np.float32) for o in meta["observations"]]
        return np.concatenate(obs)[None, :]

    world = {g for g in range(model.ngeom) if model.geom_bodyid[g] == 0}  # floor and stairs
    torso = model.body("torso_link").id
    head = model.body("head_link").id
    feet = [
        next(
            g
            for g in range(model.ngeom)
            if model.body(model.geom_bodyid[g]).name == f"{side}_foot_link"
            and model.geom_contype[g]
        )
        for side in ("left", "right")
    ]

    def feet_down() -> list[bool]:
        touching = set()
        for c in data.contact[: data.ncon]:
            if c.geom1 in world:
                touching.add(c.geom2)
            if c.geom2 in world:
                touching.add(c.geom1)
        return [f in touching for f in feet]

    renderer, frames = None, []
    if args.render:
        from PIL import Image, ImageDraw

        renderer = mujoco.Renderer(model, 540, 960)
        cam = mujoco.MjvCamera()
        cam.distance, cam.azimuth, cam.elevation = 2.6, 135, -12

    print(
        f"policy: {args.policy_dir}, trained on {meta['physics']}, {1 / meta['control_dt']:.0f} Hz"
    )

    def floor_at(x: float) -> float:
        """Height of the floor or stair step at x."""
        if not args.stairs:
            return 0.0
        x0, n, t = STAIRS["start"], STAIRS["steps"], STAIRS["tread"]
        up = np.floor((x - x0) / t) + 1
        down = np.floor((x0 + 2 * n * t + STAIRS["platform"] - x) / t) + 1
        return float(np.clip(min(up, down), 0, n) * args.stairs)

    failures, fell = [], False
    # stair rise is measured from the pelvis's walking height on the flat approach (from its
    # standing height it would count the dip of walking against the climb)
    approach, top_z, z_start = [], 0.0, data.qpos[2]
    for name, seconds, cmd in script:
        cmd = np.array(cmd, dtype=np.float32)
        steps = round(seconds / meta["control_dt"])
        vel, yaw_rate = [], []
        touchdowns, air, peak, load = 0, [0, 0], np.zeros(len(names)), []
        heat, heat_n = np.zeros(len(names)), 0  # sum of (torque / rated torque)^2
        # per-foot gait: swing and stance times, step length along the heading, swing height
        ground, lift = [0, 0], [None, None]
        swing, stance, length, height, tilt = ([[], []] for _ in range(5))
        # sole pitch (deg, toe down +): through swing, and at each touchdown
        pitch_swing, pitch_land = [[], []], [[], []]
        top = [0.0, 0.0]
        posture = []  # pelvis height above the floor under it, mean knee bend (deg)
        # side-to-side motion: pelvis and head positions, the sideways direction, pelvis and
        # torso roll (deg); at each touchdown the distance across the heading to the other foot
        sway, widths = [], []
        knees = [i for i, n in enumerate(names) if n.endswith("_knee")]
        for k in range(steps):
            last_action = session.run(None, {in_name: observe(cmd, k)})[0][0]
            target = hold.copy()
            target[act_idx] = default[act_idx] + scale * last_action
            for _ in range(substeps):
                q, qd = data.qpos[qadr], data.qvel[vadr]
                tau = kp * (target - q) - kd * qd
                hi = np.clip(effort * (1 - qd / speed), 0, effort)
                lo = np.clip(effort * (-1 - qd / speed), -effort, 0)
                data.ctrl[aid] = np.clip(tau, lo, hi)
                peak = np.maximum(peak, np.abs(data.ctrl[aid]) / effort)
                heat += np.square(data.ctrl[aid] / rated)
                heat_n += 1
                mujoco.mj_step(model, data)
            load.append(np.abs(data.ctrl[aid]) / effort)
            # a step is a touchdown after at least AIR_MIN in the air (ignores contact flicker)
            rot = data.xmat[model.body("pelvis").id].reshape(3, 3)
            top_z = max(top_z, data.qpos[2])
            if name == "stairs" and data.qpos[0] < STAIRS["start"] - 0.3:
                approach.append(data.qpos[2])
            heading = rot[:2, 0] / np.linalg.norm(rot[:2, 0])
            for i, d in enumerate(feet_down()):
                pos = data.geom_xpos[feet[i]].copy()
                frame = data.geom_xmat[feet[i]].reshape(3, 3)
                # sole height: the lowest corner of the foot box above the floor under it
                pos[2] -= np.abs(frame[2]) @ model.geom_size[feet[i]] + floor_at(pos[0])
                if d and k >= steps // 2:  # foot tilt in stance: sole normal vs vertical
                    tilt[i].append(np.degrees(np.arccos(min(1.0, frame[2, 2]))))
                pitch = np.degrees(np.arcsin(np.clip(-frame[2, 0], -1, 1)))
                if not d and k >= steps // 2:
                    pitch_swing[i].append(pitch)
                if d and air[i] * meta["control_dt"] >= AIR_MIN:
                    touchdowns += 1
                    if k >= steps // 2:
                        pitch_land[i].append(pitch)
                        other = data.geom_xpos[feet[1 - i], :2]
                        widths.append(abs(np.array([-heading[1], heading[0]]) @ (pos[:2] - other)))
                    if k >= steps // 2 and lift[i] is not None:
                        swing[i].append(air[i] * meta["control_dt"])
                        length[i].append(float(heading @ (pos[:2] - lift[i][:2])))
                        height[i].append(top[i] - lift[i][2])
                if not d and ground[i] > 0:  # lift-off
                    if k >= steps // 2:
                        stance[i].append(ground[i] * meta["control_dt"])
                    lift[i], top[i] = pos.copy(), pos[2]
                top[i] = max(top[i], pos[2])
                air[i] = 0 if d else air[i] + 1
                ground[i] = ground[i] + 1 if d else 0
            if data.qpos[2] - floor_at(data.qpos[0]) < FALL_HEIGHT or rot[2, 2] < np.cos(
                np.radians(45)
            ):
                fell = True
                break
            if k >= steps // 2:  # judge the second half, after the transition
                yaw = np.arctan2(rot[1, 0], rot[0, 0])
                c, s = np.cos(yaw), np.sin(yaw)
                vw = data.qvel[0:2]
                vel.append([c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]])
                yaw_rate.append((rot @ data.qvel[3:6])[2])  # world-frame yaw rate
                q = data.qpos[qadr]
                posture.append(
                    [data.qpos[2] - floor_at(data.qpos[0]), np.degrees(np.mean(np.abs(q[knees])))]
                )
                side = np.array([-heading[1], heading[0]])
                trot = data.xmat[torso].reshape(3, 3)
                sway.append(
                    [
                        *data.qpos[:2],
                        *data.xpos[head, :2],
                        *side,
                        np.degrees(np.arcsin(np.clip(rot[2, 1], -1, 1))),
                        np.degrees(np.arcsin(np.clip(trot[2, 1], -1, 1))),
                    ]
                )
            if renderer is not None and k % 2 == 0:
                cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2] - 0.1]
                renderer.update_scene(data, cam)
                img = Image.fromarray(renderer.render())
                yaw = np.arctan2(rot[1, 0], rot[0, 0])
                vx = np.cos(yaw) * data.qvel[0] + np.sin(yaw) * data.qvel[1]
                wz = (rot @ data.qvel[3:6])[2]
                draw = ImageDraw.Draw(img)
                draw.rectangle((8, 8, 470, 92), fill=(255, 255, 255))
                draw.text(
                    (16, 12),
                    f"{name}\ncommand  vx {cmd[0]:+.2f} m/s  wz {cmd[2]:+.2f} rad/s\n"
                    f"measured vx {vx:+.2f} m/s  wz {wz:+.2f} rad/s",
                    fill=(20, 20, 20),
                    font_size=22,
                )
                frames.append(np.asarray(img))
        if fell:
            print(f"{name:14} FELL at x {data.qpos[0]:.2f} m")
            failures.append(f"fell during '{name}'")
            break
        v, w = np.mean(vel, axis=0), float(np.mean(yaw_rate))
        rate = touchdowns / seconds
        print(
            f"{name:14} cmd vx {cmd[0]:+.2f} vy {cmd[1]:+.2f} wz {cmd[2]:+.2f}   "
            f"got vx {v[0]:+.2f} vy {v[1]:+.2f} wz {w:+.2f}   "
            f"{rate:3.1f} steps/s, torque mean {np.mean(load):3.0%} peak {peak.max():3.0%} "
            f"of peak ({names[int(peak.argmax())]})"
        )
        # heat: RMS torque over the segment against each joint's continuous rating. Copper
        # losses go with torque squared, so this is the steady-state heating, relative to the
        # rated condition, if the activity went on indefinitely
        rms = np.sqrt(heat / max(heat_n, 1))
        hot = np.argsort(-rms)[:2]
        print(
            f"{'':14} heat: RMS torque / rated "
            + ", ".join(f"{names[i]} {rms[i]:.0%}" for i in hot)
        )
        h, kb = np.mean(posture, axis=0)
        print(
            f"{'':14} posture: pelvis {h * 100:.1f} cm above the floor "
            f"(stance {STANCE_Z * 100:.1f}), knees bent {kb:.0f} deg (stance {STANCE_KNEE:.0f})"
        )
        if rms.max() > 1.0:
            failures.append(f"'{name}': {names[hot[0]]} RMS torque {rms.max():.0%} of rated")
        if cmd[0] > 0:

            def lr(x, unit, f):
                m = [np.mean(v) * f if v else float("nan") for v in x]
                return f"L {m[0]:.2f} R {m[1]:.2f} {unit} ({sym(m):.0%})"

            def sym(m):  # symmetry index: |L - R| / mean
                return abs(m[0] - m[1]) / (0.5 * (m[0] + m[1]) + 1e-9)

            print(
                f"{'':14} gait (asymmetry): swing {lr(swing, 's', 1)}, "
                f"stance {lr(stance, 's', 1)}, "
                f"step {lr(length, 'cm', 100)}, lift {lr(height, 'cm', 100)}, "
                f"stance tilt {lr(tilt, 'deg', 1)}"
            )
            if cmd[2] == 0:  # straight walks only: on a curve the pelvis drifts sideways
                # sway, 5-95 percentile range: pelvis and head across the heading about their
                # 1.2 s moving mean (so a slow change of heading doesn't count), and the roll
                sw_a = np.array(sway)
                half = round(0.6 / meta["control_dt"])
                roll = [np.ptp(np.percentile(sw_a[:, c], [5, 95])) for c in (6, 7)]
                width = np.mean(widths) if widths else np.nan
                head_cm = sideways(sw_a[:, 2:4], sw_a[:, 4:6], half) * 100
                pelvis_cm = sideways(sw_a[:, 0:2], sw_a[:, 4:6], half) * 100
                print(
                    f"{'':14} sway: head side to side {head_cm:.1f} cm, pelvis {pelvis_cm:.1f} cm, "
                    f"pelvis roll {roll[0]:.1f} deg, torso roll {roll[1]:.1f} deg, "
                    f"step width {width * 100:.1f} cm"
                )
            sw = [np.abs(p).max() if p else float("nan") for p in pitch_swing]
            ld = [np.mean(p) if p else float("nan") for p in pitch_land]
            print(
                f"{'':14} feet (toe down +): swing pitch up to L {sw[0]:.0f} R {sw[1]:.0f} deg, "
                f"at touchdown L {ld[0]:+.0f} R {ld[1]:+.0f} deg"
            )
        if cmd[0] > 0 and rate < 1.0:
            failures.append(f"'{name}': {rate:.1f} steps/s, the feet are sliding, not stepping")
        if name == "stairs":
            end = STAIRS["start"] + 2 * STAIRS["steps"] * STAIRS["tread"] + STAIRS["platform"]
            top_gain = top_z - (np.mean(approach) if approach else z_start)
            climbed = top_gain >= 0.9 * STAIRS["steps"] * args.stairs
            print(
                f"{'':14} stairs: pelvis rose {top_gain * 100:.1f} cm of "
                f"{STAIRS['steps'] * args.stairs * 100:.0f}, ended at x {data.qpos[0]:.2f} m "
                f"(stairs end at {end:.2f})"
            )
            if not climbed or data.qpos[0] < end:
                failures.append("didn't climb the stairs and come down the other side")
        elif name == "fast walk":
            if v[0] < 0.75 * cmd[0]:
                failures.append(f"'{name}': forward speed {v[0]:.2f} m/s for {cmd[0]:.2f}")
        elif abs(cmd[0]) > 0 and abs(v[0] - cmd[0]) > 0.1:
            failures.append(f"'{name}': forward speed {v[0]:.2f} m/s for {cmd[0]:.2f}")
        if abs(cmd[2]) > 0 and (np.sign(w) != np.sign(cmd[2]) or abs(w) < 0.5 * abs(cmd[2])):
            failures.append(f"'{name}': yaw rate {w:.2f} rad/s for {cmd[2]:.2f}")
        if not cmd.any() and (np.linalg.norm(v) > 0.05 or abs(w) > 0.1):
            failures.append(f"'{name}': still moving ({np.linalg.norm(v):.2f} m/s, {w:.2f} rad/s)")

    if renderer is not None and frames:
        import imageio.v2 as imageio

        imageio.mimsave(args.render, frames, fps=round(0.5 / meta["control_dt"]))
        print(f"saved {args.render}")
    if failures:
        sys.exit("FAIL: " + "; ".join(failures))
    print("PASS")


if __name__ == "__main__":
    main()
