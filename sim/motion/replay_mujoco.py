"""Replay an exported motion tracker in plain MuJoCo, clip by clip: the tracker's sim-to-sim check.

The tracker is trained in Newton (ProtoMotions). This runs the same policy, exported to ONNX by
ProtoMotions' deployment/export_bm_tracker_onnx.py, in MuJoCo on the model setup_protomotions.py
writes (the same links, collision boxes and arm collisions), with the motors as the walking
policies' MuJoCo check models them (sim/mujoco/check_policy.py): PD clipped by each motor's
torque-speed line, plus joint armature and friction, which Newton's built-in PD leaves out. The
observations are built as ProtoMotions' deployment/test_tracker_mujoco.py builds them (its
deployment helpers are imported from the checkout), plus the torso's position in the world,
which this tracker reads (EXPERIMENT_ALL in setup_protomotions.py): MuJoCo's ground truth here,
an odometry estimate on the robot. Sensor noise (--noise, a fraction of the walking check's
level) goes on the joint angles and speeds and the pelvis rate.

Each clip starts on its reference's first frame and plays to its end. The rollout is written in
ProtoMotions' motion-library format, so sim/motion/check_tracking.py scores it as it scores a
Newton rollout, and sim/motion/render_tracking.py draws it. Per clip this prints whether the
robot fell (pelvis height 25 cm off the reference's), each joint's peak torque
against its motor's peak, and the hottest joint's RMS torque against its continuous rating.

Usage (in the ProtoMotions environment, from its checkout, with onnx and onnxruntime):
  python deployment/export_bm_tracker_onnx.py --checkpoint results/<run>/last.ckpt --output <dir>
  python sim/motion/replay_mujoco.py <dir>/unified_pipeline.onnx <motions.pt> <rollout.pt>
  python sim/motion/check_tracking.py <rollout.pt> <motions.pt>
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
import torch
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "mujoco"))
sys.path.insert(0, str(Path.cwd()))  # the ProtoMotions checkout, for deployment/
from check_policy import PHYSICS_DT, SENSOR_NOISE  # noqa: E402
from deployment.motion_utils import MotionPlayer  # noqa: E402
from deployment.state_utils import quat_rotate_np  # noqa: E402
from deployment.tracker_inputs import (  # noqa: E402
    align_future_motion_refs,
    build_tracker_onnx_inputs,
    get_tracker_input_requirements,
    init_tracker_alignment,
)
from setup_protomotions import joint_specs, mjcf  # noqa: E402

FELL = 0.25  # m off the reference's pelvis height: fallen (ProtoMotions' test, on the torso)


def build() -> mujoco.MjModel:
    """The training MJCF with a floor, at the walking check's physics step."""
    xml, _ = mjcf(joint_specs())
    spec = mujoco.MjSpec.from_string(xml)
    spec.option.timestep = PHYSICS_DT
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[50, 50, 0.05],
        friction=[1, 0.005, 1e-4],
    )
    return spec.compile()


def xyzw(q: np.ndarray) -> np.ndarray:
    return np.asarray(q)[..., [1, 2, 3, 0]]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx")
    ap.add_argument("motions")
    ap.add_argument("out")
    ap.add_argument("--noise", type=float, default=0.5, help="fraction of the walking check's")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--clips", nargs="*", help="clip indices (default: all)")
    args = ap.parse_args()

    meta = yaml.safe_load(Path(args.onnx).with_suffix(".yaml").read_text())
    names, bodies = meta["robot"]["joint_names"], meta["robot"]["body_names"]
    anchor = meta["robot"]["anchor_body_index"]
    kp, kd = np.array(meta["control"]["stiffness"]), np.array(meta["control"]["damping"])
    control_dt = meta["timing"]["control_dt"]
    future = meta["motion"]["future_step_indices"]
    to_key = meta["_runtime"]["onnx_name_to_in_key"]
    needs = get_tracker_input_requirements(to_key)
    session = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    in_names = [i.name for i in session.get_inputs()]
    out_names = [o.name for o in session.get_outputs()]

    model = build()
    data = mujoco.MjData(model)
    substeps = round(control_dt / model.opt.timestep)
    specs = joint_specs()
    effort = np.array([specs[n]["effort"] for n in names])
    rated = np.array([specs[n]["rated"] for n in names])
    speed = np.array([specs[n]["speed"] for n in names])
    qadr = np.array([model.jnt_qposadr[model.joint(n).id] for n in names])
    vadr = np.array([model.jnt_dofadr[model.joint(n).id] for n in names])
    aid = np.array([model.actuator(n).id for n in names])
    bid = np.array([model.body(b).id for b in bodies])
    torso = bid[anchor]
    rng = np.random.default_rng(args.seed)

    ref = torch.load(args.motions, weights_only=False, map_location="cpu")
    clips = [int(c) for c in args.clips] if args.clips else range(len(ref["motion_files"]))
    gts, grs, dps, starts, lengths, files = [], [], [], [], [], []
    print(f"{'clip':16} {'fell':>5} {'peak torque':>22} {'hottest (RMS / rated)':>28}")
    for ci in clips:
        player = MotionPlayer(args.motions, motion_index=ci, control_dt=control_dt)
        f0 = player.get_state_at_frame(0)
        mujoco.mj_resetData(model, data)
        data.qpos[:3] = f0["body_pos"][0]
        data.qpos[3:7] = f0["body_rot"][0][[3, 0, 1, 2]]
        data.qpos[qadr] = f0["dof_pos"]
        mujoco.mj_forward(model, data)
        align = init_tracker_alignment(
            robot_anchor_pos=data.xpos[torso],
            robot_anchor_rot=xyzw(data.xquat[torso]),
            motion_anchor_pos=f0["body_pos"][anchor],
            motion_anchor_rot=f0["body_rot"][anchor],
        )
        prev = None
        peak, heat, fell = np.zeros(len(names)), np.zeros(len(names)), None
        pos, rot, q = [], [], []
        for k in range(player.total_frames):
            noise = args.noise
            dof_pos = data.qpos[qadr] + noise * SENSOR_NOISE["joint_pos"] * rng.uniform(-1, 1, 27)
            dof_vel = data.qvel[vadr] + noise * SENSOR_NOISE["joint_vel"] * rng.uniform(-1, 1, 27)
            ang = data.qvel[3:6] + noise * SENSOR_NOISE["base_ang_vel"] * rng.uniform(-1, 1, 3)
            refs = align_future_motion_refs(player.get_future_references(k, future), align)
            cur = player.get_state_at_frame(k)["body_pos"][anchor] - align.motion_anchor_start_pos
            ref_anchor = align.robot_anchor_start_pos + quat_rotate_np(align.heading_offset, cur)
            inputs = build_tracker_onnx_inputs(
                onnx_input_names=[n for n in in_names if to_key[n] != "current.anchor_pos"],
                onnx_name_to_key=to_key,
                dof_pos=dof_pos.astype(np.float32),
                dof_vel=dof_vel.astype(np.float32),
                anchor_rot=xyzw(data.xquat[torso]).astype(np.float32),
                root_local_ang_vel=ang.astype(np.float32),
                future_refs=refs,
                anchor_body_index=anchor,
                num_dofs=len(names),
                prev_actions=prev,
                ref_anchor_pos=ref_anchor.astype(np.float32) if needs.ref_anchor_pos else None,
            )
            for n in in_names:
                if to_key[n] == "current.anchor_pos":
                    inputs[n] = data.xpos[torso][None].astype(np.float32)
            target = session.run(out_names, inputs)[1][0]
            prev = target.copy()
            for _ in range(substeps):
                qq, qd = data.qpos[qadr], data.qvel[vadr]
                tau = kp * (target - qq) - kd * qd
                hi = np.clip(effort * (1 - qd / speed), 0, effort)
                lo = np.clip(effort * (-1 - qd / speed), -effort, 0)
                data.ctrl[aid] = np.clip(tau, lo, hi)
                peak = np.maximum(peak, np.abs(data.ctrl[aid]) / effort)
                heat += np.square(data.ctrl[aid] / rated) / substeps
                mujoco.mj_step(model, data)
            pos.append(data.xpos[bid].copy())
            rot.append(xyzw(data.xquat[bid]))
            q.append(data.qpos[qadr].copy())
            ref_z = player.get_state_at_frame(k)["body_pos"][0][2]
            if fell is None and abs(data.qpos[2] - ref_z) > FELL:
                fell = k * control_dt
        heat = np.sqrt(heat / player.total_frames)
        hot, top = int(np.argmax(heat)), int(np.argmax(peak))
        label = Path(ref["motion_files"][ci]).name.split("__")[0]
        label = label.removeprefix("soma_").removeprefix("proto_").split("_keypoints")[0]
        print(
            f"{label:16} {f'{fell:.1f}s' if fell is not None else 'no':>5} "
            f"{names[top]:>16} {peak[top]:4.0%} {names[hot]:>20} {heat[hot]:5.0%}",
            flush=True,
        )
        starts.append(sum(lengths))
        lengths.append(len(pos))
        files.append(ref["motion_files"][ci])
        gts.append(np.array(pos))
        grs.append(np.array(rot))
        dps.append(np.array(q))
    torch.save(
        {
            "gts": torch.tensor(np.concatenate(gts), dtype=torch.float32),
            "grs": torch.tensor(np.concatenate(grs), dtype=torch.float32),
            "dps": torch.tensor(np.concatenate(dps), dtype=torch.float32),
            "length_starts": torch.tensor(starts),
            "motion_num_frames": torch.tensor(lengths),
            "motion_dt": torch.full((len(lengths),), control_dt),
            "motion_files": files,
        },
        args.out,
    )
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
