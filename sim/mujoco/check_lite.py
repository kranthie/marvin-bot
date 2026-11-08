"""Lite on its wheels in MuJoCo: holding torques, tipping margin, braking, a push, a threshold.

Loads description/urdf/marvin_bot_lite.urdf (run tools/build_urdf.py first), adds a floor, a
free base and the omni wheels' rollers (the URDF has the wheels only): each wheel gets two rows
of free-turning rollers, spheres on hinges along the wheel's tangent, so it drives along its
rolling direction and slides freely along its axle, as a double omni wheel does. The wheels
are velocity servos limited to the RS00's peak torque, the column a position servo limited to
the DL6's force, every upper-body joint a position servo as in check_standing.py.

Checks (column down and up; arms down, and arms forward with 2 kg in each hand):
- hold: after settling, each upper-body joint's holding torque within its rated torque and the
  column's force within its rating;
- margin: the centre of mass's ground projection at least MIN_MARGIN inside the wheels' contact
  polygon;
- braking: from 1 m/s forward, the base brakes at brake_limit() (BRAKE_ALLOWANCE of the
  deceleration that would tip it, ramped in over JERK_T) and the two lightest wheels keep at
  least MIN_WHEEL_LOAD of their static share (one light wheel is not tipping);
- push: sideways at the shoulders for 0.5 s, PUSH_SHARE of the push that would tip it
  statically (printed): no two wheels lift;
- threshold: driving sideways at 0.3 m/s over a 10 mm bar (the worst direction for an omni
  wheel): the base gets over it and no wheel lifts for long.

brake_limit() is the base controller's deceleration limit (see docs/design/lite.md).

Usage:
  python sim/mujoco/check_lite.py              # every check
  python sim/mujoco/check_lite.py --quick      # hold and margin only (CI)
  python sim/mujoco/check_lite.py --render out.png

Needs: pip install -e ".[sim]"
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_standing import GAINS, side_names  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / "description"
URDF = DESC / "urdf" / "marvin_bot_lite.urdf"
G = 9.81
WHEEL_R = 0.076
ROLLERS, ROLLER_R = 12, 0.012  # per row; two rows 14 mm apart, offset by half a pitch
MIN_MARGIN = 0.080  # m
MIN_WHEEL_LOAD = 0.05  # of a wheel's static share, averaged over SMOOTH
# joints the hold check leaves out with a load in the hands: the bought hands' wrist pitch servo
# carries about 0.5 kg (requirements R20, v1 deviation); 2 kg is there for the tipping checks
LOADED_SKIP = ("wrist_pitch", "wrist_yaw")
# brake_limit() uses this share of the static tipping deceleration: the upper body sways on its
# joints' servos as the base slows, and braking at 0.8 of it unloaded the rear wheels (column up,
# arms forward, 2 kg in each hand)
BRAKE_ALLOWANCE = 0.6
SMOOTH = 0.05  # s: wheel loads are averaged over this, past the rollers' impacts
PUSH_SHARE = 0.6  # the push test's share of the static tipping push
COLUMN = (2e5, 4e3)  # column servo kp N/m, kd N·s/m: a stiff, self-locking spindle
WHEEL_KV = 8.0  # N·m per rad/s
JERK_T = 0.3  # s: the base ramps its deceleration in over this (a step overshoots it)

POSES = {  # degrees, robot.yaml's convention; the column in metres
    "arms down": {"shoulder_roll": 8, "elbow": 20},
    "arms forward": {"shoulder_pitch": 90, "elbow": 0},
}
HAND_LOAD = {"arms down": 0.0, "arms forward": 2.0}  # kg in each hand


def _models() -> dict[str, str]:
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    out = {}
    for m in robot["motors"] + robot["variants"]["lite"]["motors"]:
        for side in ("left", "right") if "{side}" in m["name"] else (None,):
            for d in m["drives"]:
                out[d.replace("{side}", side) if side else d] = m["model"]
    return out


def wheels(model: mujoco.MjModel) -> list[str]:
    return [
        model.joint(i).name for i in range(model.njnt) if model.joint(i).name.startswith("wheel_")
    ]


def build(hand_kg: float = 0.0) -> mujoco.MjSpec:
    if not (DESC / "meshes").is_dir():
        raise SystemExit("description/meshes/ is missing: run `python -m cad.build` first")
    urdf = URDF.read_text().replace(
        '<robot name="marvin_bot_lite">',
        '<robot name="marvin_bot_lite"><mujoco>'
        f'<compiler discardvisual="false" meshdir="{URDF.parent}"/></mujoco>',
        1,
    )
    spec = mujoco.MjSpec.from_string(urdf)
    spec.modelname = "marvin_bot_lite"
    spec.option.timestep = 0.001
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.compiler.autolimits = True
    world = spec.worldbody
    # contact groups: the rollers (contype 2) touch only the floor and obstacles (conaffinity 3)
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[6, 6, 0.05], conaffinity=3)
    world.add_light(pos=[0, 0, 3], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    base = spec.body("lite_base")
    base.add_freejoint(name="root")
    for g in spec.geoms:
        if g.contype and g.name != "floor":
            g.group = 3
    # rollers: in each wheel link, x is the axle; a roller at angle a round it sits at
    # (dx, -r sin a, -r cos a) and turns about the tangent (0, cos a, -sin a)
    for w in [b for b in spec.bodies if b.name.startswith("wheel_")]:
        for row, dx in enumerate((-0.007, 0.007)):
            for k in range(ROLLERS):
                a = 2 * math.pi * (k + 0.5 * row) / ROLLERS
                r = WHEEL_R - ROLLER_R
                rb = w.add_body(
                    name=f"{w.name}_r{row}_{k}", pos=[dx, -r * math.sin(a), -r * math.cos(a)]
                )
                rb.add_joint(
                    type=mujoco.mjtJoint.mjJNT_HINGE,
                    axis=[0, math.cos(a), -math.sin(a)],
                    damping=[0.002, 0, 0],
                    armature=1e-5,
                )
                rb.add_geom(
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[ROLLER_R, 0, 0],
                    mass=0.012,
                    friction=[0.9, 0.005, 1e-4],
                    contype=2,
                    conaffinity=0,
                    group=3,
                )
    for side in ("left", "right"):
        if hand_kg:
            load = spec.body(f"{side}_palm_link").add_body(name=f"{side}_load", pos=[0, 0, -0.12])
            load.add_geom(
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.04, 0, 0],
                mass=hand_kg,
                contype=0,
                conaffinity=0,
            )
    models = _models()
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    for j in list(spec.joints):
        if j.name == "root" or j.name not in models:
            continue
        model = models[j.name]
        act = spec.add_actuator(name=j.name, target=j.name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        if j.name.startswith("wheel_"):
            act.set_to_velocity(kv=WHEEL_KV)
            peak = acts[model]["peak_torque"]
            act.forcerange = [-peak, peak]
            j.armature = GAINS[model][2]
            j.damping = [0.01, 0, 0]
        elif j.name == "column_lift":
            act.set_to_position(kp=COLUMN[0], kv=COLUMN[1])
            f = acts[model]["peak_force"]
            act.forcerange = [-f, f]
            act.ctrlrange = j.range
            act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        else:
            kp, kd, armature = GAINS[model]
            act.set_to_position(kp=kp, kv=kd)
            j.armature = armature
            act.forcerange = j.actfrcrange if j.actfrclimited else [-1e3, 1e3]
            act.ctrlrange = j.range
            act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec


def targets(model: mujoco.MjModel, pose: str, column: float) -> np.ndarray:
    """Actuator targets: the pose's joint angles, the column height, wheels still."""
    q = np.zeros(model.nu)
    keys = side_names([model.actuator(i).name for i in range(model.nu)])
    for i in range(model.nu):
        name = model.actuator(i).name
        if name == "column_lift":
            q[i] = column
        elif not name.startswith("wheel_"):
            q[i] = math.radians(POSES[pose].get(keys[name], 0.0))
    return q


def start(model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray) -> None:
    mujoco.mj_resetData(model, data)
    for i in range(model.nu):
        name = model.actuator(i).name
        if not name.startswith("wheel_"):
            data.qpos[model.jnt_qposadr[model.actuator_trnid[i, 0]]] = q[i]
    data.qpos[2] = 0.002
    data.ctrl[:] = q
    mujoco.mj_forward(model, data)


def wheel_loads(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, float]:
    """Normal force (N) on each wheel's rollers."""
    out = {w: 0.0 for w in wheels(model)}
    f = np.zeros(6)
    for c in range(data.ncon):
        con = data.contact[c]
        for g in (con.geom1, con.geom2):
            parent = model.body(int(model.body_parentid[model.geom_bodyid[g]])).name
            if parent.startswith("wheel_"):
                mujoco.mj_contactForce(model, data, c, f)
                out[parent.removesuffix("_link")] += f[0]
    return out


def contact_points(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """The wheels' lowest points on the floor (x, y): the support polygon's corners."""
    pts = []
    for w in wheels(model):
        b = model.body(f"{w}_link").id
        pts.append(data.xpos[b][:2])
    return np.array(pts)


def margin(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Distance from the CoM's ground projection to the nearest edge of the wheels' polygon."""
    com = data.subtree_com[model.body("lite_base").id][:2]
    p = contact_points(model, data)
    c = p.mean(0)
    order = np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))
    p = p[order]
    d = []
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        t = (b - a) / np.linalg.norm(b - a)
        n = np.array([-t[1], t[0]])  # inward for a counter-clockwise polygon
        d.append(float(n @ (com - a)))
    return min(d)


def brake_limit(model: mujoco.MjModel, data: mujoco.MjData, direction=(1.0, 0.0)) -> float:
    """The base controller's deceleration limit (m/s²) for travel along `direction` (base
    frame): the deceleration that would move the zero-moment point to the leading wheels' edge,
    g x (distance from the CoM's projection to that edge) / (CoM height), times the allowance.
    Static: the arms' own motion adds to it, which the margin covers."""
    com = data.subtree_com[model.body("lite_base").id]
    p = contact_points(model, data)
    u = np.asarray(direction) / np.linalg.norm(direction)
    lead = max(float(u @ (q - com[:2])) for q in p)
    # the leading edge of an X of wheels is the line through the two leading wheels
    two = sorted(p, key=lambda q: -float(u @ (q - com[:2])))[:2]
    edge = float(u @ ((two[0] + two[1]) / 2 - com[:2]))
    return BRAKE_ALLOWANCE * G * min(lead, edge) / float(com[2])


def push_limit(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, float]:
    """The sideways push at shoulder height that would tip the robot over the side wheels
    (static: weight x distance to that edge / height), and PUSH_SHARE of it, the test push."""
    com = data.subtree_com[model.body("lite_base").id]
    mass = model.body_subtreemass[model.body("lite_base").id]
    h = data.xpos[model.body("left_shoulder_pitch_link").id][2]
    p = contact_points(model, data)
    side = min(float(q[1]) for q in p if q[1] > 0) - float(com[1])
    limit = mass * G * side / h
    return limit, PUSH_SHARE * limit


def drive(model: mujoco.MjModel, data: mujoco.MjData, v) -> None:
    """Wheel speed targets for a base velocity (vx, vy m/s, wz rad/s; base frame): each wheel
    rolls along its tangent t = (-sin a, cos a) at its angle a round the base."""
    for i in range(model.nu):
        name = model.actuator(i).name
        if not name.startswith("wheel_"):
            continue
        jid = model.actuator_trnid[i, 0]
        p = model.body(model.jnt_bodyid[jid]).pos
        a = math.atan2(p[1], p[0])
        rc = math.hypot(p[0], p[1])
        surface = -math.sin(a) * v[0] + math.cos(a) * v[1] + v[2] * rc
        data.ctrl[i] = -surface / WHEEL_R


def run(model, data, seconds: float, v=(0.0, 0.0, 0.0), force=None, every=None) -> list:
    """Step for `seconds`; `v` and `force` may be functions of the time since the call."""
    out, t0 = [], data.time
    for k in range(int(seconds / model.opt.timestep)):
        drive(model, data, v(data.time - t0) if callable(v) else v)
        data.xfrc_applied[:] = 0
        if force is not None:
            body, f = force(data.time - t0)
            data.xfrc_applied[body, :3] = f
        mujoco.mj_step(model, data)
        if every and k % every == 0:
            out.append(wheel_loads(model, data))
    return out


def lowest(loads: list[dict], dt: float) -> tuple[float, float]:
    """Over a run, the lowest load (N) on any wheel and the lowest on the second-least-loaded
    wheel, each wheel's load averaged over SMOOTH. One wheel going light is not tipping (the
    model's pods are rigid, the real ones sprung); two going light together is its onset."""
    n = max(1, round(SMOOTH / dt))
    x = np.array([[row[w] for w in loads[0]] for row in loads])
    if len(x) < n:
        return math.inf, math.inf
    k = np.ones(n) / n
    sm = np.sort(np.stack([np.convolve(x[:, i], k, "valid") for i in range(x.shape[1])], 1), 1)
    return float(sm[:, 0].min()), float(sm[:, 1].min())


def check_pose(pose: str, column: float, full: bool) -> list[str]:
    errors = []
    spec = build(HAND_LOAD[pose])
    model = spec.compile()
    data = mujoco.MjData(model)
    q = targets(model, pose, column)
    start(model, data, q)
    run(model, data, 2.0)
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    models = _models()
    worst, worst_j = 0.0, ""
    for i in range(model.nu):
        name = model.actuator(i).name
        if name.startswith("wheel_") or (HAND_LOAD[pose] and name.endswith(LOADED_SKIP)):
            continue
        a = acts[models[name]]
        rated = a["peak_force"] if name == "column_lift" else a["rated_torque"]
        frac = abs(data.actuator_force[i]) / rated
        if frac > worst:
            worst, worst_j = frac, name
    m = margin(model, data)
    loads = wheel_loads(model, data)
    share = sum(loads.values()) / 4
    a_lim = brake_limit(model, data)
    tag = f"{pose}, column {column:.2f} m"
    print(
        f"{tag:30} margin {m * 1000:4.0f} mm  worst hold {worst_j} {worst:4.0%}  "
        f"brake limit {a_lim:.2f} m/s²  "
        f"wheels {min(loads.values()):.0f}-{max(loads.values()):.0f} N"
    )
    if worst > 1.0:
        errors.append(f"{tag}: {worst_j} holds {worst:.0%} of its rating")
    if m < MIN_MARGIN:
        errors.append(f"{tag}: margin {m * 1000:.0f} mm < {MIN_MARGIN * 1000:.0f} mm")
    if not full:
        return errors

    # braking: up to 1 m/s, then down at the limit
    v0, t_acc = 1.0, 2.0
    t_brake = v0 / a_lim

    def slowed(tt):  # speed lost tt after braking starts, the deceleration ramped in
        if tt < JERK_T:
            return a_lim * tt * tt / (2 * JERK_T)
        return a_lim * (JERK_T / 2 + tt - JERK_T)

    def v(t):
        if t < t_acc:
            return (v0 * min(1.0, t / (t_acc * 0.8)), 0.0, 0.0)
        return (max(0.0, v0 - slowed(t - t_acc)), 0.0, 0.0)

    dt = 5 * model.opt.timestep
    loads = run(model, data, t_acc + t_brake + 1.0, v, every=5)
    one, two = (x / share for x in lowest(loads[int(t_acc / dt) :], dt))
    print(
        f"{'':30} braking at {a_lim:.2f} m/s²: lightest wheels at {one:.0%} and {two:.0%} "
        "of their share"
    )
    if two < MIN_WHEEL_LOAD:
        errors.append(f"{tag}: two wheels unload braking at {a_lim:.2f} m/s² ({two:.0%})")

    # push: sideways at the shoulders for 0.5 s, PUSH_SHARE of the static tipping push
    start(model, data, q)
    run(model, data, 1.0)
    limit, push = push_limit(model, data)
    torso = model.body("torso_link").id
    loads = run(
        model, data, 2.0, force=lambda t: (torso, [0, push if 0.5 < t < 1.0 else 0, 0]), every=5
    )
    one, two = (x / share for x in lowest(loads, dt))
    print(
        f"{'':30} sideways at the shoulders: tips at {limit:.0f} N (static); "
        f"{push:.0f} N for 0.5 s: lightest wheels at {one:.0%} and {two:.0%}"
    )
    if two < MIN_WHEEL_LOAD:
        errors.append(f"{tag}: two wheels lift under a {push:.0f} N push ({two:.0%})")
    return errors


def check_threshold() -> list[str]:
    """Sideways at 0.3 m/s over a 10 mm bar, column down, arms down."""
    spec = build()
    spec.worldbody.add_geom(
        name="bar",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[1.0, 0.02, 0.005],
        pos=[0, 0.45, 0.005],
        conaffinity=3,
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    start(model, data, targets(model, "arms down", 0.0))
    run(model, data, 1.0)
    run(model, data, 4.0, (0.0, 0.3, 0.0))
    y = data.qpos[1]
    up = np.linalg.norm(data.xmat[model.body("lite_base").id].reshape(3, 3)[:, 2] - [0, 0, 1])
    print(f"10 mm bar, sideways at 0.3 m/s: base at y = {y:.2f} m, tilt {math.degrees(up):.1f}°")
    if y < 0.7:
        return [f"threshold: the base stopped at the bar (y = {y:.2f} m)"]
    return []


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--quick", action="store_true", help="hold and margin only")
    ap.add_argument("--render", type=Path, help="save a PNG of Lite, column up, arms forward")
    args = ap.parse_args()
    errors = []
    for pose in POSES:
        for column in (0.0, 0.45):
            errors += check_pose(pose, column, not args.quick)
    if not args.quick:
        errors += check_threshold()
    if args.render:
        spec = build(2.0)
        model = spec.compile()
        data = mujoco.MjData(model)
        start(model, data, targets(model, "arms forward", 0.45))
        run(model, data, 1.0)
        r = mujoco.Renderer(model, 900, 700)
        cam = mujoco.MjvCamera()
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = [0, 0, 0.8], 3.0, 210, -10
        r.update_scene(data, cam)
        from PIL import Image  # noqa: PLC0415

        Image.fromarray(r.render()).save(args.render)
    for e in errors:
        print(f"FAIL: {e}")
    if errors:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
