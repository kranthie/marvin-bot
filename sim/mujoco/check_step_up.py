"""Can the legs step up a stair at all? A static feasibility check, no policy.

A slow step up a stair of height h needs three poses to exist, each with the pelvis upright,
the feet flat and the robot balanced (whole-body CoM over the supporting foot, 2 cm inside its
edge, front to back):

  1. swing over the edge: standing on the rear foot, the swing toe 3 cm above the stair edge
     (a swinging foot may pitch; only its toe has to clear)
  2. weight on the stair: rear foot on the floor, front foot on the stair (heel 1 cm past the
     edge), CoM over the front foot, so the rear foot can lift; the rear heel may rise (the
     foot pivots on its toe, up to the ankle's plantarflexion limit), as on real stairs
  3. rear foot over the edge: standing on the stair, the rear toe 3 cm above the stair edge

For each, the script searches pelvis positions (forward, sideways and drop) for one where
inverse kinematics puts both feet in place within the joint ranges in the URDF (from
robot.yaml). On one foot (poses 1 and 3) the CoM must also lie over that foot's width, with the
pelvis kept upright, so the stance leg's static torques (reported against the actuators' peak
and rated torques) include the weight shift a real step needs.
The upper body holds the crouch pose of check_standing.py; MuJoCo is used for kinematics only.

--hip-height scales the thigh and shin together (ankle height and masses unchanged) to compare
leg lengths before the leg CAD is fixed.

Usage:
  python sim/mujoco/check_step_up.py                       # 10 to 20 cm, current legs
  python sim/mujoco/check_step_up.py --heights 0.18        # exit 1 if not possible (CI)
  python sim/mujoco/check_step_up.py --hip-height 0.62 0.65 0.69 --heights 0.15 0.18

Needs: pip install -e ".[sim]"
"""

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_standing as cs  # noqa: E402

FOOT = cs.yaml.safe_load((cs.DESC / "skeleton.yaml").read_text())
_foot = next(c for c in FOOT["collisions"] if c["link"] == "left_foot_link")
SOLE = -(_foot["xyz"][2] - _foot["size"][2] / 2)  # ankle axis above the sole
TOE = _foot["xyz"][0] + _foot["size"][0] / 2  # sole ahead of the ankle
HEEL = _foot["size"][0] / 2 - _foot["xyz"][0]  # sole behind the ankle
CLEARANCE = 0.03  # swing toe above the stair edge
MARGIN = 0.02  # CoM at least this far inside the supporting foot (x)
SIDE_MARGIN = 0.01  # and this far inside its width on one foot (y)
HALF_WIDTH = _foot["size"][1] / 2
HEEL_LIFTS = (0.0, 0.25, 0.5)  # rear-foot pitch tried in pose 2 (rad)
TOE_TIP = (TOE, 0.0, -SOLE)  # front bottom edge of the sole, ankle frame
LEG = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"]
G = 9.81


def build(hip_height: float | None) -> tuple[mujoco.MjModel, float]:
    spec, _ = cs.build()
    robot = yaml.safe_load((cs.DESC / "robot.yaml").read_text())
    dims = robot["dimensions"]
    nominal = dims["hip_axis_height"]
    if hip_height:
        f = (hip_height - dims["ankle_height"]) / (dims["thigh"] + dims["shin"])
        for side in ("left", "right"):
            for link in ("hip_roll_link", "thigh_link", "shin_link", "ankle_link"):
                b = spec.body(f"{side}_{link}")
                b.pos = [b.pos[0], b.pos[1], b.pos[2] * f]
    return spec.compile(), hip_height or nominal


class Robot:
    def __init__(self, model: mujoco.MjModel):
        self.m, self.d = model, mujoco.MjData(model)
        self.base = model.body("pelvis").id
        self.foot = {s: model.body(f"{s}_foot_link").id for s in ("left", "right")}
        self.jnt = {s: [model.joint(f"{s}_{n}").id for n in LEG] for s in ("left", "right")}
        keys = cs.side_names([model.joint(j).name for j in range(model.njnt)])
        for j in range(model.njnt):
            name = model.joint(j).name
            if name != "root":
                deg = cs.POSES["crouch"].get(keys[name], 0.0)
                self.d.qpos[model.jnt_qposadr[j]] = np.radians(deg)
        self.d.qpos[3] = 1.0  # unit quaternion of the free base, left at the origin
        mujoco.mj_kinematics(model, self.d)
        self.q0 = self.d.qpos.copy()
        self.mass = float(sum(model.body_mass))

    def ik(self, side, target, pitch=0.0, point=(0.0, 0.0, 0.0), iters=100) -> float:
        """Place `point` (foot frame) at `target` with the foot pitched by `pitch`, or with its
        pitch free if `pitch` is None (a swinging foot). Returns the remaining error (m, plus
        0.05 m per rad); the joint angles stay in self.d.qpos, clipped to their ranges."""
        m, d, body = self.m, self.d, self.foot[side]
        adr = [m.jnt_qposadr[j] for j in self.jnt[side]]
        dof = [m.jnt_dofadr[j] for j in self.jnt[side]]
        lo, hi = m.jnt_range[self.jnt[side]].T
        c, s = np.cos(pitch or 0.0), np.sin(pitch or 0.0)
        want = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        rows = [0, 2] if pitch is None else [0, 1, 2]
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        err = 1.0
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            rot = d.xmat[body].reshape(3, 3)
            world = d.xpos[body] + rot @ np.asarray(point)
            e_p = target - world
            e_r = (0.5 * sum(np.cross(rot[:, k], want[:, k]) for k in range(3)))[rows]
            err = np.linalg.norm(e_p) + 0.05 * np.linalg.norm(e_r)
            if err < 2e-4:
                break
            mujoco.mj_jac(m, d, jp, jr, world, body)
            J = np.vstack([jp[:, dof], jr[rows][:, dof]])
            e = np.concatenate([e_p, 0.5 * e_r])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(len(e)), e)
            d.qpos[adr] = np.clip(d.qpos[adr] + dq, lo, hi)
        return err

    def com(self) -> np.ndarray:
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)
        return self.d.subtree_com[0].copy()

    def stance_torques(self, side: str) -> dict[str, float]:
        """Static joint torques of a leg holding the whole robot up from its foot."""
        m, d = self.m, self.d
        out = {}
        com = d.subtree_com[0]
        for j in self.jnt[side]:
            child = m.jnt_bodyid[j]
            m_below = m.body_subtreemass[child]
            c_below = d.subtree_com[child]
            m_rest = self.mass - m_below
            c_rest = (com * self.mass - c_below * m_below) / m_rest
            arm = c_rest - d.xanchor[j]
            out[m.joint(j).name] = abs(d.xaxis[j] @ np.cross(arm, [0, 0, -m_rest * G]))
        return out


def search(robot: Robot, place, support_x, stance: str | None, foot_y: float):
    """Find a pelvis pose (forward, sideways, drop) that places the feet and balances the CoM.
    `place(robot, px, py, pz)` runs the IK with the pelvis moved by (px, py, -pz).
    Returns the best: (IK error, CoM margin, pelvis drop, pelvis side shift, torques)."""
    best = None
    sides = np.arange(0.0, 0.1401, 0.01) * np.sign(foot_y) if stance else [0.0]
    for pz in np.arange(0.0, 0.2501, 0.01):
        for py in sides:
            for px in np.arange(-0.10, 0.301, 0.01):
                robot.d.qpos[:] = robot.q0
                err = place(robot, px, py, pz)
                c = robot.com() + [px, py, -pz]
                margin = min(c[0] - support_x[0], support_x[1] - c[0]) - MARGIN
                if stance:
                    margin = min(margin, HALF_WIDTH - abs(c[1] - foot_y) - SIDE_MARGIN)
                score = (err < 1e-3, margin if err < 1e-3 else -err)
                if best is None or score > best[0]:
                    torques = robot.stance_torques(stance) if stance else {}
                    best = (score, err, margin, pz, py, torques)
                if err < 1e-3 and margin > 0:
                    return best[1:]
    return best[1:]


def check(model: mujoco.MjModel, h: float) -> list[tuple]:
    robot = Robot(model)
    feet0 = {s: robot.d.xpos[robot.foot[s]].copy() for s in ("left", "right")}
    floor = feet0["left"][2] - SOLE
    fx = feet0["left"][0]
    edge = fx + TOE + 0.01  # stair edge 1 cm ahead of the toes
    step = edge + 0.01 + HEEL - fx  # front ankle so its heel is 1 cm past the edge
    yl, yr = feet0["left"][1], feet0["right"][1]
    rear = (fx - HEEL, fx + TOE)
    front = (fx + step - HEEL, fx + step + TOE)

    def at(x, y, z):
        return np.array([x, y, z + floor])

    def pose1(r, px, py, pz):
        shift = np.array([px, py, -pz])
        e1 = r.ik("right", feet0["right"] - shift)
        e2 = r.ik("left", at(edge, yl, h + CLEARANCE) - shift, pitch=None, point=TOE_TIP)
        return e1 + e2

    def pose2(lift):
        def place(r, px, py, pz):  # rear toe stays put while its heel rises by `lift`
            shift = np.array([px, py, -pz])
            toe = feet0["right"] + [TOE, 0, -SOLE]
            back = np.array([np.cos(lift) * TOE + np.sin(lift) * SOLE, 0, np.sin(lift) * TOE])
            back[2] -= np.cos(lift) * SOLE
            e1 = r.ik("right", toe - back - shift, pitch=lift)
            e2 = r.ik("left", at(fx + step, yl, SOLE + h) - shift)
            return e1 + e2

        return place

    def pose3(r, px, py, pz):
        shift = np.array([px, py, h - pz])
        e1 = r.ik("left", at(fx + step, yl, SOLE + h) - shift)
        e2 = r.ik("right", at(edge, yr, h + CLEARANCE) - shift, pitch=None, point=TOE_TIP)
        return e1 + e2

    out = []
    for name, place, support, stance in [
        ("1 swing over edge", pose1, rear, "right"),
        ("2 weight on stair", None, front, None),
        ("3 rear over edge", pose3, front, "left"),
    ]:
        foot_y = yr if stance == "right" else yl
        if place is None:  # try a flat rear foot first, then rising heels
            for lift in HEEL_LIFTS:
                res = search(robot, pose2(lift), support, None, foot_y)
                if res[0] < 1e-3 and res[1] > 0:
                    break
            name = f"{name} (heel {np.degrees(lift):.0f}°)"
        else:
            res = search(robot, place, support, stance, foot_y)
        err, margin, drop, side, torques = res
        out.append((name, err < 1e-3 and margin > 0, err, margin, drop, side, torques))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--heights", type=float, nargs="+", default=[0.10, 0.13, 0.15, 0.17, 0.18, 0.20]
    )
    ap.add_argument(
        "--hip-height", type=float, nargs="+", default=[None], help="hip axis to sole (m)"
    )
    args = ap.parse_args()

    _, rated = cs.build()
    failed = False
    for hh in args.hip_height:
        model, hip = build(hh)
        peak = {model.joint(j).name: model.jnt_actfrcrange[j][1] for j in range(model.njnt)}
        print(f"\nhip axis to sole {hip:.2f} m (foot {TOE + HEEL:.2f} m long, ankle {SOLE:.2f} m)")
        for h in args.heights:
            rows = check(model, h)
            ok = all(r[1] for r in rows)
            failed |= not ok
            print(f"  {h * 100:.0f} cm stair: {'possible' if ok else 'not possible'}")
            for name, ok, err, margin, drop, side, torques in rows:
                t = ""
                if torques:
                    w = max(torques, key=lambda k: torques[k] / peak[k])
                    tw = torques[w]
                    t = (
                        f", worst {w.split('_', 1)[1]} {tw:.0f} N·m = {tw / peak[w]:.0%} "
                        f"of peak, {tw / rated[w]:.0%} of rated"
                    )
                print(
                    f"    {name:26} {'yes' if ok else 'no ':3} IK {err * 1000:4.1f} mm, CoM "
                    f"{'inside' if margin > 0 else 'outside'} by {abs(margin) * 1000:3.0f} mm, "
                    f"pelvis down {drop * 100:4.1f} cm, aside {abs(side) * 100:3.0f} cm{t}"
                )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
