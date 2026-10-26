"""Check the left-right mirror rule used for symmetry augmentation in training.

sim/isaaclab/marvin_bot/tasks/symmetry.py mirrors a pose by swapping left and right joints,
keeping their values, and negating the joints listed in its FLIP_SIGN. This script takes random
poses, mirrors them with that rule, and checks with MuJoCo forward kinematics that every link of
the mirrored pose sits at the mirror image (y negated) of its counterpart in the original.

Usage:
  python sim/mujoco/check_symmetry.py
"""

import ast
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_standing as cs  # noqa: E402

SYMMETRY = cs.ROOT / "sim" / "isaaclab" / "marvin_bot" / "tasks" / "symmetry.py"
TOL = 1e-6


def flip_sign() -> set[str]:
    """FLIP_SIGN from symmetry.py, read without importing it (it needs torch)."""
    for node in ast.parse(SYMMETRY.read_text()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "FLIP_SIGN":
            return set(ast.literal_eval(node.value))
    raise KeyError("FLIP_SIGN not found in symmetry.py")


def mirror(name: str) -> str:
    for a, b in (("left_", "right_"), ("right_", "left_")):
        if name.startswith(a):
            return b + name[len(a) :]
    return name


def main() -> int:
    spec, _ = cs.build()
    model = spec.compile()
    data = mujoco.MjData(model)
    joints = [model.joint(j).name for j in range(model.njnt) if model.joint(j).name != "root"]
    flip = flip_sign()
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(20):
        q = {}
        for name in joints:
            lo, hi = model.jnt_range[model.joint(name).id]
            q[name] = rng.uniform(lo, hi)
        mq = {mirror(n): (-v if n in flip else v) for n, v in q.items()}
        pos = []
        for pose in (q, mq):
            data.qpos[:] = 0
            data.qpos[3] = 1.0
            for n, v in pose.items():
                data.qpos[model.jnt_qposadr[model.joint(n).id]] = v
            mujoco.mj_kinematics(model, data)
            pos.append({model.body(b).name: data.xpos[b].copy() for b in range(1, model.nbody)})
        for body, p in pos[0].items():
            m = pos[1][mirror(body)]
            worst = max(worst, float(np.linalg.norm(p * [1, -1, 1] - m)))
    print(f"mirrored poses vs mirrored link positions, 20 random poses: worst {worst:.2e} m")
    if worst > TOL:
        print("FAIL: the mirror rule in symmetry.py does not match the robot")
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
