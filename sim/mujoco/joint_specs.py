"""Per-joint motor and gain table shared by the MuJoCo checks and the motion pipeline.

Reads the description files and robot_cfg.py's GAINS table without importing Isaac Lab.
"""

import ast
import math
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
DESC = ROOT / "description"
ROBOT_CFG = ROOT / "sim" / "isaaclab" / "marvin_bot" / "robot_cfg.py"


def joint_specs() -> dict[str, dict]:
    """Per joint: peak torque, no-load speed, and the training gains, armature and friction
    (robot_cfg.py's GAINS table, read without importing Isaac Lab)."""
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    link = yaml.safe_load((DESC / "skeleton.yaml").read_text())["ankle_linkage"]
    gains = None
    for node in ast.parse(ROBOT_CFG.read_text()).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "GAINS":
            gains = ast.literal_eval(node.value)
    out = {}
    for m in robot["motors"]:
        a = acts[m["model"]]
        speed = a["no_load_speed_rpm"] * 2 * math.pi / 60
        for side in ("left", "right") if "{side}" in m["name"] else (None,):
            drives = [d.replace("{side}", side) if side else d for d in m["drives"]]
            for d in drives:
                r = 1.0
                if len(drives) == 2:
                    r = link["pitch_ratio"] if d.endswith("pitch") else link["roll_ratio"]
                effort = a["peak_torque"] * (2 * r if len(drives) == 2 else 1)
                rated = a["rated_torque"] * (2 * r if len(drives) == 2 else 1)
                kp, kd, armature, friction = next(v for k, v in gains.items() if re.fullmatch(k, d))
                out[d] = dict(
                    effort=effort,
                    rated=rated,
                    speed=speed / r,
                    kp=kp,
                    kd=kd,
                    armature=armature,
                    friction=friction,
                )
    return out
