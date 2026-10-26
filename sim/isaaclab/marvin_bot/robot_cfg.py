"""Isaac Lab articulation config for Marvin Bot.

Isaac Lab 3.0 API. Everything joint-related comes from the description files: joint order and
default stance from description/robot.yaml, motor torque and speed from actuators.yaml, the
ankle linkage ratios from skeleton.yaml (as tools/build_urdf.py does for the URDF).
sim/isaaclab/convert_urdf.py turns description/urdf/marvin_bot.urdf into the USD loaded here.

Actuators are DC motor models: torque saturates at the motor's peak and falls linearly to zero
at its no-load speed. The two ankle joints, driven by a pair of motors through a linkage, are
modelled in joint space with the pair's combined torque times the linkage ratio.

Gains, armature and friction are placeholders until the phase 2 bench test fits them. They
follow comparable open humanoids of this size (kp 100-150 on the 120 N·m hip and knee joints,
~40 on the ankles). sim/mujoco/check_standing.py uses stiffer gains because it holds a pose
with no balance controller.
"""

import math
from pathlib import Path

import isaaclab.sim as sim_utils
import yaml
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

ROOT = Path(__file__).resolve().parents[3]
DESC = ROOT / "description"
URDF_PATH = DESC / "urdf" / "marvin_bot.urdf"
# USD converted from the URDF by convert_urdf.py (git-ignored); loading it needs no Isaac Sim
USD_DIR = Path(__file__).resolve().parents[1] / "usd"
USD_PATH = USD_DIR / "marvin_bot" / "marvin_bot" / "marvin_bot.usda"

_robot = yaml.safe_load((DESC / "robot.yaml").read_text())
_acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
_linkage = yaml.safe_load((DESC / "skeleton.yaml").read_text())["ankle_linkage"]
SIDES = ("left", "right")


def _expand(name: str) -> list[str]:
    return [name.replace("{side}", s) for s in SIDES] if "{side}" in name else [name]


# Joint order of every policy and of the robot, on every physics backend (PhysX and Newton order
# joints differently by default): robot.yaml's order, left before right.
JOINT_ORDER = tuple(n for j in _robot["joints"] for n in _expand(j["name"]))
STANCE_DEG = _robot["stance"]
STANCE_PELVIS_Z = _robot["stance_pelvis_height"]

LEG_JOINTS = [
    ".*_hip_pitch",
    ".*_hip_roll",
    ".*_hip_yaw",
    ".*_knee",
    ".*_ankle_pitch",
    ".*_ankle_roll",
]
ARM_JOINTS = [
    ".*_shoulder_pitch",
    ".*_shoulder_roll",
    ".*_shoulder_yaw",
    ".*_elbow",
    ".*_wrist_yaw",
]
FEET = ".*_foot_link"

# IMU on the pelvis, as the MuJoCo model's site
IMU_BODY = "pelvis"
IMU_OFFSET = (0.0, 0.0, 0.06)

# (kp N·m/rad, kd N·m·s/rad, armature kg·m², friction N·m) per joint pattern. Placeholders.
GAINS = {
    ".*_hip_pitch": (150.0, 5.0, 0.03, 0.3),
    ".*_hip_roll": (150.0, 5.0, 0.03, 0.3),
    ".*_hip_yaw": (100.0, 3.3, 0.02, 0.2),
    ".*_knee": (150.0, 5.0, 0.03, 0.3),
    ".*_ankle_pitch": (40.0, 2.0, 0.02, 0.2),
    ".*_ankle_roll": (40.0, 2.0, 0.02, 0.2),
    "waist_yaw": (150.0, 5.0, 0.02, 0.2),
    ".*_shoulder_pitch": (40.0, 2.0, 0.02, 0.2),
    ".*_shoulder_roll": (40.0, 2.0, 0.02, 0.2),
    ".*_shoulder_yaw": (30.0, 1.5, 0.01, 0.1),
    ".*_elbow": (30.0, 1.5, 0.01, 0.1),
    ".*_wrist_yaw": (15.0, 0.8, 0.005, 0.05),
    "neck_.*": (10.0, 0.5, 0.002, 0.02),
    ".*_gripper": (10.0, 0.5, 0.002, 0.02),
}


def joint_limits() -> dict[str, tuple[float, float]]:
    """(peak torque N·m, no-load speed rad/s) per joint, as in the URDF."""
    out = {}
    for m in _robot["motors"]:
        a = _acts[m["model"]]
        speed = a["no_load_speed_rpm"] * 2 * math.pi / 60
        for side in SIDES if "{side}" in m["name"] else (None,):
            drives = [d.replace("{side}", side) if side else d for d in m["drives"]]
            if len(drives) == 1:
                out[drives[0]] = (a["peak_torque"], speed)
            else:  # two motors through the ankle linkage
                for d in drives:
                    r = _linkage["pitch_ratio"] if d.endswith("pitch") else _linkage["roll_ratio"]
                    out[d] = (2 * a["peak_torque"] * r, speed / r)
    return out


def _actuators() -> dict[str, DCMotorCfg]:
    import re

    limits = joint_limits()
    out = {}
    for pattern, (kp, kd, armature, friction) in GAINS.items():
        joints = [j for j in JOINT_ORDER if re.fullmatch(pattern, j)]
        torques = {limits[j] for j in joints}
        if len(torques) != 1:
            raise ValueError(f"{pattern}: joints with different motors {torques}")
        torque, speed = torques.pop()
        out[pattern.replace(".*", "").strip("_") or pattern] = DCMotorCfg(
            joint_names_expr=[pattern],
            saturation_effort=torque,
            actuator_effort_limit=torque,
            actuator_velocity_limit=speed,
            joint_effort_limit=torque,
            joint_velocity_limit=speed,
            stiffness=kp,
            damping=kd,
            armature=armature,
            friction=friction,
        )
    return out


def _stance() -> dict[str, float]:
    out = {}
    for j in JOINT_ORDER:
        key = j.split("_", 1)[1] if j.startswith(SIDES) else j
        if key in STANCE_DEG:
            out[j] = math.radians(STANCE_DEG[key])
    return out


MARVIN_BOT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        activate_contact_sensors=True,
        rigid_props=PhysxRigidBodyCfg(max_depenetration_velocity=1.0),
        articulation_props=[
            # self-collision on: with it off, an earlier prototype's policies learned to pass their
            # legs through each other when turning
            PhysxArticulationCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=4,
            ),
            NewtonArticulationCfg(self_collision_enabled=True),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, STANCE_PELVIS_Z + 0.01),
        joint_pos=_stance(),  # every other joint at 0
        joint_vel={".*": 0.0},
    ),
    joint_ordering=JOINT_ORDER,
    soft_joint_pos_limit_factor=0.95,
    actuators=_actuators(),
)
