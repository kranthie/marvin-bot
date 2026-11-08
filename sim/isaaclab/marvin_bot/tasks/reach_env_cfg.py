# Portions adapted from Isaac Lab (https://github.com/isaac-sim/IsaacLab), Copyright (c) 2022-2026,
# The Isaac Lab Project Developers, BSD-3-Clause (LICENSES/BSD-3-Clause-IsaacLab.txt). Modified for
# Marvin Bot; the rest Apache-2.0.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
"""MarvinLite-Reach: Lite's left hand reaches targets around it, using its column, waist and arm.

Built on Isaac Lab's reach task (isaaclab_tasks.core.reach). Lite's base is fixed (its wheels are
not driven here); the policy moves the column, the waist and the left arm's six joints to put the
left palm on a target, position only, anywhere from low in front to high at the side. Besides
Isaac Lab's tracking rewards, it is penalised for moving the whole robot's centre of mass more
than COM_FREE from the column's axis, the start of the tipping margin that sim/mujoco/check_lite.py
measures (163 mm with the arms down). The point of the task is to check Lite's model and actuator
settings in Isaac Lab and give its arm a first learned skill; manipulation models come later.

Trained and scored on Isaac Lab 3.0 (Newton): sim/isaaclab/README.md. Unlike the base task, an
episode does not end when the hand is within 5 cm, so the policy learns to hold each target.
"""

import isaaclab.envs.mdp as mdp
import torch
from isaaclab.assets import Articulation, AssetBaseCfg
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab_tasks.core.reach.reach_env_cfg import ReachEnvCfg

from ..robot_cfg import MARVIN_LITE_CFG, _lite

MARVIN_LITE_JOINTS = _lite["joints"]

HAND = "left_palm_link"
JOINTS = ["column_lift", "waist_yaw", "left_shoulder_.*", "left_elbow", "left_wrist_.*"]


def _robot_column_range() -> tuple[float, float]:
    lite = next(j for j in MARVIN_LITE_JOINTS if j["name"] == "column_lift")
    return tuple(lite["range"])


COM_FREE = 0.06  # m: the centre of mass may move this far from the column's axis unpenalised


def com_offset(env, asset_cfg: SceneEntityCfg, free: float) -> torch.Tensor:
    """How far (m) the whole robot's centre of mass is from the base's vertical axis, past
    `free`."""
    asset: Articulation = env.scene[asset_cfg.name]
    m = asset.data.body_mass.torch.to(env.device)  # (envs, bodies)
    com = asset.data.body_com_pos_w.torch  # (envs, bodies, 3)
    c = (m.unsqueeze(-1) * com).sum(1) / m.sum(1, keepdim=True)
    d = torch.linalg.norm(c[:, :2] - asset.data.root_pos_w.torch[:, :2], dim=1)
    return torch.clamp(d - free, min=0.0)


@configclass
class MarvinLiteReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # the arms' delayed DC motors run in Isaac Lab's actuator path, as in the walking tasks
        self.sim.use_newton_actuators = False
        newton_mjwarp = self.sim.physics.newton_mjwarp
        newton_mjwarp.solver_cfg.njmax = 300  # the arms may touch the torso and base
        newton_mjwarp.solver_cfg.nconmax = 60
        self.sim.physics.default = newton_mjwarp
        self.scene.robot = MARVIN_LITE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.table = None
        self.scene.ground = AssetBaseCfg(
            prim_path="/World/ground",
            spawn=self.scene.ground.spawn,
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
        )
        self.scene.env_spacing = 3.0
        # targets in the base frame (on the floor at the column): in front and to the left, from
        # a table's height to above the shoulder; position only
        cmd = self.commands.ee_pose
        cmd.body_name = HAND
        cmd.ranges.pos_x = (0.25, 0.60)
        cmd.ranges.pos_y = (-0.05, 0.50)
        cmd.ranges.pos_z = (0.75, 1.45)
        cmd.ranges.roll = (0.0, 0.0)
        cmd.ranges.pitch = (0.0, 0.0)
        cmd.ranges.yaw = (0.0, 0.0)
        cmd.orientation_success_threshold = 4.0  # orientation is not asked for
        # the column's target stays inside its travel: past its stops the stiff spindle model
        # would push on them at full force
        lo, hi = _robot_column_range()
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=JOINTS,
            scale=0.5,
            use_default_offset=True,
            clip={"column_lift": (lo, hi)},
        )
        r = self.rewards
        r.end_effector_position_tracking.params["asset_cfg"].body_names = [HAND]
        r.end_effector_orientation_tracking = None
        r.end_effector_position_tracking_fine = RewTerm(
            func=mdp.position_command_error_tanh,
            weight=1.0,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=[HAND]),
                "std": 0.05,
                "command_name": "ee_pose",
            },
        )
        # hold the hand on the target for the whole 4 s: the base task ends an episode with a
        # bonus as soon as the hand is within 5 cm, so its policies never learn to stay there
        self.terminations.success = None
        r.success = None
        r.com_offset = RewTerm(
            func=com_offset,
            weight=-5.0,
            params={"asset_cfg": SceneEntityCfg("robot"), "free": COM_FREE},
        )
        r.joint_vel.params["asset_cfg"] = SceneEntityCfg("robot", joint_names=JOINTS)
        self.events.reset_robot_joints.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=JOINTS[1:]
        )
