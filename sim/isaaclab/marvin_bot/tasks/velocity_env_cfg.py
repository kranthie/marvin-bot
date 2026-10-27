# Portions adapted from Isaac Lab (https://github.com/isaac-sim/IsaacLab), Copyright (c) 2022-2026,
# The Isaac Lab Project Developers, BSD-3-Clause (LICENSES/BSD-3-Clause-IsaacLab.txt). Modified for
# Marvin Bot; the rest Apache-2.0.
# SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
"""Velocity-tracking walking tasks for Marvin Bot's legs, after Isaac Lab's G1 velocity
tasks.

MarvinBot-Velocity-Rough trains on stairs, slopes and rough ground (marvin_bot/tasks/terrains.py)
with a terrain curriculum; MarvinBot-Velocity-Flat is the same task on a plane. Both widen the
forward-speed command as tracking improves, and train with left-right mirror augmentation.

Decoupled WBC style (see GR00T-WholeBodyControl): the policy drives the 12 leg joints only. The
waist, neck and arms are held by their servos, and the arm and waist targets jump to random poses
every few seconds so the legs learn to balance while the upper body moves (later driven by IK).

The actor sees only what the robot has: the pelvis IMU gyro, gravity from the IMU orientation, the
command, joint positions and velocities, its last action and, on rough ground, a height map
around the feet as the robot's terrain map would give it (camera depth fused with odometry, with
drift, holes, noise and 10 Hz updates; observations.py). The IMU and joint readings reach it
up to one control step (20 ms) late, a random lag per episode. The critic also gets the base
linear velocity and the exact height map.
"""

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import ImuCfg, RayCasterCfg, patterns
from isaaclab.utils import configclass, replace
from isaaclab.utils.noise import UniformNoiseCfg as Unoise
from isaaclab_tasks.core.velocity import mdp
from isaaclab_tasks.core.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)

from ..robot_cfg import (
    ARM_JOINTS,
    FEET,
    IMU_BODY,
    IMU_OFFSET,
    LEG_JOINTS,
    MARVIN_BOT_CFG,
    STANCE_PELVIS_Z,
)
from .curriculums import forward_speed_levels, terrain_levels_tracking
from .events import randomize_joint_targets
from .observations import mapped_height_scan
from .rewards import (
    feet_flat,
    feet_width,
    foot_clearance,
    forward_progress,
    pelvis_height,
    pelvis_roll,
    stand_pose,
    sustained_torque,
    swing_feet_level,
)
from .terminations import base_below_ground
from .terrains import MARVIN_BOT_ROUGH_TERRAINS_CFG

BODIES = "Geometry"
UPPER_BODY = [*ARM_JOINTS, "waist_yaw"]
PELVIS_HEIGHT = STANCE_PELVIS_Z  # m, default stance
SCAN_AHEAD = 0.25  # m, height map centre ahead of the pelvis
FOOT_FRAME_ABOVE_SOLE = 0.060  # m (description/skeleton.yaml)
OBS_LAG = {"delay_min_lag": 0, "delay_max_lag": 1}  # control steps, sampled per episode


@configclass
class MarvinObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        base_ang_vel = ObsTerm(
            func=mdp.imu_ang_vel,
            params={"asset_cfg": SceneEntityCfg("imu")},
            noise=Unoise(n_min=-0.2, n_max=0.2),
            **OBS_LAG,
        )
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05), **OBS_LAG
        )
        velocity_commands = ObsTerm(
            func=mdp.generated_commands, params={"command_name": "base_velocity"}
        )
        joint_pos = ObsTerm(
            func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01), **OBS_LAG
        )
        joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-0.5, n_max=0.5), **OBS_LAG)
        actions = ObsTerm(func=mdp.last_action)
        # rough ground: the terrain map around the feet, with map errors (observations.py)
        height_scan = ObsTerm(
            func=mapped_height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner"), "offset": PELVIS_HEIGHT},
            noise=Unoise(n_min=-0.01, n_max=0.01),
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    @configclass
    class CriticCfg(PolicyCfg):
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner"), "offset": PELVIS_HEIGHT},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class MarvinBotRewards(RewardsCfg):
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_yaw_frame_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.25},
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_world_exp,
        weight=1.5,
        params={"command_name": "base_velocity", "std": 0.5},
    )
    # single-support time: at 2.0 the policies dropped double support (0-2% of the stride) and
    # rocked their whole weight across onto each foot
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight=0.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
            "threshold": 0.35,
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.1,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
        },
    )
    dof_pos_limits = RewTerm(
        func=mdp.joint_pos_limits,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=[".*_ankle_.*", ".*_knee"])},
    )
    # gait shaping (a first policy in earlier experiments stepped at ~9 steps/s, knee at stall)
    joint_vel_legs = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1.0e-3,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    torque_limits = RewTerm(
        func=mdp.applied_torque_limits,
        weight=-0.2,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    # gait symmetry and clearance (earlier walking policies limped and shuffled)
    feet_air_time_variance = RewTerm(
        func=mdp.feet_air_time_variance,
        weight=-1.0,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
        },
    )
    foot_clearance = RewTerm(
        func=foot_clearance,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
            "command_name": "base_velocity",
            "target_height": 0.08,
            "std": 0.001,
        },
    )
    feet_flat = RewTerm(
        func=feet_flat,
        weight=-2.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
        },
    )
    swing_feet_level = RewTerm(
        func=swing_feet_level,
        weight=-2.0,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=FEET),
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
        },
    )
    # progress along the command, paid linearly so a slow stair climb still earns (rough ground)
    forward_progress = RewTerm(
        func=forward_progress, weight=0.0, params={"command_name": "base_velocity"}
    )
    # heat: holding leg torque above the continuous rating (a slow stair step put the knee at
    # ~108% of its rating in the static check, sim/mujoco/check_step_up.py)
    sustained_torque = RewTerm(
        func=sustained_torque,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS)},
    )
    # posture: walk and climb upright, stand in the stance pose (rough v6 stood with its knees
    # at 95 degrees, its pelvis 19 cm low)
    pelvis_height = RewTerm(
        func=pelvis_height,
        weight=-10.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
            "target": PELVIS_HEIGHT - FOOT_FRAME_ABOVE_SOLE,
            "tolerance": 0.02,
        },
    )
    stand_pose = RewTerm(
        func=stand_pose,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINTS),
            "command_name": "base_velocity",
        },
    )
    # side-to-side sway: flat v6 and rough v8 leaned 7-8 degrees over each stance foot with
    # their feet ~20 cm apart (docs/decisions.md). Keep the pelvis level, the feet nearer the
    # centre line, and the hip and ankle roll near their stance angles
    pelvis_roll = RewTerm(func=pelvis_roll, weight=-10.0)
    feet_width = RewTerm(
        func=feet_width,
        weight=-5.0,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FEET),
            "command_name": "base_velocity",
            "target": 0.16,
            "tolerance": 0.02,
        },
    )
    joint_deviation_hip = RewTerm(
        func=mdp.joint_deviation_l1,
        weight=-0.3,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", joint_names=[".*_hip_yaw", ".*_hip_roll", ".*_ankle_roll"]
            )
        },
    )


@configclass
class MarvinBotRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    observations: MarvinObservationsCfg = MarvinObservationsCfg()
    rewards: MarvinBotRewards = MarvinBotRewards()

    def __post_init__(self):
        super().__post_init__()
        # physics: 15 collision boxes per robot, so modest contact buffers
        newton_mjwarp = self.sim.physics.newton_mjwarp
        newton_mjwarp.solver_cfg.njmax = 400
        newton_mjwarp.solver_cfg.nconmax = 80
        self.sim.physics.default = newton_mjwarp
        # motors computed by Isaac Lab, not Newton's kernels: those take no command delay on a DC
        # motor model (robot_cfg.py's DelayedDCMotor)
        self.sim.use_newton_actuators = False
        # scene
        self.scene.env_spacing = 2.0
        self.scene.robot = replace(MARVIN_BOT_CFG, prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.imu = ImuCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODIES}/{IMU_BODY}",
            offset=ImuCfg.OffsetCfg(pos=IMU_OFFSET),
        )
        # the converted USD nests the links under Geometry/, beside their visual boxes
        self.scene.contact_forces.prim_path = f"{{ENV_REGEX_NS}}/Robot/{BODIES}/.*_link"
        # terrain: household stairs, slopes and rough ground, easiest rows first
        self.scene.terrain.terrain_generator = MARVIN_BOT_ROUGH_TERRAINS_CFG
        self.scene.terrain.max_init_terrain_level = 2
        self.scene.height_scanner = replace(
            self.scene.height_scanner,
            prim_path=f"{{ENV_REGEX_NS}}/Robot/{BODIES}/{IMU_BODY}",
            # 1.4 x 0.8 m grid from 0.45 m behind to 0.95 m ahead of the pelvis, yaw-aligned,
            # refreshed at 10 Hz like a camera-fed terrain map
            offset=RayCasterCfg.OffsetCfg(pos=(SCAN_AHEAD, 0.0, 20.0)),
            pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.4, 0.8]),
            update_period=0.1,
        )
        # actions: legs only
        self.actions.joint_pos.joint_names = LEG_JOINTS
        # stepping up needs bigger, faster leg moves than flat walking: a larger action scale and
        # lighter smoothness penalties than flat ground (the flat task sets its own below)
        self.actions.joint_pos.scale = 0.5
        # commands: start at up to 0.5 m/s and widen forward speed as tracking improves; turn,
        # and stand still in 20% of envs
        cmd = self.commands.base_velocity
        cmd.heading_command = False
        cmd.rel_heading_envs = 0.0
        cmd.rel_standing_envs = 0.2
        cmd.resampling_time_range = (8.0, 12.0)  # long enough to walk off a stair platform
        cmd.marker_pos_offset = (0.0, 0.0, 0.8)
        cmd.ranges.lin_vel_x = (-0.3, 0.5)
        cmd.ranges.lin_vel_y = (-0.3, 0.3)
        cmd.ranges.ang_vel_z = (-1.0, 1.0)
        cmd.ranges.heading = None
        # rewards
        self.rewards.lin_vel_z_l2.weight = 0.0  # climbing stairs moves the pelvis up and down
        self.rewards.flat_orientation_l2.weight = -1.0
        self.rewards.action_rate_l2.weight = -0.01
        self.rewards.joint_vel_legs.weight = -5.0e-4
        # a sharp clearance reward (in earlier experiments a loose one let the feet barely rise)
        self.rewards.foot_clearance.params["std"] = 1.0e-4
        # make standing still in front of stairs a poor answer to a walk command: linear progress
        # reward plus a tracking width that still pays partial speed (at std 0.15 rough v3
        # learned to stop at 10 cm stairs)
        self.rewards.forward_progress.weight = 1.0
        self.rewards.track_lin_vel_xy_exp.params["std"] = 0.25
        self.rewards.termination_penalty.weight = -50.0
        # knees ran at 73-81% of rated on 15 cm stairs (rough v6): heat costs more on rough ground
        self.rewards.sustained_torque.weight = -2.0
        self.rewards.dof_acc_l2.weight = -1.0e-7
        self.rewards.dof_acc_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=[".*_hip_.*", ".*_knee"]
        )
        # G1-class torques (up to 120 N·m), so G1's torque penalty scale
        self.rewards.dof_torques_l2.weight = -2.0e-5
        self.rewards.dof_torques_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", joint_names=LEG_JOINTS
        )
        self.rewards.undesired_contacts.params["sensor_cfg"] = SceneEntityCfg(
            "contact_forces", body_names=[".*_thigh_link", ".*_shin_link"]
        )
        # terminations: a fall is a tilted or low pelvis. Not body contact: with self-collision on,
        # the arms and thighs touch the torso and pelvis (it collapsed training earlier)
        self.terminations.base_contact = None
        self.terminations.bad_orientation = DoneTerm(
            func=mdp.bad_orientation, params={"limit_angle": 1.0}
        )
        self.terminations.base_height = DoneTerm(
            func=base_below_ground, params={"minimum_height": 0.35}
        )
        # curriculum: terrain levels judged by command tracking, and forward speed
        # 8 m tiles with a 2 m central platform: 2 m from the spawn point is past the first stairs
        self.curriculum.terrain_levels = CurrTerm(
            func=terrain_levels_tracking, params={"min_distance": 2.0}
        )
        self.curriculum.forward_speed = CurrTerm(
            func=forward_speed_levels, params={"max_speed": 1.2, "step": 0.05}
        )
        # events: torso payload (dev kit, battery) mass and CoM, friction, pushes
        self.events.physics_material.params["static_friction_range"] = (0.4, 1.0)
        self.events.physics_material.params["dynamic_friction_range"] = (0.4, 0.8)
        self.events.add_base_mass.params["asset_cfg"].body_names = "torso_link"
        self.events.add_base_mass.params["mass_distribution_params"] = (1 / 1.15, 1.15)
        self.events.base_com.params["asset_cfg"].body_names = "torso_link"
        self.events.base_com.params["com_range"] = {
            "x": (-0.03, 0.03),
            "y": (-0.03, 0.03),
            "z": (-0.02, 0.02),
        }
        self.events.base_external_force_torque.params["asset_cfg"].body_names = IMU_BODY
        self.events.reset_base.params["pose_range"] = {
            "x": (-0.5, 0.5),
            "y": (-0.5, 0.5),
            "yaw": (-3.14, 3.14),
        }
        self.events.reset_base.params["velocity_range"] = {
            k: (-0.2, 0.2) for k in ("x", "y", "z", "roll", "pitch", "yaw")
        }
        self.events.reset_robot_joints.params["position_range"] = (0.9, 1.1)
        self.events.push_robot.interval_range_s = (5.0, 10.0)
        self.events.push_robot.params["velocity_range"] = {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}
        self.events.upper_body_motion = EventTerm(
            func=randomize_joint_targets,
            mode="interval",
            interval_range_s=(1.0, 3.0),
            params={"asset_cfg": SceneEntityCfg("robot", joint_names=UPPER_BODY), "fraction": 0.5},
        )

    def play_mode(self):
        super().play_mode()
        self.scene.num_envs = 32


@configclass
class MarvinBotFlatEnvCfg(MarvinBotRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        newton_mjwarp = self.sim.physics.newton_mjwarp
        newton_mjwarp.solver_cfg.njmax = 300
        newton_mjwarp.solver_cfg.nconmax = 60  # feet, plus leg-on-leg and arm-on-body contacts
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.commands.base_velocity.resampling_time_range = (4.0, 8.0)
        self.rewards.lin_vel_z_l2.weight = -0.2
        # flat ground: strong smoothness penalties (as in earlier experiments). Action scale 0.5
        # as on rough ground: at 0.25 the 37 kg robot never found a stepping gait
        self.actions.joint_pos.scale = 0.5
        self.rewards.action_rate_l2.weight = -0.05
        self.rewards.joint_vel_legs.weight = -1.0e-3
        self.rewards.foot_clearance.params["std"] = 0.001
        self.rewards.track_lin_vel_xy_exp.params["std"] = 0.25
        self.rewards.termination_penalty.weight = -200.0
        self.observations.critic.height_scan = None
        self.curriculum.terrain_levels = None
