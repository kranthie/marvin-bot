# Isaac Lab

Marvin Bot's Isaac Lab 3.0 package: the articulation config (`marvin_bot/robot_cfg.py`), the walking tasks, and scripts to convert the URDF, check the robot and export policies. It is ported from an earlier prototype's package. Isaac Lab runs on an RTX desktop GPU or a cloud instance, not on the Jetson.

| File | What it does |
| --- | --- |
| `marvin_bot/robot_cfg.py` | `MARVIN_BOT_CFG`: USD path, DC motor models from `description/actuators.yaml` (the ankles in joint space through the linkage ratios), training gains, joint order and default stance from `robot.yaml`, IMU mount |
| `convert_urdf.py` | Converts `description/urdf/marvin_bot.urdf` to `usd/` (git-ignored) with Isaac Sim's URDF importer |
| `check_robot.py` | Checks mass, joint order and limits, kinematics against MuJoCo, and the default stance |
| `marvin_bot/tasks/` | `MarvinBot-Velocity-Flat` and `MarvinBot-Velocity-Rough` |
| `export_policy.py` | Exports a checkpoint to `policy.onnx` plus `policy.json` (joint order, default pose, action scale, observation layout, height-map layout) |

## Setup

Isaac Lab 3.0 (branch `release/3.0.0`, Isaac Sim 6.2, Python 3.12) installs with uv, outside this repo:

```bash
git clone https://github.com/isaac-sim/IsaacLab.git --branch release/3.0.0
cd IsaacLab
uv sync --extra isaacsim --extra importers   # Newton is the default backend; isaacsim adds PhysX and Kit
uv pip install --no-deps -e <path to this repo>/sim/isaaclab
export OMNI_KIT_ACCEPT_EULA=YES              # the first Isaac Sim launch asks for the Omniverse EULA
```

`uv sync` removes packages it doesn't manage, so reinstall this package after any sync and use `uv run --no-sync` for everything else.

## Convert and check

```bash
uv run --no-sync python <repo>/sim/isaaclab/convert_urdf.py      # after any URDF change
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py       # Newton
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py --backend physx
```

Results (2026-10-26, RTX PRO 6000 Blackwell):

| Check | Newton (MuJoCo-Warp) | PhysX |
| --- | --- | --- |
| Total mass | 37.322 kg | 37.322 kg |
| Joint order, ranges, effort and velocity limits vs the description files | match | match |
| Link positions vs MuJoCo FK, 8 random poses | worst 0.24 mm | worst 0.17 mm |
| Default stance, 5 s, holding gains | holds, pelvis 0.634 m, tilt 1.3°, worst joint 9% of peak | holds, 0.634 m, 1.2°, 9% |

The standing check uses the stiff holding gains of `sim/mujoco/check_standing.py`. With the training gains (ankle kp 40) the robot falls without a policy, as expected: the two ankles' stiffness is below m·g·h, and the policy balances actively.

## Walking tasks

Both follow the decoupled whole-body-control split (GR00T-WholeBodyControl), as on an earlier prototype:

- **Actions:** the policy drives the 12 leg joints (position targets around the default stance, 50 Hz).
- **Upper body:** the waist and arm targets jump to random poses within half their range every 1–3 s, so the legs learn to balance while the upper body moves.
- **Actor observations:** pelvis IMU gyro, gravity from the IMU orientation, the command, joint positions and velocities, the last action; on rough ground a 1.4 × 0.8 m terrain height map at 0.1 m resolution with map drift, holes, noise and 10 Hz updates.
- **Critic observations:** the same plus base linear velocity and the exact height map.
- **Commands:** forward −0.3 m/s up to a curriculum limit that starts at 0.5 m/s and grows to 1.2 m/s, sideways ±0.3 m/s, yaw ±1.0 rad/s; 20% of envs stand still.
- **Terrain (rough):** household stairs with 5–20 cm risers and 28 cm treads, random boxes to 10 cm, rough ground to 5 cm, slopes to 0.4 rad, with a tracking-and-distance terrain curriculum.
- **Heat:** a penalty on leg joints whose ~1 s running average of (torque / rated torque)² exceeds 0.8, so the policy avoids holding torque above the continuous rating (the static stair check put the knee at ~108%).
- **Gait shaping, falls, symmetry, randomization:** as an earlier prototype's tasks, scaled: 8 cm swing clearance, fall below 0.35 m pelvis height or past 1 rad of tilt, left-right mirror augmentation (rule checked by `sim/mujoco/check_symmetry.py`), pushes up to 0.5 m/s, torso mass ±15% and CoM ±3 cm.

```bash
uv run --no-sync isaaclab train --rl_library rsl_rl --task MarvinBot-Velocity-Flat --num_envs 4096 --max_iterations 2000
uv run --no-sync isaaclab train --rl_library rsl_rl --task MarvinBot-Velocity-Rough --num_envs 4096 --max_iterations 3000
uv run --no-sync python <repo>/sim/isaaclab/export_policy.py --checkpoint logs/rsl_rl/marvin_bot_flat/<run>/model_1999.pt
python sim/mujoco/check_policy.py <run>/exported --fast 1.0       # from the repo, in its own venv
python sim/mujoco/check_policy.py <run>/exported --stairs 0.15    # rough-ground policies
```

## Results

| Run | Training (Isaac Lab, Newton) | MuJoCo cross-check (`check_policy.py`) |
| --- | --- | --- |
| flat v2 (2026-10-26) | survives 99.6% of episodes, but barely steps (air-time reward ~0) | stands and turns, does not walk: the clearance reward paid planted feet in full, so standing still won. Fixed: only swinging feet earn it; flat action scale 0.25 → 0.5 |
| rough v2 (2026-10-26, pre-CAD masses) | survives 96.6%; terrain curriculum stalled at level ~1 of 10 | **passes flat ground**: walk 0.44 m/s for 0.5, turn 0.75 rad/s for 0.8, walk-and-turn, stop; 1.5 steps/s, symmetric (2%), 7 cm steps, 6 cm foot lift, knee peaks 63% of peak torque. Stops in front of 10 cm stairs |
| flat v3 (2026-10-26, clearance fix, action scale 0.5) | survives 99.3%; speed curriculum reached 1.2 m/s; air-time reward 0.33 (was ~0) | **passes**, including a 0.8 m/s fast walk: walk 0.49 for 0.5 m/s, turn 0.75 for 0.8 rad/s, walk-and-turn exact, stop; 1.5-1.8 steps/s, 5 cm foot lift, symmetric (1-3%); hottest joint ankle pitch at 73% of rated (RMS). Trained on the 37.98 kg first-pass model, checked on the 39.55 kg second-pass leg |

Policies are cross-checked against the current URDF, so rough v2 also coped with the CAD leg masses (0.66 kg heavier) it was not trained on.
