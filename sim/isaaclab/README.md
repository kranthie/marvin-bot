# Isaac Lab

Marvin Bot's Isaac Lab 3.0 package: the articulation config (`marvin_bot/robot_cfg.py`), the walking tasks, and scripts to convert the URDF, check the robot and export policies. Isaac Lab runs on an RTX desktop GPU or a cloud instance, not on the Jetson.

| File | What it does |
| --- | --- |
| `marvin_bot/robot_cfg.py` | `MARVIN_BOT_CFG`: USD path, DC motor models from `description/actuators.yaml` (the ankles in joint space through the linkage ratios), training gains, joint order and default stance from `robot.yaml`, IMU mount |
| `convert_urdf.py` | Converts `description/urdf/marvin_bot.urdf` to `usd/` (git-ignored) with Isaac Sim's URDF importer |
| `check_robot.py` | Checks mass, joint order and limits, kinematics against MuJoCo, and the default stance |
| `marvin_bot/tasks/` | `MarvinBot-Velocity-Flat` and `MarvinBot-Velocity-Rough` |
| `../mujoco/check_policy.py` | The MuJoCo cross-check of an exported policy: walk, turn, stop, fast walk or stairs, with posture, sway, heat and gait figures; `--render` saves a video |
| `export_policy.py` | Exports a checkpoint to `policy.onnx` plus `policy.json` (joint order, default pose, action scale, observation layout, height-map layout) |

## Setup

Isaac Lab 3.0 (Isaac Sim 6.2, Python 3.12) installs with uv, outside this repo. The results below
used commit `961760bcb91e3af48e7d116292e57e8ecaefc31e` of the moving `release/3.0.0` branch, on one
NVIDIA RTX PRO 6000 Blackwell (96 GB, driver 595); a smaller RTX GPU should work with fewer robots
(`--num_envs`), which has not been tried.

```bash
git clone https://github.com/isaac-sim/IsaacLab.git && cd IsaacLab
git checkout 961760bcb91e3af48e7d116292e57e8ecaefc31e
uv sync --extra isaacsim --extra importers   # Newton is the default backend; isaacsim adds PhysX and Kit
uv pip install --no-deps -e <path to this repo>/sim/isaaclab
export OMNI_KIT_ACCEPT_EULA=YES              # only if you have read and accept NVIDIA's Isaac Sim licence
```

Isaac Sim is NVIDIA software under its own licence, not open source. Read it before the first
launch; setting `OMNI_KIT_ACCEPT_EULA=YES` means you accept it.

`uv sync` removes packages it doesn't manage, so reinstall this package after any sync and use `uv run --no-sync` for everything else.

## Convert and check

The URDF refers to the part meshes, which are generated, not committed: first, in this repo's own
environment, run `pip install -e ".[cad]" && python -m cad.build`.

```bash
uv run --no-sync python <repo>/sim/isaaclab/convert_urdf.py      # after any URDF change
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py       # Newton
uv run --no-sync python <repo>/sim/isaaclab/check_robot.py --backend physx
```

Results (2026-10-26, RTX PRO 6000 Blackwell; before the hands were added, not rerun on the current 40.35 kg model):

| Check | Newton (MuJoCo-Warp) | PhysX |
| --- | --- | --- |
| Total mass | 39.132 kg | 37.322 kg (earlier model, not rerun) |
| Joint order, ranges, effort and velocity limits vs the description files | match | match |
| Link positions vs MuJoCo FK, 8 random poses | worst 0.25 mm | worst 0.17 mm |
| Default stance (knee 15°), 5 s, holding gains | holds, pelvis 0.644 m, tilt 0.0°, worst joint 4% of peak | knee 25° stance (not rerun): 0.634 m, 1.2°, 9% |

The standing check uses the stiff holding gains of `sim/mujoco/check_standing.py`. With the training gains (ankle kp 40) the robot falls without a policy, as expected: the two ankles' stiffness is below m·g·h, and the policy balances actively.

## Walking tasks

Both follow the decoupled whole-body-control split (GR00T-WholeBodyControl):

- **Actions:** the policy drives the 12 leg joints (position targets around the default stance, 50 Hz).
- **Upper body:** the waist and arm targets jump to random poses within half their range every 1–3 s, so the legs learn to balance while the upper body moves.
- **Actor observations:** pelvis IMU gyro, gravity from the IMU orientation, the command, joint positions and velocities, the last action; on rough ground a 1.4 × 0.8 m terrain height map at 0.1 m resolution with map drift, holes, noise and 10 Hz updates.
- **Critic observations:** the same plus base linear velocity and the exact height map.
- **Commands:** forward −0.3 m/s up to a curriculum limit that starts at 0.5 m/s and grows to 1.2 m/s, sideways ±0.3 m/s, yaw ±1.0 rad/s; 20% of envs stand still.
- **Terrain (rough):** household stairs with 5–20 cm risers and 28 cm treads, random boxes to 10 cm, rough ground to 5 cm, slopes to 0.4 rad, with a tracking-and-distance terrain curriculum.
- **Heat:** a penalty on leg joints whose ~1 s running average of (torque / rated torque)² exceeds 0.8, so the policy avoids holding torque above the continuous rating (the static stair check puts the knee at ~118% of its rating on the current model).
- **Posture:** a penalty on the pelvis sitting more than 2 cm below its stance height above the lower foot (so it holds on stairs), on the leg joints' distance from the stance pose while commanded to stand, and on foot tilt, in contact and in swing.
- **No waddle:** a penalty on pelvis roll, on step width beyond 16 ± 2 cm while walking, and on hip yaw, hip roll and ankle roll away from the stance; the air-time reward is small (0.5), since a large one paid for long single-support phases, which the robot spent tipping over the standing foot.
- **Delays:** each robot gets a random lag of 0-20 ms on the IMU and joint observations and 0-10 ms on its motor commands. The delayed motor model runs in Isaac Lab's Python actuator path (`use_newton_actuators = False`), as Newton's own actuators take only the stock motor configs: about 2.5x slower per iteration.
- **Gait shaping, falls, symmetry, randomization:** scaled to this robot: 8 cm swing clearance, fall below 0.35 m pelvis height or past 1 rad of tilt, left-right mirror augmentation (rule checked by `sim/mujoco/check_symmetry.py`), pushes up to 0.5 m/s, torso mass ±15% and CoM ±3 cm.

```bash
uv run --no-sync isaaclab train --rl_library rsl_rl --task MarvinBot-Velocity-Flat --num_envs 4096 --max_iterations 2000
uv run --no-sync isaaclab train --rl_library rsl_rl --task MarvinBot-Velocity-Rough --num_envs 4096 --max_iterations 3000
uv run --no-sync python <repo>/sim/isaaclab/export_policy.py --checkpoint logs/rsl_rl/marvin_bot_flat/<run>/model_1999.pt
uv run --no-sync python <repo>/sim/isaaclab/export_policy.py --task MarvinBot-Velocity-Rough \
    --checkpoint logs/rsl_rl/marvin_bot_rough/<run>/model_2999.pt
# then from this repo, in its own venv (logs/ is in the IsaacLab checkout):
python sim/mujoco/check_policy.py <IsaacLab>/logs/rsl_rl/marvin_bot_flat/<run>/exported --fast 1.0
python sim/mujoco/check_policy.py <IsaacLab>/logs/rsl_rl/marvin_bot_rough/<run>/exported --stairs 0.15
```

Each run takes about 2.2-2.3 s an iteration on the GPU above (about 1.3 h for 2000 iterations),
with rsl_rl's default seed (42). The training logs and checkpoints go to `logs/rsl_rl/` in the
IsaacLab checkout (the rough runs take about 2 GB).

**Reproducing the published policies.** The trained policies are not committed. Flat v7 and rough
v9 were not trained from scratch: each run resumed the previous one (rsl_rl's `--resume`,
`--load_run` and `--checkpoint` options) after a change to the rewards, as the results table below
records (flat v7: about 6000 iterations in all; rough v9: about 14,000), and each stage's task
config is in the git history at that date. Training from scratch with the current config has not
been tried yet, and both policies were trained on the model before the hands (39.13 kg). The
clips in the README replay them with `check_policy.py --render` on the current model.

## Results

| Flat v7 in MuJoCo | Rough v9 on 15 cm stairs |
|---|---|
| ![Flat-ground policy](../../docs/media/flat_walk.webp) | ![Stair policy](../../docs/media/stairs_15cm.webp) |

| Run | Training (Isaac Lab, Newton) | MuJoCo cross-check (`check_policy.py`) |
| --- | --- | --- |
| flat v2 (2026-10-26) | survives 99.6% of episodes, but barely steps (air-time reward ~0) | stands and turns, does not walk: the clearance reward paid planted feet in full, so standing still won. Fixed: only swinging feet earn it; flat action scale 0.25 → 0.5 |
| rough v2 (2026-10-26, pre-CAD masses) | survives 96.6%; terrain curriculum stalled at level ~1 of 10 | **passes flat ground**: walk 0.44 m/s for 0.5, turn 0.75 rad/s for 0.8, walk-and-turn, stop; 1.5 steps/s, symmetric (2%), 7 cm steps, 6 cm foot lift, knee peaks 63% of peak torque. Stops in front of 10 cm stairs |
| flat v3 (2026-10-26, clearance fix, action scale 0.5) | survives 99.3%; speed curriculum reached 1.2 m/s; air-time reward 0.33 (was ~0) | **passes**, including a 0.8 m/s fast walk: walk 0.49 for 0.5 m/s, turn 0.75 for 0.8 rad/s, walk-and-turn exact, stop; 1.5-1.8 steps/s, 5 cm foot lift, symmetric (1-3%); hottest joint ankle pitch at 73% of rated (RMS). Trained on the 37.98 kg first-pass model, checked on the 39.55 kg second-pass leg |
| rough v3 (2026-10-26, clearance fix) | survives 96.5%; terrain curriculum plateaued at level ~1 of 10 (robots learned to wait at stair bottoms: few falls, little promotion) | passes flat ground (walk 0.48, turn, stop); **climbs 5 cm stairs** up and down (pelvis +27 cm); stops in front of 10 and 15 cm stairs. Next (v4): linear progress reward, tracking width 0.25, stairs on 55% of tiles from 4 cm, 6000 iterations |
| rough v4 (2026-10-26) | **collapsed**: 99% of episodes end in falls, terrain level 0. The new progress reward had no upper cap, so lunging forward and falling paid more than walking. Fixed (capped at the commanded speed); retrained as v5 | not usable |
| **rough v5** (2026-10-26, capped progress reward, stair-heavy terrain, heat penalty, 6000 iterations) | survives 97.7%; terrain curriculum reached level ~6 of 10 (v3 stalled at 1) | **climbs 10, 15, 18 and 20 cm stairs** up and down (5 steps each way, 28 cm treads, 0.4 m/s command); passes flat ground (walk 0.51, turn 0.77, stop). Knees on stairs at 73-75% of rated (RMS), hip roll ~60%. 5 cm stairs crossed but pelvis rise 21.5 of 25 cm, just under the check's 90% mark. Trained on the 37.98 kg model, checked on 39.06 kg |
| **flat v4** (2026-10-26, roll-first hip, real actuator outlines, 2000 iterations) | survives 99.5% | **passes**, including a 1.0 m/s fast walk (97 cm steps, symmetric within 1%): walk-and-turn 0.57 for 0.5 m/s, stop; 1.6 steps/s, 6 cm foot lift; hottest joints hip roll ~60% and ankle pitch 58% of rated (RMS), peaks under 70% of peak torque. Trained and checked on the 39.13 kg model |
| rough v6 (2026-10-26, same model as flat v4, 6000 iterations) | survives 97.3%; terrain level 6.0 of 10 (as v5) | passes flat ground and **15 cm stairs** up and down (pelvis +70.8 of 75 cm). Crosses 5 and 10 cm stairs but the pelvis rise (18.5 of 25, 43.5 of 50 cm) is under the check's 90% mark; falls on the way down 18 cm stairs and on the way up 20 cm. Knees at 73-81% of rated (RMS), hotter than v5. Weaker than v5 was on its own model (10-20 cm); rough v5 run on the current model climbs 10 and 15 cm, falls on 18 cm. Next: retrain from v6 with more 16-20 cm stair tiles and a knee heat penalty |
| flat v5 (2026-10-26, from flat v4 + 1000 iterations with the posture terms) | survives 99.8% | **passes**; stands in the stance pose (pelvis 63.7 cm, knees 24°; v4 stood at 18°), walks at 61.4 cm as v4; fast walk 0.87 for 1.0 m/s (v4 1.02), 93 cm steps; hottest joint ankle pitch 64% of rated |
| **rough v7** (2026-10-26, from rough v6 + 3000 iterations: pelvis-height and stand-pose terms, heat penalty x2, steep 14-20 cm stairs on a tenth of the tiles on every row) | survives 97.7%; terrain level 5.9 of 10 | **passes flat ground and 5, 10, 15, 18 and 20 cm stairs** up and down (pelvis rise 23.0/25, 48.0/50, 72.9/75, 87.7/90, 97.8/100 cm). Upright: stands at 63.6 cm with the knees at 24° (v6: 44 cm, 95°); on stairs the pelvis averages 60.5 cm and the knees 53° (v6: 55.8 cm, 67°). Knees at 19% of rated standing (v6 crouched); hottest joint on stairs ankle pitch 64-67% of rated (RMS), knees 73-81% on v6 |
| flat v6 (2026-10-26, from flat v5 + 1000 iterations: knee 15° stance, swing foot-level term) | survives 99.9% | **passes**; stands at 64.5 cm with the knees at 13° (stance 64.5, 15°); walks at 62.6 cm (v5 61.4); feet swing at most 11-16° toe down (v5 29-31°, the tilted heels), land 5-9° heel first; 1.7 steps/s, 62 cm steps; fast walk 0.88 for 1.0 m/s |
| **rough v8** (2026-10-26, from rough v7 + 2000 iterations: knee 15° stance, swing foot-level term) | survives 98.9%; terrain level 6.0 of 10 | **passes flat ground and 5, 10, 15, 18 and 20 cm stairs** up and down (rise measured from the walking height: 25.1/25, 50.1/50, 75.3/75, 90.8/90, 100.2/100 cm). Stands in the stance pose (64.5 cm, 15°); walks and climbs at 60.7 cm, still 3.8 cm under stance, at the edge of the pelvis-height term's 4 cm allowance. Feet swing at most 14-16° toe down (v7 27-29°). Hottest joint on stairs ankle pitch ~60% of rated (RMS). Flat v6 and rough v8 trained on a USD converted before the last CAD changes (38.78 kg, 0.35 kg light); the MuJoCo checks run the current 39.13 kg model, and the USD is now reconverted |
| **flat v7** (2026-10-27, from flat v6 + 2000 iterations: pelvis-roll and step-width terms, hip and ankle roll held, air-time reward 2.0 → 0.5, pelvis allowance 2 cm, 0-20 ms sensor and 0-10 ms motor delays, 39.13 kg USD) | survives 99.9% | **passes**, fast walk 1.01 for 1.0 m/s. The side-to-side wobble is gone: pelvis roll 6.0° peak to peak (v6 20.3°), head sway 7.9 cm (v6 16.5 cm), step width 14.7 cm (v6 20.8). 2.2 steps/s, 0.46 s swing (v6 1.7, 0.6 s); walks at 63.4 cm (v6 62.6) |
| **rough v9** (2026-10-27, from rough v8 + 3000 iterations, same changes as flat v7) | survives 94.7%; terrain level 5.9 of 10 | **passes flat ground and 5, 10, 15, 18 and 20 cm stairs** (seed 0: 18 cm falls, other seeds pass); pelvis roll 6.0° (v8 22.5°), head sway 6.8 cm, step width 15.4 cm; walks and climbs at 62.7-62.8 cm (v8 60.7). Over seeds 0-4 it fails 2 of 15 stair runs from 15 to 20 cm, as v8 does. 2.7 steps/s at 0.5 m/s, 42 cm steps |

Policies are cross-checked against the current URDF, so rough v2 also coped with the CAD leg masses (0.66 kg heavier) it was not trained on.
