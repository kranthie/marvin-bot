# Marvin Bot

An open humanoid robot, about 1.29 m tall and 40 kg, with 27 powered joints and two 8-servo four-finger hands. It is built from off-the-shelf quasi-direct-drive actuators, a CNC-machined aluminium frame and printed structural shells, and uses an NVIDIA Jetson AGX Orin for onboard compute.

**Status:** phase 1, robot in simulation. Nothing has been built yet. The full robot is modelled in code-CAD, and walking and stair-climbing policies trained in Isaac Lab pass a cross-check in MuJoCo; a motion-tracking pipeline (human motion to robot skills) is being brought up. See [docs/roadmap.md](docs/roadmap.md).

> [!WARNING]
> **Work in progress, not yet built or tested on hardware.** The designs, policies and scripts
> here are shared for reference, research and learning. They have only run in simulation; the
> motor driver and bench scripts are untested. A robot of this size (about 40 kg, 48 V,
> joints up to 120 N·m) can cause serious injury, fire or damage if built or run wrongly. Build
> or power any part of it only if you are experienced with high-power robotics and batteries,
> follow [docs/safety.md](docs/safety.md), and at your own risk: everything is provided as is,
> without warranty, under the licences below. Report safety issues privately
> ([SECURITY.md](SECURITY.md)).

![Marvin Bot, bare structure and with its clip-on covers (MuJoCo render of the generated model)](docs/media/robot.webp)

## In simulation

| Walking, turning and stopping (flat v7) | 15 cm stairs up and down (rough v9) |
|---|---|
| ![Flat-ground policy in MuJoCo](docs/media/flat_walk.webp) | ![Stair policy in MuJoCo](docs/media/stairs_15cm.webp) |

Policies are trained in Isaac Lab on Newton and replayed in MuJoCo with sensor noise (`sim/mujoco/check_policy.py --render`, palms turned in with `--wrist-yaw 90`); results for every run are in [sim/isaaclab/README.md](sim/isaaclab/README.md). Both were trained before the hands were added; the clips replay them on the current model, hands included, where they still pass their checks.

## Goals

- Walk, turn and stop on flat ground and rough terrain; climb standard household stairs (17–18 cm steps); recover from pushes and get up after a fall.
- Carry and manipulate light objects with two 6-DOF arms and four-finger hands (1–2 kg is the goal; the v1 hands manage about 0.5 kg), learned from teleoperated demonstrations with a vision-language-action model (GR00T N1.7).
- Learn skills beyond walking by tracking human motion (gestures, whole-body teleoperation), trained in simulation before the robot is built ([roadmap](docs/roadmap.md#skills-beyond-walking-simulation-before-and-during-the-build)).
- Look finished without its clip-on covers: the visible parts are the structure, and the covers are an optional add-on.
- Stay buildable by one person with a desktop CNC mill and a desktop 3D printer, for about $12k in parts (the current BOM).
- Be fully open: CAD, electronics, firmware, simulation and training code.

The full list is in [docs/requirements.md](docs/requirements.md).

## Built on

Marvin Bot is a new design, but it stands on open work by others:

- **Layout and sizing:** the open humanoids [Asimov 1](https://github.com/menloresearch/asimov-1) and [Roboto Origin](https://github.com/Roboparty/roboto_origin) (ideas and reference figures only), with lessons on low-cost building from [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) and [ToddlerBot](https://github.com/hshi74/toddlerbot).
- **Hands:** the [AmazingHand](https://github.com/pollen-robotics/AmazingHand) by Pollen Robotics (CC-BY-4.0), bought as kits; its meshes are in `description/third_party/`.
- **Training and simulation:** NVIDIA [Isaac Lab](https://github.com/isaac-sim/IsaacLab) and Newton, [ProtoMotions](https://github.com/NVlabs/ProtoMotions) with [PyRoki](https://github.com/chungmin99/pyroki) for motion tracking, [MuJoCo](https://github.com/google-deepmind/mujoco) for the cross-checks, and [GR00T](https://github.com/NVIDIA/Isaac-GR00T) planned for manipulation.
- **Actuators:** [RobStride](https://github.com/RobStride/Product_Information)'s published manuals and specifications.

Every source, with its licence and what was used from it, is in [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md).

## Design at a glance

| | |
|---|---|
| Height, mass | ~1.29 m; 39.9 kg bare and 40.7 kg with the body shells (estimate, `tools/check_description.py`); 35 kg target bare, 41 kg limit with the shells |
| Joints | 6 per leg, 6 per arm (wrist yaw and pitch), waist yaw, 2 neck, plus an 8-DOF four-finger hand per arm |
| Actuators | Robstride RS04 (hip pitch and roll, knees), RS03 (hip yaw, ankles, waist, shoulders), RS06 (shoulder yaw, elbows), RS00 (wrists); serial-bus servos for the neck and wrist pitch; eight small servos in each hand |
| Compute | Jetson AGX Orin 64 GB; desktop GPU for training and, at first, for the vision-language-action model |
| Sensing | Pelvis IMU, head-mounted active-stereo depth camera, joint encoders |
| Power | 48 V nominal (13S Li-ion, ~630 Wh), quick-release pack with a keep-alive supply for the computer |
| Comms | 6 CAN buses for the actuators, 1 auxiliary CAN bus for the battery; Feetech serial buses for the neck and wrist servos and for each hand |
| Software | Isaac Lab for training, MuJoCo as a cross-check, ProtoMotions for motion tracking, ROS 2 Jazzy on the robot, GR00T N1.7 for manipulation |

The joint list, ranges and actuators are defined in [description/robot.yaml](description/robot.yaml), the single source of truth for everything else.

## Repository layout

```
docs/          plan, requirements, roadmap, safety, decisions, design notes, media (renders, clips)
description/   robot.yaml (joints, sizes, CAN buses), actuators.yaml, mass_budget.yaml,
               skeleton.yaml (frames, mass placement), urdf/ (generated),
               third_party/ (AmazingHand meshes, CC-BY-4.0)
cad/           code-CAD part generators (legs, pelvis, torso, arms and hand mounts, head, covers),
               the mass and size build, and the strength check
electrical/    wiring harness (harness.yaml, generated by tools/build_harness.py)
bom/           bill of materials (bom.csv); generated build lists of made parts and actuator bolts
tools/         checks and generators (description check, BOM summary, URDF, harness, hand
               meshes, README render, concept sketch)
sim/mujoco/    MuJoCo checks (standing, step-up, symmetry, policy cross-check)
sim/isaaclab/  Isaac Lab robot config, walking tasks, robot check, policy export
sim/motion/    motion pipeline: human motion retargeted to Marvin Bot, tracking policies (ProtoMotions)
robot/         onboard software (Robstride CAN driver so far)
bench/         actuator bench tests (phase 2)
tests/         unit tests
```

Folders for CAD, electronics, the onboard software, simulation and bench tests are added as each phase reaches them; [docs/plan.md](docs/plan.md) describes the full layout.

## Checks

```bash
pip install -e ".[dev]"
ruff check . && ruff format --check .
python tools/check_description.py   # joints, actuators, CAN load, mass budget, BOM counts
python tools/bom_summary.py         # BOM totals by phase and group
python tools/build_urdf.py --check  # URDF matches the description files
python tools/build_harness.py       # wiring harness routes, gauges and fuses (electrical/)

pip install -e ".[cad]"
python -m cad.build                 # part meshes, masses and inertias; part sizes vs mill and printer
python -m cad.strength              # hand-calc strength check of the frame and printed parts
python -m cad.covers --check        # shells and parts swept through the joint ranges
python -m cad.export                # STL per part, DXF per flat part (outputs/), build BOM (bom/)

pip install -e ".[sim]"
python sim/mujoco/check_standing.py # stands in MuJoCo; joint torques against ratings
python tools/render_robot.py        # the robot picture above (docs/media/robot.webp)
```

CI runs the same checks on every push.

## Licences

- Software and documentation: [Apache-2.0](LICENSE)
- Hardware designs (CAD, drawings, electronics): [CERN-OHL-S-2.0](LICENSES/CERN-OHL-S-2.0.txt)

Third-party files keep their own licences: the AmazingHand meshes in [description/third_party/](description/third_party/amazing_hand/README.md) are CC-BY-4.0 (Pollen Robotics). Vendor CAD models are not redistributed here; see [CONTRIBUTING.md](CONTRIBUTING.md). Credits for the projects and products this design learns from are in [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md).

## Safety

A 40 kg robot on a 48 V battery can injure people. Read [docs/safety.md](docs/safety.md) before powering any actuator. Report safety or security issues privately, as described in [SECURITY.md](SECURITY.md).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). Everyone taking part is expected to follow the [code of conduct](CODE_OF_CONDUCT.md).
