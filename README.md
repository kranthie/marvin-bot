# Marvin Bot

An open humanoid robot, about 1.2 m tall and 35 kg, with 25 powered joints. It is built from off-the-shelf quasi-direct-drive actuators, a CNC-machined aluminium frame and printed structural shells, and uses an NVIDIA Jetson AGX Orin for onboard compute.

**Status:** phase 1, robot in simulation. Nothing has been built yet. See [docs/roadmap.md](docs/roadmap.md).

![Concept layout v0.2](docs/design/concept_v0.svg)

## Goals

- Walk, turn and stop on flat ground and rough terrain; climb standard household stairs (17–18 cm steps).
- Carry and manipulate light objects (1–2 kg) with two 5-DOF arms.
- Look good with or without its outer shell: the shells are part of the structure.
- Stay buildable by one person with a desktop CNC mill and a desktop 3D printer, for roughly $10–12k in parts.
- Be fully open: CAD, electronics, firmware, simulation and training code.

The full list is in [docs/requirements.md](docs/requirements.md).

## Design at a glance

| | |
|---|---|
| Height, mass | ~1.22 m, ≤35 kg target (current estimate in [description/mass_budget.yaml](description/mass_budget.yaml)) |
| Joints | 6 per leg, 5 per arm, waist yaw, 2 neck, plus a 1-DOF gripper per arm |
| Actuators | Robstride RS04 (hip pitch and roll, knees), RS03 (hip yaw, ankles, waist, shoulders), RS06 (shoulder yaw, elbows), RS00 (wrists); serial-bus servos for neck and grippers |
| Compute | Jetson AGX Orin 64 GB; desktop GPU for training and, at first, for the vision-language-action model |
| Sensing | Pelvis IMU, head-mounted active-stereo depth camera, joint encoders |
| Power | 48 V nominal (13S Li-ion, ~630 Wh), quick-release pack with a keep-alive supply for the computer |
| Comms | 6 CAN buses for the actuators, 1 auxiliary CAN bus for the battery |
| Software | Isaac Lab for training, MuJoCo as a cross-check, ROS 2 Jazzy on the robot, GR00T N1.7 for manipulation |

The joint list, ranges and actuators are defined in [description/robot.yaml](description/robot.yaml), the single source of truth for everything else.

## Repository layout

```
docs/          plan, requirements, roadmap, safety, decisions, design notes
description/   robot.yaml (joints, sizes, CAN buses), actuators.yaml, mass_budget.yaml,
               skeleton.yaml (frames, mass placement), urdf/ (generated)
bom/           bill of materials (bom.csv)
tools/         checks and generators (description check, BOM summary, URDF, concept sketch)
sim/mujoco/    MuJoCo checks (standing, step-up, symmetry, policy cross-check)
sim/isaaclab/  Isaac Lab robot config, walking tasks, robot check, policy export
```

Folders for CAD, electronics, the onboard software, simulation and bench tests are added as each phase reaches them; [docs/plan.md](docs/plan.md) describes the full layout.

## Checks

```bash
pip install -e ".[dev]"
ruff check . && ruff format --check .
python tools/check_description.py   # joints, actuators, CAN load, mass budget, BOM counts
python tools/bom_summary.py         # BOM totals by phase and group
python tools/build_urdf.py --check  # URDF matches the description files

pip install -e ".[sim]"
python sim/mujoco/check_standing.py # stands in MuJoCo; joint torques against ratings
```

CI runs the same checks on every push.

## Licences

- Software and documentation: [Apache-2.0](LICENSE)
- Hardware designs (CAD, drawings, electronics): [CERN-OHL-S-2.0](LICENSES/CERN-OHL-S-2.0.txt)

Vendor CAD models are not redistributed here; see [CONTRIBUTING.md](CONTRIBUTING.md). Credits for the projects and products this design learns from are in [ACKNOWLEDGMENTS.md](ACKNOWLEDGMENTS.md).

## Safety

A 35 kg robot on a 48 V battery can injure people. Read [docs/safety.md](docs/safety.md) before powering any actuator.
