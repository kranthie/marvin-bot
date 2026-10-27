# Acknowledgments

Marvin Bot is designed from scratch, but it learns from many open projects, published specifications and tools. This file credits them. Nothing from them is copied into this repository unless a line below says so and the licence allows it.

## Humanoid designs (ideas and reference figures)

- **Asimov 1** by Menlo Research: https://github.com/menloresearch/asimov-1 (hardware CERN-OHL-S-2.0, software GPL-2.0). The main reference for the overall layout: a 1.2 m, 35 kg, 25-DOF humanoid with 6-DOF legs, 5-DOF arms, waist yaw and a 2-DOF neck, a two-motor parallel-linkage ankle, and per-joint torque classes. Used for ideas and sizing only; no Asimov CAD, drawings or code are used here.
- **Roboto Origin** by RoboParty: https://github.com/Roboparty/roboto_origin (hardware and robot description CERN-OHL-W-2.0, training code BSD-3-Clause, onboard software GPL-3.0). An open 1.25 m, 34 kg, 23-DOF humanoid; reference for an all-machined structure, two-motor ankles with a parallel linkage, an off-the-shelf 48 V pack, a published BOM and per-joint actuator gains for Isaac Lab. Used for ideas and reference figures only.
- **Berkeley Humanoid Lite** (UC Berkeley Hybrid Robotics): https://github.com/HybridRobotics/Berkeley-Humanoid-Lite and arXiv:2504.17249. Lessons on low-cost actuation, printed structures that route load through metal fasteners, and sim-to-real training.
- **ToddlerBot** (Stanford University): https://github.com/hshi74/toddlerbot. Lessons on zero-point calibration and data collection for a low-cost open humanoid.
- **K-Scale Labs** (K-Bot): https://github.com/kscalelabs. Reference for an all-Robstride humanoid and open actuator drivers.
- **Unitree Robotics**: published specifications of the G1 and R1 (https://www.unitree.com) used as size, mass and joint-range benchmarks; the cost-driven design idea of actuator housings carrying structural load.
- **Booster Robotics**: published specifications of the K1 (https://www.booster.tech) and its design language, where structural limb shells and exposed actuator caps make the robot look finished without a separate cover.

## Software and models

- **NVIDIA Isaac Lab, Isaac Sim, Newton and Isaac ROS** (https://github.com/isaac-sim/IsaacLab): training environments and the onboard perception stack.
- **NVIDIA Isaac GR00T N1.7** (https://github.com/NVIDIA/Isaac-GR00T) and **GR00T-WholeBodyControl** (https://github.com/NVlabs/GR00T-WholeBodyControl): the planned vision-language-action model and the decoupled whole-body control approach.
- **MuJoCo** and **MuJoCo Menagerie** (Google DeepMind): simulation cross-check.
- **rsl_rl** (ETH Zurich Robotic Systems Lab): PPO training.
- **ProtoMotions** (NVIDIA, https://github.com/NVlabs/ProtoMotions, Apache-2.0): the motion pipeline in `sim/motion/` runs its keypoint extraction, PyRoki retargeter (G1 version, with Marvin Bot's link map and scaling), converters, motion library and motion-tracking training. Its sample human motions come from the BONES-SEED dataset (Bones Studio, gated licence); they are used locally and not redistributed.
- **PyRoki** (https://github.com/chungmin99/pyroki, MIT): the kinematics optimiser behind the retargeter.
- **BeyondMimic** (https://github.com/HybridRobotics/whole_body_tracking, arXiv:2508.08241): the motion-tracking formulation ProtoMotions' tracker follows. **Kimodo** (NVIDIA, https://github.com/nv-tlabs/kimodo) and **GMR** (https://github.com/YanjieZe/GMR): planned motion generation and an alternative retargeter.
- **NVIDIA Cosmos** (world models and open physical-AI datasets), **GR00T-Mimic** and **SONIC** (GR00T-WholeBodyControl): planned sources of synthetic manipulation data and of a general whole-body tracker.
- Papers behind the planned skills in `docs/roadmap.md`: HoST (arXiv 2502.08378) and arXiv 2502.12152 (getting up), TWIST and HOMIE (teleoperation and loco-manipulation), Hiking in the Wild (arXiv 2601.07718) and PIM (arXiv 2411.14386) (stairs from depth), PhysHSI (arXiv 2510.11072) (carrying and sitting), PAiD (arXiv 2602.05310) and the RoboCup 2025 soccer work (kicking). Ideas only; no code from them is used.
- **mjlab** (https://github.com/mujocolab/mjlab, Apache-2.0): its Unitree G1 velocity task showed the anti-sway terms used in the walking rewards (an upright term and tight hip and ankle roll while walking, no single-support reward).
- **bd_warehouse** (https://github.com/gumyr/bd_warehouse, Apache-2.0) and **build123d**: planned sources of parametric fasteners, bearings and code-CAD.

## Component specifications

- **RobStride** official manuals and specifications: https://github.com/RobStride/Product_Information. Actuator figures in `description/actuators.yaml` come from these manuals unless marked otherwise.
- **Feetech** STS3250 datasheet (neck and wrist pitch servos).
- The open-source 8-DOF four-finger hand kit sold by **Seeed Studio** (SKUs 100062181 and 100063642): its design is CC-BY-4.0 and its code Apache-2.0. Its published mass and outline are used for the envelope in `cad/arm.py`; none of its CAD or code is copied here.
- **Orbbec** Gemini 335L product documentation (head depth camera).

## Predecessor

- **an earlier prototype**, the project's 66 cm servo-driven humanoid, which developed the training pipeline (Isaac Lab velocity and rough-terrain tasks, symmetry augmentation, MuJoCo cross-checks, ONNX export) that this robot reuses.
