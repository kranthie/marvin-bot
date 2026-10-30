---
title: Tutorials
description: "An interactive guide to humanoid robotics through Marvin Bot: 23 playgrounds from degrees of freedom to sim-to-real, for readers new to the field."
---

The **interactive guide** teaches humanoid robotics from first principles, using Marvin Bot as the worked example. Each tutorial is a *playground*: a page with sliders and a live picture, so you change something (a battery, a joint angle, a reward weight) and see what happens, then read why. Each one explains the idea, shows what Marvin Bot does and the alternatives it considered, and ends with the decisions you could make yourself.

It is written for readers with school maths and physics and no robotics background. [Open the guide's map](/marvin-bot/guide/) for a clickable robot and the learning path, or pick a playground below.

:::note[How the guide is written]
- **Sourced.** Every non-obvious claim links to a numbered source: papers, official documentation and open-source projects. All of them are collected on the [sources page](/marvin-bot/guide/sources.html). Where a published source and the repository disagree, the page says so.
- **Open references only.** Following the project's rules, it compares Marvin Bot only with open-source projects and names bought parts as parts.
- **Dated.** Marvin Bot's numbers are the repository's as of 2026-10-29; each page names the file a number comes from, so it can be checked against the current version.
- **Simplified on purpose.** The playgrounds use small models (2-D statics, first-order heat models, illustrative gaits) to teach an idea. Each says what it leaves out; none replaces the project's own checks.
:::

## Start here

- [**00 · Marvin Bot in 3-D**](/marvin-bot/guide/playgrounds/00-3d-model.html): Turn the real model around, X-ray the motors, electronics and battery, step through the assembly order, and play skills with live motor loads.

## Stage 1: Foundations

- [**01 · Degrees of freedom and kinematic chains**](/marvin-bot/guide/playgrounds/01-dof-kinematics.html): Joints, ranges and how 27 of them add up; bend a leg and see where the foot goes.
- [**02 · Balance: centre of mass, support polygon, ZMP and capture point**](/marvin-bot/guide/playgrounds/02-balance.html): Why a robot stays up, why it falls, and where to step to recover from a push.
- [**03 · System architecture: from battery to policy**](/marvin-bot/guide/playgrounds/03-architecture.html): Every block from battery to neural network, and one 20 ms control tick traced end to end.

## Stage 2: Mechanics

- [**04 · Actuators: quasi-direct drive, torque, speed and heat**](/marvin-bot/guide/playgrounds/04-actuators.html): Gear ratios, torque–speed curves, rated vs peak torque, motor heating, and why each RobStride model sits where it does.
- [**05 · Leg loads and stairs**](/marvin-bot/guide/playgrounds/05-leg-loads-stairs.html): Bend a leg on a stair and watch the knee torque climb past its rating.
- [**06 · The parallel-linkage ankle**](/marvin-bot/guide/playgrounds/06-ankle-linkage.html): Two motors on the shin push rods that tilt the foot; see how they share pitch and roll.
- [**07 · Mass budget and proportions**](/marvin-bot/guide/playgrounds/07-mass-proportions.html): Where the 41 kg goes, how scaling changes torque, and the 1.44 m redesign.
- [**08 · Structure and code-CAD**](/marvin-bot/guide/playgrounds/08-structure-cad.html): Aluminium or plastic, safety factors, pocketed plates, and sweeping joints for collisions.

## Stage 3: Electrical, sensing and safety

- [**09 · Power and battery**](/marvin-bot/guide/playgrounds/09-power-battery.html): Runtime calculator, regenerative-braking voltage margin, and the power path from pack to motors.
- [**10 · CAN buses and bandwidth**](/marvin-bot/guide/playgrounds/10-can-bus.html): How many motors fit on one bus at 500 Hz, worked out frame by frame.
- [**11 · Sensors and compute**](/marvin-bot/guide/playgrounds/11-sensors-compute.html): IMU noise and delay, stereo depth error, and the Jetson AGX Orin compute budget.
- [**12 · Safety**](/marvin-bot/guide/playgrounds/12-safety.html): Fall energy, pinch hazards, the e-stop chain and what catches each fault.

## Stage 4: Model and simulation

- [**13 · The robot description: one source of truth**](/marvin-bot/guide/playgrounds/13-description.html): robot.yaml becomes URDF and MJCF; explore the coupled shoulder limit.
- [**14 · Simulation: MuJoCo, Isaac Lab, Newton and PhysX**](/marvin-bot/guide/playgrounds/14-simulation.html): Timesteps, contacts and integrators, and why a second simulator cross-checks the first.
- [**15 · PD control, gains and armature**](/marvin-bot/guide/playgrounds/15-pd-control.html): Tune a joint’s stiffness and damping and watch it overshoot, settle or go unstable.

## Stage 5: Learning

- [**16 · Reinforcement learning for walking**](/marvin-bot/guide/playgrounds/16-rl-walking.html): PPO, observations, actions and a reward-shaping sandbox where you can make the robot cheat.
- [**17 · Motion tracking: Kimodo, retargeting and BeyondMimic**](/marvin-bot/guide/playgrounds/17-motion-tracking.html): From a text prompt to a tracked skill: scale a human onto the robot and tune reward kernels.
- [**18 · Whole-body control and VLA models**](/marvin-bot/guide/playgrounds/18-wbc-vla.html): Decoupled control versus one general tracker, and how vision-language-action models drive the arms.
- [**19 · Hands**](/marvin-bot/guide/playgrounds/19-hands.html): The four-finger hands, the wrist, and how much the arm can actually hold.

## Stage 6: Toward hardware

- [**20 · Sim-to-real and system identification**](/marvin-bot/guide/playgrounds/20-sim-to-real.html): Fit a motor model to bench data and see why randomisation ranges matter.
- [**21 · Bill of materials and costs**](/marvin-bot/guide/playgrounds/21-bom-costs.html): Explore the ≈$17.5k bill of materials by group, phase and cost per newton-metre.
- [**22 · Build phases and roadmap**](/marvin-bot/guide/playgrounds/22-roadmap.html): Seven phases from foundations to perception, what each must prove, and which risks it retires.

## Reference pages

- [Glossary](/marvin-bot/guide/glossary.html): every term the playgrounds use, in plain language.
- [Sources](/marvin-bot/guide/sources.html): every cited paper, datasheet, document and project, with what it was used for.

## Improving the guide

The guide is plain HTML in `site/public/guide/`, one self-contained page per playground with no build step. Corrections are welcome as issues or pull requests (see [contributing](/marvin-bot/project/contributing/)); please cite a source for any change to a number, and keep to the project's naming rules. The guide is part of the project's documentation and shares its Apache-2.0 licence.
