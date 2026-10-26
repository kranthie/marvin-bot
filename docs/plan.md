# Plan

Marvin Bot is built in stages. Each stage ends with a pass check that decides whether to move on, and parts are bought only when the next stage needs them, so a wrong assumption costs one stage rather than the whole budget.

## Principles

1. **Simulation before hardware.** The robot must stand, walk and climb stairs in simulation, with torques inside the actuators' ratings, before more than two test actuators are bought.
2. **Test the riskiest part with the smallest build:** one actuator, then one leg, then the lower body, then the full robot.
3. **Buy in stages.** Spend is committed one phase at a time (see `bom/bom.csv`, column `phase`).
4. **Measure, then update the model.** Every hardware stage measures the real thing (torque, speed, heat, latency, mass) and feeds it back into `description/` and the simulation.

## Phases

| # | Phase | Work | Pass check | Indicative time (part-time) |
|---|---|---|---|---|
| 0 | Foundations | Repo, requirements, decision log, licences, v0 design: joints, sizes, mass budget, BOM | Requirements and v0 layout agreed | 1 week |
| 1 | Robot in simulation | Code-CAD of the frame, URDF and masses generated from it; standing, step-up and torque/thermal checks; walking and stair policies in Isaac Lab, cross-checked in MuJoCo | Walks and climbs 15–18 cm steps in simulation; peak torques ≤70% of rating; continuous torques within rating | 3–5 weeks |
| 2 | Actuator bench | One RS04 and one RS03, CAN adapter, bench supply with regen clamp. Measure torque constant, speed, heating, CAN timing, built-in position control; fit actuator models and feed them back into simulation | Manual figures confirmed; models match measurements; CAN at the planned rates works on the Orin | 2–3 weeks |
| 3 | One leg | Remaining actuators for one leg; machine and print one leg; test on a jig: trajectory tracking, ankle linkage range, gait-like loads for heat, simple hops | Leg tracks simulated motion; linkage reaches its range; nothing overheats | 4–6 weeks |
| 4 | Lower body | Second leg, pelvis, battery and power system, e-stop, Orin, IMU. Stand on a gantry, walk on the gantry, then walk free | Walks at 0.5 m/s, turns, stops, recovers from pushes, handles low steps | 6–8 weeks |
| 5 | Torso, arms, head | Waist, arms, grippers, neck, camera; whole-body controller; fall recovery | Walks while carrying objects; gets up after a fall | 4–6 weeks |
| 6 | Perception and intelligence | Terrain map from the depth camera for stairs, teleoperation, data collection, GR00T N1.7 fine-tuning; finish the shells | Climbs real stairs using vision; picks up objects on command | ongoing |

## Workstreams

- **Mechanical:** code-CAD part generators, CNC aluminium cores, printed structural shells.
- **Electrical:** battery and BMS, power distribution, e-stop chain, CAN wiring.
- **Onboard software:** a real-time control process on the Orin (CAN I/O, policy inference at 50 Hz, safety) kept outside ROS; ROS 2 around it for the camera, logging and teleoperation.
- **Learning:** Isaac Lab tasks (velocity, rough terrain, stairs), MuJoCo cross-check, ONNX export; later motion tracking and the VLA.
- **Docs:** decision log, BOM, build guides, test results.

## Repository growth

Folders are added as phases reach them:

```
docs/           plan, requirements, roadmap, decisions, safety, design/, build/ (phase 3+)
description/    robot.yaml, actuators.yaml, mass_budget.yaml, urdf/ (generated, phase 1)
cad/            code-CAD part generators, purchased-part envelopes, print and machining lists (phase 1)
electrical/     power, wiring and CAN maps, custom boards (phase 2+)
bom/            bom.csv
robot/          onboard software: control loop, drivers, ROS 2 packages, config (phase 2+)
sim/            Isaac Lab tasks, MuJoCo checks, exported policies (phase 1)
bench/          bench-test scripts and summarised results (phase 2+)
tools/          generators and checks
```
