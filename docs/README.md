# Documentation

| Document | What it is |
|---|---|
| [plan.md](plan.md) | Phases, their pass checks and the repository's planned layout |
| [roadmap.md](roadmap.md) | Where each phase stands, open items, skills planned beyond walking |
| [requirements.md](requirements.md) | What the robot must and should do |
| [decisions.md](decisions.md) | Every design decision, newest first, with reasons and alternatives (older entries describe the design as it was then) |
| [safety.md](safety.md) | Safety rules for benches, batteries and walking tests |
| [design/concept.md](design/concept.md) | The v0.2 concept (historical) |
| [design/stairs.md](design/stairs.md) | Stair climbing: status, open issues and options |
| [design/assembly_wiring.md](design/assembly_wiring.md) | Modules, assembly order, power and wiring |
| [design/g1_capability_study.md](design/g1_capability_study.md) | What matching Unitree G1's joints and ranges would take: limits, tiers of change, mass and cost |
| [../sim/isaaclab/README.md](../sim/isaaclab/README.md) | Training the walking and stair policies, and every run's results |
| [../sim/motion/README.md](../sim/motion/README.md) | The motion-tracking pipeline |
| [../bench/README.md](../bench/README.md) | Actuator bench tests (phase 2) |
| [../bom/README.md](../bom/README.md) | Bill of materials and how its prices are sourced |

## Glossary

- **DOF:** degree of freedom, one powered joint axis. Marvin Bot has 27, plus 8 in each hand.
- **QDD (quasi-direct drive):** an actuator with a low gear ratio (about 9:1 here), so it can be back-driven and sense force through its current.
- **RobStride RS00/RS03/RS04/RS06:** the actuator models used, from small (wrist) to large (hips, knees).
- **URDF, MJCF, USD:** robot description formats: URDF is generated here from `description/`; MuJoCo reads URDF or MJCF; Isaac Sim uses USD.
- **Code-CAD:** parts written as Python code (`cad/`) instead of drawn in a CAD program, so they regenerate from the description.
- **SF (safety factor):** yield strength divided by the calculated stress; the strength check targets 2.
- **Rated vs peak torque:** a motor can hold its rated torque continuously; peak torque only briefly. RMS torque over a motion is compared with the rated value for heat.
- **CoM:** centre of mass.
- **IMU:** inertial measurement unit (gyro and accelerometer) in the pelvis.
- **BMS:** battery management system.
- **CAN:** the bus the actuators are commanded over.
- **PPO:** the reinforcement-learning algorithm used to train the walking policies.
- **Policy:** the trained neural network that turns sensor readings into joint targets.
- **Flat v7, rough v9:** the 7th flat-ground and 9th rough-terrain training runs; each resumed the one before after a change.
- **Sim-to-real:** making a policy trained in simulation work on the real robot.
- **Cross-check:** replaying a policy trained in Isaac Lab in a second simulator (MuJoCo) to catch simulator-specific tricks.
- **Motion tracking (BeyondMimic):** training a policy to follow a reference motion, such as a retargeted human walk.
- **Retargeting:** converting a human motion to the robot's body and joints.
- **VLA:** vision-language-action model (GR00T N1.7 here), which turns camera images and an instruction into arm motions.
