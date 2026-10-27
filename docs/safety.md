# Safety

Marvin Bot weighs about 40 kg, runs on a 48 V lithium-ion battery that can deliver well over 100 A, and has joints that produce up to 120 N·m. It can break fingers, cause serious injury when it falls, and start a battery fire. These rules apply from the first actuator bench test onward. They are a first version and will grow with each phase.

## Before any actuator is powered

- Clamp the actuator or limb to the bench. Keep hands out of the range of motion whenever the driver is enabled.
- Start with low torque limits and low gains; raise them only after the motion has been checked at low limits.
- Know where the e-stop is and test it before each session.

## E-stop and power

- The hardware e-stop opens a contactor that removes power from all actuators. It does not cut the computer's supply, so logs and state survive a stop.
- There are two stops in series: a latching wired mushroom button and a wireless stop. Either one cuts actuator power.
- A watchdog relay also opens the contactor if the control loop stops sending heartbeats.
- The main battery path has a Class-T fuse, a precharge circuit and the contactor. Each limb's branch has its own fuse.
- When running from a bench supply, a regen clamp (brake resistor or TVS) must be fitted. Braking joints push energy back into the supply, and many supplies are damaged by it.

## Battery

- Charge the pack only on a non-flammable surface, attended, with a charger set to 4.1 V per cell for everyday use (4.2 V only when maximum range is needed).
- Store at about 50% charge, in a fire-resistant container, away from exits.
- Never use a pack that is swollen, damaged or has been in a hard fall until it has been inspected.
- The BMS must report to the computer; the robot sits down and powers off its actuators at about 15% charge.

## Walking tests

- All early standing and walking happens on a gantry with a harness that catches the robot before its knees touch the floor.
- Free walking starts only after the gantry pass check in [plan.md](plan.md), in a clear area with padding and nobody within 2 m.
- One person operates; one person holds the e-stop and does nothing else.
- Policies run first in simulation (Isaac Lab and the MuJoCo cross-check) with the same limits, rates and latency as the robot.

## Software safety layers

- Commands from any high-level model (VLA, teleoperation) are clamped to joint, velocity and torque limits.
- If no new command arrives within a timeout, the robot holds its pose; if the controller itself stops, the watchdog cuts actuator power.
- Fall detection switches all joints to a damping mode.
- Motor faults (over-temperature, over-current, encoder errors) are logged and stop the robot.
