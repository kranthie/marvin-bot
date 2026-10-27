# Requirements

"Must" items gate the design; "should" items are targets that may move.

## Size and mass

| ID | Requirement | Target | Level |
|---|---|---|---|
| R1 | Standing height | 1.20–1.30 m | must |
| R2 | Total mass with battery | ≤35 kg target bare; ≤41 kg hard limit for the full robot, hands and body shells included (40 kg bare from 2026-10-26; 41 kg with the shells from 2026-10-27, when the hands came in) | must |
| R3 | Hip axis to sole | ≥0.65 m (needed for 17–18 cm stairs) | must |

## Mobility

| ID | Requirement | Target | Level |
|---|---|---|---|
| R10 | Walk on flat ground, turn in place, stop on command | 0.5 m/s walk, 0.5 rad/s turn | must |
| R11 | Fast walk | 1.0 m/s | should |
| R12 | Climb and descend stairs | 17–18 cm risers, 25 cm treads | must |
| R13 | Rough ground, slopes | ±10° slopes, 3 cm obstacles | should |
| R14 | Push recovery | recovers from a 30 N·s push while standing | should |
| R15 | Fall recovery | gets up unaided from prone and supine | should |

## Manipulation

| ID | Requirement | Target | Level |
|---|---|---|---|
| R20 | Arm payload | 1 kg per hand at full reach, 2 kg close to the body | must |
| R21 | Hands | open 8-DOF four-finger hands (bought kits) on a wrist pitch joint in v1; stronger hands later | must |
| R22 | Task learning | fine-tune a VLA (GR00T N1.7) from teleoperation data | should |

## Power and endurance

| ID | Requirement | Target | Level |
|---|---|---|---|
| R30 | Runtime | ≥1 h mixed walking and standing | must |
| R31 | Battery swap | quick-release pack; onboard computer stays up during a swap | should |
| R32 | Bus voltage margin | pack voltage plus regen spikes stay below the actuators' 60 V limit | must |

## Sensing and compute

| ID | Requirement | Target | Level |
|---|---|---|---|
| R40 | Onboard computer | Jetson AGX Orin 64 GB | must |
| R41 | Depth perception | head-mounted depth camera usable on low-texture floors and stairs | must |
| R42 | IMU | ≥500 Hz gyro and accelerometer at the pelvis | must |
| R43 | Control rates | 50 Hz policy; 500 Hz leg motor commands; ≥250 Hz arm motor commands | must |

## Safety

| ID | Requirement | Target | Level |
|---|---|---|---|
| R50 | Hardware e-stop | wired and wireless; cuts actuator power without rebooting the computer | must |
| R51 | Software safety | command watchdog, joint and torque limits, fall detection with damping mode | must |
| R52 | Test fixtures | gantry and harness for all early walking | must |
| R53 | Fall protection | padding at likely impact points | should |

## Build and openness

| ID | Requirement | Target | Level |
|---|---|---|---|
| R60 | Fabrication | parts fit a desktop CNC mill (about 200 × 150 mm usable) and a desktop FDM printer, or can be ordered from online machining services | must |
| R61 | Parts cost | ≈$10–12k for one robot, excluding the onboard computer | should |
| R62 | Openness | all designs, code and models in this repo under its licences; vendor CAD linked, not copied | must |
| R63 | Serviceability | each actuator replaceable without disassembling more than one limb | should |
