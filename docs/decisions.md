# Decisions

Newest first. Each entry gives the decision, the reason and the alternatives considered. Re-open a decision only with a new reason, and record the change here.

## 2026-10-25: v0.2: shorter legs, RS06 ankles, hip yaw stays RS03, ranges checked

Compared with two open humanoids of the same class (Asimov 1, from its simulation model; Roboto Origin, from its URDF, BOM and Isaac Lab config):

| | Asimov 1 | Roboto Origin | Marvin v0.1 | Marvin v0.2 |
|---|---|---|---|---|
| Height, mass | 1.2 m, 35.5 kg | 1.25 m, 33.8 kg | 1.255 m, 37.5 kg est. | 1.215 m, 36.2 kg est. |
| Thigh / shin / hip to sole | ~0.25 / 0.295 / ~0.58 m | 0.25 / 0.30 / ~0.62 m | 0.32 / 0.30 / 0.69 m | 0.30 / 0.29 / 0.65 m |
| Hip order from pelvis | pitch, roll, yaw | yaw, roll, pitch | pitch, roll, yaw | pitch, roll, yaw |
| Hip yaw actuator | limited to 28 N·m in sim | 120 N·m class | RS03, 60 N·m | RS03, 60 N·m |
| Ankle | two motors, 40 / 17 N·m pitch / roll in sim | two 27 N·m motors on a linkage | two RS03 (60 N·m) | two RS06 (36 N·m) |
| Knee range | 0–86° (sim) | −11° to 143° | 0–140° | 0–140° |
| Ankle pitch / roll range | ±20° / ±6° (sim) | ±34° / ±29° | −45/+35° / ±20° | −40/+35° / ±20° |

- **Legs:** hip axis to sole goes from 0.69 m to 0.65 m (thigh 0.30 m, shin 0.29 m, ankle 0.06 m), and overall height from 1.255 m to 1.215 m. 0.69 m was longer than both references, adding leg inertia and mass; 0.65 m still meets requirement R3 for 17–18 cm stairs. Phase 1 simulation compares 0.62, 0.65 and 0.69 m on stairs before the leg CAD is fixed.
- **Ankles:** two RS06 (36 N·m peak, 11 N·m rated) per ankle instead of two RS03. Roboto Origin walks and runs at 34 kg on two 27 N·m motors per ankle. Saves ~1.1 kg of distal mass. Risk: continuous stance torque is close to the RS06's rating, so phase 1 checks ankle torque and heating, and RS03 remains the fallback (same bus, similar mounting).
- **Hip yaw:** stays RS03 (60 N·m), between Asimov's 28 N·m limit and Roboto Origin's 120 N·m actuator.
- **Hip order:** stays pitch, roll, yaw (as Asimov 1), which suits deep hip flexion for stairs; Roboto Origin's yaw-first order is noted as an alternative.
- **Ranges:** hip pitch extension widened to 40° (Roboto Origin allows 45°); ankle plantarflexion set to 40°, a realistic target for a two-motor linkage. Our knee and ankle ranges are closer to Roboto Origin's than to Asimov's simulation limits, which suits stairs.
- **Bench test:** phase 2 tests an RS04 and an RS06 (the RS06 now covers 8 joints).

## 2026-10-25: mass target kept at 35 kg; estimate is 37.5 kg

_Updated by v0.2 above: the estimate is now 36.2 kg._

The v0.1 estimate (`tools/check_description.py`) is 21.8 kg of motors plus 15.7 kg of everything else. That is over the 35 kg target and under the 38 kg hard limit. Reductions to test in phase 1: RS06 instead of RS03 at the ankles (−1.1 kg), the Orin module on a compact carrier board instead of the developer kit (−1.0 kg), lighter leg cores once real loads are known.

## 2026-10-25: structure: aluminium load path, printed structural shells

Machined 6061 cores carry the load through each leg and the pelvis; 7075 only for highly loaded flanges and ankle cranks. Printed carbon-fibre-nylon shells bolt through to metal inserts and add stiffness, so the robot looks finished without a separate cover. Arms, which carry light loads, use printed structural shells with aluminium rings at the actuators. A second (far-side) bearing supports the hip roll, hip yaw and knee joints; a crossed-roller bearing supports the waist. Parts must fit a desktop CNC mill of about 200 × 150 mm usable area, so thigh and shin cores are split into pinned segments or ordered from an online machining service.
Alternatives: all-machined structure (heavier, more machining time); fully printed structure (load must still go through metal; heat near actuators softens PETG-class plastics).

## 2026-10-25: leg layout: hip pitch, roll, yaw; knee at the joint; two-motor ankle

_Updated by v0.2 above: hip to sole 0.65 m, RS06 ankle motors._

Hip order from the pelvis is pitch, roll, yaw, which allows deep hip flexion for stairs and sitting. The knee motor sits at the knee for v1 (simplest). Each ankle uses two RS03 motors mounted high on the calf, driving pitch and roll through push rods with spherical rod ends, which keeps the shin light. Hip axis to sole is 0.69 m (55% of height), sized for 17–18 cm stairs.
Alternatives: knee motor near the hip with a linkage (lower leg inertia, more parts; revisit if simulation shows a need); serial ankle pitch and roll motors (heavier shin).

## 2026-10-25: arms use RS06 at shoulder yaw and elbow

Robstride's RS06 (36 N·m peak, 0.621 kg) replaces the RS02 (17 N·m). Roboto Origin uses 27 N·m-peak actuators at every arm joint; Asimov 1's simulation model limits shoulder yaw to 20 N·m and the elbow to 12 N·m. The RS02's 17 N·m peak is below the first, and the RS06 leaves margin for push-ups and getting up after a fall. Cost +~¥600 for four, mass +~0.96 kg. Revisit with simulated arm torques in phase 1.

## 2026-10-25: Robstride actuators, RS04 / RS03 / RS06 / RS00

Robstride gives the most torque per dollar of the quasi-direct-drive actuators compared, publishes manuals and specifications in a public repository, and has the best open software support among budget brands (Python, Rust and ROS 2 drivers). RS04 (120 N·m) at hip pitch, hip roll and knee; RS03 (60 N·m) at hip yaw, ankles, waist and shoulder pitch and roll; RS06 at shoulder yaw and elbow; RS00 at the wrists. Second source for the 120 N·m class: Damiao DM-J10010L (also MIT-mode CAN). Ask RobStride for a direct quote: China list prices are roughly half of US reseller prices before shipping and tariffs.
Alternatives: CubeMars (2–3× the price per N·m), Damiao (fewer verified specs), Encos and MyActuator (several times the price), self-built cycloidal actuators (more work, lower torque).

## 2026-10-25: six motor CAN buses plus one battery bus

Robstride uses classic CAN 2.0B with 29-bit identifiers at 1 Mbit/s, and each motor needs a command and a reply frame per cycle. Six motors on one bus at 500 Hz would load it to roughly 80–90%. Each leg gets two buses (hip: 3 motors; shank: knee and two ankle motors) at 500 Hz; each arm gets one bus at 250 Hz, with the waist on the right arm's bus. All stay at or below 45% load. A seventh bus carries battery telemetry. The Orin's two on-board CAN controllers (with 3.3 V transceivers) and isolated USB-CAN adapters (gs_usb, SocketCAN) provide the buses; a microcontroller bridge is the fallback if bench tests show too much jitter.

## 2026-10-25: power: 13S3P quick-release pack, keep-alive supply, 4.1 V charging

A 13S3P pack of 21700 high-discharge cells (~630 Wh, ~3.4 kg) gives about 1.5× the runtime of 13S2P for ~1.1 kg more. It sits on a quick-release cradle with an anti-spark blind-mate connector. A small keep-alive battery feeds only the computer through an ideal-diode OR, so the computer stays up during a swap. A full 13S pack is 54.6 V against the actuators' 60 V limit, so regen spikes are a real risk: charge to 4.1 V per cell (53.3 V) for daily use and fit a clamp at about 56–58 V. DC-DC converters are rated for 75 V input, and the computer's supply is wired upstream of the motor contactor so an e-stop never reboots it. Main path: Class-T fuse, precharge, contactor.
Alternatives: 13S2P (lighter, ~1 h); two packs OR-ed for swapping while walking (v2).

## 2026-10-25: sensing: active-stereo depth camera, low-latency IMU

Head camera: Orbbec Gemini 335L. It computes depth on the camera, leaving the Orin's GPU free; its IR pattern gives depth on plain floors and carpeted stairs; it connects over USB-C; and Isaac Sim 6.2 has a built-in model of it. Driver support on JetPack 7.2 must be confirmed before ordering.
IMU: the locomotion policy needs a low-latency gyro at 500 Hz or more. A low-cost USB AHRS module tops out around 200 Hz, so it is the backup; the primary will be an industrial AHRS or a BMI088 read by a microcontroller, decided after a bench test.
Alternatives: ZED X Mini (needs a capture card and computes depth on the GPU), RealSense D455 (USB stability reports on Jetson).

## 2026-10-25: compute and software stack

Onboard: Jetson AGX Orin 64 GB developer kit (already available), JetPack 7.2, Isaac ROS 4.6 and ROS 2 Jazzy. Isaac ROS 5.0 is skipped for now because of reported regressions on Orin. A real-time control process (CAN I/O, 50 Hz policy, safety) runs outside ROS. Training: Isaac Lab 3.0 with rsl_rl, MuJoCo as a cross-check. Manipulation: GR00T N1.7, served from a desktop GPU first and moved to the Orin with TensorRT later. Whole-body control starts with the RL lower body plus IK arms; a motion-tracking controller (retargeting with GMR, BeyondMimic-style training) comes later. NVIDIA's SONIC checkpoints are for another robot and need far more compute to retrain, so it is deferred.
Alternatives: Jetson Thor T4000 (same module price in 2026; worth considering for a second robot).

## 2026-10-25: vendor CAD is linked, not committed

Robstride's STEP models are public but carry no licence, and several CAD sites forbid redistribution or derived designs. The repo will fetch vendor models into a git-ignored folder with a script and use its own simplified envelopes in committed designs. Fasteners and bearings come from parametric libraries with open licences (bd_warehouse, Apache-2.0).

## 2026-10-25: built in stages, simulation first

See [plan.md](plan.md). Parts are bought per phase; only two actuators are bought before the simulation pass check.

## 2026-10-25: from scratch, with open designs as references

Marvin Bot is a new design. Asimov v1 (Menlo Research) is the main reference for layout and sizing and is used for ideas only, never its CAD. Other references are listed in `ACKNOWLEDGMENTS.md`.
