# Roadmap

Current phase: **1, robot in simulation** (started 2026-10-25). Phases and pass checks are defined in [plan.md](plan.md).

| Phase | Status |
|---|---|
| 0 Foundations | done |
| 1 Robot in simulation | in progress |
| 2 Actuator bench | not started |
| 3 One leg | not started |
| 4 Lower body | not started |
| 5 Torso, arms, head | not started |
| 6 Perception and intelligence | not started |

## Phase 0

- [x] Repository, licences, acknowledgements
- [x] Plan, requirements, safety rules
- [x] v0.1 joint layout, actuator assignment and CAN map (`description/robot.yaml`)
- [x] Mass budget and preliminary BOM
- [x] v0.2: compared with Asimov 1 and Roboto Origin; shorter legs, RS06 ankles
- [x] v0.2 layout locked for phase 1

## Phase 1

- [x] Skeleton (`description/skeleton.yaml`) and URDF generated from the description files (`tools/build_urdf.py`)
- [x] MuJoCo standing check: crouch and straight poses hold, worst holding torque 40% of rated (`sim/mujoco/check_standing.py`)
- [x] Static step-up check: 10-20 cm stairs possible with 40° ankle dorsiflexion; leg length (0.62-0.69 m) barely matters (`sim/mujoco/check_step_up.py`)
- [x] Ankle linkage geometry and push-rod loads (`tools/ankle_linkage.py`); ankles back to RS03
- [x] Isaac Lab package: robot config, flat and rough walking tasks, robot check on Newton and PhysX (`sim/isaaclab/`), mirror rule checked (`sim/mujoco/check_symmetry.py`)
- [x] First walking policy passes the MuJoCo flat-ground cross-check (rough v2: walk, turn, stop)
- [x] Flat policy v3 passes the MuJoCo cross-check up to 0.8 m/s, joints within their continuous ratings
- [x] Stairs: rough v5 climbs 10-20 cm stairs up and down in the MuJoCo cross-check, knees at 73-75% of rated (RMS)
- [x] Heat: RMS torque vs continuous rating per joint in the MuJoCo policy check (walking: knees 60-64% of rated); sustained-torque penalty in training
- [x] Rough-terrain and stair policies; MuJoCo cross-check (rough v7-v9: 5-20 cm stairs up and down, upright posture)
- [x] Packaging check with STEP-model actuator envelopes (`tools/check_packaging.py`); thigh layout, knee range and motor positions fixed
- [x] Code-CAD of the legs, first pass (`cad/leg.py`, `cad/build.py`): every part fits the mill or printer (foot base plates cut by a service); masses and inertias in the URDF
- [x] Strength check, hand calcs on the CAD sections (`cad/strength.py`): landing case fails five one-sided parts
- [x] Leg CAD second pass: far-side bearings (knee, hip roll), channel-flanged hip block, ribbed foot; passes the strength check (worst SF 2.4); robot 39.55 kg, limit raised to 40 kg
- [x] Finite-element check of every load-carrying part (`cad/fea.py`; decisions 2026-10-30): most of the leg, pelvis and shoulder structure is below SF 2 in torsion or through the motor rings' posts
- [ ] Strengthen the failing parts (closed sections or ribs against twist; rings bolted to plates, not on posts) and fix the CAD faults the check found (shin spacers, upper arm shell, computer plate)
- [ ] Leg CAD: fasteners, bearing seats, hip yaw far-side support
- [x] Pelvis and torso CAD, first pass (`cad/torso.py`): 0.86 + 1.46 kg (estimates were 1.0 + 1.8); pelvis frame SF 37 in the landing case; robot 39.06 kg
- [x] Arm and head CAD, first pass (`cad/arm.py`, `cad/head.py`): arms 1.47 kg and grippers 0.18 kg for both (estimates 1.0 and 0.4), head 0.40 kg with the camera; shoulder plates SF 2.4 in a hand-load case; robot 39.20 kg
- [x] Clip-on covers as an optional add-on (`cad/covers.py`, 0.59 kg, separate URDF) and a generated wiring harness (`tools/build_harness.py`); see [design/assembly_wiring.md](design/assembly_wiring.md)
- [x] Slim shin with the ankle motors staggered on its centre line (~95 mm wide, was ~132), same ankle torque; plates reduced to rings on the motors' bolt circles and straps; robot 38.71 kg
- [x] Styled body: helmet head with a full-face visor, full body-shell set (torso, neck, hip girdle, legs, arms, toes) as an add-on, 1.10 kg; shell clearance check through every joint range in CI (`python -m cad.covers --check`); 39.80 kg with shells
- [x] More human proportions: computer stood on edge (torso frame 155 mm deep, was 190), helmet head 142 mm wide and 170 mm tall with the camera at eye height, neck 20 mm longer, torso shell sloping into the neck; 1.29 m, 38.66 kg (39.77 kg with shells)
- [x] Narrower shoulders: shoulder pitch motors 30 mm deeper in the torso, arms 158 mm off the centre line (was 188), two-plate side frames; shoulder roll range 0-120°; 38.80 kg (39.88 kg with shells)
- [x] Roll-first hip: hip roll RS04s fixed in the pelvis behind the hips, pitch motors at the hip joints on the roll brackets; knee 0-130° (was 125°), hip pitch -30-120° (was -40°); torso 25 mm ahead of the hips; part-to-part sweep added to `python -m cad.covers --check`; 38.69 kg (39.78 kg with shells)
- [x] Thigh top a closed box round the hip pitch motor; roll bracket bars over the roll motors; no shells over the roll motors; 38.78 kg (39.74 kg with shells)
- [x] Retrain the flat and stair policies on the current model (39.13 kg) and rerun the MuJoCo cross-checks (flat v7, rough v9)
- [x] No side-to-side waddle (pelvis roll 20° to 6° peak to peak, head sway 16.5 to 7-8 cm); 0-20 ms sensor and 0-10 ms motor delays in training
- [ ] Stair robustness: rough v9 falls in 2 of 15 runs on 15-20 cm stairs over five noise seeds
- [ ] Motion tracking (`sim/motion/`): one tracker follows all 25 Kimodo clips on the 1.44 m model (locomotion, sidesteps, jog, march, kick, squat, pick-up, gestures), every clip upright, the head steadier and the arms retargeted collision-free ([skills page](skills.md)). Next: a jog that leaves the ground, closer two-handed clips (clap, two-hand wave, arms raise) and squat depth, and the torso position it reads, which the robot will have only from odometry. Replayed in MuJoCo (sensor noise, torque-speed limits, joint friction): every clip upright, hands 6.7 cm against 6.8 in Newton
- [ ] Fasteners, clips, harness clip points and connector footprints in the CAD; covers and harness loops in the packaging check
- [x] Battery cradle, blind-mate connector and electronics bay layout (`cad/electrical.py`)

## Phase 2 preparation

- [x] Robstride CAN protocol driver from the official manuals, frame encoding unit-tested (`robot/drivers/robstride.py`)
- [x] Bench scripts and procedure: scan, latency, step response, thermal (`bench/`, untested)

## Skills beyond walking (simulation, before and during the build)

Each can be trained in simulation now and retrained once the bench (phase 2) has fitted the actuator models. Published sim-to-real results for comparable humanoids are cited for each.

- [ ] Get up from a fall, face up and face down (HoST, arXiv 2502.08378; arXiv 2502.12152). Phase 5's pass check; also a shoulder and hip torque check at Marvin Bot's mass
- [x] Expressive clips through `sim/motion/`: gestures (waving, pointing, clapping, bowing), squats, a kick and a dance are generated with Kimodo, retargeted with arm self-collision and tracked in simulation ([skills page](skills.md))
- [ ] A general motion tracker for whole-body teleoperation (SONIC, GR00T-WholeBodyControl; TWIST, HOMIE): one policy that follows any human pose from VR or motion capture, which is how GR00T demonstrations get recorded. Needs far more clips than the 61 samples (the full BONES-SEED set, gated) or SONIC adapted to Marvin Bot
- [ ] Walk, squat and reach with any arm pose (HOMIE): the waist only turns, so reaching low means squatting
- [ ] Stairs from depth images: a student policy on simulated depth with camera noise and holes, distilled from rough v9 (Hiking in the Wild, arXiv 2601.07718; PIM, arXiv 2411.14386)
- [ ] Carry a box between both arms, sit on and stand up from a chair (PhysHSI, arXiv 2510.11072)
- [ ] Walk to and kick a ball, with the camera's detection noise and latency modelled (RoboCup 2025 soccer work; PAiD, arXiv 2602.05310)
- [ ] Manipulation rehearsal in Isaac Sim: teleoperated demonstrations, GR00T-Mimic to multiply them, Cosmos Transfer for photoreal variety, GR00T N1.7 fine-tuning. Cosmos (NVIDIA's world models and open datasets) belongs here, as a source of synthetic training video; the walking and tracking policies do not use camera images
- Out of scope for this robot: parkour and tall jumps (past the 70%-of-peak torque rule), floor work such as crawling until the shells are in the collision model

## Hands

The 1-DOF grippers are replaced by two bought 8-servo four-finger hand kits (owned; SCS0009 servos at 5 V, 0.40 kg each as published) on a wrist pitch joint driven by the former gripper STS3250 (see decisions.md, 2026-10-27).

- [x] Wrist pitch cradle, hand mount and hand envelope in the CAD; masses, URDF, harness and checks updated; forearm 0.188 m to the wrist pitch axis
- [x] Real hand meshes (AmazingHand, CC-BY-4.0, `description/third_party/`) in the CAD, URDF and renders; mount plate drilled to its wrist interface (6 × M3 on Ø42)
- [ ] Weigh the hands; fit a 12 V to 5 V converter at each wrist
- [ ] A hand model for manipulation in simulation (the kit's MJCF has placeholder masses and closed linkages; reduce each finger to its two servo joints)
- [ ] Retrain the walking and stair policies on the current model (the hands and the 1.44 m body): flat v8 done and passing; rough v10 climbed worse than v9 (6 of 15 stair runs against 14 of 15 for v9 on the new body), so v9 stays until a retrain beats it
- Later upgrades, in order: compliant fingertips or pads; the kit's stronger-servo variant (STS3032 at 6 V, same protocol as the STS3250s); a second wrist servo for side-to-side tilt; tactile fingertips

## Open items to verify before ordering

1. RobStride direct quote, lead time and current import tariffs; compare with Seeed Studio (Seeed's live prices confirmed 2026-10-29: list price, ~5% off at 10+ of one model). Price the unquoted lines in `bom/bom.xlsx` (online machining, IMU, BMS, charger, power parts) before each phase's order.
2. Robstride output-bearing type, backlash and moment ratings (decides the far-side bearing design).
3. Robstride bus current and regen behaviour (sizes the fuse, clamp and precharge).
4. Real CAN timing with 3–5 motors per bus on the Orin under JetPack 7.2, and the on-board CAN controllers' pin setup.
5. Depth camera driver support on JetPack 7.2; camera baseline figure (sources disagree).
6. Primary IMU choice (industrial AHRS or a BMI088 on a microcontroller).
7. Cell availability (genuine stock) and the built pack's mass.
8. Desktop-CNC feeds and achievable accuracy in 6061 (coupon cuts).
9. Ankle push-rod loads and rod-end sizing (simulation with gait loads).
10. GR00T N1.7 closed-loop latency on the Orin while the 50 Hz controller runs.
11. Knee and hip-roll heating on slow stairs (knee 47 N·m, 118% of rated, and hip roll 105% in the static check on the 1.44 m model, decisions 2026-10-29; the parked 1.5:1 knee stage would also cover longer legs): see [design/stairs.md](design/stairs.md) for the issue, options and how it gets settled.
12. Hand: weigh the kits, confirm the 6 × M3 on Ø42 wrist pattern on the real part, and measure serial bus timing with 8 servos per hand.
13. CAN interfaces: robot.yaml has 7 CAN buses (4 leg, 2 arm, battery). The BOM now covers them with the Orin's 2 on-board ports (arms), one dual-channel isolated USB-CAN adapter per leg and a single-channel one for the battery, which fit the electronics bay's 3 adapter outlines. Confirm with the CAN timing test (item 4) that a dual-channel adapter carries two 500 Hz buses on one USB link, and find a live US price for it (none was found in October 2026); otherwise use 5 single-channel adapters and make room for 2 more in the bay.
