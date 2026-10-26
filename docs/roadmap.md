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
- [ ] Stairs: rough-terrain policy that climbs 15-18 cm stairs in Isaac Lab and in MuJoCo
- [x] Heat: RMS torque vs continuous rating per joint in the MuJoCo policy check (walking: knees 60-64% of rated); sustained-torque penalty in training
- [ ] Rough-terrain and stair policies; MuJoCo cross-check
- [x] Packaging check with STEP-model actuator envelopes (`tools/check_packaging.py`); thigh layout, knee range and motor positions fixed
- [x] Code-CAD of the legs, first pass (`cad/leg.py`, `cad/build.py`): every part fits the mill or printer (foot base plates cut by a service); masses and inertias in the URDF
- [x] Strength check, hand calcs on the CAD sections (`cad/strength.py`): landing case fails five one-sided parts
- [x] Leg CAD second pass: far-side bearings (knee, hip roll), channel-flanged hip block, ribbed foot; passes the strength check (worst SF 2.4); robot 39.55 kg, limit raised to 40 kg
- [ ] Leg CAD: fasteners, bearing seats, hip yaw far-side support
- [x] Pelvis and torso CAD, first pass (`cad/torso.py`): 0.86 + 1.46 kg (estimates were 1.0 + 1.8); pelvis frame SF 37 in the landing case; robot 39.06 kg
- [ ] CAD for arms and head

## Phase 2 preparation

- [x] Robstride CAN protocol driver from the official manuals, frame encoding unit-tested (`robot/drivers/robstride.py`)
- [x] Bench scripts and procedure: scan, latency, step response, thermal (`bench/`, untested)

## Open items to verify before ordering

1. RobStride direct quote, lead time and current import tariffs; compare with Seeed Studio (list price, ~5% off at 10+ of one model) and confirm Seeed's live prices.
2. Robstride output-bearing type, backlash and moment ratings (decides the far-side bearing design).
3. Robstride bus current and regen behaviour (sizes the fuse, clamp and precharge).
4. Real CAN timing with 3–5 motors per bus on the Orin under JetPack 7.2, and the on-board CAN controllers' pin setup.
5. Depth camera driver support on JetPack 7.2; camera baseline figure (sources disagree).
6. Primary IMU choice (industrial AHRS or a BMI088 on a microcontroller).
7. Cell availability (genuine stock) and the built pack's mass.
8. Desktop-CNC feeds and achievable accuracy in 6061 (coupon cuts).
9. Ankle push-rod loads and rod-end sizing (simulation with gait loads).
10. GR00T N1.7 closed-loop latency on the Orin while the 50 Hz controller runs.
11. Knee and hip-roll heating on slow stairs (knee 115% of rated in the static check at 39.55 kg): see [design/stairs.md](design/stairs.md) for the issue, options and how it gets settled.
