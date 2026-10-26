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

## Open items to verify before ordering

1. RobStride direct quote, lead time and current import tariffs; compare with US resellers.
2. Robstride output-bearing type, backlash and moment ratings (decides the far-side bearing design).
3. Robstride bus current and regen behaviour (sizes the fuse, clamp and precharge).
4. Real CAN timing with 3–5 motors per bus on the Orin under JetPack 7.2, and the on-board CAN controllers' pin setup.
5. Depth camera driver support on JetPack 7.2; camera baseline figure (sources disagree).
6. Primary IMU choice (industrial AHRS or a BMI088 on a microcontroller).
7. Cell availability (genuine stock) and the built pack's mass.
8. Desktop-CNC feeds and achievable accuracy in 6061 (coupon cuts).
9. Ankle push-rod loads and rod-end sizing (simulation with gait loads).
10. GR00T N1.7 closed-loop latency on the Orin while the 50 Hz controller runs.
