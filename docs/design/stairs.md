# Stair climbing: status and open issues

Requirement R12: climb and descend household stairs with 17–18 cm risers and 25 cm treads. This note collects what the checks and training runs have shown so far, the issues still open, and the plan for closing them. Status as of 2026-10-26, design v0.2 with the second-pass leg CAD and the first-pass pelvis and torso CAD (39.06 kg).

## What the robot can do in principle

`sim/mujoco/check_step_up.py` checks that a slow, static step up is kinematically possible and balanced (three poses: swing over the edge, weight onto the stair, rear foot over the edge; CoM over the supporting foot; pelvis upright). With the current legs:

| Riser | Possible | Stance knee | Stance hip roll |
|---|---|---|---|
| 10–18 cm | yes | 45 N·m: 37% of peak, **112% of continuous rating** | 37 N·m: 31% of peak, 92% of rating |
| 20 cm | no in the static check (since the leg CAD masses); **the trained policy (rough v5) climbs 20 cm dynamically** | | |

How the design got here:

- 35° of ankle dorsiflexion could not climb 17 cm at any leg length; 38° is the minimum, and the design has 40° (decisions, 2026-10-26).
- Leg length (0.62–0.69 m hip to sole) barely matters in this check; the hip height stays 0.65 m.
- The two-motor ankle (two RS03 staggered on the shin's centre line, 60 mm cranks, rods 60 mm behind and ±40 mm beside the ankle axis) gives 32 N·m pitch and 25 N·m roll continuous at the worst pose, against ~22 and ~14 N·m needed (`tools/ankle_linkage.py`).
- The knee range is 0–125°: beyond that the folded calf meets the hip roll motor (`tools/check_packaging.py`).

## Issue 1: knee heat on slow stairs

On a slow step the stance knee holds ~45 N·m against the RS04's 40 N·m continuous rating. Heating goes with torque squared, so ~1.3× the rated heat while it lasts. Three things make the real margin smaller:

- The 40 N·m rating assumes a 345 × 345 mm aluminium heat sink; the thigh plate is much smaller, so the motor's real continuous rating in the leg may be 30–35 N·m.
- Holding a bent knee still gives no cooling from motion.
- Each extra kilogram of robot adds ~1.1 N·m at the knee in this pose.

**Measured on real climbs (rough v5 in MuJoCo, 2026-10-26): the knees average 73–75% of their continuous rating (RMS) on 10–20 cm stairs, hip roll ~60%.** The trained gait loads each knee for only part of each step, so the static 112% is a worst case the policy avoids. What remains is confirming the RS04's real rating in a thigh-sized mount on the bench (if it is 30–35 N·m rather than 40, stair RMS would be ~85–100%).

What it is not: a peak-torque problem (38% of peak), or a walking problem. The flat policy v3 walks at up to 0.8 m/s with its knees at ~60% of rated (RMS) and its hottest joint, ankle pitch, at 73% (`sim/mujoco/check_policy.py`).

The static check is the worst case: slow, pausing on one bent leg. Dynamic climbing loads each knee for only part of each step. The real figure comes from a trained stair policy (issue 2) and the bench test.

How comparable robots avoid it (neither publishes a stair heat analysis):

| | Mass | Knee actuator | Thigh | Approach |
|---|---|---|---|---|
| Asimov 1 | 35.5 kg | 45 N·m limit in its simulation model, knee range 0–86° | ~0.25 m | never bends the knee deeply, so the lever arm stays short |
| Roboto Origin | 33.8 kg | Damiao DM-10010L, 120 / 40 N·m (same class as the RS04) | 0.25 m | ~6 kg lighter than Marvin Bot; trained for dynamic walking and running |
| Marvin Bot | 39.55 kg | Robstride RS04, 120 / 40 N·m | 0.30 m | |

## Issue 2: the stair policy stops at the first step

| Run | Result in MuJoCo |
|---|---|
| rough v2 | walks on flat ground; stops in front of 10 cm stairs |
| rough v3 (clearance reward fixed) | climbs 5 cm stairs up and down; stops in front of 10 and 15 cm |
| rough v4 | collapsed: the progress reward had no upper cap, so lunging forward and falling paid best |
| **rough v5** | **climbs 10, 15, 18 and 20 cm stairs up and down** (five steps each way, 28 cm treads), and still walks, turns and stops on flat ground |

In v3 the terrain curriculum plateaued at level ~1 of 10 with very few falls: waiting at the bottom of a stair was safe, and the sharp tracking reward paid almost nothing for a slow climb, so nothing pulled the robots upward. v5 fixed it with a linear progress reward along the command (capped at the commanded speed), a wider tracking reward on rough ground (std 0.25), stairs from 4 to 20 cm on 55% of the tiles, the heat penalty and 6000 iterations; its curriculum reached level ~6 of 10. **Issue 2 is resolved in simulation.** Remaining ideas, if real stairs need more margin:

- A stairs-only curriculum run, started from the v4 or flat policy.
- A reward for height gained on stair tiles.
- A minimum forward command on stair tiles (no standing commands there).
- A finer height map (0.05 m) near the feet; the current one is 1.4 × 0.8 m at 0.1 m.

## Issue 3: mass

The leg strength pass (`cad/strength.py`: far-side bearings at the knee and hip roll, a channel-flanged hip block, a ribbed foot) put the robot at 39.55 kg; the pelvis and torso CAD brought it to 39.06 kg, the arm and head CAD to 39.20 kg, and the slimmer ring-and-strap plates and staggered-ankle shin to 38.71 kg (39.30 kg with the optional covers). The hard limit was raised to 40 kg, with the 35 kg target unchanged (decisions, 2026-10-26). Mass feeds issue 1 directly. Savings that keep the computer and the battery as they are:

| Option | Saves | Trade-off |
|---|---|---|
| Shoulder pitch and roll RS03 → RS06 | ~1.1 kg | shoulder peak 60 → 36 N·m (Asimov limits these joints to 30 / 25 N·m); less margin for push-ups and getting up |
| Electronics and arms from CAD instead of estimates (pelvis and torso done: −0.5 kg) | ±0.5 kg | unknown until drawn |
| 7075 leg plates, thinned to the same stiffness | ~0.3–0.5 kg | cost, sourcing |
| Carbon-fibre tube cores for thigh and shin | ~0.5 kg | design work, bonded joints |

## Options for the knee, if it is needed

| Option | Effect | Cost |
|---|---|---|
| Better heat path: thigh plate as heat sink, thermal paste, a small fan | +10–20% continuous (bench to confirm) | cheap; no design change |
| Policy that spares the knee: the heat penalty in training (in v4) | lower knee RMS on stairs | free |
| 1.5:1 reduction stage (belt or linkage) between the RS04 and the knee | 60 N·m continuous, 180 N·m peak, ~14 rad/s | ~0.2 kg, some backlash, thigh redesign; a linkage could also move the motor up the thigh |
| A larger actuator (CubeMars AKE90-8, specs unconfirmed; Encos EC-A10020) | maybe 55 N·m continuous or more | second protocol, cost (\$484 to \$1,250+), mass |

RobStride has nothing larger than the RS04, and the Damiao DM-J10010L is the same class.

## Decision for now, and how it gets settled

Keep the current design (RS04 knee, no reduction stage). Settle it from three measurements:

1. **Trained stair policy:** done (rough v5): knees at 73–75% of rated (RMS) on 10–20 cm stairs.
2. **Phase 2 bench test:** an RS04 holding 40–45 N·m, bolted to a plate the size of the thigh plate (`bench/actuator/bench_robstride.py thermal`), to measure its real continuous rating in the leg.
3. **Mass:** torso and arm CAD, replacing estimates.

If the knee's measured RMS on stairs exceeds its measured continuous rating by more than the cooling options cover, add the 1.5:1 knee stage before ordering the leg actuators (phase 3).
