# Decisions

Newest first. Each entry gives the decision, the reason and the alternatives considered. Re-open a decision only with a new reason, and record the change here.

## 2026-10-27: bought four-finger hands on a wrist pitch joint

Two open-source 8-DOF four-finger hand kits (bought from Seeed Studio, SKUs 100062181 and 100063642; eight Feetech SCS0009 servos each at 5 V, 0.40 kg as published, mechanical design CC-BY-4.0) replace the 1-DOF grippers. The gripper's STS3250 becomes a wrist pitch joint (±60°) under the RS00 wrist yaw, so the hand both turns and tilts: the servo hangs from the RS00's output in a printed cradle, its case running up from its shaft, and a printed U on its horn and an idler pin carries a plate for the hand's wrist interface (`cad/arm.py`). The hands are the AmazingHand (Pollen Robotics): its CC-BY-4.0 design lets the real hand meshes ship in `description/third_party/amazing_hand/` with attribution, under their own licence, and they are used for the visuals and the joint sweep at the published mass; the mount plate carries the hand's six M3 holes on a 42 mm circle. Holding the hand level takes about 0.4 N·m (25% of the servo's rated torque), and with 0.5 kg at the fingertips about 1.2 N·m (75% of rated, 24% of stall): enough for pick-and-place, not for holding a load out for minutes. Alternatives considered: an RS00 for pitch (overkill, +0.24 kg at the end of the arm), an STS3215 (2.9 N·m stall, too little margin), and keeping the gripper with compliant fingers (cheaper, but the hands were already bought).

Proportions: with the pitch servo's case above its shaft, the wrist pitch axis sits 0.063 m under the wrist yaw (the gripper's pivot was 0.090), so the forearm is 0.188 m to the wrist axis, a person's at Marvin Bot's height (0.146 of height). The shoulder-to-fingertip length is 0.62 m, about 9% longer than a person's at this height, all of it from the hand, which is about 30% larger than a person's scaled down; nothing smaller is on hand.

Mass: the hands and mounts add 0.73 kg (grippers 0.19 kg out, hands 0.80 kg and mounts 0.12 kg in): 39.86 kg bare, 40.69 kg with the body shells. The hard limit now applies to the robot as it would run, hands and shells included, and is set to 41 kg (`tools/check_description.py`). The hands' servos need a 12 V to 5 V converter at each wrist and a serial bus of their own (SCS protocol; the other servos use STS), one USB adapter per hand. The joint sweep counts the palm with the arm. The walking and stair policies were trained without the hands and are to be retrained; the motion pipeline's arm scale is now the reach to the wrist pitch axis (0.79 of the source's).

## 2026-10-27: no side-to-side waddle; command and sensor delays

Flat v6 and rough v8 tipped their whole body over each stance foot: pelvis and torso roll 20-22° peak to peak (a person walks with about 5-8°), which swung the head 16.5 cm side to side while the pelvis itself moved under 2 cm. The waist only turns, so the torso rolls with the pelvis; the only roll penalty was the general flat-orientation term (0.02 at 8°); the air-time reward (2.0) paid for time on one foot, so the gait had no double support (0-2% of the stride) and slow 0.6-0.75 s swings; and the feet landed 21-24 cm apart. mjlab's Unitree G1 task handles the same problem with an upright term, tight hip and ankle roll while walking and no single-support reward. The walking tasks now penalize pelvis roll (−10 on its squared sine) and a step width more than 2 cm from 16 cm, hold the hip and ankle roll near stance (−0.3, was −0.1 on hip roll and yaw), and pay air time at 0.5. The pelvis-height allowance is 2 cm (was 4). Commands also reach the motors 0-10 ms late and the IMU and joint readings reach the policy 0-20 ms late (random per episode), which Isaac Lab's DC motor model has no option for: `robot_cfg.py`'s `DelayedDCMotor` adds it, and since Newton's own actuator kernels only take a fixed delay on a plain PD motor, the motors run in Isaac Lab (about 2.5× slower per iteration). Flat v7 and rough v9 (resumed from v6 and v8) roll 4-6°, swing the head 3-8 cm, step 14-15 cm wide and pass every check; rough v9 is as reliable on 15-20 cm stairs over five seeds as v8 (2 falls in 15 runs each). `check_policy.py` prints head and pelvis sway, roll and step width. Alternative considered: moving training to mjlab, whose G1 task suggested the fix; not needed, since Isaac Lab 3.0 on Newton already runs the same physics.

## 2026-10-27: motion-tracking pipeline (in progress)

For skills beyond walking, motion tracking (BeyondMimic) scales better than reward design: a new skill is a new clip. `sim/motion/` adds Marvin Bot to NVIDIA's ProtoMotions (generated model, retargeting URDF and robot config) and retargets human motion with its PyRoki retargeter, overriding the link map and body scale. Eight walking, jogging and turning clips from ProtoMotions' samples retarget within every joint limit, with 7-14° of pelvis roll; ProtoMotions' height fix assumes G1's foot frame, so `fix_sole_height.py` puts the soles on the floor. The first tracker (5,800 epochs on Newton) counted all eight clips as tracked, but ProtoMotions' test only checks torso height: it shuffled on the spot, covering 0-32% of the reference's distance with its knees at about half the reference's bend. Two model problems were behind it: the gripper boxes overlap the thighs even with the arms hanging, so the arms held the legs (the arms now collide with the ground only; the retargeted arms still pass through the body and need a collision-aware retarget before hardware), and the scaled human feet were 12-14 cm apart for 11 cm wide feet (knees and feet now move out 1.5 and 3 cm). `check_tracking.py` compares distance, joints and body positions with the reference. A second tracker is training. The command-driven walking and stair policies stay: a tracker needs a reference motion to follow. Motion data is not committed (the samples are BONES-SEED, gated).

## 2026-10-26: straighter stance; level feet in swing

The default stance's knee goes from 25° to 15° (hip pitch 12° to 6°, ankle pitch 13° to 9°, which keeps the foot flat and the CoM 106 mm inside the feet; pelvis 0.636 to 0.645 m), so the robot stands straighter and the pelvis-height term aims higher. The policies also swung their feet toe down by up to 30° (the heel high), because the foot-level term only covered feet in contact; a matching term now penalizes the tilt of feet in the air, and the MuJoCo check prints swing pitch and touchdown pitch. Flat v6 and rough v8 (resumed from v5 and v7) swing their feet at most 11-16° toe down and pass every check. The stair check now measures the climb from the walking height on the approach rather than the standing height, which counted the dip of walking against the climb (it failed rough v8 on 5 cm stairs it crossed). Not done: the rough policy still walks 3.8 cm under its stance height, inside the term's 4 cm allowance; a tighter allowance is the next lever.

## 2026-10-26: posture terms in the walking rewards

The stair policy (rough v6) climbed 15 cm stairs in a deep crouch: it stood with its knees at 95° and its pelvis 19 cm below the stance height, and climbed 8 cm low, which also ran its knees at 73-81% of their rated torque. Nothing in the rewards asked for height, and a crouch is stable. Two terms were added to both walking tasks: a linear penalty on the pelvis sitting more than 4 cm below its stance height above the lower foot (measured from the lower foot so it holds on stairs), and an L1 penalty on the leg joints' distance from the stance pose while commanded to stand. The rough task also doubled the heat penalty and put 14-20 cm stairs on a tenth of the tiles on every curriculum row. Resumed from v6 for 3000 iterations, rough v7 stands in the stance pose, climbs 4 cm taller, and passes 5-20 cm stairs (v6 fell on 18 and 20 cm). The posture check is now printed by `sim/mujoco/check_policy.py`. Alternatives considered: a straighter default stance (fewer degrees at the knee), which changes the standing check and every policy, left as an option; a base-height target from the terrain map, rejected because the map averages the stairs ahead.

## 2026-10-26: electronics bay, battery cradle and limb-root bulkheads

The bought electronics were one estimate box each in the torso and pelvis. They are now laid out at typical outlines for their class (`cad/electrical.py`), placed so nothing overlaps the frame, the motors or each other, and in the order the power flows: the battery slides out of the back of the torso between printed side guides and a latch plate, and its blind-mate connector seats in a socket over the computer, beside its buttons; the bay above the computer holds the contactor and precharge, the torso power board, the e-stop relay, the computer's DC-DC (fed ahead of the contactor) and the three USB-CAN adapters beside the computer's USB ports; the pelvis channel under the waist motor holds the 12 V servo DC-DC and the IMU, and the leg power board sits between the hip roll motors. A 150 W chassis-mount 12 V module does not fit the channel, so the BOM now asks for a compact 10 A brick (60 × 34 × 25 mm at most). Each limb's harness ends at a bulkhead on its parent module (legs under the pelvis board, arms on the shoulder plates) with one power and one signal connector, so a limb unplugs as a unit; the generated harness names the bulkhead and a clip pair (parent link, child link) for every service loop, and its leg chain now follows the roll-first order. The chest shell went to 2 mm and the connector masses moved from the harness allowance to the bulkheads to stay inside the limit: 39.13 kg bare, 39.96 kg with shells. Alternative considered: a single electronics box on the back; rejected because it would sit outside the battery's slide-out path and lengthen every harness.

## 2026-10-26: real actuator outlines and mounts

The actuators were modelled as plain cylinders at their overall size. RobStride's outline drawings (manual §1.1) and STEP models show each is a full-diameter body with the output on its front face and a smaller driver cap (with the cable exit) proud of its rear face: RS04 Ø120 body, Ø94 × 13.2 mm cap; RS03 Ø98 (ears to ~Ø107), ~Ø78 × 12.5 mm cap; RS06 Ø85, Ø70 × 14 mm cap; RS00 a plain Ø57 cylinder. Their stator hole circles are larger than the rings drawn before (RS04 10 × M4 on Ø106, RS03 8 × M4 on Ø98 in the ears, RS06 8 × M3 on Ø82). Each motor's output direction is now in `robot.yaml`, and its stepped outline is used in the URDF visuals, the packaging check and the part sweep. Every mount was reworked to bolt where the motor really bolts (`cad/mounts.py`): a 4 mm ring on the body's rear face with the stator holes and a hole for the cap; where a plate carries a far-side bearing stub over the cap (hip roll, hip pitch, knee, ankles) or must stay solid for strength (shoulder roll), posts join the ring to that plate, which became a light three-spoke spider where it no longer carries bolts; output flanges carry the output hole circle and a 1 mm relief outside the rotating flange. The Feetech servos were modelled with their shaft along the wrong axis; their shaft runs along their 35.8 mm height, so the neck bracket (now a cradle under the pitch servo), the head frame's plates (on the horn and idler sides), the neck sleeve (oval, 84 mm across) and the gripper palm were rebuilt round them. A rest-pose audit of every part against every motor (the sweep only reports overlaps beyond those at rest) found and fixed a shin spacer inside an ankle motor and the shoulder roll bracket clipping the shoulder yaw motor. The computer stands on edge with its fan forward (vents in the chest shell), its I/O ports toward the sides and its buttons up, bolted by its feet to a plate behind it (outline and port positions from NVIDIA's P3730 developer kit model, used locally and not redistributed). Robot 39.12 kg bare, 39.94 kg with shells (now 1.2 mm).

## 2026-10-26: box thigh top; roll bracket bar on top

The thigh hung from the hip pitch motor in a U of two plates (an outer plate on the output, an inner plate on a far-side bearing): weak in twist and visibly a bracket. It is now a closed box round the pitch motor, with front and back walls joining the two plates below the motor and the yaw motor under its floor: much stiffer (worst SF 5.6 with a 6 mm outer wall, was 5.6 with 8 mm and no box) and one solid piece at the top of the thigh. A single-sided thigh on the pitch motor's output alone would look cleaner still, but it puts the whole landing moment on the actuator's output bearing, whose moment rating is still an open item; it stays an option once that is known. The roll brackets' bars now run over the roll motors instead of outboard: the pelvis back plate is full width only below the roll axes, with a narrow spine above them, so the bars clear it when a leg rolls out 45° (checked by the part sweep). The shells over the roll motors are dropped; they are left bare. Robot 38.78 kg bare, 39.74 kg with shells.

## 2026-10-26: roll-first hip

The hip was pitch, then roll, then yaw: the hip roll RS04 sat behind the hip on the leg and swung with every hip flexion, a box hanging off the back of the thigh that also stopped the knee at 125°. It is now roll, then pitch, then yaw, as on several current humanoids: the roll RS04s are fixed in the pelvis behind the hips (a static block, like a pelvis), and each roll bracket (a U round its roll motor, with a far-side bearing behind it and a bar on its outer side) reaches forward to carry the pitch RS04 at the hip joint by its stator face; the thigh hangs from the pitch motor in a U-yoke with a far-side bearing on its inner side. A full yaw-roll-pitch stack with the yaw motor in the pelvis was ruled out: it needs ~60 mm more above the hips, past the 1.30 m height limit or below the 0.65 m leg length the stairs need.

What it changed: 1.4 kg of motor per leg stops swinging with the leg; the knee now reaches 130° (beyond it the upper ankle motor meets the thigh shell); hip extension is -30° (was -40°; beyond it the thigh yoke meets the roll motor; walking uses ~-20°); the pelvis is a back plate carrying both roll motors, a top plate under the waist motor and two gussets, all kept clear of the hip pitch motors, which now swing round the roll axes. With 2.8 kg of roll motors 98 mm behind the hips, the static step-up check lost 18 and 20 cm stairs; moving the torso 25 mm forward over the hips (the waist joint) restores 10-20 cm (knee 108% of rated, hip roll 100% on a slow 18 cm step). A part-to-part sweep of the real CAD through every joint range is now part of `python -m cad.covers --check` (the packaging check sees only motors and boxes); it found the roll bracket's bar swinging into the pelvis (first over the motor, then under it into the thigh; it now sits outboard), the thigh yoke against the pelvis gussets at full flexion (gussets moved inward), and the forearm folding into the upper-arm shell from 90° of elbow (a relief cut in the shell). Arms against the pelvis are now counted as self-collision, like arms against legs. Robot 38.69 kg bare, 39.78 kg with shells.

## 2026-10-26: head profile; hip and upper-thigh shells

The head is now deeper than it is wide (160 x 142 mm), sitting forward of the neck with the chin forward and low and the crown back and high; the visor face leans back 18°, and its underside rises from the chin to where the neck enters. The camera sits behind a flush window (optical-grade, IR-transparent tinted polycarbonate) instead of an open slot, so the visor reads as one smooth face; the window's effect on depth noise is a phase 6 test. Shells now cover the hip roll motor behind each hip (a dark glute, rounded over the yoke plates, above the roll axis where the roll bracket never swings) and the hip yaw motor at the top of each thigh. Shells thinned to 1.4 mm to stay inside the 40 kg limit with them on (39.93 kg).

## 2026-10-26: narrower shoulders

The arms hung ~188 mm off the centre line (a person this height: ~150 mm), set by the shoulder roll RS03 (Ø100 mm) sitting outside the shoulder pitch RS03. Putting the roll motor behind the pitch motor, as at the hip, does not help: it would sweep through the torso as the arm pitches. Instead both shoulder pitch motors sit 30 mm deeper in the torso (centres at ±80 mm, joints at ±110 mm), so the arms hang 158 mm off the centre line and the shoulders are 416 mm across (was 476). The battery (140 mm wide) is wider than the space now left between the shoulder motors, so each side frame is two plates: a lower A-frame outside the battery and an upper plate on the shoulder motor's inner face, above the battery and computer, joined by two spacers clear of the motor; the battery sits 4.5 mm lower, the electronics and keep-alive bays are narrower. With the arms closer, below 0° of shoulder roll the forearm meets the hip motor, so the roll range is now 0-120° (was -5-120°); the default stance uses 8°. Robot 38.80 kg bare, 39.88 kg with shells (0.12 kg under the 40 kg limit).

## 2026-10-26: torso shell follows the frame

The torso shell was the convex hull of everything in the torso, so the shoulder motors (±138 mm, at the top) held its sides out all the way down: up to 44 mm of air beside a frame ±88 mm wide. It now follows the frame slice by slice, ~13 mm outside it everywhere (4 mm gap, the corner rounding and the skin), blending between slices at most 0.7 mm across per mm up: broad at the shoulders, slimmer at the chest and waist.

## 2026-10-26: more human proportions

Compared with human proportions at this height the robot's heights were already right (shoulder axis at 0.79 of standing height, chin at ~0.85), but it was too deep in the chest, too wide in the head and wide at the shoulders: parts that do not shrink with the robot (the computer developer kit, the battery, Ø120 mm knee motors, the 124 mm-wide depth camera) sit in a body three quarters the size of current commercial humanoids (~1.7 m). Changes that need no new parts: the computer stands on edge (71 mm deep instead of 110), so the torso frame is 155 mm deep instead of 190 and the shell no longer flares at the chest; the head is a narrower, taller helmet (142 mm wide, 170 mm tall) with the camera at eye height, half way up the face; the neck is 20 mm longer, and the torso shell's shoulders slope up into a collar round a flared neck sleeve. Standing height 1.29 m (requirement 1.20-1.30 m). Not changed, and open: narrower shoulders (the shoulder roll motor sits outboard of the shoulder pitch motor, so the arms hang ~190 mm off the centre line against ~150 mm for a person of this height) would need a new shoulder layout; a smaller head would need a narrower depth camera. The actuator envelopes were re-measured with a STEP reader and match the description (the RS03 and RS06 have a lug that reaches Ø107 and Ø90 mm at one point).

## 2026-10-26: helmet head and full body shells

The head is now a helmet (taller than it is wide, narrowing to the chin) with a full-face glossy visor, a slot in front of the depth camera's optics, and its bottom open over a dark neck sleeve. The add-on covers became a full body-shell set in the style of current commercial humanoids (a light torso widening to the shoulders, a dark hip girdle, long leg shells, arm sleeves, toe caps), with our own shapes. Because full shells sit close to moving parts, they are now checked automatically: `python -m cad.covers --check` sweeps every joint through its range plus combined poses and fails on any shell contact (in CI). The torso shell is in eight pieces so each fits a 250 mm printer. Shells are 1.6 mm (1.10 kg); the robot is 38.70 kg bare and 39.80 kg with shells, inside the 40 kg limit. The policies still train on the bare robot.

## 2026-10-26: staggered ankle motors; ring-and-strap plates

The two ankle RS03 sat side by side, making the shin ~132 mm wide. They are now staggered on the shin's centre line between two side plates: one high and forward just under the knee (like a kneecap), one low just behind the ankle, each bolted to one plate with its crank outside the other; the plates' lower ends are the ankle fork cheeks. The linkage search (`tools/ankle_linkage.py --search`, now with a separate position per motor) found 60 mm cranks and rods 60 mm behind and ±40 mm beside the ankle axis: 32 N·m pitch and 25 N·m roll continuous at the worst pose (side by side: 31 and 26), ratios 0.99 / 0.67 at neutral, transmission ≥ 0.53, rod force ≤ 1.9 kN. A layout with the high motor behind (a calf) was also searched: it only reached 23 / 20 N·m, because the low motor's rod becomes very short. The shin is ~95 mm wide and its lower front stays within 40 mm of the ankle axis, which matters on stair nosings. The aluminium plates elsewhere were reduced to rings on the motors' mounting-hole circles (measured from the STEP models: RS04 stator ~86 mm, RS03 stator ~72 mm) joined by straps sized by `cad/strength.py` (worst SF 2.2), instead of hulls as wide as the motors; the torso uprights became A-frames. Robot 38.71 kg (was 39.20 kg). The policies must be retrained for the new ankle ratios.

## 2026-10-26: arms, head, covers and harness, first pass

Arms: two-plate shoulder brackets (an L in one piece is too tall for 60 mm stock), aluminium rings on every motor face, printed upper-arm and forearm shells that carry the harness inside; 7 mm shoulder plates pass a new hand-load case (0.5 × body weight at the gripper: catching a fall or pushing up) at SF 2.4. Head: an egg-shaped printed shell with a flat-cut front and a dark visor, with a slot in front of the depth camera's optics rather than material over them. Covers are an add-on, not structure: the robot must look finished without them, so they are generated as offset skins over the parts, kept out of the default URDF (the policies train on the bare robot) and written to a second URDF with their 0.56 kg. The harness is generated from the description (one harness per limb, one power and one signal connector at each limb root, so a limb comes off as a unit) instead of being drawn by hand, so it follows layout changes. Robot 39.20 kg bare, 39.76 kg with covers. Details in [design/assembly_wiring.md](design/assembly_wiring.md).

## 2026-10-26: stairs work in simulation; knee stays an RS04 pending the bench test

The stairs policy rough v5 climbs 10, 15, 18 and 20 cm stairs up and down in the MuJoCo cross-check (five steps each way, 28 cm treads) and still walks, turns and stops on flat ground. On stairs the knees average 73–75% of their continuous rating (RMS) and hip roll ~60%, against 112% in the static worst-case check: the trained gait loads each knee only part of the time. The 1.5:1 knee stage is not needed on this evidence; the remaining check is the RS04's real continuous rating in a thigh-sized mount (phase 2 bench test). Details in [design/stairs.md](design/stairs.md).

## 2026-10-26: pelvis and torso structure, first pass

`cad/torso.py`: the pelvis is a box frame of pocketed plates (the hip pitch RS04s on the side plates' inner faces, the waist RS03 on the top plate, the pelvis electronics inside); the torso has a waist flange and base plate, two uprights carrying the shoulder pitch RS03s (flat profiles of 265 mm, cut by an online service), a bridge over the neck yaw servo, and a printed chest shell open at the back for the battery. The computer, battery and electronics stay as estimate boxes, moved so they sit inside the structure without overlaps. Mass: pelvis 0.86 kg and torso 1.46 kg against estimates of 1.0 and 1.8 kg; the robot is 39.06 kg, and the stance knee on a slow 18 cm step is at 112% of rated. The pelvis frame carries a one-leg landing at SF 37 as a box section, so it can be lightened later.

## 2026-10-26: keep the second-pass leg; mass limit 40 kg; knee decided after measurement

The second-pass leg passes `cad/strength.py` (worst safety factor 2.4) and puts the robot at 39.55 kg. The computer (Jetson AGX Orin developer kit) and the battery (13S3P) stay as they are, so the hard mass limit goes from 38 to 40 kg; the 35 kg target stays. Effect on the actuators: peak torques stay at or below 38% of peak in the static stair check and below 76% while walking; the continuous rating binds at the knee on slow stairs (46 N·m, 115% of the RS04's rating) and is close at hip roll (92%).

The knee stays an RS04 for now. Whether it needs a 1.5:1 reduction stage, better cooling or nothing is decided from the stair policy's measured knee RMS, the phase 2 bench thermal test and the torso and arm CAD masses, before the leg actuators are ordered. The issues, options and comparison with Asimov 1 and Roboto Origin are in [design/stairs.md](design/stairs.md).

## 2026-10-26: strength check finds one-sided joints too weak; second pass adds far-side supports

`cad/strength.py` cuts each load-carrying leg part at stations along its load path, takes the real cross section of the CAD solid (holes included), and checks bending-plus-axial stress against yield (target safety factor 2, torsion and stress concentrations excluded) for two cases: a landing load at the ankle (3 × body weight up, 0.5 × fore-aft, 0.3 × sideways, straight leg) and each joint's peak torque. Peak torques pass everywhere (SF 4–9). The landing case fails five first-pass parts: foot base plate (SF 0.3), hip yoke back plate (0.3), hip roll front plate (0.6), shin knee flange (0.8), hip yoke side plate (1.4). Shin web and thigh plate pass (2.7–2.8).

Cause: every joint in the first pass hangs on the motor's output on one side, so landing loads bend thin plates out of their plane. The concept already called for far-side bearings at hip roll, hip yaw and knee; the second pass adds them (the shin as a yoke around the knee, a stiffer machined hip block around the roll motor) and a stiffening rib under the foot base, then re-runs this check and the mass budget.

## 2026-10-26: buy actuators from Seeed Studio; bulk order after the bench test

Seeed Studio stocks every Robstride model in the BOM at RobStride's list prices, with about 5% off at 10+ of one model (October 2026, from Seeed's product listings; Seeed's pages could not be opened directly to confirm): RS04 $255 ($242 at 10+), RS03 $225 ($214), RS06 $210 ($200), RS00 $138.99 ($131.99). RobStride's own site links to Seeed and Amazon for ordering. The BOM's 26 actuators come to ~$5,800 at single prices, against ~$6,250 at the US resellers used before.

Only the RS03 (12 with a spare) reaches the 10+ tier on its own. Ordering it all at once would save ~$130 but commit ~$2,600 before the phase 2 bench test. So the bench units (one RS04, one RS03) are bought singly, and the remaining 11 RS03 are ordered together after the bench test passes, which still qualifies for the 10+ price. RS04 (7) and RS06 (5) stay below 10 unless a second robot or extra spares are planned. A direct quote from RobStride for the full set is still worth asking for. Seeed also sells RobStride's $15 USB-to-CAN debug adapter, worth having for RobStride's configuration tool; the robot's buses stay on isolated SocketCAN adapters until the debug adapter is shown to work as one.

## 2026-10-26: leg CAD first pass; mass at the 38 kg limit

`cad/leg.py` draws the legs in code (manifold3d), around the motors at their STEP-model sizes: a flange, side plate and back plate at the hip; a two-plate bracket for hip roll; a flange, outer side plate and printed shell on the thigh; on the shin a knee flange, a central web that both ankle RS03 bolt to, and a fork for the ankle pitch bearings; a hollow ankle cross, 7075 cranks, rods with rod ends; and a foot of a pocketed base plate, heel posts, roll-bearing lugs and a TPU sole. Plates are 6 mm and pocketed with lightening holes; flanges 6 mm. `cad/build.py` computes each part's mass and inertia (into the URDF), and checks sizes: every machined part fits the desktop mill (200 × 150 mm, 60 mm stock), except the 230 mm foot base plates, which are flat and go to an online laser or water-jet service.

Result: 1.72 kg of structure per leg plus a 0.46 kg foot (budget was 1.5 + 0.35), so the robot estimate is 37.98 kg, at the 38 kg hard limit with no margin. No fasteners, bearings or strength check yet; the second pass adds them and then lightens where the stresses allow. Further options if needed: the Orin module on a carrier board (−1.0 kg), a 13S2P pack (−1.1 kg, ~1 h instead of ~1.5 h). With the CAD masses, 20 cm stairs no longer pass the static check (17–18 cm, the requirement, still do) and the stance knee on a slow step is at 108% of rated.

## 2026-10-26: packaging fixes: thigh layout, knee 125°, motors moved along their axes

`tools/check_packaging.py` places every motor (STEP-model envelopes) and collision box on its link and checks for overlaps over each joint's range and in combined poses (deep squat, stair step, sitting). Within a limb, or between a limb and the body it is mounted on, an overlap is a packaging error; between different limbs it is ordinary self-collision, which the simulators model. Fixes:

- **Hip roll RS04** moved 90 mm back along its own axis, behind the hip pitch RS04 (they overlapped at the joint origin).
- **Thigh layout:** hip roll to hip yaw 60 mm (was 100), hip yaw to knee 155 mm (was 115); thigh length unchanged at 300 mm. With the yaw motor low on the thigh, the calf's ankle motors hit it from 125° of knee flexion; the roll motor no longer sits above the yaw motor, so the yaw joint can move up.
- **Knee range 0–125°** (was 140°): beyond 125° the folded calf reaches the hip roll motor behind the hip. Stairs need far less; Roboto Origin allows 143°, Asimov's simulation model 86°.
- **Shoulder pitch RS03** pulled 30 mm into the torso along its axis, clear of the shoulder roll RS03 and the head.
- **Shoulder roll range −5° to 120°** (was −10°): at −10° the upper arm entered the torso.
- **Neck yaw servo** lowered 25 mm into the torso top, clear of the head.

All stairs to 20 cm still pass; the stance knee on a slow 18 cm step is now at 42 N·m, 106% of the RS04's rated torque (bench item: knee heat path).

## 2026-10-26: ankle motors side by side, wider push rods; real actuator envelopes

Packaging the leg against the actuators' STEP models (fetched locally by `tools/fetch_vendor_cad.py`; RS04 Ø120 × 55.7 mm, RS03 Ø100 × 56.6 mm, RS06 Ø85 × 50.5 mm, RS00 Ø57 × 51.4 mm) showed the previous ankle layout could not be built: two RS03 50 mm apart overlap (each is 56.6 mm thick), and 105 mm below the knee they hit the knee motor. They now sit side by side 64 mm apart, 125 mm below the knee, with the cranks on their outer faces. The search (`tools/ankle_linkage.py --search`) then finds 65 mm cranks and rods 55 mm behind and ±55 mm beside the ankle axis (foot widened to 110 mm): ratios 0.86 pitch / 0.83 roll at neutral, 31 N·m pitch and 26 N·m roll continuous at the worst pose (needs ~22 and ~14), transmission ≥ 0.58, rod force ≤ 1.3 kN. The wider rods trade some pitch for much more roll. All stairs from 10 to 20 cm still pass; the stance knee is at 99% of rated on a slow step (bench item).

The same check found that the hip roll RS04 at its joint origin would overlap the hip pitch RS04; it will sit behind it along its own axis, which leaves the kinematics unchanged. The leg CAD places every motor along its axis this way.

## 2026-10-26: ankle linkage geometry; ankles back to RS03

`tools/ankle_linkage.py` solves the two-crank, two-rod ankle exactly over the full range (pitch ±40°, roll ±20°) and searches crank radius, rod placement and crank tilt for the best worst-pose torque with a transmission (sine of the crank–rod angle) of at least 0.5. Best geometry: 45 mm cranks tilted 10°, rods ~188 mm long attached 45 mm behind, ±25 mm beside and 10 mm below the ankle axis; cranks swing −53° to +54°. Joint torque per motor torque is 1.02 (pitch) and 0.56 (roll) at neutral, and at the worst pose the pair gives about 2.0× (pitch) and 1.0× (roll) one motor's torque. Narrowing roll to ±15° hardly changes this.

With RS06 that is 22 N·m pitch and 11 N·m roll continuous, against ~22 N·m pitch and ~14 N·m roll that slow stair climbing and standing on one foot need (`check_step_up.py`). With RS03 it is 39 N·m and 20 N·m. The ankles therefore go back to RS03, the fallback named in v0.2; mass estimate rises by 1.1 kg to ~37.3 kg (limit 38 kg). The largest rod force for a 60 N·m pitch / 30 N·m roll demand is ~2 kN, which sizes the rod ends (M6 steel-ball rod ends or larger) and push rods.
Alternatives: RS06 ankles relying on walking duty cycle to stay within rating (rejected: slow stairs and one-foot standing are sustained); a larger crank-to-rod ratio (limited by the ±40° pitch range and transmission).

## 2026-10-26: ankle dorsiflexion 40°; hip height stays 0.65 m

`sim/mujoco/check_step_up.py` (static: three poses of a slow step, pelvis upright, CoM over the supporting foot, stance torques) shows:

- With 35° of ankle dorsiflexion no leg length could climb 17 cm: in the "weight on the stair" pose the CoM cannot get over the front foot. Widening hip extension, knee flexion or plantarflexion did not help; dorsiflexion did.
- The smallest dorsiflexion that climbs 17–20 cm is 38° at every leg length tried (0.62, 0.65, 0.69 m hip to sole). Leg length barely matters in this test, so the hip height stays 0.65 m.
- The ankle pitch range becomes −40°/+40°; the linkage must reach it. All stairs from 10 to 20 cm are then possible; the stance leg's worst static torque is the knee at 37 N·m (31% of peak, 94% of rated).
- Single-support ankle roll needs ~14 N·m, about the RS06 pair's rated roll torque with the placeholder 0.6 roll ratio. The linkage design must give a higher roll ratio, or the ankles go back to RS03.

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

Marvin Bot is a new design. Asimov v1 is the main reference for layout and sizing and is used for ideas only, never its CAD. Other references are listed in `ACKNOWLEDGMENTS.md`.
