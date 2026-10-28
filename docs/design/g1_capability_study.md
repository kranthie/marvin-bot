# Matching Unitree G1's capabilities: what a redesign would take

A study, not a redesign: nothing in `description/`, `cad/` or the training code changes here. Status as of 2026-10-28, against the current CAD (39.86 kg bare, 40.69 kg with the body shells; limit 41 kg with the shells).

Why: the motion trackers (`sim/motion/`) showed that the joint ranges limit the skills. The retargeted Kimodo references pin shoulder roll, shoulder yaw and hip roll at their limits in the clap, the two-hand wave, the reach and the sidesteps, and the bow, squat and pick-up have no waist pitch to bend with. Unitree's G1 runs the same kind of tracking with the same kinds of skills, so it is the natural yardstick.

**Short answer.** Most of what the skills need is free. The CAD clears G1-like shoulder yaw (±150°), hip yaw (±158°), waist yaw (±150°), wrist twist (±113°) and hip roll adduction (−30°). Shoulder roll clears −30° with the arm forward, where the clap needs it. These are range changes plus a pose-dependent limit, with no parts, mass or cost added. The real cost is G1's two extra waist joints. A two-RS03 waist adds about 2.5 kg and about $630, against 0.31 kg of mass margin, and probably 40–60 mm of height against 12 mm of margin. It is worth building only if a simulation experiment shows the bow, squat and pick-up need it; hip flexion may do most of that work. Wrist yaw is cheap (about 0.23 kg, $90) but none of the current skills needs it.

Sources:
- **G1 joints, ranges and torques**: ProtoMotions' `g1_bm_no_mesh_box_feet.xml` (Apache-2.0) and the `g1_29dof_rev_1_0.xml` model shipped with NVIDIA's SOMA retargeter. Joint speeds come from Isaac Lab's `G1_29DOF_CFG` (BSD-3-Clause). Both models give 33.3 kg.
- **G1 product figures**: Unitree's G1 page (https://www.unitree.com/g1, read 2026-10-28), cited, not copied.
- **Marvin Bot**: `description/robot.yaml`, `actuators.yaml`, `mass_budget.yaml`, the generated URDF, `bom/bom.csv`, and the clearance checks rerun with widened ranges in a scratch copy (method below).

## The two robots

| | Unitree G1 (29-DOF) | Marvin Bot |
|---|---|---|
| Height | 1.32 m | 1.288 m (R1: 1.20–1.30 m) |
| Width | 450 mm overall | 416 mm across the shoulder actuators |
| Mass | about 35 kg with battery (models: 33.3 kg) | 39.86 kg bare, 40.69 kg with shells (R2: 41 kg limit) |
| Body DOF | 29: legs 2 × 6, waist 3, arms 2 × 7 | 27: legs 2 × 6, waist 1, neck 2, arms 2 × 6 |
| Hands | optional three-finger hands, 7 DOF each | bought four-finger hands, 8 DOF each |
| Battery | 13S, 9 Ah, quick release (~420 Wh at 3.6 V per cell); about 2 h | 13S3P 21700, ~630 Wh, quick release; ~1.5 h estimated |
| Compute | 8-core CPU; an Orin-class module optional | Jetson AGX Orin 64 GB developer kit |
| Sensing | depth camera, 3D lidar, microphone array | active-stereo depth camera, IMU |
| Arm payload | about 2 kg (EDU about 3 kg) | about 0.5 kg with the bought hands (R20 deviation: the wrist pitch servo) |
| Price | US $13.5k base (EDU on request) | ~$12,000 in parts (`bom/`), without labour |

Peak torque and speed per joint. For Marvin Bot: peak and rated torque from `actuators.yaml`, and no-load speed. For G1: the models' effort limits and Isaac Lab's velocity limits; the configs used here give no arm speeds.

| Joint | G1 N·m | G1 rad/s | Marvin Bot actuator | N·m peak (rated) | rad/s |
|---|---|---|---|---|---|
| Hip pitch | 88 | 32 | RS04 | 120 (40) | 20.9 |
| Hip roll | 88–139 (the two models differ) | 32 | RS04 | 120 (40) | 20.9 |
| Hip yaw | 88 | 32 | RS03 | 60 (20) | 20.4 |
| Knee | 139 (the page: 90, EDU 120) | 20 | RS04 | 120 (40) | 20.9 |
| Ankle pitch / roll | 50 / 50 | 37 | two RS03 on a linkage | 32 / 25 continuous at the worst pose | ~20 |
| Waist yaw | 88 | 32 | RS03 | 60 (20) | 20.4 |
| Waist roll, pitch | 50, 50 | 37 | none | | |
| Shoulder pitch, roll | 25 | | RS03 | 60 (20) | 20.4 |
| Shoulder yaw, elbow | 25 | | RS06 | 36 (11) | 50.3 |
| Wrist roll (Marvin Bot's `wrist_yaw`, the forearm twist) | 25 | | RS00 | 14 (5) | 33.0 |
| Wrist pitch | 5 | | STS3250 | 4.9 (1.57) | 7.9 |
| Wrist yaw | 5 | | none | | |

What this says: Marvin Bot's legs are about as strong as G1's and its arms are stronger, but it is slower. Its hip actuators reach about two thirds of G1's hip speed (20.9 against 32 rad/s); the knee speed is about the same. It is 5–6 kg heavier at a similar height, mostly in the developer-kit computer, the larger battery and the stronger arm motors. The gaps that matter are the ranges and the two missing waist joints, and the rest of this note is about them.

## How the limits were found

Each joint was widened, one at a time, to intermediate and G1-like ranges in a scratch copy of the repository, and both clearance checks were rerun:

- `python -m cad.covers --check` checks the shells and the real part solids. Each joint goes to both ends of its range and half of each end, with the others at zero, plus the check's own combined poses. It counts limb-against-body contacts, arms against torso and pelvis included. Leg-against-leg contacts are left to the simulators.
- `tools/check_packaging.py` checks the motor envelopes and collision boxes. It reads the ranges from the URDF, so `tools/build_urdf.py` was run first. An overlap within a limb is a packaging error; one between limbs is self-collision, which the simulators model.

A third run checked combined poses from the skills, each pose on its own: the arm forward and adducted (clap), out and yawed (wave), forward and yawed (reach), the hip yawed with the knee bent, and the waist turned with the arms forward. The scripts were scratch work and are not in the repository. All figures below are overlap volumes beyond what the parts already share at rest.

| Widened to | Result |
|---|---|
| Shoulder roll −15°, −30°, −91° (arm at the side) | the upper-arm shell touches the torso shell from −8°; the upper arm itself hits the torso from −15° (4 cm³), and by −30° the roll bracket and upper arm do too (61 cm³) |
| Shoulder roll −15° / −30° / −45° with the arm pitched 90° | clear / clear / the torso shell touches the roll bracket (1.8 cm³) |
| Shoulder roll −30° with the arm at 60° / 45° / 30° | shell touch 0.8 cm³ / 3.7 cm³ and the bracket 0.6 cm³ / the bracket meets the torso (7 cm³) |
| Shoulder yaw ±120°, ±150° | clear, also with the arm out (wave), forward (reach) and up |
| Shoulder pitch −90° (extension) | clear |
| Elbow 150° | the forearm meets the shoulder roll link (16 cm³): 135° stays |
| Wrist twist ±113° | clear |
| Wrist pitch ±93° | +93°: the forearm and palm collision boxes overlap (9 cm³); −93°: the hand mount touches the wrist cradle (1.5 cm³) |
| Waist yaw ±150° | clear for parts and shells, with the arms down or forward; hanging arms already meet the pelvis and hip motors from 45–68° at today's ±90° (self-collision) |
| Hip roll −30° | clear for parts and shells. In the collision boxes the shins already overlap from about −11° to −20° when the other leg is straight, at today's limit too (leg-against-leg self-collision) |
| Hip roll +90°, +170° | the roll bracket meets the pelvis from about +85° (2 cm³) |
| Hip yaw ±90°, ±158° | clear, also with the knee bent and the hip flexed 60° |
| Hip pitch −45° (extension) | the thigh yoke meets the pelvis (13 cm³), as recorded |
| Hip pitch +145° (flexion) | the thigh meets the pelvis (4.5 cm³) and the torso's collision box (88 cm³) |
| Knee 145°, 165° | at 145° the shin meets the thigh and hip pitch link, and the foot meets the pelvis and hip roll motor (up to 8.5 cm³); at 165°, 104 cm³ |

## Per joint

Ranges use the `robot.yaml` convention (left side; pitch + = flexion, roll + = abduction, yaw + = towards the midline). G1's own signs differ (its hip and shoulder pitch are negative for flexion), so they are converted here. Tiers are defined in the next section. Costs use the BOM's prices.

| Joint | G1 | Marvin Bot | What limits Marvin Bot | What a change takes | Tier | Mass, cost | Skills helped |
|---|---|---|---|---|---|---|---|
| Shoulder roll | −91° to 129° | 0° to 120° | The torso, with the arm at the side: the shoulders were narrowed (decisions, 2026-10-26), so the arm hangs 158 mm off the centre line, close to the torso. With the arm forward, −30° is clear. | Range to −30°, with a limit that depends on pitch: about 0° below 30° of pitch, −30° from about 75°. It is enforced by self-collision in training and a keep-out in the safety layer (R51). −45° needs a torso shell trim at the roll bracket. −91° at the side would need the wider shoulders back (+60 mm). | 1 (−30°); 2 (−45°) | 0 | clap, hands together, hand across the chest |
| Shoulder yaw | ±150° | ±90° | Nothing in the CAD; no reason is recorded for ±90°. The hand meets the thigh at +90° with the arm down (self-collision, as today). The service loop grows from 0.11 to 0.18 m. | Range only | 1 | 0 | two-hand wave, reach |
| Shoulder pitch | 177° flexion, 153° extension | 180°, 60° | Nothing at −90° | Range only (optional) | 1 | 0 | none listed (arm swing, pushing up) |
| Elbow | 180° of travel (about another zero) | 0° to 135° | The forearm meets the shoulder roll link beyond 135° | No change needed | n/a | 0 | none |
| Wrist twist (G1: wrist roll) | ±113° | ±90° | Nothing in the CAD. The hand's power and serial bus cross the RS00. | Range only | 1 | 0 | turning the palm up and in |
| Wrist pitch | ±92.5° | ±60° | The hand mount against the wrist cradle at −93° (1.5 cm³); the forearm and palm boxes at +93° | A relief in the cradle and mount; the range between ±60° and ±93° was not tested in steps | 2 (small) | ~0 | manipulation |
| Waist yaw | ±150° | ±90° | Nothing in the CAD. The trunk harness (power and 5 CAN pairs) loop grows from 0.11 to 0.18 m. | Range only | 1 | 0 | looking back, turning gestures, dance |
| Hip roll | −30° to 170° | −20° to 45° | Adduction: leg against leg (the shins meet with the other leg straight), not the CAD. Abduction: the roll bracket meets the pelvis near +85°. | Range to −30° (abduction to ~80° if wanted); the simulators keep the legs apart | 1 | 0 | sidesteps (in part, see below) |
| Hip yaw | ±158° | ±45° | Nothing in the CAD to ±158°, even with the leg bent. The leg harness loop grows; the walking rewards hold yaw near stance. | Range to ±90° (the skills need no more) | 1 | 0 | turns on the spot, kick, dance |
| Hip pitch | 145° flexion, 165° extension | 120°, 30° | Flexion: the thigh against the pelvis and torso beyond about 130°. Extension: the thigh yoke against the hip roll motor (roll-first hip, decisions 2026-10-26). | Flexion to ~130°: pelvis reliefs. Extension: moving the hip roll motors, which would reopen the roll-first hip. | 2 | ~0 (reliefs) | deeper squat and pick-up |
| Knee | −5° to 165° | 0° to 130° | Beyond 130° the upper ankle motor meets the thigh shell (recorded); at 145° the shin meets the thigh and the foot meets the pelvis | A new shin and ankle-motor layout; 165° is out of reach with the motors on the shin | 2 (major) | unknown | kneeling, deep squat (the deep-squat pose uses 125°) |
| Ankle pitch, roll | 50° up, 30° down; ±15° | ±40°; ±20° | | No gap: Marvin Bot is wider | n/a | | |
| Waist roll, pitch | ±30° each | none | No joint: the waist is a single RS03 on the pelvis top plate | New joints (below) | 3 | +2.5 kg, +$630 | bow, squat, pick-up, leaning |
| Wrist yaw | ±92.5° | none | No joint | New joint (below) | 3 | +0.23 kg, +$90 | manipulation |
| Neck yaw, pitch | none | ±60°, −20° to 40° | | Marvin Bot's extra | n/a | | looking around |

## Tiers

### Tier 1: range-only changes to existing parts

These go in `robot.yaml`, with the URDF, harness and checks regenerated:

- shoulder roll to −30° with a pitch-dependent keep-out;
- shoulder yaw to ±150°;
- hip roll to −30°;
- hip yaw to ±90°;
- waist yaw to ±150°;
- optionally, wrist twist to ±113° and shoulder pitch to −90°.

No mass, no cost, no CAN change. What else moves:

- **Checks**: `cad.covers`' combined poses should gain the arm-forward-and-adducted poses, and the single-joint sweep needs to know about the pitch-dependent shoulder roll limit. At −30° with the arm at its side, the sweep would otherwise fail on contacts the robot never makes.
- **Harness**: `tools/build_harness.py` sizes each service loop from the range (range × 35 mm), so the loops at shoulder yaw, waist yaw and hip yaw grow by about 0.04–0.07 m each. Wider yaw loops need checking for flex life (open question).
- **Training**: the walking, stair and tracking policies retrain on the new ranges. Shoulder roll and hip roll rely on arm-to-torso and leg-to-leg self-collision being on in training (open question).
- **Hardware**: a joint limit that depends on another joint's angle cannot be set in the actuators. It belongs in the real-time control process's limit checks (R51).

### Tier 2: part redesigns

- **Torso shell trim at the shoulder roll bracket** lets shoulder roll reach −45° with the arm forward. Small; do it at the next pass on the torso shell.
- **Wrist cradle and hand-mount relief** lets wrist pitch reach about ±90°. Small; it is a printed part.
- **Pelvis reliefs for hip flexion to ~130°.** Moderate, and the pelvis plates are machined. Worth it only if the squat and pick-up references pin hip pitch at 120°.
- **Hip extension beyond −30°** would move the hip roll motors and reopen the roll-first hip; not recommended.
- **Knee beyond 130°** would need a new shin and ankle-motor layout; not recommended. No listed skill needs it.
- **Wider shoulders** would bring back shoulder roll below 0° at the side. That reverses the narrower-shoulders decision (+60 mm across, arms 30 mm further out), and the clap does not need it.

### Tier 3: new joints

**Waist roll and pitch** (G1: ±30° each, 50 N·m).

- **Load**: everything above the waist is 17.45 kg, with its CoM 0.117 m above the waist joint with the arms down (generated URDF). Leaning 30° takes about 10 N·m about the waist yaw joint's height, or 13 N·m if the new axes sit 40 mm lower. With both arms reaching forward (about 3.7 kg of moving arm each, about 0.2 m out) it takes about 25 N·m. That is above an RS03's 20 N·m rating and well inside its 60 N·m peak.
- **Actuators**: two RS03 driving a cross through cranks and push rods, like the ankle. Both motors share the pitch load, and the ankle's version gives 32 N·m pitch and 25 N·m roll continuous at its worst pose (`tools/ankle_linkage.py` can search a waist version). A single RS04 (40 N·m rated) would do for pitch alone. RobStride is the right family here; the STS3250 servos are far too weak.
- **Mass**: +1.80 kg of motors, about 0.4 kg of linkage (the ankle's mounts, cranks, rods and cross weigh 0.41 kg per ankle), and about 0.2–0.3 kg for the cross bearings and a stiffer waist flange. About **+2.5 kg** in all, an estimate until it is drawn. Pitch alone on one RS04: about +1.7 kg.
- **Cost**: two RS03 at $214 (the RS03 order goes from 11 to 13, still in the 10+ tier), so $428. About $60 of rod ends and bearings, about $50 of stock, and one more isolated USB-CAN adapter ($90). About **+$630**, 5% of the BOM.
- **The 41 kg limit**: 40.69 + 2.5 ≈ 43.2 kg, so `tools/check_description.py` fails by about 2.2 kg. The recorded offsets cover it only together:
  - the Orin module on a compact carrier board instead of the developer kit, −1.0 kg (the board's cost is not yet quoted);
  - lighter leg cores, −0.5 kg;
  - a 13S2P pack, −1.1 kg, which cuts the runtime to about 1 h against G1's 2 h.

  The other choice is to raise R2.
- **What else moves**:
  - **Height**: in series, a cross between pelvis and torso adds an estimated 40–60 mm, against 12 mm left under R1's 1.30 m. A yaw-roll-pitch stack at the hip was already ruled out for the same reason (decisions, 2026-10-26). Shortening the torso or neck, or moving R1, is open.
  - **Pelvis**: it holds the waist RS03, the hip roll RS04s, the 12 V DC-DC and IMU channel and the leg power board, so two more motors and a linkage need a new pelvis or lower-torso layout. The pelvis is built in phase 4, so this has to be decided before it is machined.
  - **CAN**: the right arm bus already carries six motors at 45%; eight would reach the 60% limit. A waist bus at 500 Hz with all three waist motors would run at 45%.
  - **Harness**: the trunk harness crosses two more joints.
  - **Shells**: the chest shell and hip girdle need a gap for ±30° of lean.
  - **Torques**: the extra 2.5 kg raises the knee and hip roll loads by about 6%. The slow-stair knee goes from about 109% to about 116% of its rating, already a phase 2 bench item.
  - **Policies and retargeter**: 29 actions instead of 27, and the retargeter's link map gains the joints.

**Wrist yaw** (G1: ±92.5°, 5 N·m).

- **Actuator**: an STS3250 like the wrist pitch servo (4.9 N·m stall), turning the hand side to side under the pitch joint. An RS00 (0.31 kg, $139) is too heavy for the end of the arm.
- **Mass**: +0.075 kg of servo and about 0.04 kg of cradle per side, so about **+0.23 kg**. That nearly fills the 0.31 kg margin, and would not fit with the waist joints unless they bring their own offsets.
- **Cost**: two STS3250 at $45, so about **+$90**, plus printed parts. They join the existing servo bus, with no CAN change.
- **What else moves**: the wrist grows by about 35–45 mm, so the arm, already 9% longer than a person's at this height, grows further. The lever on the wrist pitch servo grows too, from 75% of its rating with 0.5 kg at the fingertips to perhaps 85–90%. The forearm shell and hand mount are redrawn.

## Which tiers the motion skills need

- **Clap** (shoulder roll below 0°): **tier 1**. With the arms forward, as in a clap, −30° is clear. Only the pitch-dependent limit is new.
- **Two-hand wave and reach** (shoulder yaw pinned at 90°): **tier 1**. ±150° is clear in the wave and reach poses.
- **Sidesteps** (hip roll at −20°): **tier 1** helps, but only so far. With the other leg straight, the legs already touch inside today's range, so the extra 10° is usable only when the other leg is out. The bigger cause is recorded in decisions (2026-10-28): Kimodo tends to cross or close the feet in sidesteps. Wider stance prompts or a foot-spacing filter in `generate_kimodo.py` belong alongside the range change.
- **Bow, squat, pick-up** (no waist pitch): **tier 3** for a G1-like motion. But G1's waist pitch is only ±30°, so even on G1 most of a bow or pick-up is hip flexion, and Marvin Bot has 120° of that. Before any hardware, the retargeter could put the reference's torso pitch into the hips, and a simulation-only variant of the description with a waist pitch joint could show how much tracking it buys. Hip flexion to ~130° (tier 2) may help the pick-up and squat as much.

## Recommendation

1. **Now (simulation, no parts).** Apply tier 1:
   - shoulder roll −30° with the pitch-dependent keep-out;
   - shoulder yaw ±150°;
   - hip roll −30°;
   - hip yaw ±90°;
   - waist yaw ±150°;
   - optionally, wrist twist ±113°.

   Add the arm-forward poses to `cad.covers`, check that arm-to-torso and leg-to-leg self-collision is on in training, and retrain the trackers on the clap, wave, reach and sidesteps. This needs a decision in `docs/decisions.md`, since it widens settled ranges. Cost and mass: zero.
2. **Next (simulation experiment).** Retarget torso pitch into hip pitch. Separately, try a description variant with waist pitch, and roll if wanted, at RS03 strength and +2.5 kg. Compare the bow, squat and pick-up trackers. Decide on the waist joints from that comparison, before the pelvis is machined in phase 4.
3. **Only if the experiment shows a clear gain.** Draw the two-RS03 linkage waist and settle three things together: the mass offsets or a new R2, the height against R1, and a waist CAN bus. Order the two extra RS03 with the batch of 11 that already follows the bench test (no parts are ordered by this study).
4. **When those parts are next touched.** Do the small tier 2 items: the torso shell trim and the wrist cradle relief.
5. **Leave for now.** Hip extension, knee range and wider shoulders, which reopen settled layout decisions and help none of the listed skills. Leave wrist yaw until teleoperation and manipulation data (phase 6) show a need.

## Open questions

- Do the tracking and walking simulations model arm-to-torso and leg-to-leg self-collision? The tier 1 shoulder roll and hip roll changes rely on it.
- How should the pitch-dependent shoulder roll limit be expressed: in the policies (self-collision only), in the control process's limit checks, and in `cad.covers` (a coupled range rather than a box)?
- Do the RS03 and RS06 keep their position across ±150° of output travel through a power cycle, or do they need homing? Check the manuals before relying on yaw ranges past ±90°.
- Will the wider service loops at the shoulder, waist and hip yaw last? Bend loops of 0.15–0.18 m were not sized for flex life.
- Where does a two-motor waist fit, and what does the robot give up in height (R1) or mass (R2, or runtime with a 13S2P pack)?
- Is Marvin Bot's hip speed (20.9 rad/s against G1's 32) enough for the faster skills (jog, kick, dance)? The trackers' peak joint speeds would show it.
- The two G1 models disagree on hip roll torque (88 against 139 N·m). It does not change this study, but either figure should be cited as the model's, not as Unitree's specification.
