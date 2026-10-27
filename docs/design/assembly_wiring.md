# Assembly, wiring and covers

How the robot comes apart, how the wiring runs, and what the optional covers add. Status as of 2026-10-26: first-pass CAD of every structural part (`cad/`), the electronics bay, battery cradle and limb-root bulkheads at their parts' outlines (`cad/electrical.py`), a generated harness (`tools/build_harness.py` → `electrical/harness.yaml`), and the covers as an add-on (`cad/covers.py`). Fastener holes are in the mounts; clip bosses and the bought parts' datasheet outlines are still to come.

## Modules

The robot is built from seven modules that are assembled and tested on their own, then joined at a small number of bolted interfaces and connectors:

| Module | Contents | Joins at | Connectors at the joint |
|---|---|---|---|
| Torso core | torso frame, chest shell, Orin, electronics bay, battery cradle, shoulder pitch and neck yaw motors | waist RS03 output flange (bolted) | trunk harness: power plus 5 CAN pairs |
| Pelvis | back plate, top plate and gussets, waist RS03, hip roll RS04s, pelvis power board, 12 V servo DC-DC, IMU | waist flange; leg flanges | leg bulkheads |
| Leg (×2) | roll bracket to foot: 5 actuators (hip pitch, yaw, knee, two ankle), linkage, foot | hip roll RS04 output flange and the far-side bearing behind it | 1 power, 1 signal, at the leg bulkhead |
| Arm (×2) | shoulder bracket to hand: 5 actuators, wrist pitch servo, hand | shoulder pitch RS03 output flange | 1 power, 1 signal, at the arm bulkhead |
| Head | neck bracket, pitch servo, frame, shell, visor, camera | neck yaw servo horn | servo bus, camera USB-C |
| Battery | 13S pack with BMS | slides into the cradle from the back | blind-mate power and BMS connector |
| Body shells | clip-on shells (optional) | clips on the parts below | none |

Each limb's first actuator stays on the parent module (hip roll on the pelvis, shoulder pitch on the torso), so a limb comes off by removing the bolts on one output flange and unplugging two connectors. That meets R63: any actuator in a limb can be changed with only that limb off the robot.

## Making the parts

`python -m cad.export` writes an STL of every made part (mm, sitting on z = 0) and a DXF outline of every flat part (a plate of one thickness: the laser or water-jet service's input, and the mill's 2D toolpaths) to `outputs/parts/`, and two build lists that CI keeps current: `bom/build_parts.csv` (each made part once, mirror pairs as quantity 2, with process, material, stock thickness, mass and size; bought parts and shells listed for completeness) and `bom/build_fasteners.csv` (the actuator bolts from each motor's stator and output hole circles). Right-side parts are the left ones mirrored and are exported as their own files.

## Assembly order

1. Pelvis: back plate with the hip roll RS04s, top plate and gussets, waist RS03, pelvis power board; bench-test the three motors on a short CAN stub.
2. Legs: build each leg from the hip roll bracket down; set the ankle linkage (rod lengths from `tools/ankle_linkage.py`) with the foot flat at zero; test each leg on the single-leg jig before joining.
3. Torso core: frame, shoulder pitch RS03s, neck yaw servo, electronics bay, Orin, battery cradle and the e-stop chain; power up without limbs.
4. Join the torso core to the pelvis at the waist flange; plug in the trunk harness through its service loop.
5. Arms and head onto their flanges and connectors.
6. Calibrate joint zeros with the robot hung from the gantry (docs/safety.md), then add the covers if wanted.

## Electronics layout

| Where | What, in power-flow order |
|---|---|
| Torso, back | 13S pack on the base plate between two printed side guides; a latch plate behind it. It slides out backwards with the back shell off; its blind-mate connector (AS150 class) is at its front-top corner and seats in a socket over the computer, beside the computer's buttons. |
| Torso, middle | the computer on edge: fan forward (vents in the chest shell), I/O ports to the sides, buttons up, bolted by its feet to a plate behind it. |
| Torso, bay above the computer | main contactor and precharge, torso power board (bus bars, arm fuses), e-stop receiver and watchdog relay; the computer's DC-DC (fed ahead of the contactor); three isolated USB-CAN adapters beside the computer's USB ports. |
| Torso, behind the neck | keep-alive allowance (the parts that stay powered during an e-stop). |
| Shoulder plates | the arm bulkheads, behind the shoulder pitch motors. |
| Pelvis channel, under the waist motor | 12 V DC-DC for the servos (a compact 10 A brick, at most 60 × 34 × 25 mm), the IMU near the hip axes. |
| Pelvis, between the hip roll motors | pelvis power board (leg fuses) with the leg bulkheads under it. |

The outlines are typical sizes for each class of part; each is replaced with the chosen part's datasheet outline when it is bought, and the sweep and rest-pose audit rerun.

## Power and signal topology

```mermaid
flowchart LR
  BAT[13S battery + BMS] -->|AS150-class blind mate| FUSE[Class-T fuse 100 A]
  FUSE --> PRE[precharge + contactor]
  ESTOP[wired + wireless e-stop, watchdog] -.opens.-> PRE
  FUSE --> DCC[DC-DC 19-20 V: Orin, always on]
  PRE --> TPDB[torso power board]
  TPDB -->|20 A| LARM[left arm harness]
  TPDB -->|20 A| RARM[right arm harness]
  TPDB --> DC12[DC-DC 12 V: servos]
  TPDB -->|80 A, across the waist| PPDB[pelvis power board]
  PPDB -->|40 A| LLEG[left leg harness]
  PPDB -->|40 A| RLEG[right leg harness]
  PPDB --> WAIST[waist RS03]
  ORIN[Orin] -->|USB-CAN / on-board CAN| BUSES[6 CAN buses + BMS bus]
  ORIN -->|USB serial| SERVO[Feetech bus: neck, wrist pitch]
  ORIN -->|USB-C| CAM[depth camera]
```

The computer's supply is taken before the contactor, so an e-stop cuts every actuator but keeps the Orin, logs and state alive (docs/safety.md). The torso power board feeds the arms; the legs' power crosses the waist once, as a trunk, to a second board in the pelvis. The waist motor sits on the right-arm CAN bus (robot.yaml) but takes its power from the pelvis board.

## Harness

`tools/build_harness.py` generates `electrical/harness.yaml` from the description files, so the harness follows any change to motor positions, joint ranges or buses. CI checks it is up to date. Current figures:

| Harness | Branch current | Fuse | Length | Carries |
|---|---|---|---|---|
| Main (battery to torso board) | 78.5 A | 100 A | — | 6 AWG |
| Trunk (torso to pelvis, across the waist) | 51.3 A | 80 A | 0.60 m | 8 AWG, 4 leg CAN pairs, waist CAN |
| Leg (each) | 24.3 A | 40 A | 1.82 m | 12 AWG at the root down to 16 AWG; hip and shank CAN |
| Arm (each) | 13.6 A | 20 A | 1.79 m | 16 AWG; arm CAN; 12 V and serial for the wrist pitch servo and the hand |
| Head | — | — | 0.36 m | 12 V and serial for the neck servos; camera USB-C |

The currents are continuous estimates: rated torque at half the no-load speed, 80% efficiency, 48 V, with half the branch's motors at rated output at once. Peak currents are several times higher for short periods; the fuse and gauge choices are first-pass, and the real bus currents come from the phase 2 bench test (open item 3 in [roadmap.md](../roadmap.md)).

Rules the harness follows:

- **One harness per limb**: power is daisy-chained motor to motor in kinematic order, with the limb's CAN pair(s) twisted alongside. Each motor's own power and CAN connectors plug into the chain, so a motor swap needs no crimping.
- **Service loops at every joint**: a loop of length range × 35 mm around each joint the harness crosses (0.04–0.15 m per joint). The loop runs beside the motor housing, on the side away from the output flange, inside the clearance the packaging check keeps. The waist and shoulder pitch, the joints with the largest ranges, get the longest loops.
- **No wire carries load**: the harness is clipped to the structure on both sides of each loop (`harness.yaml` lists the clip pair, parent link and child link, for every loop), so motion only flexes the loop.
- **CAN**: twisted pair, 120 Ω termination at the hub end and on the last motor of each bus, shield or drain grounded at the hub only. Each leg carries two buses (hip and shank), so its signal connector has 6 pins.
- **Limb-root connectors**: a keyed 2-pin power connector rated above the branch fuse (XT60 class for legs and arms) and a latching signal connector (CAN pairs, ground, a spare), both mounted on a bulkhead on the parent module next to the limb's first flange (`elec_bulkhead_*` in `cad/electrical.py`).
- **Paths**: the leg harness leaves the bulkhead under the pelvis board, runs to the hip roll motor behind the hip, then past the hip pitch and yaw motors, down the back of the thigh shell and between the shin's side plates to the ankle motors. The arm harness leaves the torso beside the shoulder pitch motor, runs inside the upper-arm and forearm shells (they leave a cable channel; the upper-arm shell has an exit at the elbow for the loop), and ends at the wrist pitch servo, beside which a 12 V to 5 V converter feeds the hand's servos (their serial bus is separate: they use Feetech's SCS protocol, the other servos its STS protocol).

## Body shells (add-on)

The bare robot is meant to look finished: graphite printed shells (thigh, upper arm, forearm, chest), bead-blasted aluminium plates, the actuators' own housings and a glossy helmet head. The body shells are an optional layer that gives it a finished body in the style of current commercial humanoids: a helmet head with a full-face glossy visor over a slim dark neck, a smooth light torso that widens from the waist to the shoulders, a dark hip girdle, long light leg shells, arm sleeves and toe caps, with the joints left visible between them. The style is the general language only; the shapes are this project's own.

| Shell | Pieces | Notes |
|---|---|---|
| Torso | 8 (front and back, left and right, chest and waist) | follows the torso frame slice by slice (rounded sections ~13 mm outside it, blended where the width changes), so it is broad at the shoulders and slimmer at the chest and waist; the shoulders slope up into a collar round the neck; flat side walls with round openings for the shoulder motors; the back comes off for the battery |
| Neck sleeve | 1, dark, flexible print | flares from the helmet down into the collar; turns with neck yaw inside the torso's neck opening; the head pitches over it |
| Hip girdle | 1, dark | round the pelvis frame and waist motor; reaches over the hip motors in front, stays clear of the hip roll motors that swing round the hip behind |
| Thigh, shin | 1 each per leg | thigh all round except a strip at the back of the inner side (harness), and over the front of the knee motor; shin front and sides, open at the back where the cranks and rods move |
| Shoulder | 1 per side | over the top of the shoulder roll motor only, above the swing of the roll bracket (the hip roll motors behind the hips are left bare) |
| Upper arm, forearm, toe | 1 each per side | sleeves between the joints (the upper arm's runs down the back of the elbow motor); a cap over the front of the foot |

How they are made and checked:

- Each shell is a 1.2 mm printed skin at a 4 mm gap from the parts it covers (an offset of their hull, rounded; the torso is lofted from rounded sections), so the shells refit automatically when the structure changes.
- `python -m cad.covers --check` (run in CI) moves every joint to both ends and to half of each end of its range, plus combined poses (deep squat, knee up, arm up, hand to chest, looking down while turned), and fails if any shell touches a part or shell of another link. Shells on one limb are not checked against the other limbs; limb-to-limb contact is the policy's job.
- They clip on (clip and boss details come with the fastener pass), so they come off without tools, and they carry no load; they add some fall protection (R53).

The shells weigh 0.83 kg in all. The default model (`description/urdf/marvin_bot.urdf`, 39.86 kg with the hands) is the bare robot, and the policies are trained on it. `marvin_bot_covers.urdf` (40.68 kg) adds the shells' mass and visuals, inside the 41 kg limit, which applies to the robot with them; a robot that runs with them should be trained on it.

## Still to do

- Bearing seats, clip bosses on the clip points `harness.yaml` lists, and connector footprints on the bulkheads.
- Loop guides; check the loops against the packaging envelopes through the full joint ranges.
- Torso shell seams: clip and alignment details at the eight pieces' joins.
- Datasheet outlines for the bought electronics once chosen (the layout uses typical sizes).
- Measured bus currents (phase 2) to confirm the gauges and fuses.
