# Bench tests

Phase 2 of [the plan](../docs/plan.md): measure one RS04 and one RS03 before buying the rest. The scripts here are **untested** until the actuators arrive; they stay marked so until they have run.

Read [docs/safety.md](../docs/safety.md) first. On the bench: the motor bolted to a fixture, the e-stop in reach, low torque limits for the first run.

## Setup

- Bench supply at 48 V with a regen clamp (brake resistor or TVS at ~56–58 V), through the e-stop contactor.
- One isolated USB-CAN adapter (or the Orin's on-board CAN with a 3.3 V transceiver), 120 Ω termination at both ends of the bus.
- Bring the bus up: `sudo ip link set can0 up type can bitrate 1000000`
- `pip install -e ".[bench]"` (python-can)

The driver is [robot/drivers/robstride.py](../robot/drivers/robstride.py) (Robstride's private protocol from the official manuals; frame encoding covered by `tests/test_robstride.py`).

## Tests

| Test | Command | What it decides |
|---|---|---|
| Scan | `bench_robstride.py scan` | Motor IDs and wiring |
| Latency | `bench_robstride.py latency --channel can0 --id 1 --model RS04 --rate 500` | Command-to-feedback round trip on the Orin; whether USB-CAN adapters are good enough or a microcontroller bridge is needed ([decisions](../docs/decisions.md): six motor buses) |
| Step | `bench_robstride.py step --channel can0 --id 1 --model RS04 --kp 150 --kd 5` | Fits armature, friction and effective gains for `sim/isaaclab/marvin_bot/robot_cfg.py` and the MuJoCo checks |
| Thermal | `bench_robstride.py thermal --id 1 --model RS04 --torque 47 --minutes 10` | Whether the RS04 holds the ~47 N·m slow-stair knee load (118% of rated) without overheating, and how much heat sinking the knee needs ([roadmap](../docs/roadmap.md) open item 11) |

Lite's column ([design/lite.md](../docs/design/lite.md)), before a second one or the base is
bought:

| Test | How | What it decides |
|---|---|---|
| Direct drive | the DL6 on the bench, its motor on a 24 V H-bridge (current-limited supply), its Hall sensor on a microcontroller; run it up and down at low duty first | Whether it runs without LINAK's control box, which is what Lite is designed around (the TiMOTION TL18 is the fallback) |
| Speed and position | stroke against the Hall count, loaded with 25 kg | Its counts per mm, speed under load, and repeatability for the column joint's model |
| Bending at full stroke | clamp its foot, extend 450 mm, hang 10 kg 0.5 m off its axis (about 50 N·m) | Sway at the top, and play, against Lite's 52 N·m worst case |

Also to measure by hand: output backlash (dial indicator on a lever arm), the actual mass, and the output flange and mounting dimensions against the STEP model.

Raw logs go to `bench/data/` (git-ignored); write a short summary of each run to `bench/results/` and update `description/actuators.yaml` with measured values (`verified: bench`).
