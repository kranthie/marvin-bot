# Bench tests

Phase 2 of [the plan](../docs/plan.md): measure one RS04 and one RS03 before buying the rest. The scripts here are **untested** until the actuators arrive; they stay marked so until they have run.

Read [docs/safety.md](../docs/safety.md) first. On the bench: the motor bolted to a fixture, the e-stop in reach, low torque limits for the first run.

## Setup

- Bench supply at 48 V with a regen clamp (brake resistor or TVS at ~56–58 V), through the e-stop contactor.
- One isolated USB-CAN adapter (or the Orin's on-board CAN with a 3.3 V transceiver), 120 Ω termination at both ends of the bus.
- Bring the bus up: `sudo ip link set can0 up type can bitrate 1000000`
- `pip install python-can`

The driver is [robot/drivers/robstride.py](../robot/drivers/robstride.py) (Robstride's private protocol from the official manuals; frame encoding covered by `tests/test_robstride.py`).

## Tests

| Test | Command | What it decides |
|---|---|---|
| Scan | `bench_robstride.py scan` | Motor IDs and wiring |
| Latency | `bench_robstride.py latency --rate 500` | Command-to-feedback round trip on the Orin; whether USB-CAN adapters are good enough or a microcontroller bridge is needed ([decisions](../docs/decisions.md): six motor buses) |
| Step | `bench_robstride.py step --kp 150 --kd 5` | Fits armature, friction and effective gains for `sim/isaaclab/marvin_bot/robot_cfg.py` and the MuJoCo checks |
| Thermal | `bench_robstride.py thermal --torque 38 --minutes 10` | Whether the RS04 holds the ~38 N·m stair-step knee load (96% of rated) without overheating, and how much heat sinking the knee needs ([roadmap](../docs/roadmap.md) open item 11) |

Also to measure by hand: output backlash (dial indicator on a lever arm), the actual mass, and the output flange and mounting dimensions against the STEP model.

Raw logs go to `bench/data/` (git-ignored); write a short summary of each run to `bench/results/` and update `description/actuators.yaml` with measured values (`verified: bench`).
