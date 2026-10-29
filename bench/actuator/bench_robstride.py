"""Phase 2 actuator bench tests for one Robstride motor on a SocketCAN bus. UNTESTED.

Subcommands (see bench/README.md for the fixture and the safety rules):

  scan      list the motors that answer on the bus
  latency   command -> feedback round-trip times at a given rate (CAN timing on the Orin)
  step      position steps with the training gains; logs position, velocity, torque (fits the
            sim actuator model: armature, friction, effective kp/kd)
  thermal   hold a constant torque against the locked output and log temperature
            (continuous-rating check for the knee and hip-roll stair loads)

Every run writes a CSV to bench/data/ (git-ignored); summaries go to bench/results/.

Usage:
  python bench/actuator/bench_robstride.py scan --channel can0
  python bench/actuator/bench_robstride.py latency --channel can0 --id 1 --model RS04 --rate 500
  python bench/actuator/bench_robstride.py step --channel can0 --id 1 --model RS04 --kp 150 --kd 5
  python bench/actuator/bench_robstride.py thermal --id 1 --model RS04 --torque 47 --minutes 10

Needs: pip install -e ".[bench]"; a CAN interface up at 1 Mbit/s (ip link set can0 up type can
bitrate 1000000).
"""

import argparse
import csv
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "robot" / "drivers"))
import robstride as rs  # noqa: E402

DATA = ROOT / "bench" / "data"
MAX_TEMP_C = 90.0  # stop thermal runs well below the motor's 145 °C winding limit


def writer(name: str):
    DATA.mkdir(parents=True, exist_ok=True)
    path = DATA / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    f = path.open("w", newline="")
    w = csv.writer(f)
    w.writerow(["t", "position", "velocity", "torque", "temperature", "command", "faults"])
    return path, f, w


def run_scan(args) -> None:
    bus = rs.RobstrideBus(args.channel, {i: args.model for i in range(1, 128)})
    for motor_id in range(1, 128):
        bus.stop(motor_id)
        time.sleep(0.002)
    found = sorted({fb.motor_id for fb in bus.poll(0.2)})
    print("motors answering:", found or "none")
    bus.close()


def run_latency(args) -> None:
    bus = rs.RobstrideBus(args.channel, {args.id: args.model})
    bus.enable(args.id)
    times, period = [], 1.0 / args.rate
    for _ in range(int(args.seconds * args.rate)):
        t0 = time.perf_counter()
        bus.command(args.id, 0.0, 0.0, 0.0, 0.0, 0.0)  # zero gains: motor stays limp
        while not bus.poll(period):
            if time.perf_counter() - t0 > period:
                break
        times.append(time.perf_counter() - t0)
        time.sleep(max(0.0, period - (time.perf_counter() - t0)))
    bus.stop(args.id)
    times.sort()
    n = len(times)
    print(
        f"{n} commands at {args.rate} Hz: round trip median {times[n // 2] * 1e3:.2f} ms, "
        f"p99 {times[int(n * 0.99)] * 1e3:.2f} ms, max {times[-1] * 1e3:.2f} ms"
    )
    bus.close()


def run_step(args) -> None:
    bus = rs.RobstrideBus(args.channel, {args.id: args.model})
    path, f, w = writer(f"step_{args.model}_{args.id}")
    bus.enable(args.id)
    t0, period = time.perf_counter(), 1.0 / args.rate
    targets = [0.0, args.amplitude, 0.0, -args.amplitude, 0.0]
    try:
        for target in targets:
            t_end = time.perf_counter() + args.hold
            while time.perf_counter() < t_end:
                bus.command(args.id, target, 0.0, args.kp, args.kd, 0.0)
                for fb in bus.poll(period):
                    w.writerow(
                        [
                            time.perf_counter() - t0,
                            fb.position,
                            fb.velocity,
                            fb.torque,
                            fb.temperature,
                            target,
                            ";".join(fb.faults),
                        ]
                    )
    finally:
        bus.stop(args.id)
        f.close()
        bus.close()
    print(f"wrote {path.relative_to(ROOT)}")


def run_thermal(args) -> None:
    bus = rs.RobstrideBus(args.channel, {args.id: args.model})
    path, f, w = writer(f"thermal_{args.model}_{args.id}_{args.torque:.0f}Nm")
    bus.enable(args.id)
    t0 = time.perf_counter()
    try:
        while time.perf_counter() - t0 < args.minutes * 60:
            bus.command(args.id, 0.0, 0.0, 0.0, 0.0, args.torque)  # pure torque, output locked
            for fb in bus.poll(0.01):
                w.writerow(
                    [
                        time.perf_counter() - t0,
                        fb.position,
                        fb.velocity,
                        fb.torque,
                        fb.temperature,
                        args.torque,
                        ";".join(fb.faults),
                    ]
                )
                if fb.temperature > MAX_TEMP_C or fb.faults:
                    print(f"stopping: {fb.temperature:.1f} °C, faults {fb.faults}")
                    return
    finally:
        bus.stop(args.id)
        f.close()
        bus.close()
        print(f"wrote {path.relative_to(ROOT)}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("scan", "latency", "step", "thermal"):
        p = sub.add_parser(name)
        p.add_argument("--channel", default="can0")
        p.add_argument("--model", choices=sorted(rs.MODELS), default="RS04")
        if name != "scan":
            p.add_argument("--id", type=int, required=True)
        if name == "latency":
            p.add_argument("--rate", type=float, default=500.0)
            p.add_argument("--seconds", type=float, default=10.0)
        if name == "step":
            p.add_argument("--kp", type=float, default=150.0)
            p.add_argument("--kd", type=float, default=5.0)
            p.add_argument("--amplitude", type=float, default=0.3, help="rad")
            p.add_argument("--hold", type=float, default=1.0, help="s per step")
            p.add_argument("--rate", type=float, default=500.0)
        if name == "thermal":
            p.add_argument("--torque", type=float, required=True, help="N·m")
            p.add_argument("--minutes", type=float, default=10.0)
    args = ap.parse_args()
    {"scan": run_scan, "latency": run_latency, "step": run_step, "thermal": run_thermal}[args.cmd](
        args
    )


if __name__ == "__main__":
    main()
