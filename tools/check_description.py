"""Check that description/ and the BOM agree with each other, and print a summary.

Checks:
- every joint has a valid range and is driven by exactly one motor, or by a pair of motors
  through a linkage;
- every motor's model exists in actuators.yaml and its bus exists in robot.yaml;
- CAN bus load stays at or below the configured maximum at each bus's command rate;
- the total mass (motors from actuators.yaml plus mass_budget.yaml, plus the add-on body shells
  from cad_model.yaml) is under the hard limit (a warning above the target);
- bom/bom.xlsx lists exactly as many of each motor model as robot.yaml uses (the qty column;
  spares are a column of their own);
- Lite (robot.yaml variants.lite): its own joints and motors are consistent, its buses' CAN
  load is within the maximum, the BOM's Lite qty column lists its motors, and its URDF with the
  shells is under its mass limit.

Usage:
  python tools/check_description.py
"""

import sys
from collections import Counter
from pathlib import Path

import yaml
from bom.book import read_book

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
CAD_MODEL = DESC / "cad_model.yaml"
SIDES = ("left", "right")


def expand(entries: list[dict]) -> list[dict]:
    """Expand entries whose fields contain "{side}" into a left and a right copy."""
    out = []
    for e in entries:
        if any("{side}" in str(v) for v in e.values()):
            for side in SIDES:
                out.append(
                    {
                        k: (
                            [x.replace("{side}", side) for x in v]
                            if isinstance(v, list) and v and isinstance(v[0], str)
                            else v.replace("{side}", side)
                            if isinstance(v, str)
                            else v
                        )
                        for k, v in e.items()
                    }
                )
        else:
            out.append(dict(e))
    return out


def expand_buses(buses: dict) -> dict:
    out = {}
    for name, spec in buses.items():
        for side in SIDES if "{side}" in name else (None,):
            out[name.replace("{side}", side) if side else name] = spec
    return out


def lite(robot: dict, acts: dict, motors: list[dict], buses: dict, errors: list[str]) -> None:
    """Lite keeps the motors driving Max's joints from its keep_from joint up and adds its
    own; the kept joints come from the Lite URDF (tools/build_urdf.py)."""
    import xml.etree.ElementTree as ET  # noqa: PLC0415

    v = robot["variants"]["lite"]
    urdf = ET.parse(DESC / "urdf" / "marvin_bot_lite_covers.urdf").getroot()
    joints = {j.get("name") for j in urdf.iter("joint")}
    own = {j["name"] for j in v["joints"]}
    if not own <= joints:
        errors.append(f"Lite URDF lacks joints {sorted(own - joints)}: run tools/build_urdf.py")
    kept = [m for m in motors if all(d in joints for d in m["drives"])]
    lite_motors = kept + v["motors"]
    lite_buses = {b: buses[b] for b in {m["bus"] for m in kept}} | v["buses"]
    driven = {d for m in lite_motors for d in m["drives"]}
    if driven != joints:
        errors.append(f"Lite: joints without a motor or motors without a joint: {driven ^ joints}")
    for m in v["motors"]:
        if m["model"] not in acts:
            errors.append(f"Lite motor {m['name']}: unknown model {m['model']}")
        elif acts[m["model"]]["interface"] != lite_buses[m["bus"]]["type"]:
            errors.append(f"Lite motor {m['name']}: {m['model']} cannot run on bus {m['bus']}")
    can = robot["can"]
    per_bus = Counter(m["bus"] for m in lite_motors)
    print(f"\nLite: {len(joints)} joints, {len(lite_motors)} motors")
    for name, spec in lite_buses.items():
        if spec["type"] != "can":
            continue
        n = per_bus.get(name, 0)
        load = n * spec["rate_hz"] * can["frames_per_motor_cycle"] * can["bits_per_frame"]
        load /= can["bitrate"]
        print(f"{name:14} {n:6d} {spec['rate_hz']:8d} {load:6.0%}")
        if load > can["max_load"]:
            errors.append(f"Lite bus {name}: load {load:.0%} > {can['max_load']:.0%}")
    want = Counter(m["model"] for m in lite_motors)
    in_bom: Counter = Counter()
    for r in read_book()["lines"]:
        if r["model"]:
            in_bom[r["model"]] += int(r.get("lite_qty") or 0)
    for k in sorted(set(want) | set(in_bom)):
        if want.get(k, 0) != in_bom.get(k, 0):
            errors.append(f"BOM lists {in_bom.get(k, 0)} x {k} for Lite, it uses {want.get(k, 0)}")
    mass = sum(float(m.get("value")) for m in urdf.iter("mass"))
    print(f"Lite with the shells: {mass:.2f} kg, limit {v['mass_limit_kg']:.1f} kg")
    if mass > v["mass_limit_kg"]:
        errors.append(f"Lite: {mass:.2f} kg with the shells over its {v['mass_limit_kg']} kg limit")


def main() -> int:
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    budget = yaml.safe_load((DESC / "mass_budget.yaml").read_text())
    errors: list[str] = []
    warnings: list[str] = []

    joints = {j["name"]: j for j in expand(robot["joints"])}
    motors = expand(robot["motors"])
    can = robot["can"]
    buses = expand_buses(can["buses"])
    if len(joints) != len(expand(robot["joints"])):
        errors.append("duplicate joint names")

    for name, j in joints.items():
        lo, hi = j["range"]
        if not lo < hi:
            errors.append(f"{name}: range {j['range']} is not increasing")
    for c in expand(robot.get("coupled_limits", [])):
        if c["joint"] not in joints or c["by"] not in joints:
            errors.append(f"coupled limit {c['joint']} by {c['by']}: unknown joint")
            continue
        lo, hi = joints[c["joint"]]["range"]
        blo, bhi = joints[c["by"]]["range"]
        bys = [b for b, _ in c["min"]]
        if bys != sorted(bys) or not all(blo <= b <= bhi and lo <= q <= hi for b, q in c["min"]):
            errors.append(
                f"coupled limit {c['joint']} by {c['by']}: points outside the ranges or unsorted"
            )

    drivers: dict[str, list[str]] = {n: [] for n in joints}
    for m in motors:
        if m["model"] not in acts:
            errors.append(f"motor {m['name']}: unknown model {m['model']}")
        if m["bus"] not in buses:
            errors.append(f"motor {m['name']}: unknown bus {m['bus']}")
        elif m["model"] in acts and acts[m["model"]]["interface"] != buses[m["bus"]]["type"]:
            errors.append(f"motor {m['name']}: {m['model']} cannot run on bus {m['bus']}")
        for j in m["drives"]:
            if j not in joints:
                errors.append(f"motor {m['name']}: drives unknown joint {j}")
            else:
                drivers[j].append(m["name"])
    for j, ms in drivers.items():
        if len(ms) == 1:
            continue
        linked = [m for m in motors if m["name"] in ms]
        if len(ms) == 2 and all(len(m["drives"]) == 2 for m in linked):
            continue  # parallel linkage: two motors share two joints
        errors.append(f"joint {j}: driven by {ms or 'nothing'}")

    dof = sum(1 for j in joints.values() if j.get("powered_dof", True))
    print(f"joints: {len(joints)} ({dof} body DOF), motors: {len(motors)}")

    # CAN load
    print(f"\n{'bus':14} {'motors':>6} {'rate Hz':>8} {'load':>6}")
    per_bus = Counter(m["bus"] for m in motors)
    for name, spec in buses.items():
        if spec["type"] != "can":
            continue
        n = per_bus.get(name, 0)
        load = n * spec["rate_hz"] * can["frames_per_motor_cycle"] * can["bits_per_frame"]
        load /= can["bitrate"]
        flag = "  <-- over max" if load > can["max_load"] else ""
        print(f"{name:14} {n:6d} {spec['rate_hz']:8d} {load:6.0%}{flag}")
        if load > can["max_load"]:
            errors.append(f"bus {name}: load {load:.0%} > {can['max_load']:.0%}")

    # mass
    models = Counter(m["model"] for m in motors)
    motor_mass = sum(acts[k]["mass"] * n for k, n in models.items() if k in acts)
    other = sum(v["mass"] for v in budget["items"].values())
    total = motor_mass + other
    # the add-on body shells (cad/covers.py) count towards the limit, not the bare robot
    cad = yaml.safe_load(CAD_MODEL.read_text()) if CAD_MODEL.exists() else {}
    shells = sum(p["mass"] for p in cad.values() if p["budget"] == "covers")
    target, limit = robot["robot"]["mass_target_kg"], robot["robot"]["mass_limit_kg"]
    print(f"\n{'model':10} {'qty':>4} {'kg':>7}")
    for k, n in sorted(models.items()):
        print(f"{k:10} {n:4d} {acts[k]['mass'] * n:7.2f}")
    print(f"motors {motor_mass:.2f} kg + other {other:.2f} kg = {total:.2f} kg bare")
    print(f"with the body shells ({shells:.2f} kg): {total + shells:.2f} kg")
    print(f"target {target:.1f} kg bare, hard limit {limit:.1f} kg with the shells")
    if total + shells > limit:
        errors.append(f"mass {total + shells:.2f} kg with the shells over the {limit} kg limit")
    elif total > target:
        warnings.append(f"mass {total:.2f} kg is over the {target} kg target")

    # BOM motor counts
    in_bom: Counter = Counter()
    for r in read_book()["lines"]:
        if r["model"]:
            in_bom[r["model"]] += int(r["qty"])
    for k in sorted(set(models) | set(in_bom)):
        if models.get(k, 0) != in_bom.get(k, 0):
            errors.append(f"BOM lists {in_bom.get(k, 0)} x {k}, robot.yaml uses {models.get(k, 0)}")

    lite(robot, acts, motors, buses, errors)

    for w in warnings:
        print(f"WARNING: {w}")
    for e in errors:
        print(f"ERROR: {e}")
    if errors:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
