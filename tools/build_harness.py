"""Generate the wiring harness from the description: routes, lengths, wire gauges, fuses.

Reads robot.yaml (motors, buses, joint ranges), actuators.yaml (torque, speed) and
skeleton.yaml (frames, motor positions, equipment), and writes electrical/harness.yaml.

Topology (docs/design/assembly_wiring.md):
- The battery feeds a fuse, the e-stop contactor and the torso power distribution board (PDB).
  The torso PDB feeds the two arm branches and a trunk across the waist to the pelvis PDB,
  which feeds the two leg branches and the waist motor.
- Each limb has one harness: a power pair daisy-chained motor to motor in kinematic order, and
  the CAN pair(s) of that limb's bus(es) beside it (the Feetech serial bus and 12 V for the
  wrist pitch servo and the hand ride in the arm harness; the hand's servos run at 5 V from a
  converter at the wrist, on a serial bus of their own). It plugs in at the limb root through
  one power connector and one signal connector, so a limb comes off as a unit.
- Every joint the harness crosses gets a service loop sized for that joint's full range.

Sizing assumptions (first pass, to be replaced by bench measurements):
- a motor's continuous DC current is its rated torque at half its no-load speed, at 80 %
  efficiency, from a 48 V bus; a branch carries half the sum of its motors' currents (they are
  never all at rated output together);
- wire gauge from a conservative ampacity table for bundled silicone wire, but no finer than 16 AWG
  for actuator power (headroom for peak current, and the size the actuators' power connectors take);
  the wrist pitch servo and the hand get a 12 V pair from the servo DC-DC in the pelvis; fuses at
  the next standard rating above 1.25 x the branch current;
- run length is the straight distance between the motors' cable exits (a RobStride's driver cap,
  at its rear end) in the default pose x 1.3, plus each crossed joint's service loop
  (range x 35 mm), plus 80 mm for termination.

Each service loop is tied down on both sides, to a named plate of the parent link and one of the
child link (CLIPS: a cable tie through one of the plate's lightening pockets, or a P-clip under
one of its screws); the names are checked against description/cad_model.yaml, so a renamed or
removed part fails the check.

Lite (robot.yaml variants.lite) gets its own block, `lite`: the battery in the base feeds a base
PDB beside the column, which feeds the four wheel motors (one daisy chain round the base, the
wheels' CAN bus beside it), the column's driver, and a riser up the column. The riser stops at
the waist motor on the column head and goes on across the waist to the torso PDB, which feeds the
arms and head exactly as on Max (the same harnesses). The riser's slack over the column's stroke
is a coiled cable or drag chain as long as the stroke.

Usage:
  python tools/build_harness.py           # write electrical/harness.yaml and print a summary
  python tools/build_harness.py --check   # exit 1 if it is out of date (CI)
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_urdf as bu  # noqa: E402

ROOT = bu.ROOT
OUT = ROOT / "electrical" / "harness.yaml"
BUS_V = 48.0
EFFICIENCY = 0.8
DIVERSITY = 0.5
ROUTE_FACTOR = 1.3
LOOP_RADIUS = 0.035  # m: harness loop radius around a joint
TERMINATION = 0.08  # m per segment
# conservative continuous ampacity of bundled silicone wire (A) by AWG
AMPACITY = [(22, 3), (20, 5), (18, 10), (16, 15), (14, 22), (12, 30), (10, 45), (8, 70), (6, 100)]
MIN_POWER_AWG = 16
SERVO_AWG = 20  # 12 V pair for a Feetech servo
FUSES = [5, 10, 15, 20, 25, 30, 40, 50, 60, 80, 100, 125, 150]
# where the power distribution boards sit (link, xyz m): in the torso electronics bay and the
# pelvis frame (skeleton.yaml's electronics boxes)
HUBS = {
    "torso_pdb": ("torso_link", [0.005, 0.001, 0.187]),
    "pelvis_pdb": ("pelvis", [-0.096, 0.0, 0.094]),
}
# each crossed joint's service loop is tied to a plate either side (left side and centre; the
# right side mirrors): (parent link's plate, child link's plate)
CLIPS = {
    "left_hip_roll": ("pelvis_back_clip_l", "left_hip_roll_web"),
    "left_hip_pitch": ("left_hip_roll_web", "left_hip_pitch_outer_clip"),
    "left_hip_yaw": ("left_hip_pitch_back", "left_thigh_floor"),
    "left_knee": ("left_thigh_plate", "left_shin_inner"),
    "waist_yaw": ("pelvis_top", "torso_base"),
    "neck_yaw": ("torso_bridge_clip", "neck_bracket"),
    "left_shoulder_pitch": ("torso_shoulder_plate_l", "left_shoulder_flange"),
    "left_shoulder_roll": ("left_shoulder_flange", "left_shoulder_roll_front_clip"),
    "left_shoulder_yaw": ("left_shoulder_roll_front_clip", "left_upper_arm_strap"),
    "left_elbow": ("left_upper_arm_strap", "left_forearm_frame"),
    "left_wrist_yaw": ("left_forearm_frame", "left_wrist_cradle_back"),
}
# Lite: the base PDB (cad/lite_base.py elec_power) and the plates the riser is tied to either side
# of the column's stroke and the waist
LITE_PDB = ("lite_base", [-0.150, 0.0, 0.059])
LITE_CLIPS = {
    "column_lift": ("column_collar", "column_head_plate"),
    "waist_yaw": ("column_head_waist_cage", "torso_base"),
}
WHEEL_ORDER = ["wheel_rear_left", "wheel_front_left", "wheel_front_right", "wheel_rear_right"]
WHEEL_INBOARD = 0.0145 + 0.0085  # m: half the wheel's width plus the hub, to the motor's face
POD_LOOP = 0.05  # m: slack across each pod's pivot (about +-8 mm of travel at the wheel)
SPINDLE_EFFICIENCY = 0.3  # the column's motor and spindle, a first guess
LIMBS = {  # harness: hub, motors in kinematic order
    "{side}_leg": (
        "pelvis_pdb",
        ["hip_roll", "hip_pitch", "hip_yaw", "knee", "ankle_a", "ankle_b"],
    ),
    "{side}_arm": (
        "torso_pdb",
        ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_pitch"],
    ),
}


def awg(current: float) -> int:
    return next(g for g, a in AMPACITY if a >= current)


def fuse(current: float) -> int:
    return next(f for f in FUSES if f >= 1.25 * current)


def build() -> str:
    robot = yaml.safe_load((bu.DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((bu.DESC / "actuators.yaml").read_text())
    skel = yaml.safe_load((bu.DESC / "skeleton.yaml").read_text())
    joints = bu.mirrored(skel["joints"], key_is_name=True)
    ranges = {j["name"]: j["range"] for j in bu.expand(robot["joints"])}
    motors = {m["name"]: m for m in bu.motor_placements()}
    cad = yaml.safe_load((bu.DESC / "cad_model.yaml").read_text())

    def clip_parts(joint: str) -> list[str]:
        left = joint.replace("right_", "left_")
        pair = CLIPS.get(left)
        if pair is None:
            raise SystemExit(f"no clip plates for the service loop at {joint} (CLIPS)")
        out = []
        for name in pair:
            if joint.startswith("right_"):
                name = name.replace("left_", "right_").replace("_plate_l", "_plate_r")
                name = name.replace("_clip_l", "_clip_r")
            if name not in cad:
                raise SystemExit(f"clip plate {name} for {joint} is not in cad_model.yaml")
            out.append(name)
        return out

    def exit_xyz(m: dict) -> list[float]:
        """Where a motor's cable comes out: a RobStride's driver cap, at its rear end, or the side
        of a capless body `cable_back` mm in front of its rear face (on the axis here)."""
        shape = m.get("shape") or {}
        t = m["envelope"][2]
        if shape.get("cap_d"):
            back = t / 2
        elif shape.get("cable_back") is not None:
            back = t / 2 - shape["cable_back"] / 1000
        else:
            return list(m["xyz"])
        return [x - o * back for x, o in zip(m["xyz"], m["output"], strict=True)]

    bus_of = {m["name"]: m["bus"] for m in bu.expand(robot["motors"])}

    # link origins in the pelvis frame, default pose (all frames aligned)
    origin = {"pelvis": [0.0, 0.0, 0.0]}
    parent_joint = {}
    pending = dict(joints)
    while pending:
        for name, j in list(pending.items()):
            if j["parent"] in origin:
                origin[j["child"]] = [
                    a + b for a, b in zip(origin[j["parent"]], j["xyz"], strict=True)
                ]
                parent_joint[j["child"]] = name
                del pending[name]

    def at(link, xyz):
        return [a + b for a, b in zip(origin[link], xyz, strict=True)]

    def path(a: str, b: str) -> list[str]:
        """Joints between links a and b."""

        def up(link):
            chain = [link]
            while link in parent_joint:
                link = joints[parent_joint[link]]["parent"]
                chain.append(link)
            return chain

        ua, ub = up(a), up(b)
        common = next(x for x in ua if x in ub)
        return [parent_joint[x] for x in ua[: ua.index(common)] + ub[: ub.index(common)]]

    def current(name: str) -> float:
        a = acts[motors[name]["model"]]
        if a["interface"] != "can":
            return 0.0
        w = a["no_load_speed_rpm"] * 2 * math.pi / 60 / 2
        return a["rated_torque"] * w / EFFICIENCY / BUS_V

    def segment(frm, frm_link, frm_xyz, to, to_link, to_xyz, carries, downstream):
        p, q = at(frm_link, frm_xyz), at(to_link, to_xyz)
        crossed = path(frm_link, to_link)
        loops = sum(math.radians(ranges[j][1] - ranges[j][0]) * LOOP_RADIUS for j in crossed)
        amps = DIVERSITY * sum(current(m) for m in downstream)
        return dict(
            frm=frm,
            to=to,
            length_m=round(math.dist(p, q) * ROUTE_FACTOR + loops + TERMINATION, 2),
            crosses=crossed,
            service_loop_m=round(loops, 2),
            # a clip on each side of every loop: on a plate of the parent link and one of the
            # child link, so the joint only flexes the loop
            clips=[clip_parts(j) for j in crossed],
            current_a=round(amps, 1),
            power_awg=min(awg(amps), MIN_POWER_AWG) if amps else SERVO_AWG,
            signals=carries,
        )

    harnesses = {}
    for template, (hub, chain) in LIMBS.items():
        for side in bu.SIDES:
            names = [f"{side}_{m}" for m in chain]
            buses = []
            for n in names:
                if bus_of[n] not in buses:
                    buses.append(bus_of[n])
            segs = []
            frm, frm_link, frm_xyz = hub, *HUBS[hub]
            for i, n in enumerate(names):
                m = motors[n]
                rest = names[i:]
                carries = [b for b in buses if any(bus_of[r] == b for r in rest)]
                segs.append(
                    segment(frm, frm_link, frm_xyz, n, m["link"], exit_xyz(m), carries, rest)
                )
                frm, frm_link, frm_xyz = n, m["link"], exit_xyz(m)
            amps = segs[0]["current_a"]
            harnesses[template.format(side=side)] = dict(
                hub=hub,
                buses=buses,
                branch_current_a=amps,
                branch_fuse_a=fuse(amps),
                root_connectors=dict(
                    bulkhead=f"elec_bulkhead_{template.split('_')[1]}_{side[0]}",
                    power=f"2-pin, rated >= {fuse(amps)} A (XT60 class), keyed",
                    signal=f"{2 * len(buses) + 2}-pin latching (CAN pairs, ground, spare)",
                ),
                total_length_m=round(sum(s["length_m"] for s in segs), 2),
                segments=segs,
            )
    # trunk and centre loads
    waist = motors["waist_yaw"]
    legs = [h for k, h in harnesses.items() if k.endswith("_leg")]
    arms = [h for k, h in harnesses.items() if k.endswith("_arm")]
    waist_a = DIVERSITY * current("waist_yaw")
    trunk_a = sum(h["branch_current_a"] for h in legs) + waist_a
    main_a = trunk_a + sum(h["branch_current_a"] for h in arms)
    trunk = segment(
        "torso_pdb",
        *HUBS["torso_pdb"],
        "pelvis_pdb",
        *HUBS["pelvis_pdb"],
        ["left_hip", "left_shank", "right_hip", "right_shank", "right_arm (waist motor)"],
        [],
    )
    trunk.update(current_a=round(trunk_a, 1), power_awg=awg(trunk_a), fuse_a=fuse(trunk_a))
    waist_seg = segment(
        "pelvis_pdb",
        *HUBS["pelvis_pdb"],
        "waist_yaw",
        waist["link"],
        exit_xyz(waist),
        ["right_arm"],
        ["waist_yaw"],
    )
    neck = [motors["neck_yaw"], motors["neck_pitch"]]
    head = segment(
        "torso_pdb",
        *HUBS["torso_pdb"],
        "neck_pitch",
        neck[1]["link"],
        neck[1]["xyz"],
        ["servo (12 V + serial)", "camera USB-C"],
        [],
    )
    data = dict(
        bus_voltage_v=BUS_V,
        main=dict(current_a=round(main_a, 1), power_awg=awg(main_a), fuse_a=fuse(main_a)),
        trunk=trunk,
        waist_motor=waist_seg,
        head=head,
        limbs=harnesses,
    )
    data["lite"] = build_lite(robot, acts, skel, harnesses, current, exit_xyz)
    header = (
        "# Generated by tools/build_harness.py from description/*.yaml. Do not edit.\n"
        "# Lengths in m, currents in A (continuous, first-pass estimates), wire gauges in AWG.\n"
    )
    return header + yaml.safe_dump(data, sort_keys=False, width=100, default_flow_style=None)


def build_lite(robot, acts, skel, max_limbs, current, exit_xyz) -> dict:
    """Lite's harness: base PDB, wheels, column driver and the riser (see the module docstring);
    the arms and head are Max's."""
    joints = bu.variant_joints(robot, skel, "lite")
    lite = robot["variants"]["lite"]
    ranges = {j["name"]: j["range"] for j in bu.expand(robot["joints"])}
    ranges |= {j["name"]: j.get("range") for j in lite["joints"]}
    kinds = {j["name"]: j.get("type", "revolute") for j in lite["joints"]}
    motors = {m["name"]: m for m in bu.motor_placements("lite")}
    cad = yaml.safe_load((bu.DESC / "cad_model.yaml").read_text())
    cad |= yaml.safe_load((bu.DESC / "cad_model_lite.yaml").read_text())

    origin, parent_joint = {"lite_base": [0.0, 0.0, 0.0]}, {}
    pending = dict(joints)
    while pending:
        for name, j in list(pending.items()):
            if j["parent"] in origin:
                origin[j["child"]] = [
                    a + b for a, b in zip(origin[j["parent"]], j["xyz"], strict=True)
                ]
                parent_joint[j["child"]] = name
                del pending[name]

    def at(link, xyz):
        return [a + b for a, b in zip(origin[link], xyz, strict=True)]

    def clips(joint):
        out = list(LITE_CLIPS[joint])
        for n in out:
            if n not in cad:
                raise SystemExit(f"clip plate {n} for Lite's {joint} is not in the CAD models")
        return out

    def seg(frm, p, to, q, crossed, amps, carries, extra_loop=0.0, clip_list=None):
        loops = extra_loop
        for j in crossed:
            lo, hi = ranges[j]
            # a prismatic joint's slack is its stroke; a revolute's a loop round the joint
            loops += (
                (hi - lo) if kinds.get(j) == "prismatic" else math.radians(hi - lo) * LOOP_RADIUS
            )
        return dict(
            frm=frm,
            to=to,
            length_m=round(math.dist(p, q) * ROUTE_FACTOR + loops + TERMINATION, 2),
            crosses=crossed,
            service_loop_m=round(loops, 2),
            clips=clip_list if clip_list is not None else [clips(j) for j in crossed],
            current_a=round(amps, 1),
            power_awg=min(awg(amps), MIN_POWER_AWG) if amps else SERVO_AWG,
            signals=carries,
        )

    pdb = at(*LITE_PDB)
    # wheels: the motor's cable exit, inboard of the wheel along its axle (the wheel link's x)
    rs00 = acts["RS00"]
    w = rs00["no_load_speed_rpm"] * 2 * math.pi / 60 / 2
    wheel_a = rs00["rated_torque"] * w / EFFICIENCY / BUS_V
    exits = {}
    for n in WHEEL_ORDER:
        j = joints[n]
        yaw = j["rpy"][2]
        back = WHEEL_INBOARD + rs00["envelope_mm"][2] / 1000
        exits[n] = [
            j["xyz"][0] - back * math.cos(yaw),
            j["xyz"][1] - back * math.sin(yaw),
            j["xyz"][2],
        ]
    wheel_segs, frm, p = [], "base_pdb", pdb
    for i, n in enumerate(WHEEL_ORDER):
        a = DIVERSITY * wheel_a * (len(WHEEL_ORDER) - i)
        ang = round(math.degrees(joints[n]["rpy"][2])) % 360
        clip = [f"pod_clevis_{ang}", f"pod_inner_{ang}"]
        for c in clip:
            if c not in cad:
                raise SystemExit(f"clip plate {c} for {n} is not in cad_model_lite.yaml")
        wheel_segs.append(
            seg(frm, p, n, exits[n], [], a, ["wheels"], extra_loop=POD_LOOP, clip_list=[clip])
        )
        frm, p = n, exits[n]
    wheels_a = wheel_segs[0]["current_a"]

    # the column's driver, beside the base PDB; its motor at the column's foot
    dl6 = acts["DL6"]
    column_a = dl6["peak_force"] * dl6["speed"] / SPINDLE_EFFICIENCY / BUS_V
    col = seg("base_pdb", pdb, "column", [0.0, 0.0, 0.08], [], column_a, ["column (Hall)"])
    col["power_awg"] = min(
        awg(column_a * BUS_V / dl6["voltage"][0]), MIN_POWER_AWG
    )  # at 24 V after the converter

    # the riser: base PDB -> waist motor -> torso PDB
    arms = [h for k, h in max_limbs.items() if k.endswith("_arm")]
    arms_a = sum(h["branch_current_a"] for h in arms)
    waist = motors["waist_yaw"]
    waist_a = DIVERSITY * current("waist_yaw")
    exit_w = at(waist["link"], exit_xyz(waist))
    carries = ["left_arm", "right_arm", "servo (12 V + serial)", "camera USB-C"]
    riser = seg("base_pdb", pdb, "waist_yaw", exit_w, ["column_lift"], arms_a + waist_a, carries)
    riser["fuse_a"] = fuse(arms_a + waist_a)
    torso = at(*HUBS["torso_pdb"])
    upper = seg("waist_yaw", exit_w, "torso_pdb", torso, ["waist_yaw"], arms_a, list(carries))
    main_a = wheels_a + column_a + arms_a + waist_a
    return dict(
        main=dict(current_a=round(main_a, 1), power_awg=awg(main_a), fuse_a=fuse(main_a)),
        wheels=dict(
            hub="base_pdb",
            buses=["wheels"],
            branch_current_a=wheels_a,
            branch_fuse_a=fuse(wheels_a),
            total_length_m=round(sum(x["length_m"] for x in wheel_segs), 2),
            segments=wheel_segs,
        ),
        column_driver=col,
        riser=[riser, upper],
        arms_and_head="as Max's left_arm, right_arm and head, from the torso PDB",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    text = build()
    if args.check:
        if not OUT.exists() or OUT.read_text() != text:
            print("electrical/harness.yaml is out of date: run python tools/build_harness.py")
            return 1
        print("harness.yaml up to date")
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text)
    d = yaml.safe_load(text)
    print(
        f"main: {d['main']['current_a']} A, {d['main']['power_awg']} AWG,"
        f" fuse {d['main']['fuse_a']} A"
    )
    t = d["trunk"]
    print(f"trunk across the waist: {t['current_a']} A, {t['power_awg']} AWG, {t['length_m']} m")
    for name, h in d["limbs"].items():
        print(
            f"{name:10} {h['branch_current_a']:5.1f} A  fuse {h['branch_fuse_a']:3d} A  "
            f"{h['total_length_m']:.2f} m  buses {', '.join(h['buses'])}"
        )
        for s in h["segments"]:
            print(
                f"    {s['frm']:>20} -> {s['to']:22} {s['length_m']:.2f} m"
                f" (loops {s['service_loop_m']:.2f}) {s['current_a']:5.1f} A {s['power_awg']} AWG"
            )
    lt = d["lite"]
    print(
        f"lite main: {lt['main']['current_a']} A, fuse {lt['main']['fuse_a']} A; wheels "
        f"{lt['wheels']['branch_current_a']} A, {lt['wheels']['total_length_m']} m; riser "
        + " + ".join(f"{s['length_m']:.2f} m" for s in lt["riser"])
        + f" ({lt['riser'][0]['current_a']} A, {lt['riser'][0]['power_awg']} AWG)"
    )
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
