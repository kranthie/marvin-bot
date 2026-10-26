"""Design and check the two-motor ankle linkage.

Each ankle has two motors on the calf, axes along y, each turning a crank of radius r. The
motors are staggered: both on the shin's centre line, each at its own (x, z): a with its crank
on the outer side (+y), b with its crank on the inner side. A push rod of
fixed length runs from each crank tip to a ball joint on the foot, behind the ankle axis
(rod_back behind it, rod_side to the side of it, rod_below below it). Moving both cranks the
same way pitches the foot; moving them opposite ways rolls it.

For a candidate geometry the script solves the linkage exactly over the whole joint range
(ankle pitch and roll from description/robot.yaml), and reports:
- whether every pose is reachable, and the crank angle range it needs;
- the worst transmission (sine of the angle between crank and rod; 1 is ideal, near 0 the
  linkage locks up), which must stay above MIN_TRANSMISSION;
- the joint torque the two motors can produce (peak and rated) at the worst pose;
- the largest rod force for a given joint torque demand, to size the rod ends.

The pitch and roll ratios at the neutral pose are what description/skeleton.yaml's
ankle_linkage section must hold (tools/build_urdf.py uses them for the URDF's ankle limits).

Usage:
  python tools/ankle_linkage.py            # report the geometry in skeleton.yaml
  python tools/ankle_linkage.py --search   # grid-search crank radius and rod placement
  python tools/ankle_linkage.py --check    # exit 1 if skeleton.yaml's ratios are stale (CI)
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
DESC = ROOT / "description"
MIN_TRANSMISSION = 0.5  # sin(30°)
DEMAND = (60.0, 30.0)  # joint torque demand for rod sizing: pitch, roll (N·m)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


class Linkage:
    """Left ankle, in the shin frame: origin on the ankle axis, x forward, y left, z up."""

    def __init__(self, g: dict):
        self.r = g["crank_radius"]
        self.motor = [
            np.array([g[k][0], s * g["crank_y"], g[k][1]])
            for k, s in (("motor_a", 1), ("motor_b", -1))
        ]
        self.attach = [
            np.array([-g["rod_back"], s * g["rod_side"], -g["rod_below"]]) for s in (1, -1)
        ]
        self.theta0 = math.pi + g.get("crank_tilt", 0.0)  # crank points back at neutral
        self.length = [np.linalg.norm(self.tip(i, self.theta0) - self.attach[i]) for i in (0, 1)]

    def tip(self, i: int, theta: float) -> np.ndarray:
        return self.motor[i] + self.r * np.array([math.cos(theta), 0.0, math.sin(theta)])

    def foot_point(self, i: int, pitch: float, roll: float) -> np.ndarray:
        # robot.yaml: + pitch = dorsiflexion (toes up) = rotation about -y; + roll = about +x
        return rot_y(-pitch) @ rot_x(roll) @ self.attach[i]

    def crank(self, i: int, pitch: float, roll: float, guess: float) -> float | None:
        """Crank angle putting rod i at its length, on the branch nearest `guess`."""
        a = self.foot_point(i, pitch, roll)
        d = a - self.motor[i]
        # |m + r(cos t, 0, sin t) - a|^2 = L^2  ->  P cos t + Q sin t = K
        p, q = -2 * self.r * d[0], -2 * self.r * d[2]
        k = self.length[i] ** 2 - d @ d - self.r**2
        amp = math.hypot(p, q)
        if amp < 1e-12 or abs(k) > amp:
            return None
        base, spread = math.atan2(q, p), math.acos(k / amp)
        sols = [base + spread, base - spread]
        return min(sols, key=lambda t: abs(math.remainder(t - guess, 2 * math.pi)))

    def transmission(self, i: int, theta: float, pitch: float, roll: float) -> float:
        tangent = np.array([-math.sin(theta), 0.0, math.cos(theta)])
        rod = self.foot_point(i, pitch, roll) - self.tip(i, theta)
        return abs(tangent @ rod) / np.linalg.norm(rod)

    def jacobian(self, pitch: float, roll: float, guess: tuple, h: float = 1e-5):
        """d(crank angles)/d(pitch, roll), 2 x 2."""
        j = np.zeros((2, 2))
        for k, (dp, dr) in enumerate(((h, 0.0), (0.0, h))):
            for i in (0, 1):
                hi = self.crank(i, pitch + dp, roll + dr, guess[i])
                lo = self.crank(i, pitch - dp, roll - dr, guess[i])
                j[i, k] = math.remainder(hi - lo, 2 * math.pi) / (2 * h)
        return j


def evaluate(g: dict, pitch_range, roll_range, peak: float, rated: float, steps: int = 17):
    lk = Linkage(g)
    worst_tx, cap_peak, cap_rated, rod_force = 1.0, [math.inf] * 2, [math.inf] * 2, 0.0
    lo_t, hi_t = math.inf, -math.inf
    for pitch in np.linspace(*pitch_range, steps):
        for roll in np.linspace(*roll_range, steps):
            guess = (lk.theta0, lk.theta0)
            th = [lk.crank(i, pitch, roll, guess[i]) for i in (0, 1)]
            if None in th:
                return None
            for i in (0, 1):
                worst_tx = min(worst_tx, lk.transmission(i, th[i], pitch, roll))
                dt = math.remainder(th[i] - lk.theta0, 2 * math.pi)
                lo_t, hi_t = min(lo_t, dt), max(hi_t, dt)
            jac = lk.jacobian(pitch, roll, tuple(th))
            # joint torque = J^T motor torque; best case per axis with |motor torque| <= limit
            for k in (0, 1):
                col = np.abs(jac[:, k]).sum()
                cap_peak[k] = min(cap_peak[k], peak * col)
                cap_rated[k] = min(cap_rated[k], rated * col)
            try:
                motor = np.linalg.solve(jac.T, np.array(DEMAND))
            except np.linalg.LinAlgError:
                return None
            for i in (0, 1):
                tx = lk.transmission(i, th[i], pitch, roll)
                rod_force = max(rod_force, abs(motor[i]) / (lk.r * max(tx, 1e-6)))
    jac0 = lk.jacobian(0.0, 0.0, (lk.theta0, lk.theta0))
    ratios = (np.abs(jac0[:, 0]).sum() / 2, np.abs(jac0[:, 1]).sum() / 2)
    return {
        "transmission": worst_tx,
        "crank_range_deg": (math.degrees(lo_t), math.degrees(hi_t)),
        "peak": cap_peak,
        "rated": cap_rated,
        "rod_force": rod_force,
        "ratios": ratios,
        "rod_length": lk.length,
    }


def load():
    robot = yaml.safe_load((DESC / "robot.yaml").read_text())
    acts = yaml.safe_load((DESC / "actuators.yaml").read_text())
    skel = yaml.safe_load((DESC / "skeleton.yaml").read_text())
    ranges = {j["name"]: j["range"] for j in robot["joints"]}
    pitch = [math.radians(x) for x in ranges["{side}_ankle_pitch"]]
    roll = [math.radians(x) for x in ranges["{side}_ankle_roll"]]
    model = next(m["model"] for m in robot["motors"] if m["name"] == "{side}_ankle_a")
    return skel, pitch, roll, acts[model]["peak_torque"], acts[model]["rated_torque"], model


def report(g, res, model):
    print(
        f"crank r {g['crank_radius'] * 1000:.0f} mm, rods {g['rod_back'] * 1000:.0f} mm behind, "
        f"±{g['rod_side'] * 1000:.0f} mm aside, {g['rod_below'] * 1000:.0f} mm below the "
        f"ankle axis; motors at x, z = {[round(v * 1000) for v in g['motor_a']]} and "
        f"{[round(v * 1000) for v in g['motor_b']]} mm"
    )
    if res is None:
        print("  NOT reachable over the whole range")
        return
    lo, hi = res["crank_range_deg"]
    rods = " and ".join(f"{x * 1000:.0f}" for x in res["rod_length"])
    print(f"  rod lengths {rods} mm, crank swing {lo:+.0f}°..{hi:+.0f}°")
    print(f"  worst transmission {res['transmission']:.2f} (min {MIN_TRANSMISSION})")
    print(f"  ratios at neutral: pitch {res['ratios'][0]:.2f}, roll {res['ratios'][1]:.2f}")
    print(
        f"  worst-pose torque, two {model}: pitch {res['peak'][0]:.0f} N·m peak / "
        f"{res['rated'][0]:.0f} rated, roll {res['peak'][1]:.0f} peak / {res['rated'][1]:.0f} rated"
    )
    print(
        f"  largest rod force for {DEMAND[0]:.0f}/{DEMAND[1]:.0f} N·m demand: "
        f"{res['rod_force']:.0f} N"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--search", action="store_true")
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    skel, pitch, roll, peak, rated, model = load()
    g = skel["ankle_linkage"]["geometry"]

    if args.search:
        best = None
        # Shin frame: the knee axis is ankle_z above the ankle axis. The high motor must clear
        # the knee RS04 (radius 60 mm) by 6 mm, the two RS03 (radius 50 mm) must clear each
        # other by 6 mm, and the low motor must stay 30 mm above the ankle cross's sweep.
        knee_z = -skel["joints"]["left_ankle_pitch"]["xyz"][2]
        radius = 0.050
        # b is the high motor (below the knee), a the low one (above the ankle)
        for bx in np.arange(-0.060, 0.0501, 0.010):
            for bz in np.arange(0.160, 0.2101, 0.010):
                if math.hypot(bx, knee_z - bz) < 0.060 + radius + 0.006:
                    continue
                for ax in np.arange(-0.060, 0.0301, 0.010):
                    for az in np.arange(0.080, 0.1301, 0.010):
                        if math.hypot(ax - bx, az - bz) < 2 * radius + 0.006:
                            continue
                        for r in np.arange(0.045, 0.0651, 0.005):
                            for back in np.arange(0.040, 0.0801, 0.010):
                                for side in (0.040, 0.045, 0.050, 0.055):
                                    for tilt in np.radians([-20, 0, 20]):
                                        cand = {
                                            **g,
                                            "motor_a": [ax, az],
                                            "motor_b": [bx, bz],
                                            "crank_radius": r,
                                            "rod_back": back,
                                            "rod_side": side,
                                            "crank_tilt": tilt,
                                        }
                                        res = evaluate(cand, pitch, roll, peak, rated, steps=7)
                                        if res is None or res["transmission"] < MIN_TRANSMISSION:
                                            continue
                                        score = min(res["rated"][0] / 30.0, res["rated"][1] / 14.0)
                                        if best is None or score > best[0]:
                                            best = (score, cand)
        if best is None:
            print("no geometry meets the transmission limit")
            return 1
        cand = best[1]
        report(cand, evaluate(cand, pitch, roll, peak, rated), model)
        print("geometry:", {k: np.round(v, 4).tolist() for k, v in cand.items()})
        return 0

    res = evaluate(g, pitch, roll, peak, rated)
    report(g, res, model)
    if res is None or res["transmission"] < MIN_TRANSMISSION:
        print("FAIL: geometry does not work over the whole range")
        return 1
    stored = (skel["ankle_linkage"]["pitch_ratio"], skel["ankle_linkage"]["roll_ratio"])
    if any(abs(a - round(b, 2)) > 1e-9 for a, b in zip(stored, res["ratios"], strict=True)):
        print(
            f"skeleton.yaml ankle_linkage ratios {stored} differ from the geometry's "
            f"({res['ratios'][0]:.2f}, {res['ratios'][1]:.2f}): update them"
        )
        return 1 if args.check else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
