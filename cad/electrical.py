"""Electronics, power and battery layout: where each bought part sits, at its outline.

A first layout of the parts in the BOM (bom/bom.xlsx), as boxes at their outlines (typical sizes
for the class of part; replace with the chosen part's datasheet outline when it is bought), in
the torso and pelvis frames (mm). Placed so nothing overlaps the frame, the motors or each
other (`python -m cad.covers --check` and the rest-pose audit), with the wiring in mind:

- the battery slides out of the back of the torso between two side guides and a latch plate;
  it sits high, on a shelf under the shoulder motors; its anti-spark blind-mate connector
  (AS150 class) sits at its front-top corner and seats in a socket at the front of its bay;
- the torso bay above the computer holds the main power path in order (contactor and
  precharge, then the torso power board), the computer's DC-DC (fed before the contactor, so
  an e-stop keeps it up), the e-stop relay and the three USB-CAN adapters next to the
  computer's USB ports;
- the pelvis channel under the waist motor holds the 12 V DC-DC for the servos and the IMU;
  the leg power board with the leg fuses sits between the hip roll motors behind the hips.

The harness itself is a mass allowance in each bay. Masses are estimates for the class of
part until weighed.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from .lib import Part, box, union

ROOT = Path(__file__).resolve().parent.parent
_skel = yaml.safe_load((ROOT / "description" / "skeleton.yaml").read_text())

# name, link, size (x, y, z), centre, mass (g), what
TORSO = [
    (
        "elec_contactor",
        (50, 35, 45),
        (5, -24.5, 194.5),
        180,
        "main contactor, 48 V DC, opened by the e-stop chain",
    ),
    (
        "elec_precharge",
        (40, 35, 12),
        (0, -24.5, 225),
        30,
        "precharge resistor and relay, across the contactor",
    ),
    (
        "elec_power_board_torso",
        (50, 12, 30),
        (5, 1, 187),
        90,
        "torso power board: bus bars, arm branch fuses",
    ),
    (
        "elec_estop_relay",
        (14, 35, 40),
        (38, -24.5, 192),
        60,
        "wireless e-stop receiver and watchdog relay",
    ),
    ("elec_can_usb_1", (60, 22, 12), (15, 21, 178), 35, "isolated USB-CAN adapter"),
    ("elec_can_usb_2", (60, 22, 12), (15, 21, 192), 35, "isolated USB-CAN adapter"),
    ("elec_can_usb_3", (60, 22, 12), (15, 21, 206), 35, "isolated USB-CAN adapter"),
    (
        "elec_dcdc_compute",
        (60, 40, 13),
        (15, 22, 220),
        120,
        "DC-DC 48 V to 19-20 V, 120 W+, ahead of the contactor",
    ),
    (
        "elec_battery_socket",
        (31, 21, 11),
        (-16, 40, 243),
        15,
        "battery connector, cradle half (AS150 class), at the front of the battery bay",
    ),
    (
        "elec_harness_torso",
        (10, 10, 10),
        (40, 30, 230),
        270,
        "harness allowance: torso (limb-root connectors on the bulkheads)",
    ),
]
PELVIS = [
    ("elec_dcdc_servo", (60, 34, 25), (-30, 0, 63), 110, "DC-DC 48 V to 12 V, 10 A (servos)"),
    ("elec_imu", (40, 30, 20), (25, 0, 60), 60, "IMU on the pelvis, near the hip axes"),
    (
        "elec_power_board_pelvis",
        (50, 34, 80),
        (-94, 0, 10),
        150,
        "pelvis power board: leg branch fuses",
    ),
    (
        "elec_harness_pelvis",
        (10, 10, 10),
        (-94, 0, 60),
        220,
        "harness allowance: pelvis (limb-root connectors on the bulkheads)",
    ),
]


def parts() -> list[Part]:
    out: list[Part] = []
    for link, items in (("torso_link", TORSO), ("pelvis", PELVIS)):
        for name, size, centre, grams, what in items:
            out.append(
                Part(name, link, "bought", box(size, centre), "bought", mass_g=grams, notes=what)
            )

    # limb-root bulkheads: each limb's harness ends in one power and one signal connector on a
    # small printed bracket on the body it hangs from, so a limb unplugs as a unit (legs beside
    # the pelvis power board, below the hip roll motors; arms on the shoulder plates behind the
    # shoulder motors)
    for side, s in (("l", 1), ("r", -1)):
        out.append(
            Part(
                f"elec_bulkhead_leg_{side}",
                "pelvis",
                "bought",
                box((30, 12, 28), (-94, s * 36, -45)),
                "bought",
                mass_g=15,
                notes="leg power (XT60 class) and signal connectors",
            )
        )
        out.append(
            Part(
                f"elec_bulkhead_arm_{side}",
                "torso_link",
                "bought",
                box((30, 10, 28), (-62, s * 48, 100)),
                "bought",
                mass_g=15,
                notes="arm power and signal connectors",
            )
        )

    # battery cradle: side guides on the battery shelf, a latch plate at the back (the pack slides
    # out backwards once the latch is lifted)
    bat = next(b for b in _skel["bodies"] if b["budget"] == "battery")
    bx, by, bz = (v * 1000 for v in bat["xyz"])
    sx, sy, sz = (v * 1000 for v in bat["size"])
    base = bz - sz / 2
    guides = [box((sx, 3, 20), (bx, s * (sy / 2 + 2), base + 10)) for s in (1, -1)]
    out.append(Part("torso_battery_guides", "torso_link", "pa_cf", union(*guides), "print"))
    latch = box((4, sy - 10, 60), (bx - sx / 2 - 3, 0, base + 30))
    out.append(Part("torso_battery_latch", "torso_link", "pa_cf", latch, "print"))
    return out
