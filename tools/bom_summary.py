"""Print BOM totals by phase and by group from bom/bom.csv, and check it and bom/sourcing.csv.

Prices are estimates or reseller listings (see the price_basis column). Owned items carry their
current price, so the total is the full cost to replicate the robot; the summary also gives what
this build still has to buy.

Usage:
  python tools/bom_summary.py
"""

import csv
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOM = ROOT / "bom" / "bom.csv"
SOURCING = ROOT / "bom" / "sourcing.csv"
COLUMNS = [
    "id",
    "group",
    "item",
    "model",
    "qty",
    "unit_usd",
    "price_basis",
    "phase",
    "status",
    "spare",
    "source",
    "notes",
]
SOURCING_COLUMNS = [
    "id",
    "vendor",
    "kind",
    "url",
    "unit_usd",
    "tiers",
    "availability",
    "checked",
    "confirmed",
    "notes",
]
STATUSES = {"planned", "ordered", "received", "owned"}


def main() -> int:
    with BOM.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != COLUMNS:
            print(f"ERROR: columns are {reader.fieldnames}, expected {COLUMNS}")
            return 1
        rows = list(reader)

    errors = []
    ids = set()
    by_phase: dict[str, float] = defaultdict(float)
    by_group: dict[str, float] = defaultdict(float)
    owned = 0.0
    for r in rows:
        if r["id"] in ids:
            errors.append(f"duplicate id {r['id']}")
        ids.add(r["id"])
        if r["status"] not in STATUSES:
            errors.append(f"{r['id']}: unknown status {r['status']}")
        if r["spare"] not in ("yes", "no"):
            errors.append(f"{r['id']}: spare must be yes or no")
        try:
            cost = int(r["qty"]) * float(r["unit_usd"])
        except ValueError:
            errors.append(f"{r['id']}: qty or unit_usd is not a number")
            continue
        by_phase[r["phase"]] += cost
        by_group[r["group"]] += cost
        if r["status"] == "owned":
            owned += cost

    with SOURCING.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != SOURCING_COLUMNS:
            errors.append(f"sourcing.csv columns are {reader.fieldnames}")
        for r in reader:
            if r["id"] not in ids:
                errors.append(f"sourcing.csv: {r['id']} is not in bom.csv")
            if r["confirmed"] not in ("yes", "no"):
                errors.append(f"sourcing.csv: {r['id']}: confirmed must be yes or no")

    total = sum(by_phase.values())
    print(f"{'phase':8} {'USD':>9}")
    for k in sorted(by_phase):
        print(f"{k:8} {by_phase[k]:9,.0f}")
    print(f"\n{'group':12} {'USD':>9}")
    for k, v in sorted(by_group.items(), key=lambda kv: -kv[1]):
        print(f"{k:12} {v:9,.0f}")
    print(f"\ntotal {total:,.0f} USD ({len(rows)} lines), the full cost to replicate")
    print(f"owned {owned:,.0f} USD, still to buy {total - owned:,.0f} USD")

    for e in errors:
        print(f"ERROR: {e}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
