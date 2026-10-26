"""Print BOM totals by phase and by group from bom/bom.csv, and check the file's columns.

Prices are estimates or reseller listings (see the price_basis column); owned items count as 0.

Usage:
  python tools/bom_summary.py
"""

import csv
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOM = ROOT / "bom" / "bom.csv"
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

    total = sum(by_phase.values())
    print(f"{'phase':8} {'USD':>9}")
    for k in sorted(by_phase):
        print(f"{k:8} {by_phase[k]:9,.0f}")
    print(f"\n{'group':12} {'USD':>9}")
    for k, v in sorted(by_group.items(), key=lambda kv: -kv[1]):
        print(f"{k:12} {v:9,.0f}")
    print(f"\ntotal {total:,.0f} USD ({len(rows)} lines)")

    for e in errors:
        print(f"ERROR: {e}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
