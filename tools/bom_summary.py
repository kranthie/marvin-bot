"""Print BOM totals by phase and by group from bom/bom.xlsx, and check its rows.

Prices are estimates or reseller listings (see each line's price basis). Owned items carry their
current price, so the total is the full cost to replicate the robot; the summary also gives what
is owned, ordered and still to buy, from each line's status.

Usage:
  python tools/bom_summary.py
"""

import sys
from collections import defaultdict

from bom_book import read_book, validate


def main() -> int:
    data = read_book()
    errors = validate(data)
    rows = [r for r in data["lines"] if isinstance(r.get("qty"), int)]
    by_phase: dict[str, float] = defaultdict(float)
    by_group: dict[str, float] = defaultdict(float)
    by_status: dict[str, float] = defaultdict(float)
    for r in rows:
        try:
            cost = r["qty"] * r["unit_usd"]
        except TypeError:
            continue
        by_phase[r["phase"]] += cost
        by_group[r["group"]] += cost
        by_status[r["status"]] += cost

    total = sum(by_phase.values())
    print(f"{'phase':8} {'USD':>9}")
    for k in sorted(by_phase):
        print(f"{k:8} {by_phase[k]:9,.0f}")
    print(f"\n{'group':12} {'USD':>9}")
    for k, v in sorted(by_group.items(), key=lambda kv: -kv[1]):
        print(f"{k:12} {v:9,.0f}")
    print(f"\ntotal {total:,.0f} USD ({len(data['lines'])} lines), the full cost to replicate")
    print(
        f"owned {by_status['owned']:,.0f} USD, ordered {by_status['ordered']:,.0f} USD, "
        f"still to buy {by_status['to buy']:,.0f} USD"
    )

    for e in errors:
        print(f"ERROR: {e}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
