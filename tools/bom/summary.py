"""Print BOM totals by group from bom/bom.xlsx, and the quantity of each model; check its rows.

Prices are estimates or reseller listings (see each line's price basis). Owned items carry their
current price, so the total is the full cost to replicate the robot; the summary also gives what
is owned, ordered and still to buy, from each line's status.

Usage:
  python tools/bom/summary.py
"""

import sys
from collections import defaultdict

from book import line_value, read_book, validate


def main() -> int:
    data = read_book()
    errors = validate(data)
    rows = [r for r in data["lines"] if isinstance(r.get("qty"), int)]
    by_group: dict[str, float] = defaultdict(float)
    by_status: dict[str, float] = defaultdict(float)
    for r in rows:
        cost = line_value(r)
        by_group[r["group"]] += cost
        by_status[r["status"] or "to buy"] += cost

    total = sum(by_group.values())
    print(f"{'model':8} {'qty':>4} {'spares':>6}")
    for r in sorted((r for r in rows if r.get("model")), key=lambda r: r["model"]):
        print(f"{r['model']:8} {r['qty']:4d} {r['spares']:6d}")
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
