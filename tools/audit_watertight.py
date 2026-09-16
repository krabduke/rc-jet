"""Every part must be a closed surface.

    python3 tools/audit_watertight.py

A part whose every edge is shared by exactly two faces bounds a volume. One
that has edges with a single face on them does not: it is a sheet, and it has
no inside. That matters three times over here.

It cannot be manufactured. It has no volume, so nothing can weigh it. And --
the one that hid for a long time -- the intersection audit works by asking
whether a point is inside a part, which means firing a ray at it and counting
crossings, which only means anything on a closed surface. So every open part
is skipped as a container, and whatever is inside it is invisible.

That was seventeen parts on this engine, including every compressor and
turbine blade row: the platform each blade stands on was written as a box per
angular step, which leaves a wall between every pair of steps and no caps on
the ends.
"""

import os
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import _intersect

PKG = "plane/parts"

# Parts allowed to be open, each with the reason. Keep it empty if you can.
ALLOWED = {}


def main():
    parts = _intersect.load_parts(ROOT, PKG)
    bad = []
    for name, (verts, faces) in sorted(parts.items()):
        if name in ALLOWED:
            continue
        edges = Counter()
        for f in faces:
            for i in range(len(f)):
                a, b = f[i], f[(i + 1) % len(f)]
                edges[(min(a, b), max(a, b))] += 1
        n = sum(1 for c in edges.values() if c != 2)
        if n:
            bad.append((n, len(edges), name))
    bad.sort(reverse=True)
    print(f"{len(parts)} parts checked for closed surfaces")
    if bad:
        print(f"\n{len(bad)} are open:")
        for n, tot, name in bad[:25]:
            print(f"  {n:7d} of {tot:8d} edges have one face   {name}")
    print("\n" + "=" * 62)
    print("PASS  every part is a closed surface" if not bad
          else "FAIL  parts are sheets, not solids")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
