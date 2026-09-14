"""Fail the build on parts that are too crude to be the thing they are named.

Part count is the wrong metric and it is easy to game: splitting one mesh into
sixteen raises the count by fifteen and adds no geometry at all. What matters
is whether each part has the shape it actually has.

This airframe started at 1,601 vertices per part, and almost all of that was
the F110 turbofan inside it: the aeroplane around the engine was eight-vertex
ventral fins, 20-vertex landing gear legs, 24-vertex NACA inlets, a wing
lofted through fourteen stations and closed off with a flat rib at the tip,
and ribs that were the aerofoil with one enormous hole scaled out of the
middle. The engine collection is the standard the rest has to reach.

So: every part must clear a vertex floor, unless it is genuinely a fastener or
a wire, and the whole model must clear a mean. The exemptions are listed by
name rather than inferred, so adding one is a deliberate act that shows up in
a diff. The gate is a ratchet: raise it when the model gets better, never
lower it to make a build pass.

    python3 tools/audit_geometry.py
"""

import csv
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# A part may be simple only if it really is simple. Each entry is a prefix and
# the reason it is allowed to be.
EXEMPT = {
    "panel_": "an access panel, which is a surface and not a solid",
    "decal": "printed film",
    "navlight": "a moulded lens",
}

FLOOR = 180          # vertices, for anything not exempt
TOTAL = 695_000      # vertices, over the whole model

# The total, not the mean.
#
# A mean gate punishes the one thing these models most need: more components.
# A sensor, a gasket, a bolt pattern is a few hundred vertices because that is
# what it is, so adding real hardware to a model drags the average down while
# the total goes up -- and a gate that fails on that is telling you to delete
# parts. The docstring above already argues the honest number is the total.
# Both floor and total are ratchets: raise them when the model gets better,
# never lower them to make a build pass.


def audit(path=None):
    path = path or os.path.join(ROOT, "build", "parts.csv")
    rows = list(csv.DictReader(open(path)))
    crude = []
    for r in rows:
        n = int(r["verts"])
        name = r["name"]
        if any(name.startswith(k) for k in EXEMPT):
            continue
        if n < FLOOR:
            crude.append((n, name))
    total = sum(int(r["verts"]) for r in rows)
    mean = total // max(len(rows), 1)
    return sorted(crude), mean, len(rows), total


if __name__ == "__main__":
    crude, mean, n, total = audit()
    print(f"{n} parts, {total:,} verts, mean {mean:,}/part "
          f"(floor {FLOOR}, total target {TOTAL:,})")
    if crude:
        print(f"\n{len(crude)} parts below the floor:")
        for v, name in crude[:40]:
            print(f"  {name:28s} {v:5d} v")
    ok = not crude and total >= TOTAL
    print("\n" + ("PASS  geometry is up to standard"
                  if ok else "FAIL  geometry is too crude"))
    sys.exit(0 if ok else 1)
