import sys, os
sys.path.insert(0, 'plane'); sys.path.insert(0, 'tools')
import _intersect
HERE = os.path.abspath('.')
cuts = {}
parts = _intersect.load_parts(HERE, "plane/parts", cuts)
mine = set()
sys.path.insert(0, 'plane')
from parts import systems
mine = set(systems.build().keys())
rows = []
for name, (v, f) in parts.items():
    if name in mine: continue
    lo = [min(p[i] for p in v) for i in range(3)]
    hi = [max(p[i] for p in v) for i in range(3)]
    # does it overlap the fuselage bay region I care about?
    if hi[0] < 100 or lo[0] > 310: continue
    if lo[1] > 40 or hi[1] < -40: continue
    if lo[2] > 34 or hi[2] < -30: continue
    rows.append((lo[0], name, lo, hi))
rows.sort()
print(f"{len(rows)} existing parts overlap x[100,310] y[-40,40] z[-30,34]")
print(f"{'part':32s} {'x':>15s} {'y':>15s} {'z':>15s}")
for _, name, lo, hi in rows:
    print(f"{name:32s} {lo[0]:6.1f}..{hi[0]:6.1f} {lo[1]:6.1f}..{hi[1]:6.1f} {lo[2]:6.1f}..{hi[2]:6.1f}")
