"""Nothing may be in the air the engine breathes.

    python3 tools/audit_duct.py

The intersection audit cannot answer this. It works by ray-casting a part's
vertices against a closed container, and the duct is a tube -- open at the
lip, open at the engine face -- so it is skipped as a container and everything
inside it is invisible to the check. That is not a small blind spot: on a
nose-intake model the duct is the largest single volume in the fuselage, and
it is what decides where everything else can go. The main gear retract sat
inside it, driving a leg whose trunnion is twenty-three millimetres further
out; the nose gear strut passed straight through it.

There is no need to ray-cast anything. The duct's section is an analytic
superellipse -- intake.duct_bore(x) is the same function the duct itself is
lofted from -- so the test is one expression per vertex.
"""

import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))
sys.path.insert(0, os.path.join(HERE, "tools"))

import _intersect
from parts import intake

# The duct, and what is bonded to or through it on purpose.
ALLOWED = {
    "duct_inlet": "is the duct",
    "duct_frames": "the hoops the duct is carried on",
    "duct_seam": "the moulding seam down the duct",
    "duct_coupling": "the flange at the engine face",
    "intake_guard": "the FOD guard, which stands in the throat by design",
    "intake_lip": "the lip, which is the duct's mouth",
    "intake_lip_ring": "the lip, which is the duct's mouth",
    "bl_diverter": "splits the boundary layer off ahead of the throat",
    # The duct's whole purpose is to deliver to these: its aft end IS the
    # engine's inlet, and the two share the same plane.
    "engine_inlet": "the engine face, which the duct delivers to",
    "engine_casing_inlet": "the engine face, which the duct delivers to",
    # A seam is a line ON a surface. Where that surface bounds the duct -- the
    # skin at the chin inlet, the duct's own mouldings -- so is the seam.
    "seam_ring": "a moulding seam on the skin",
    "seam_lengthwise": "a moulding seam on the skin",
    "wing_seams": "a moulding seam on the wing",
    "fuselage_skin": "the chin inlet is a hole cut in it",
}

# A part may graze the wall by this much without being called an obstruction:
# the duct's inner wall is a lofted superellipse and the parts around it are
# placed from the same function, so a shared surface reads as a fraction of a
# millimetre of overlap either way.
MARGIN = 0.4


def main():
    cuts = {}
    parts = _intersect.load_parts(HERE, "plane/parts", cuts)
    bad = []
    for name, (verts, _f) in sorted(parts.items()):
        if any(name.startswith(k) for k in ALLOWED):
            continue
        # a vertex in material the build cuts away is not in the airflow: the
        # frames are bored for the duct, and before they were the duct had
        # five ply webs across it
        solids = cuts.get(name)
        n_in = sum(1 for p in verts if intake.in_duct(p, MARGIN)
                   and not (solids and _intersect.machined_away(p, solids)))
        if n_in:
            bad.append((n_in, len(verts), name))
    bad.sort(reverse=True)

    print(f"{len(parts)} parts checked against the duct's air path")
    if bad:
        print(f"\n{len(bad)} parts are in the airflow:")
        for n_in, n, name in bad[:20]:
            print(f"  {n_in:6d}/{n:<7d} {100*n_in/n:5.1f}%   {name}")
    print("\n" + "=" * 62)
    print("PASS  the duct carries air and nothing else" if not bad
          else "FAIL  parts are standing in the intake")
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
