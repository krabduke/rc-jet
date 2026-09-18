"""Fail the build if docs/FULL_SCALE.md quotes a figure spec.py disagrees with.

The design-point table was wrong for exactly this reason: it was typed once and
then spec.py moved under it. Every number below is computed from spec and then
required, verbatim, in the prose. If a figure changes, this fails and tells you
the string the doc should now contain.

    python3 tools/check_docs.py
"""

import io
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from plane import spec                                     # noqa: E402

DOC = os.path.join(ROOT, "docs", "FULL_SCALE.md")

TANKS = ("fuel_forward", "fuel_wing", "fuel_aft")
CREW = ("pilot", "oil_unusable")
RHO = 1.225                 # sea level, ISA
G = 9.80665
CLMAX = 1.2                 # stated in the doc as an assumption, not a result


def _kg(v):
    """11006 -> '11 006', the thin-space form the doc uses."""
    return f"{v:,.0f}".replace(",", " ").replace(" ", " ")


def claims():
    S = spec.wing_area_m2()
    span = spec.m_full(spec.SPAN)
    mac = spec.m_full(spec.mean_aero_chord())
    full = spec.total_mass_kg()
    fuel = sum(m for (n, _, m) in spec.MASS_FULL if n in TANKS)
    crew = sum(m for (n, _, m) in spec.MASS_FULL if n in CREW)
    half = full - fuel / 2.0
    thrust = spec.ENGINE_FULL["thrust_ab_n"]

    def corner(m, cl=CLMAX):
        return math.sqrt(4.0 * m * G / (0.5 * RHO * S * cl))

    def tw(m):
        return thrust / (m * G)

    out = [
        ("span",                f"**{span:.2f} m**"),
        ("wing area",           f"**{S:.1f} m²**"),
        ("aspect ratio",        f"**{span ** 2 / S:.2f}**"),
        ("MAC",                 f"**{mac:.2f} m**"),
        ("mass, full fuel",     f"**{_kg(full)} kg**"),
        ("mass, half fuel",     f"**{_kg(half)} kg**"),
        ("wing loading, full",  f"**{full / S:.0f} kg/m²**"),
        ("wing loading, half",  f"**{half / S:.0f} kg/m²**"),
        ("thrust/weight, full", f"**{tw(full):.2f}**"),
        ("thrust/weight, half", f"**{tw(half):.2f}**"),
        ("corner, full fuel",   f"**{corner(full):.0f} m/s**"),
        ("corner, half fuel",   f"**{corner(half):.0f} m/s**"),
        ("empty mass",          f"{_kg(spec.empty_mass_kg())} kg"),
        ("pilot and oil",       f"{_kg(crew)} kg"),
        ("internal fuel",       f"{_kg(fuel)} kg"),
        ("thrust",              f"{thrust / 1000.0:.0f} kN"),
        ("fan diameter",        f"{spec.ENGINE_FULL['fan_diameter_m']:.2f} m"),
        ("CLmax assumed",       f"CLmax {CLMAX:.1f}"),
    ]
    # The stated sensitivity either side of the assumed CLmax.
    for cl in (1.1, 1.3):
        out.append((f"corner at CLmax {cl:.1f}",
                    f"{corner(full, cl):.0f} m/s"))
    # Each fuel cell, in the order the doc lists them.
    for name, label in (("fuel_forward", "forward"),
                        ("fuel_wing", "wing"),
                        ("fuel_aft", "aft")):
        m = dict((n, v) for (n, _, v) in spec.MASS_FULL)[name]
        out.append((f"{label} cell", f"{label} {_kg(m)}"))
    return out


text = io.open(DOC, encoding="utf-8").read()

fails = 0
for label, want in claims():
    ok = want in text
    print(f"  {'ok  ' if ok else 'x   '}{label:22s} {want}")
    if not ok:
        fails += 1

print("\n" + ("PASS  every figure in FULL_SCALE.md matches spec.py"
              if not fails else
              f"FAIL  {fails} figures in FULL_SCALE.md disagree with spec.py"))
sys.exit(0 if not fails else 1)
