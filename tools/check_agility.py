"""The agility study has to describe an aeroplane that could exist.

    python3 tools/check_agility.py

Energy-manoeuvrability numbers are easy to produce and easy to produce
nonsensically, because nothing about a plausible turn rate says whether the
aeroplane could hold level flight at that speed or whether the load factor
quoted is one the wing can actually reach. These are the things that would
make the study a lie.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "aero"))
sys.path.insert(0, os.path.join(HERE, "plane"))

import agility

fails = []


def main():
    j = agility.Jet()

    # 1. a corner speed a fighter could use
    vc = j.corner_speed(0.0)
    if not 100.0 <= vc <= 250.0:
        fails.append("CORNER SPEED: %.0f m/s is outside 100-250. Either the "
                     "lift model or the structural limit is wrong." % vc)

    # 2. it has to be able to sustain a turn at all
    ns = j.n_sustained(vc, 0.0)
    if ns <= 1.0:
        fails.append("SUSTAINED TURN: at the corner speed the engine holds "
                     "only %.2f g, so the aeroplane cannot turn without "
                     "losing energy at any speed." % ns)

    # 3. it has to be able to fly level at cruise
    ps_cruise = j.ps(200.0, 0.0, 1.0)
    if ps_cruise <= 0.0:
        fails.append("LEVEL FLIGHT: specific excess power at 200 m/s and 1 g "
                     "is %.0f m/s. The aeroplane cannot hold height." % ps_cruise)

    # 4. the structural limit must be enforced, not merely mentioned
    worst = 0.0
    for i in range(80):
        v = 60.0 + (agility.MACH_LIMIT * agility.atmosphere(0.0)[1] - 60.0) * i / 79.0
        worst = max(worst, j.n_available(v, 0.0))
    if worst > agility.N_STRUCTURAL + 1e-6:
        fails.append("STRUCTURE: load factor reaches %.2f g against a stated "
                     "limit of %.1f and is not being clamped"
                     % (worst, agility.N_STRUCTURAL))

    # 5. the study must stay inside the method's range
    if agility.MACH_LIMIT > 0.72:
        fails.append("METHOD: the study runs to M %.2f. The panel chain is "
                     "incompressible and Prandtl-Glauert stops being "
                     "defensible above about M 0.7."
                     % agility.MACH_LIMIT)

    if fails:
        for f in fails:
            print("FAIL  " + f)
        sys.exit(1)

    nc = j.n_available(vc, 0.0)
    print("PASS  corner speed %.0f m/s (%.0f kt) at %.1f g, %.1f deg/s"
          % (vc, vc * 1.944, nc, j.turn_rate(vc, nc)))
    print("PASS  sustained %.2f g at the corner, %.1f deg/s"
          % (ns, j.turn_rate(vc, ns)))
    print("PASS  level flight at cruise, Ps %.0f m/s at 1 g" % ps_cruise)
    print("PASS  the %.0f g structural limit is enforced" % agility.N_STRUCTURAL)
    print("PASS  the study stops at M %.2f, inside the method's range"
          % agility.MACH_LIMIT)


if __name__ == "__main__":
    main()
