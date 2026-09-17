"""How hard can this aeroplane be made to turn, and where does it run out.

    python3 aero/agility.py

Standard energy manoeuvrability -- the Boyd method -- not anything invented
here. The output that matters is the doghouse plot: turn rate against speed
with the lift limit, the structural limit and the thrust limit drawn on it.
One chart carries more about an aeroplane's fighting ability than any table of
single numbers, because every interesting question is about where two of those
limits cross.

The lift limit is not a stall. At aspect ratio 1.81 this wing works by holding
a leading-edge vortex, so it keeps making lift far past where an attached flow
gives up, and the right model is Polhamus's leading-edge-suction analogy:

    CL = Kp sin(a) cos^2(a) + Kv cos(a) sin^2(a)

the first term the potential-flow lift the wing would make with full leading-
edge suction, the second the lift recovered by the vortex when that suction is
lost. NASA TN D-3767. Kp is taken from slender-wing theory at this aspect
ratio and Kv from the same source's correlation; both are printed so a reader
can check them rather than take them on trust.

The drag polar's zero-lift term is an assumption and is labelled as one. This
repo's viscous build-up is a panel-method chain that has not been re-run at
full scale, so rather than quote it as though it had, CD0 is stated flat with
its basis and the sensitivity is shown.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec

G = 9.80665
RHO0 = 1.225
A0 = 340.3

# Assumed, and stated. A clean fighter of this size and wetted area sits
# between 0.018 and 0.024; 0.022 is the middle of that. Every headline number
# below is repeated at both ends of that range so the reader can see what it
# is worth.
CD0 = 0.022
CD0_BAND = (0.018, 0.024)

# Oswald efficiency for a low-aspect-ratio delta carrying a vortex. A slender
# delta is poor by this measure -- the vortex costs induced drag even as it
# buys lift -- so 0.70 rather than the 0.85 a straight wing would get.
OSWALD = 0.70

N_STRUCTURAL = 9.0          # g, the airframe limit
MACH_LIMIT = 0.70           # above this the incompressible panel chain, and
                            # the Prandtl-Glauert correction over it, stop
                            # being defensible


def atmosphere(h):
    """(density, speed of sound) in the troposphere."""
    t = 288.15 - 0.0065 * h
    rho = RHO0 * (t / 288.15) ** 4.2561
    return rho, 20.0468 * math.sqrt(t)


def thrust(h, afterburner=True):
    """Installed thrust at altitude. Turbofan thrust falls roughly with
    density to the 0.7, which is the usual first-order rule and is stated as
    such rather than dressed up as a deck."""
    rho, _ = atmosphere(h)
    t0 = (spec.ENGINE_FULL["thrust_ab_n"] if afterburner
          else spec.ENGINE_FULL["thrust_dry_n"])
    return t0 * (rho / RHO0) ** 0.7


class Jet:
    def __init__(self):
        self.m = spec.total_mass_kg()
        self.W = self.m * G
        self.S = spec.wing_area_m2()
        self.b = spec.SPAN * spec.SCALE_TO_FULL / 1000.0
        self.AR = self.b * self.b / self.S
        # slender-wing potential lift slope, per radian
        self.Kp = math.pi * self.AR / 2.0
        # Polhamus vortex term; for AR below about 2 it approaches pi
        self.Kv = math.pi * (1.0 - 0.18 * self.AR / 2.0)

    def cl(self, alpha):
        s, c = math.sin(alpha), math.cos(alpha)
        return self.Kp * s * c * c + self.Kv * c * s * s

    def cl_max(self):
        best = 0.0
        for i in range(1, 900):
            a = math.radians(i / 10.0)
            best = max(best, self.cl(a))
        return best

    def n_lift(self, v, h):
        rho, _ = atmosphere(h)
        q = 0.5 * rho * v * v
        return q * self.S * self.cl_max() / self.W

    def n_available(self, v, h):
        return min(self.n_lift(v, h), N_STRUCTURAL)

    def drag(self, v, h, n, cd0=CD0):
        rho, _ = atmosphere(h)
        q = 0.5 * rho * v * v
        cl = n * self.W / max(q * self.S, 1e-6)
        cdi = cl * cl / (math.pi * self.AR * OSWALD)
        return q * self.S * (cd0 + cdi)

    def n_sustained(self, v, h, cd0=CD0):
        """The load factor the engine can hold: thrust equals drag."""
        t = thrust(h)
        lo, hi = 1.0, self.n_available(v, h)
        if self.drag(v, h, lo, cd0) > t:
            return 0.0
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if self.drag(v, h, mid, cd0) > t:
                hi = mid
            else:
                lo = mid
        return lo

    def turn_rate(self, v, n):
        if n <= 1.0:
            return 0.0
        return math.degrees(G * math.sqrt(n * n - 1.0) / v)

    def ps(self, v, h, n, cd0=CD0):
        return v * (thrust(h) - self.drag(v, h, n, cd0)) / self.W

    def corner_speed(self, h=0.0):
        """Where the lift limit meets the structural limit -- the speed at
        which the aeroplane turns fastest, and the single most useful number
        in a turning fight."""
        lo, hi = 50.0, 400.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if self.n_lift(mid, h) < N_STRUCTURAL:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)


def doghouse(j, h=0.0, width=64, height=20):
    """Turn rate against speed, as a chart, with the three limits on it."""
    _, a = atmosphere(h)
    vmax = MACH_LIMIT * a
    vs = [60.0 + (vmax - 60.0) * i / (width - 1) for i in range(width)]
    inst = [j.turn_rate(v, j.n_available(v, h)) for v in vs]
    sus = [j.turn_rate(v, j.n_sustained(v, h)) for v in vs]
    top = max(max(inst), 1.0)
    grid = [[" "] * width for _ in range(height)]
    for i, v in enumerate(vs):
        for series, ch in ((inst, "#"), (sus, "o")):
            row = height - 1 - int(round((height - 1) * series[i] / top))
            row = min(max(row, 0), height - 1)
            if grid[row][i] == " ":
                grid[row][i] = ch
    out = []
    out.append("   deg/s")
    for r in range(height):
        val = top * (height - 1 - r) / (height - 1)
        out.append("  %5.1f |%s" % (val, "".join(grid[r])))
    out.append("        +" + "-" * width)
    lab = "        "
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        i = int(frac * (width - 1))
        lab += " " * max(0, i + 1 - (len(lab) - 8)) + "%.0f" % vs[i]
    out.append(lab + "  m/s")
    out.append("        # instantaneous (lift or structure limited)   "
               "o sustained (thrust limited)")
    return "\n".join(out)


def ps_table(j, h, loads=(1.0, 2.0, 3.0, 5.0, 7.0, 9.0)):
    _, a = atmosphere(h)
    vmax = MACH_LIMIT * a
    vs = [80.0 + (vmax - 80.0) * i / 7.0 for i in range(8)]
    lines = ["   n \\ V   " + "".join("%8.0f" % v for v in vs) + "   m/s"]
    lines.append("   " + "-" * (9 + 8 * len(vs)))
    for n in loads:
        row = "   %4.1f g  " % n
        for v in vs:
            if n > j.n_available(v, h) + 1e-6:
                row += "%8s" % "--"
            else:
                row += "%8.0f" % j.ps(v, h, n)
        lines.append(row)
    lines.append("   (m/s of specific excess power; -- means the aeroplane "
                 "cannot pull that g at that speed)")
    return "\n".join(lines)


def main():
    j = Jet()
    print("=" * 78)
    print("ENERGY MANOEUVRABILITY -- VX-J1")
    print("=" * 78)
    print()
    print("  mass %.0f kg, wing %.2f m2, span %.2f m, aspect ratio %.2f"
          % (j.m, j.S, j.b, j.AR))
    print("  wing loading %.0f kg/m2, thrust/weight %.2f on one %s"
          % (spec.wing_loading_kg_m2(), spec.thrust_to_weight(),
             spec.ENGINE_FULL["designation"]))
    print("  Polhamus Kp %.2f, Kv %.2f, CLmax %.2f at %.0f deg alpha"
          % (j.Kp, j.Kv, j.cl_max(),
             max(range(1, 900), key=lambda i: j.cl(math.radians(i / 10.0))) / 10.0))
    print("  CD0 %.3f assumed (band %.3f-%.3f), Oswald e %.2f"
          % (CD0, CD0_BAND[0], CD0_BAND[1], OSWALD))
    print()

    vc = j.corner_speed(0.0)
    nc = j.n_available(vc, 0.0)
    print("  CORNER SPEED %.0f m/s (%.0f kt, M %.2f) at %.1f g, turning %.1f deg/s"
          % (vc, vc * 1.944, vc / A0, nc, j.turn_rate(vc, nc)))
    print("  That is where the lift limit meets the %.0f g structural limit."
          % N_STRUCTURAL)
    print()

    print("  DOGHOUSE PLOT, sea level")
    print(doghouse(j, 0.0))
    print()

    for h in (0.0, 5000.0, 10000.0):
        print("  SPECIFIC EXCESS POWER at %.0f m" % h)
        print(ps_table(j, h))
        print()

    print("  SENSITIVITY -- best sustained turn rate at sea level")
    for cd0 in (CD0_BAND[0], CD0, CD0_BAND[1]):
        best, bv = 0.0, 0.0
        for i in range(60):
            v = 80.0 + (MACH_LIMIT * A0 - 80.0) * i / 59.0
            r = j.turn_rate(v, j.n_sustained(v, 0.0, cd0))
            if r > best:
                best, bv = r, v
        print("    CD0 %.3f -> %.1f deg/s at %.0f m/s" % (cd0, best, bv))
    print()
    print("  Capped at M %.2f. Above that the incompressible panel chain this"
          % MACH_LIMIT)
    print("  aeroplane's aerodynamics come from, and the Prandtl-Glauert")
    print("  correction over it, stop being defensible. No transonic method")
    print("  exists in this repo, so no transonic numbers are quoted.")


if __name__ == "__main__":
    main()
