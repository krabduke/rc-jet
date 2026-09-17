"""compressibility — subcritical compressibility screening (Prandtl-Glauert).

APPROXIMATE. This is a first-order subcritical screening correction only.
It is NOT a transonic, separated-flow or high-alpha delta validation, and it
says nothing about proving agility. Do not use it near or beyond Mach 1.
"""

import math


_M_MAX = 0.7


def pg_factor(mach):
    if not math.isfinite(mach) or not 0.0 <= mach <= _M_MAX:
        raise ValueError(
            "pg_factor defined only for finite 0 <= M <= %.1f, got %r"
            % (_M_MAX, mach))
    return 1.0 / math.sqrt(1.0 - mach * mach)


def main():
    print("Prandtl-Glauert screening factor 1/sqrt(1-M^2)")
    print("APPROXIMATE and subcritical only; inapplicable to proving agility.")
    print("Valid screening domain: 0 <= M <= 0.7; not a validated drag-rise curve.")
    print("Not a transonic, separated-flow or high-alpha delta validation.")
    print("%-8s %-12s" % ("M", "pg_factor"))
    for m in (0.0, 0.3, 0.5, 0.7):
        print("%-8.1f %-12.4f" % (m, pg_factor(m)))


if __name__ == "__main__":
    main()
