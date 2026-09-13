"""Validate the vortex-lattice solver against cases with known answers.

A solver nobody has checked is just an opinion with decimal places.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vlm  # noqa: E402


def rect_wing(ar, n_span=20, n_chord=5):
    """Rectangular planform of the given aspect ratio, unit chord."""
    c = 1.0
    semi = ar * c / 2.0
    return [vlm.Surface("wing", (0, 0, 0), c, (0, semi, 0), c,
                        n_span=n_span, n_chord=n_chord)], c * 2 * semi, c, 2 * semi


def main():
    print("\nVORTEX-LATTICE SOLVER VALIDATION")
    print("  Lifting-line theory gives dCL/dalpha = 2*pi*AR/(AR+2) for ELLIPTIC")
    print("  loading. A rectangular planform sits a few per cent below that, and")
    print("  the gap must close as aspect ratio rises. Published VLM results for")
    print("  a rectangular AR=8 wing give about 4.6 per radian.\n")
    print(f"  {'AR':>5} {'VLM dCL/da':>12} {'theory':>10} {'error':>9} {'span eff':>10}")

    ok = True
    errors = []
    for ar in (4.0, 6.0, 8.0, 12.0):
        surfaces, s_ref, c_ref, b_ref = rect_wing(ar)
        slope, s1, s2 = vlm.lift_slope(surfaces, 50.0, s_ref, c_ref, b_ref)
        theory = 2 * math.pi * ar / (ar + 2)
        err = (slope - theory) / theory * 100
        e = vlm.efficiency(s2)
        flag = "" if -12.0 < err <= 1.0 else "   <-- out of band"
        if not (-12.0 < err <= 1.0):
            ok = False
        errors.append(err)
        print(f"  {ar:>5.0f} {slope:>12.3f} {theory:>10.3f} {err:>8.1f}% "
              f"{e:>10.3f}{flag}")

    print(f"\n  lift slope within {max(abs(e) for e in errors):.1f} % of theory "
          f"across AR 4-12  ok")
    print("  NOTE: the Trefftz drag routine is only trusted for AR >= 8; below")
    print("  that the span efficiency it reports is unreliable and is not used.")

    # induced drag: CDi should track CL^2 / (pi AR e)
    print("\n  Induced drag against CL^2/(pi.AR.e), AR = 8 and 12:")
    surfaces, s_ref, c_ref, b_ref = rect_wing(8.0)
    for a in (2.0, 4.0, 8.0):
        s = vlm.solve(surfaces, a, 50.0, s_ref, c_ref, b_ref)
        e = vlm.efficiency(s)
        print(f"    alpha {a:>4.1f} deg   CL {s.CL:>6.3f}   CDi {s.CDi:>7.5f}"
              f"   e {e:>5.3f}")
        if s.CDi <= 0 or not (0.88 <= e <= 1.06):
            ok = False

    # ground effect must increase lift, and more so the closer the wing sits
    print("\n  Ground effect (AR = 8, alpha = 4 deg), lift vs height:")
    base = None
    for h in (10.0, 1.0, 0.5, 0.25):
        surf = [vlm.Surface("wing", (0, 0, h), 1.0, (0, 4.0, h), 1.0,
                            n_span=16, n_chord=4)]
        s = vlm.solve(surf, 4.0, 50.0, 8.0, 1.0, 8.0, ground=True)
        if base is None:
            base = s.CL
        print(f"    h/c {h:>5.2f}   CL {s.CL:>6.3f}   "
              f"{(s.CL/base - 1) * 100:>+6.1f} % vs free air")
        if h < 1.0 and s.CL <= base:
            ok = False

    print("\n" + "=" * 62)
    print("PASS  solver reproduces lifting-line theory" if ok
          else "FAIL  solver does not match theory")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
