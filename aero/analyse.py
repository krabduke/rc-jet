"""Aerodynamic analysis of the RC jet, by vortex-lattice solve.

This exists to test an assumption. The centre of gravity was placed at 24.5 %
of the mean aerodynamic chord because that is the conventional place to put it
-- but conventional wisdom is not a calculation. The neutral point solved here
is what actually decides whether the aircraft is stable, and by how much.

Run with a numpy-capable interpreter:
    python3 aero/analyse.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "plane"))

import vlm    # noqa: E402
import spec   # noqa: E402

MM = 0.001
V = 22.0      # m/s, a realistic cruise for a 330 g jet


def surfaces():
    W, H = spec.WING, spec.HTAIL
    sweep = math.tan(math.radians(W["sweep_le"]))
    wing = vlm.Surface(
        "wing",
        le_root=(W["x_root_le"] * MM, 0.0, W["z_root"] * MM),
        chord_root=W["root_chord"] * MM,
        le_tip=((W["x_root_le"] + W["semi_span"] * sweep) * MM,
                W["semi_span"] * MM, W["z_root"] * MM),
        chord_tip=W["tip_chord"] * MM,
        n_span=24, n_chord=10,
        twist_root=W["incidence"], twist_tip=W["incidence"] - W["washout"])

    hsweep = math.tan(math.radians(H["sweep_le"]))
    hrise = math.tan(math.radians(H["anhedral"]))
    tail = vlm.Surface(
        "stabilator",
        le_root=(H["x_root_le"] * MM, 0.0, H["z_root"] * MM),
        chord_root=H["root_chord"] * MM,
        le_tip=((H["x_root_le"] + H["semi_span"] * hsweep) * MM,
                H["semi_span"] * MM,
                (H["z_root"] + H["semi_span"] * hrise) * MM),
        chord_tip=H["tip_chord"] * MM,
        n_span=12, n_chord=8,
        twist_root=H["deflect"], twist_tip=H["deflect"])
    return [wing, tail], wing


def main():
    surfs, wing = surfaces()
    s_ref = spec.wing_area_mm2() * MM * MM
    c_ref = spec.mean_aero_chord() * MM
    b_ref = spec.SPAN * MM
    x_le_mac = spec.mac_leading_edge_x() * MM
    x_cg = spec.cg_x() * MM

    print(f"\nRC JET — VORTEX-LATTICE ANALYSIS")
    print(f"  reference area {s_ref*1e4:.1f} cm²   MAC {c_ref*1000:.1f} mm   "
          f"span {b_ref*1000:.0f} mm   V {V:.0f} m/s")

    print("\nLIFT")
    slope, s0, s4 = vlm.lift_slope(surfs, V, s_ref, c_ref, b_ref)
    print(f"  dCL/dalpha           {slope:>7.3f} per rad "
          f"({math.radians(1)*slope:.4f} per deg)")
    ar = b_ref ** 2 / s_ref
    print(f"  aspect ratio         {ar:>7.2f}")
    print(f"  CL at 0 deg          {s0.CL:>7.4f}")
    print(f"  CL at 4 deg          {s4.CL:>7.4f}   CDi {s4.CDi:.5f}   "
          f"e {vlm.efficiency(s4):.3f}")

    # level-flight trim
    w_N = spec.total_mass_g() / 1000.0 * 9.81
    q = 0.5 * vlm.RHO * V * V
    cl_needed = w_N / (q * s_ref)
    a_trim = math.degrees((cl_needed - s0.CL) / slope)
    print(f"\nTRIM AT {V:.0f} m/s")
    print(f"  weight               {w_N:>7.2f} N ({spec.total_mass_g():.0f} g)")
    print(f"  CL required          {cl_needed:>7.4f}")
    print(f"  alpha required       {a_trim:>7.2f} deg")

    print("\nSTABILITY")
    np_frac, _, _ = vlm.neutral_point(surfs, V, s_ref, c_ref, b_ref, x_le_mac)
    cg_frac = spec.cg_frac_mac()
    sm = np_frac - cg_frac
    print(f"  neutral point        {np_frac*100:>7.1f} % MAC   "
          f"(x = {(x_le_mac + np_frac*c_ref)*1000:.1f} mm)")
    print(f"  centre of gravity    {cg_frac*100:>7.1f} % MAC   "
          f"(x = {x_cg*1000:.1f} mm)")
    print(f"  static margin        {sm*100:>7.1f} % MAC")

    verdict = ("stable" if sm > 0 else "UNSTABLE")
    band = "comfortable" if 0.05 <= sm <= 0.20 else (
        "twitchy" if 0 < sm < 0.05 else
        "very stable, sluggish" if sm > 0.20 else "divergent")
    print(f"  verdict              {verdict}, {band}")

    print("\nSPANWISE LOADING at 4 deg")
    ys = sorted(s4.strips)
    half = [y for y in ys if y >= 0]
    peak = max(abs(s4.strips[y]) for y in half) or 1.0
    for y in half[::max(1, len(half)//10)]:
        n = int(round(abs(s4.strips[y]) / peak * 34))
        print(f"    y {y*1000:>6.0f} mm  {'#' * n}")

    ok = sm > 0.02
    print("\n" + "=" * 62)
    if ok:
        print(f"PASS  aircraft is longitudinally stable "
              f"({sm*100:.1f} % static margin)")
    else:
        print(f"FAIL  static margin {sm*100:.1f} % -- move the CG forward")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
