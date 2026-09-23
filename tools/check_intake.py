"""The inlet must actually lead to the engine.

    python3 tools/check_intake.py

Before this gate existed the mouth at station 52 spanned z -26.5 to -7.5 while
the fuselage's own underside there was at z -14.6. Seven units of the inlet
were buried in solid fuselage and twelve hung in open air with nothing round
them: the duct was a loose tube threaded through a closed body, so air
entering the mouth met structure rather than the fan.

Three things have to be true, and none of them is a matter of taste:

  1. ENCLOSURE.  From the throat back to the end of the chin, every point on
     the duct's outer wall lies inside the fuselage section. Tested against
     the section polygon `fuselage.section_ring` actually returns, not against
     an assumed superellipse, so it holds however the chin is implemented.

  2. APERTURE.  The skin is lofted closed nose to tail, so the mouth only
     exists if something cuts it. `intake.build()` must register a cutter for
     `fuselage_skin` spanning the mouth stations and covering the bore.

  3. DELIVERY.  The bore must run unbroken from the mouth to the engine face,
     arriving on the engine centreline at the fan's radius.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec
from parts import fuselage, intake

I = spec.INTAKE
fails = []


def _in_polygon(y, z, ring):
    """Is (y, z) inside the section polygon? Even-odd, on the y-z plane."""
    inside = False
    n = len(ring)
    for k in range(n):
        (_, ya, za), (_, yb, zb) = ring[k], ring[(k + 1) % n]
        if (za > z) != (zb > z):
            t = (z - za) / (zb - za)
            if y < ya + t * (yb - ya):
                inside = not inside
    return inside


def check_enclosure():
    """The body wraps the duct, all the way from the throat to the wing root."""
    worst = (0.0, None)
    for i in range(80):
        x = I["x_throat"] + (I["chin_x1"] - I["x_throat"]) * i / 79
        ring = fuselage.section_ring(x)
        w, h, zc = intake.duct_section(x)
        out = 0
        for k in range(48):
            a = 2 * math.pi * k / 48
            y = w * math.cos(a)
            z = zc + h * math.sin(a)
            if not _in_polygon(y, z, ring):
                out += 1
        if out:
            frac = out / 48
            if frac > worst[0]:
                worst = (frac, x)
    if worst[1] is not None:
        fails.append("ENCLOSURE: the duct's outer wall leaves the body -- "
                     "worst at drawing station %.0f, %.0f %% of the wall "
                     "outside it"
                     % (worst[1], worst[0] * 100))
    else:
        print("PASS  the body encloses the duct from the throat to station %.0f"
              % I["chin_x1"])


def check_aperture():
    """There is a hole in the skin where the mouth is."""
    built = intake.build()
    cut = built.get("cut:fuselage_skin")
    if cut is None:
        fails.append("APERTURE: intake.build() registers no cutter for "
                     "fuselage_skin, so the skin is still closed over the "
                     "mouth and no air can get in")
        return
    verts = cut[0]
    x0 = min(v[0] for v in verts)
    x1 = max(v[0] for v in verts)
    if x0 > I["mouth_x0"] + 0.5 or x1 < I["x_throat"] - 0.5:
        fails.append("APERTURE: the cutter spans x %.1f-%.1f but the mouth "
                     "needs x %.1f-%.1f" % (x0, x1, I["mouth_x0"],
                                            I["x_throat"]))
        return
    # and it has to be wide enough to clear the bore it is opening
    bw, bh, bz, _ = intake.duct_bore(I["x_throat"])
    near = [v for v in verts if abs(v[0] - I["x_throat"]) < 6.0]
    if not near:
        fails.append("APERTURE: the cutter has no section at the throat")
        return
    if max(abs(v[1]) for v in near) < bw - 0.2:
        fails.append("APERTURE: the cutter is narrower than the bore at the "
                     "throat (%.1f vs %.1f), so the mouth is pinched"
                     % (max(abs(v[1]) for v in near), bw))
        return
    print("PASS  the mouth is cut through the skin, x %.0f-%.0f" % (x0, x1))


def check_delivery():
    """The bore diffuses from the throat to the fan, and the lip can feed it.

    The first version of this check demanded that every station from the mouth
    back be at least 80 % of the fan face's area, and failed at station 256.
    That was the test being wrong, not the duct: a throat is *supposed* to be
    smaller than the fan face -- that is what makes it a throat -- and the
    diffuser's whole job is to get from one to the other. Asking an inlet not
    to have a throat is asking it not to be an inlet.

    What actually has to be true is that the bore never narrows once past the
    throat, that it arrives round and on the engine's centreline, and that the
    mouth is big enough to swallow what the engine eats.
    """
    w, h, zc, _ = intake.duct_bore(I["x_duct_end"])
    if abs(zc - spec.ENGINE_Z) > 0.3:
        fails.append("DELIVERY: the duct ends at z %.2f, the engine is at "
                     "z %.2f" % (zc, spec.ENGINE_Z))
    if abs(w - h) > 0.4:
        fails.append("DELIVERY: the duct is still %.1f x %.1f at the fan face "
                     "-- it has to be round there" % (w, h))

    # Areas and lengths are reported full size, like the engine's mass flow
    # they are compared with; stations stay in drawing units, which is what
    # the spec is written in. The throat used to be quoted in drawing mm^2
    # one line above a capture area in full-size m^2.
    S = spec.SCALE_TO_FULL

    # a subsonic diffuser never narrows
    prev, worst = None, None
    for i in range(120):
        x = I["x_throat"] + (I["x_duct_end"] - I["x_throat"]) * i / 119
        bw, bh, _, _ = intake.duct_bore(x)
        a = math.pi * bw * bh
        if prev is not None and a < prev - 1e-6:
            drop = prev - a
            if worst is None or drop > worst[1]:
                worst = (x, drop)
        prev = a
    if worst:
        fails.append("DELIVERY: the bore narrows by %.4f m^2 full size at "
                     "drawing station %.0f. A subsonic inlet duct diffuses "
                     "all the way to the fan; it never contracts."
                     % (worst[1] * S * S / 1e6, worst[0]))

    # and the mouth has to pass what the engine swallows
    lw, lh = I["lip_width"] / 2, I["lip_height"] / 2
    capture_m2 = math.pi * lw * lh * S * S / 1e6
    mdot = spec.ENGINE_FULL["mass_flow_kgs"]
    v_cruise = 200.0
    need = mdot / (1.225 * v_cruise)
    if capture_m2 < need:
        fails.append("DELIVERY: the mouth captures %.3f m^2 but the engine "
                     "swallows %.0f kg/s, which needs %.3f m^2 at %.0f m/s"
                     % (capture_m2, mdot, need, v_cruise))
    if not fails:
        thr = math.pi * intake.duct_bore(I["x_throat"])[0] * \
            intake.duct_bore(I["x_throat"])[1]
        fan = math.pi * I["duct_r_end"] ** 2
        print("PASS  the bore diffuses from the throat (%.3f m^2, %.0f %% of "
              "the fan face) to a %.2f m radius on the engine centreline"
              % (thr * S * S / 1e6, 100 * thr / fan, w * S / 1000.0))
        print("PASS  the mouth captures %.3f m^2 against the %.3f m^2 the "
              "engine needs at cruise" % (capture_m2, need))


def main():
    check_enclosure()
    check_aperture()
    check_delivery()
    if fails:
        for f in fails:
            print("FAIL  " + f)
        sys.exit(1)
    print("the inlet leads to the engine")


if __name__ == "__main__":
    main()
