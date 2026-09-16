"""Bubble canopy, windscreen frame and a hint of cockpit."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import fuselage

C = spec.CANOPY
SEG = 40


def build():
    out = {}
    out.update(_glass())
    out.update(_frame())
    return out


def _profile(t):
    """Canopy half-width and height as a fraction of length. Peaks just behind
    the windscreen and fairs into the spine, which is the bubble shape.

    It lives in spec because the fuselage skin has to cut its aperture to the
    same outline, and two copies of a curve is how a canopy ends up landing
    half on skin and half on nothing.
    """
    return spec.canopy_profile(t)


def _glass():
    n = 26
    x0, x1 = C["x_front"], C["x_rear"]
    rings = []
    for i in range(n):
        t = i / (n - 1)
        x = x0 + (x1 - x0) * t
        w, h = _profile(t)
        _, _, zc, _ = fuselage.station_at(x)
        base = C["z_base"] + zc * 0.15
        ring = []
        # half-dome: only the part standing above the fuselage spine
        for s in range(SEG + 1):
            a = math.pi * s / SEG
            ring.append((x, w * math.cos(a), base + h * math.sin(a)))
        rings.append(ring)

    # A moulding with a thickness, not a half-dome of single faces.
    #
    # As one layer the canopy had 130 of its 2,065 edges with a single face on
    # them -- along both sills and round the front and rear rims. It had no
    # inside, so nothing could tell whether the pilot was under it or through
    # it. A blown canopy on a model this size is about 1.2 mm.
    t = 1.2
    inner = []
    for i, r in enumerate(rings):
        tt = i / (n - 1)
        w, h = _profile(tt)
        x = r[0][0]
        zc = r[0][2] - h * math.sin(0.0)      # the sill z of this ring
        row = []
        for s_ in range(SEG + 1):
            a = math.pi * s_ / SEG
            row.append((x, max(w - t, 0.1) * math.cos(a),
                        zc + max(h - t, 0.1) * math.sin(a)))
        inner.append(row)

    per = SEG + 1
    verts = [v for r in rings for v in r] + [v for r in inner for v in r]
    off = n * per
    faces = []
    for i in range(n - 1):
        a, b = i * per, (i + 1) * per
        for s_ in range(SEG):
            faces.append((a + s_, a + s_ + 1, b + s_ + 1, b + s_))
            faces.append((off + a + s_, off + b + s_,
                          off + b + s_ + 1, off + a + s_ + 1))
        # the two sills, where the glass meets the fuselage
        for s_ in (0, SEG):
            if s_ == 0:
                faces.append((a, b, off + b, off + a))
            else:
                faces.append((a + s_, off + a + s_, off + b + s_, b + s_))
    # and the front and rear rims
    for i, flip in ((0, False), (n - 1, True)):
        a = i * per
        for s_ in range(SEG):
            q = (a + s_, a + s_ + 1, off + a + s_ + 1, off + a + s_)
            faces.append(q if flip else tuple(reversed(q)))
    return {"canopy_glass": (verts, faces)}


def _frame():
    """Windscreen bow and the canopy sill rails."""
    parts = []
    x = C["windscreen_x"]
    t = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
    w, h = _profile(t)
    _, _, zc, _ = fuselage.station_at(x)
    base = C["z_base"] + zc * 0.15
    bow = []
    for s in range(SEG + 1):
        a = math.pi * s / SEG
        bow.append((x, w * math.cos(a), base + h * math.sin(a)))
    parts.append(mesh.pipe(bow, C["frame"] * 0.7, 8, caps=True))

    for sgn in (-1.0, 1.0):
        rail = []
        for i in range(20):
            tt = i / 19
            xx = C["x_front"] + (C["x_rear"] - C["x_front"]) * tt
            ww, _ = _profile(tt)
            _, _, zz, _ = fuselage.station_at(xx)
            rail.append((xx, sgn * ww, C["z_base"] + zz * 0.15))
        parts.append(mesh.pipe(rail, C["frame"] * 0.55, 8, caps=True))

    return {"canopy_frame": mesh.join(*parts)}
