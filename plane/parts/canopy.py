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

    per = SEG + 1
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(n - 1):
        a, b = i * per, (i + 1) * per
        for s in range(SEG):
            faces.append((a + s, a + s + 1, b + s + 1, b + s))
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
