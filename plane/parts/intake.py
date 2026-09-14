"""Chin inlet and the duct that carries air back to the engine face."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import fuselage

I = spec.INTAKE
SEG = spec.RES["revolve"]


def build():
    out = {}
    out.update(_lip())
    out.update(_duct())
    return out


def _oval(x, w, h, zc, segments=SEG, squash=2.4):
    """Superellipse ring, matching the fuselage's own section language."""
    p = 2.0 / squash
    ring = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        ring.append((x, w * math.copysign(abs(ca) ** p, ca),
                     zc + h * math.copysign(abs(sa) ** p, sa)))
    return ring


def _lip():
    """Rolled inlet lip -- a closed torus-like rim, so the inlet reads as a
    real cowl rather than a hole cut in the skin."""
    x, w, h, zc = I["x_lip"], I["lip_width"] / 2, I["lip_height"] / 2, I["z_lip"]
    r = I["lip_radius"]
    inner = _oval(x + r, w - r * 0.4, h - r * 0.4, zc)
    outer = _oval(x, w + r, h + r, zc)
    back_i = _oval(x + r * 3.2, w - r * 0.4, h - r * 0.4, zc)
    back_o = _oval(x + r * 3.2, w + r, h + r, zc)
    verts = inner + outer + back_i + back_o
    n = SEG
    A, B, C, D = 0, n, 2 * n, 3 * n
    faces = []
    for s in range(n):
        s2 = (s + 1) % n
        faces.append((A + s, A + s2, B + s2, B + s))       # front rim
        faces.append((B + s, B + s2, D + s2, D + s))       # outer skin
        faces.append((D + s, D + s2, C + s2, C + s))       # back
        faces.append((C + s, C + s2, A + s2, A + s))       # inner throat
    return {"intake_lip": (verts, faces)}


def duct_section(x):
    """(half width, half height, z centre) of the duct's OUTER wall at x.

    Everything that shares the fuselage with the duct -- the tank saddles,
    the equipment trays, the avionics -- has to be placed from the duct,
    because on a nose-intake model the duct is what decides where there is
    room. Guessing at it is how the flight pack, the fuel filter and the
    access tray all ended up inside the airflow.
    """
    x0, x1 = I["x_throat"], I["x_duct_end"]
    t = min(max((x - x0) / (x1 - x0), 0.0), 1.0)
    s = t * t * (3 - 2 * t)
    w0 = I["lip_width"] / 2 - 1.0
    h0 = I["lip_height"] / 2 - 1.0
    r1 = I["duct_r_end"]
    return (w0 + (r1 - w0) * s + I["wall"],
            h0 + (r1 - h0) * s + I["wall"],
            I["z_lip"] + (spec.ENGINE_Z - I["z_lip"]) * s)


def duct_top(x, gap=1.0):
    """The lowest z a box at station x can sit at and stay out of the duct."""
    w, h, zc = duct_section(x)
    return zc + h + gap


def _duct():
    """S-duct from the chin inlet up to the engine centreline. The offset is
    real: the inlet sits below the cockpit and the engine sits on the datum, so
    the duct has to climb, which is exactly why fighters have S-ducts."""
    n_st = 22
    x0, x1 = I["x_throat"], I["x_duct_end"]
    w0, h0, z0 = I["lip_width"] / 2 - 1.0, I["lip_height"] / 2 - 1.0, I["z_lip"]
    r1 = I["duct_r_end"]

    outer_rings, inner_rings = [], []
    for i in range(n_st):
        t = i / (n_st - 1)
        # smoothstep the climb so the duct has no kink at either end
        s = t * t * (3 - 2 * t)
        w = w0 + (r1 - w0) * s
        h = h0 + (r1 - h0) * s
        zc = z0 + (spec.ENGINE_Z - z0) * s
        x = x0 + (x1 - x0) * t
        squash = 2.4 + (2.0 - 2.4) * s          # oval inlet -> round at the fan
        outer_rings.append(_oval(x, w + I["wall"], h + I["wall"], zc, SEG, squash))
        inner_rings.append(_oval(x, w, h, zc, SEG, squash))

    verts = [v for r in outer_rings for v in r] + \
            [v for r in inner_rings for v in r]
    off = n_st * SEG
    faces = []
    for i in range(n_st - 1):
        a, b = i * SEG, (i + 1) * SEG
        for s in range(SEG):
            s2 = (s + 1) % SEG
            faces.append((a + s, a + s2, b + s2, b + s))
            faces.append((off + a + s, off + b + s, off + b + s2, off + a + s2))
    last = (n_st - 1) * SEG
    for s in range(SEG):
        s2 = (s + 1) % SEG
        faces.append((s, off + s, off + s2, s2))
        faces.append((last + s, last + s2, off + last + s2, off + last + s))
    return {"duct_inlet": (verts, faces)}
