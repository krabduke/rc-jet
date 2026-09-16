"""Chin inlet and the duct that carries air back to the engine face."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import fuselage

I = spec.INTAKE
SEG = spec.RES["revolve"]


def build():
    out = {}
    out.update(_lip())
    out.update(_duct())
    out.update(_hardware())
    return out


def _hardware():
    """What holds the duct up, what joins it to the engine, and what stops it
    swallowing the runway.

    The induction system was two objects: a lip and a duct. A duct is a
    moulding, and a moulding has a bond line where its two halves meet, hoops
    where it is carried by the fuselage frames, and a flange at the end where
    it bolts to something. The inlet guard is the part that decides whether
    the engine survives its first taxi across gravel.
    """
    out = {}
    I = spec.INTAKE
    x0, x1 = I["x_throat"], I["x_duct_end"]

    # carrying hoops, at the stations the fuselage formers are near
    hoops = []
    for f in (0.10, 0.30, 0.50, 0.70, 0.88):
        x = x0 + (x1 - x0) * f
        w, h, zc = duct_section(x)
        outer = _oval(x, w + 1.8, h + 1.8, zc, SEG, _squash(f))
        inner = _oval(x, w + 0.1, h + 0.1, zc, SEG, _squash(f))
        back_o = _oval(x + 2.6, w + 1.8, h + 1.8, zc, SEG, _squash(f))
        back_i = _oval(x + 2.6, w + 0.1, h + 0.1, zc, SEG, _squash(f))
        hoops.append(_ring_prism(outer, inner, back_o, back_i))
    out["duct_frames"] = mesh.join(*hoops)

    # the bond line down each side, where the two mouldings meet
    seams = []
    for sgn in (-1.0, 1.0):
        path, scale = [], []
        for i in range(18):
            f = i / 17
            x = x0 + (x1 - x0) * f
            w, h, zc = duct_section(x)
            path.append((x, sgn * (w + 0.2), zc))
            scale.append((1.0, 1.0))
        # 1.5 mm proud, not 3.2: the saddle tanks sit against the duct's
        # flanks and a bond line is a bead, not a rail
        seams.append(shapes.swept_profile(
            path, [(-2.4, -0.7), (2.4, -0.7), (2.9, 0.0), (2.4, 0.7),
                   (-2.4, 0.7), (-2.9, 0.0)], scale, subdiv=2))
    out["duct_seam"] = mesh.join(*seams)

    # the coupling at the engine face: a flange, its bolts, and the rubber
    # bellows that takes the mismatch between a moulding and a machined case
    w, h, zc = duct_section(x1)
    r = I["duct_r_end"]
    parts = []
    fv, ff = mesh.revolve_ring(
        [(x1 - 10.0, r + 1.0), (x1 - 10.0, r + 5.5),
         (x1 - 3.0, r + 5.5), (x1 - 3.0, r + 1.0)],
        SEG // 2)
    parts.append(([(px, py, pz + zc) for (px, py, pz) in fv], ff))
    for k in range(10):
        a = 2 * math.pi * k / 10
        bv, bf = mesh.cylinder(0.0, 3.4, 1.1, 8)
        parts.append(([(pz + x1 - 12.0, px + (r + 3.4) * math.cos(a),
                        py + zc + (r + 3.4) * math.sin(a))
                       for (px, py, pz) in bv], bf))
    # the bellows itself: three convolutions between the flange and the case
    for k in range(3):
        xb = x1 - 10.0 + k * 3.2
        cv, cf = mesh.revolve_open(
            [(xb, r + 1.0), (xb + 1.6, r + 3.2), (xb + 3.2, r + 1.0)],
            SEG // 2, cap_start=False, cap_end=False)
        parts.append(([(px, py, pz + zc) for (px, py, pz) in cv], cf))
    out["duct_coupling"] = mesh.join(*parts)

    # the guard across the mouth: streamlined bars, not a mesh, because a
    # mesh at this scale blocks more air than the engine can spare
    bars = []
    xl, w0, h0 = I["x_lip"] + I["lip_radius"] * 1.6, I["lip_width"] / 2, I["lip_height"] / 2
    for k in range(5):
        u = -0.66 + 1.32 * k / 4
        y = u * (w0 - 2.0)
        z_hi = I["z_lip"] + (h0 - 1.5) * math.sqrt(max(0.0, 1.0 - u * u))
        z_lo = I["z_lip"] - (h0 - 1.5) * math.sqrt(max(0.0, 1.0 - u * u))
        bars.append(shapes.swept_profile(
            [(xl, y, z_lo), (xl + 3.0, y, (z_lo + z_hi) / 2), (xl, y, z_hi)],
            [(-0.5, -1.7), (0.5, -1.7), (0.8, 0.0), (0.5, 1.7),
             (-0.5, 1.7), (-0.8, 0.0)], subdiv=3))
    out["intake_guard"] = mesh.join(*bars)
    return out


def _squash(f):
    """The section exponent at fraction f along the duct, matching _duct."""
    s = f * f * (3 - 2 * f)
    return 2.4 + (2.0 - 2.4) * s


def _ring_prism(front_o, front_i, back_o, back_i):
    """Close two annular rings into a solid hoop."""
    n = len(front_o)
    verts = front_o + front_i + back_o + back_i
    a, b, c, d = 0, n, 2 * n, 3 * n
    faces = []
    for s in range(n):
        s2 = (s + 1) % n
        faces.append((a + s, a + s2, b + s2, b + s))
        faces.append((c + s, d + s, d + s2, c + s2))
        faces.append((a + s, c + s, c + s2, a + s2))
        faces.append((b + s, b + s2, d + s2, d + s))
    return verts, faces


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


def duct_bore(x):
    """(half width, half height, z centre, exponent) of the duct's AIR PATH.

    The inner wall, and the superellipse exponent the section is drawn with,
    so anything can ask exactly whether a point is in the airflow. The outer
    wall is duct_section(); the difference between them is the duct's own
    material.
    """
    x0, x1 = I["x_throat"], I["x_duct_end"]
    t = min(max((x - x0) / (x1 - x0), 0.0), 1.0)
    s = t * t * (3 - 2 * t)
    w0 = I["lip_width"] / 2 - 1.0
    h0 = I["lip_height"] / 2 - 1.0
    r1 = I["duct_r_end"]
    return (w0 + (r1 - w0) * s, h0 + (r1 - h0) * s,
            I["z_lip"] + (spec.ENGINE_Z - I["z_lip"]) * s,
            2.4 + (2.0 - 2.4) * s)


def in_duct(p, margin=0.0):
    """Is this point in the air the engine breathes?"""
    x, y, z = p
    if not (I["x_throat"] <= x <= I["x_duct_end"]):
        return False
    w, h, zc, n = duct_bore(x)
    w -= margin
    h -= margin
    if w <= 0 or h <= 0:
        return False
    return (abs(y / w) ** n + abs((z - zc) / h) ** n) <= 1.0


def duct_solid(pad=1.2, n_st=24):
    """The duct's outer wall as a closed solid, for cutting frames with.

    Every ply bulkhead and every former is a web filling the fuselage section,
    and the duct runs through five of them. A central lightening bore does not
    clear it: at the cockpit bulkhead the duct is down at z -13.6 and half the
    section tall, so the bore that would clear it is bigger than the frame.
    The frame has to be cut to the duct's shape, which is what a builder does
    -- forward of the S-duct's climb the frames are horseshoes open at the
    bottom, not rings.

    So the duct hands out its own shape and the frames subtract it. Before
    this the firewall closed 70 % of the intake and the aeroplane could not
    breathe.
    """
    x0, x1 = I["x_throat"] - 4.0, I["x_duct_end"] + 4.0
    rings = []
    for i in range(n_st):
        x = x0 + (x1 - x0) * i / (n_st - 1)
        w, h, zc = duct_section(x)
        _, _, _, sq = duct_bore(x)
        rings.append(_oval(x, w + pad, h + pad, zc, SEG, sq))
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(n_st - 1):
        a, b = i * SEG, (i + 1) * SEG
        for s_ in range(SEG):
            s2 = (s_ + 1) % SEG
            faces.append((a + s_, a + s2, b + s2, b + s_))
    faces.append(tuple(range(SEG - 1, -1, -1)))
    base = (n_st - 1) * SEG
    faces.append(tuple(range(base, base + SEG)))
    return verts, faces


def duct_top(x, gap=3.2):
    """The lowest z a box at station x can sit at and stay out of the duct.

    The gap is 3.2 mm, not 1: the duct is carried on hoops that stand 1.8 mm
    proud of its outer wall, and a tray resting on the wall rests on them.
    """
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
