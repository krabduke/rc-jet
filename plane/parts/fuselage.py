"""Fuselage: superellipse loft, shelled skin, bulkheads."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["fuse_sections"]
NST = spec.RES["fuse_stations"]


def build():
    out = {}
    out.update(_skin())
    out.update(_bulkheads())
    return out


# --------------------------------------------------------------------------

def _catmull(p0, p1, p2, p3, t):
    t2, t3 = t * t, t * t * t
    return (0.5 * ((2 * p1) + (-p0 + p2) * t
                   + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                   + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))


def station_at(x):
    """Interpolate (half_width, half_height, z_centre, exponent) at station x.

    Catmull-Rom through the table, so the body is smooth between the defining
    stations rather than a stack of cones.
    """
    tbl = spec.FUSELAGE
    if x <= tbl[0][0]:
        return tbl[0][1:]
    if x >= tbl[-1][0]:
        return tbl[-1][1:]
    i = 0
    while i < len(tbl) - 2 and tbl[i + 1][0] < x:
        i += 1
    x0, x1 = tbl[i][0], tbl[i + 1][0]
    t = (x - x0) / (x1 - x0)
    i0 = max(i - 1, 0)
    i3 = min(i + 2, len(tbl) - 1)
    out = []
    for k in range(1, 5):
        out.append(_catmull(tbl[i0][k], tbl[i][k], tbl[i + 1][k], tbl[i3][k], t))
    return tuple(out)


def section_ring(x, inset=0.0, segments=SEG):
    """One superellipse section. |y/w|^n + |z/h|^n = 1, centred at zc.

    The exponent is what gives a fighter its flat-sided fuselage: n = 2 is a
    plain ellipse, n ~ 3 reads as slab-sided with rounded corners.
    """
    w, h, zc, n = station_at(x)
    w = max(w - inset, 0.05)
    h = max(h - inset, 0.05)
    p = 2.0 / n
    ring = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        y = w * math.copysign(abs(ca) ** p, ca)
        z = h * math.copysign(abs(sa) ** p, sa)
        ring.append((x, y, zc + z))
    return ring


def _stations():
    x0, x1 = spec.FUSELAGE[0][0], spec.FUSELAGE[-1][0]
    # cosine spacing: more sections where the nose curvature is highest
    out = []
    for i in range(NST):
        f = i / (NST - 1)
        f = 0.5 * (1 - math.cos(math.pi * f))
        out.append(x0 + (x1 - x0) * f)
    return out


def _skin():
    """Outer and inner surfaces joined at both ends -- a real shell, so a
    cutaway shows the bays and the hardware inside them."""
    xs = _stations()
    outer, inner = [], []
    for x in xs:
        outer.extend(section_ring(x))
        inner.extend(section_ring(x, inset=spec.FUSELAGE_SKIN))

    verts = outer + inner
    off = len(outer)
    faces = []
    for i in range(len(xs) - 1):
        a, b = i * SEG, (i + 1) * SEG
        for s in range(SEG):
            s2 = (s + 1) % SEG
            faces.append((a + s, a + s2, b + s2, b + s))
            faces.append((off + a + s, off + b + s,
                          off + b + s2, off + a + s2))
    # close the nose and tail rims between the two skins
    last = (len(xs) - 1) * SEG
    for s in range(SEG):
        s2 = (s + 1) % SEG
        faces.append((s, off + s, off + s2, s2))
        faces.append((last + s, last + s2, off + last + s2, off + last + s))
    return {"fuselage_skin": (verts, faces)}


def _bulkheads():
    """Ply bulkheads filling the section at each frame station, with a
    lightening hole -- which is also how the wiring runs fore and aft."""
    out = {}
    for (name, x, t) in spec.BULKHEADS:
        w, h, zc, n = station_at(x)
        ring_f = section_ring(x - t / 2, inset=spec.FUSELAGE_SKIN)
        ring_a = section_ring(x + t / 2, inset=spec.FUSELAGE_SKIN)
        hole_r = min(w, h) * 0.42
        hole_f, hole_a = [], []
        for i in range(SEG):
            a = 2.0 * math.pi * i / SEG
            cy, cz = hole_r * math.cos(a), hole_r * math.sin(a)
            hole_f.append((x - t / 2, cy, zc + cz))
            hole_a.append((x + t / 2, cy, zc + cz))

        verts = ring_f + ring_a + hole_f + hole_a
        o_f, o_a, h_f, h_a = 0, SEG, 2 * SEG, 3 * SEG
        faces = []
        for s in range(SEG):
            s2 = (s + 1) % SEG
            # front and rear faces, as a ring between outer edge and hole
            faces.append((o_f + s, o_f + s2, h_f + s2, h_f + s))
            faces.append((o_a + s, h_a + s, h_a + s2, o_a + s2))
            # outer rim and hole wall
            faces.append((o_f + s, o_a + s, o_a + s2, o_f + s2))
            faces.append((h_f + s, h_f + s2, h_a + s2, h_a + s))
        out[name] = (verts, faces)
    return out
