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
    out["cut:fuselage_skin"] = canopy_aperture()
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


def canopy_aperture():
    """The hole the cockpit is seen through.

    The skin was lofted closed from nose to tail and the canopy sat on top of
    it, so the aeroplane had no cockpit opening at all: the tub, the seat, the
    panel and the pilot were all buried inside solid material, with the top of
    a helmet coming through the spine like a periscope. Nobody noticed while
    the cockpit was three boxes, because three boxes look much the same
    whether you can see them or not.

    The hole is the tub's outline, not the canopy's. That is the part people
    get wrong: a bubble canopy is longer and wider than the cockpit it covers,
    and forward of the windscreen base it closes over solid nose deck -- you
    see the instrument panel through the glass, not through a hole. Cut to the
    canopy instead and you get a slot either side of the tub looking straight
    down at the flight pack, and an open trench under the windscreen.

    So: the tub's plan outline, from the sill line up past the crown, with the
    skin below the sill left alone. That is the coaming the cockpit is let
    into.
    """
    C = spec.CANOPY
    K = spec.COCKPIT
    n = 40
    prism = []
    for i in range(n):
        t = i / (n - 1)
        x = K["x_front"] + (K["x_rear"] - K["x_front"]) * t
        tub = K["half_width"] + (K["half_width_aft"] - K["half_width"]) * t
        ct = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
        w = max(min(spec.canopy_profile(ct)[0] - C["frame"] * 1.1,
                    tub - K["wall"] - 0.3), 0.25)
        _, _, zc, _ = station_at(x)
        prism.append((x, w, C["z_base"] + zc * 0.15))

    verts, faces = [], []
    for (px, w, z) in prism:
        verts.append((px, -w, z - 0.6))
        verts.append((px, w, z - 0.6))
    base = len(verts)
    for (px, w, z) in prism:
        verts.append((px, -w, z + 60.0))
        verts.append((px, w, z + 60.0))
    for i in range(n - 1):
        a, b = i * 2, (i + 1) * 2
        faces.append((a, a + 1, b + 1, b))                      # floor
        faces.append((base + a, base + b, base + b + 1, base + a + 1))
        faces.append((a, b, base + b, base + a))                # left wall
        faces.append((a + 1, base + a + 1, base + b + 1, b + 1))
    faces.append((0, base, base + 1, 1))                        # front and
    last = (n - 1) * 2                                          # back caps
    faces.append((last, last + 1, base + last + 1, base + last))
    return verts, faces


def _bulkheads():
    """Ply bulkheads filling the section at each frame station.

    These carry load -- the firewall takes the engine, bhd_spar takes the wing
    -- so they are lightened less than the formers are, four holes rather than
    six and a wider rim. But they were single-hole discs, which is the one
    thing a load-bearing ply frame never is: what is cut out of it is how it
    is tuned, and the holes are also how the wiring and the pushrods get fore
    and aft past it.
    """
    out = {}
    from parts import common as pc
    for (name, x, t) in spec.BULKHEADS:
        w, h, zc, n = station_at(x)
        ring_f = section_ring(x - t / 2, inset=spec.FUSELAGE_SKIN)
        ring_a = section_ring(x + t / 2, inset=spec.FUSELAGE_SKIN)
        # the firewall keeps more material: it takes the engine's thrust
        bore = 0.30 if "firewall" in name else 0.40
        hub = bore + (0.24 if "firewall" in name else 0.20)
        holes = 4 if "firewall" in name else 5
        parts = [pc.lightened_ring(ring_f, ring_a, zc, bore, hub, holes,
                                   web_frac=0.40)]
        # A rolled flange round the outer edge, which is what stops a 2.5 mm
        # ply frame folding the first time the skin loads it. It is a lip
        # with a wall: written as a single band of faces emitted twice it
        # was a zero-thickness surface, which is not a flange and is not
        # even a closed mesh.
        lip = 3.4
        a_out = ring_a
        b_out = [(px + lip, py, pz) for (px, py, pz) in a_out]
        b_in = pc.shrink_ring(b_out, zc, 0.962)
        a_in = [(px - lip, py, pz) for (px, py, pz) in b_in]
        verts, faces = [], []
        m = len(a_out)
        for r in (a_out, b_out, b_in, a_in):
            verts.extend(r)
        for k in range(4):
            r0, r1 = k * m, ((k + 1) % 4) * m
            for j in range(m):
                j2 = (j + 1) % m
                faces.append((r0 + j, r0 + j2, r1 + j2, r1 + j))
        parts.append((verts, faces))
        out[name] = mesh.join(*parts)
    return out


def surface_point(x, angle_deg, standoff=0.0):
    """A point on (or just off) the skin, at a clock angle round the section."""
    w, h, zc, n = station_at(x)
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    p = 2.0 / n
    y = (w + standoff) * math.copysign(abs(ca) ** p, ca)
    z = (h + standoff) * math.copysign(abs(sa) ** p, sa)
    return (x, y, zc + z)


def surface_patch(x0, x1, a0, a1, height, nx=8, na=8):
    """A panel that follows the skin instead of floating above it.

    A flat box laid on a curved fuselage only touches along one line; its
    corners either sink into the body or hang off it. Sampling the section
    over the panel's own angular range and lofting the result gives a panel
    that sits down on the surface everywhere.
    """
    inner, outer = [], []
    for i in range(nx):
        x = x0 + (x1 - x0) * i / (nx - 1)
        for j in range(na):
            a = a0 + (a1 - a0) * j / (na - 1)
            inner.append(surface_point(x, a, 0.0))
            outer.append(surface_point(x, a, height))
    verts = inner + outer
    off = len(inner)
    faces = []
    for i in range(nx - 1):
        for j in range(na - 1):
            k = i * na + j
            faces.append((k, k + 1, k + na + 1, k + na))                 # base
            faces.append((off + k, off + k + na, off + k + na + 1,
                          off + k + 1))                                  # top
    for i in range(nx - 1):                       # side walls along the angle
        for j in (0, na - 1):
            k = i * na + j
            if j == 0:
                faces.append((k, k + na, off + k + na, off + k))
            else:
                faces.append((k + na, k, off + k, off + k + na))
    for j in range(na - 1):                       # end walls across the angle
        for i in (0, nx - 1):
            k = i * na + j
            if i == 0:
                faces.append((k + 1, k, off + k, off + k + 1))
            else:
                faces.append((k, k + 1, off + k + 1, off + k))
    return verts, faces
