"""Shapes that are not boxes.

Nothing on a fast vehicle is a rectangular prism. A casting has draft so it will
leave the mould, a radius on every edge because a sharp internal corner is a
crack waiting to happen, ribs where it needs stiffness, and bosses where
something bolts to it. An electronics case has fins because it has to get rid
of heat, and a connector because something plugs into it.

These are the primitives for that. They cost a few more vertices than
`mesh.box` and they are the difference between a model of an engine and a pile
of blocks the right size.
"""

import math

import mesh


def rounded_box(cx, cy, cz, sx, sy, sz, r=6.0, seg=4, draft=0.0):
    """A box with rounded vertical edges and optional draft.

    Draft is the taper a casting needs to come out of its mould -- a degree or
    two, always narrowing away from the parting line. It is a small thing and
    it is why a real casting never looks like a rendered cube.
    """
    r = min(r, sx / 2 - 0.1, sy / 2 - 0.1)
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    rings = []
    for k in range(2):
        f = k                                  # 0 at the bottom, 1 at the top
        t = math.tan(math.radians(draft)) * sz * f
        ax, ay = hx - t, hy - t
        rr = max(r - t, 0.5)
        ring = []
        for corner, (sgx, sgy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
            ox, oy = sgx * (ax - rr), sgy * (ay - rr)
            a0 = math.atan2(sgy, sgx) - math.pi / 4
            for i in range(seg + 1):
                a = a0 + (math.pi / 2) * i / seg
                ring.append((cx + ox + rr * math.cos(a),
                             cy + oy + rr * math.sin(a),
                             cz - hz + sz * f))
        rings.append(ring)
    return _loft_closed(rings)


def _loft_closed(rings):
    n = len(rings[0])
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n):
            j2 = (j + 1) % n
            faces.append((a + j, a + j2, b + j2, b + j))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = (len(rings) - 1) * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def finned_case(cx, cy, cz, sx, sy, sz, n_fins=9, fin_h=7.0, fin_t=3.0,
                r=5.0, axis="x", side=1.0):
    """A case with cooling fins across it.

    Power electronics and a battery pack both have to reject heat, and both do
    it with extruded fins. A smooth slab says the designer never thought about
    it.
    """
    parts = [rounded_box(cx, cy, cz, sx, sy, sz, r, draft=1.5)]
    span = sx if axis == "x" else sy
    for i in range(n_fins):
        f = (i + 0.5) / n_fins
        # `side` puts the fin stack on the face that actually sees air: a
        # battery slung under the engine rejects heat downwards, and fins
        # pointing up into the crankcase are just a hidden slab
        zf = cz + side * (sz / 2 + fin_h / 2)
        if axis == "x":
            parts.append(rounded_box(cx - sx / 2 + span * f, cy, zf,
                                     fin_t, sy * 0.92, fin_h, 1.2))
        else:
            parts.append(rounded_box(cx, cy - sy / 2 + span * f, zf,
                                     sx * 0.92, fin_t, fin_h, 1.2))
    return mesh.join(*parts)


def connector(cx, cy, cz, sx=26.0, sy=18.0, sz=14.0, pins=6):
    """A plug housing with pins in it. Everything electrical has one."""
    parts = [rounded_box(cx, cy, cz, sx, sy, sz, 2.5)]
    for i in range(pins):
        f = (i + 0.5) / pins
        v, fc = mesh.cylinder(0.0, sx * 0.45, 1.6, 6)
        v = [(px + cx + sx * 0.2, py + cy - sy * 0.32 + sy * 0.64 * f, pz + cz)
             for (px, py, pz) in v]
        parts.append((v, fc))
    return mesh.join(*parts)


def bolt_boss(cx, cy, cz, r=9.0, h=10.0, seg=10):
    """A raised pad with a bolt hole, where something fastens to a casting."""
    return mesh.revolve_open(
        [(0.0, r * 0.42), (0.0, r), (h * 0.6, r * 0.92), (h, r * 0.78),
         (h, r * 0.42)], seg, cap_start=True, cap_end=True)


def ribbed_cover(x0, x1, half_w, z_base, height, n_ribs=7, rib_h=5.0,
                 rib_w=7.0, crown=0.35, seg=14):
    """A cast cover: a crowned top, draft down the sides, and ribs across it.

    A cam cover is not a lid. It is a casting under a bolt flange, domed so it
    clears the valve gear, ribbed so it does not drum, with a bolt boss at
    every fastener.
    """
    parts = []
    n = 18
    rings = []
    for x in (x0, x0 + (x1 - x0) * 0.06, x1 - (x1 - x0) * 0.06, x1):
        t = 0.0 if x in (x0, x1) else 1.0
        hw = half_w * (0.90 + 0.10 * t)
        h = height * (0.72 + 0.28 * t)
        ring = []
        for i in range(n):
            a = 2 * math.pi * i / n
            ca, sa = math.cos(a), math.sin(a)
            p = 2.0 / 2.6
            y = hw * math.copysign(abs(ca) ** p, ca)
            z = h * math.copysign(abs(sa) ** p, sa)
            ring.append((x, y, z_base + h * crown + z))
        rings.append(ring)
    parts.append(_loft_closed(rings))
    for i in range(n_ribs):
        f = (i + 0.5) / n_ribs
        x = x0 + (x1 - x0) * f
        parts.append(rounded_box(x, 0.0, z_base + height * (crown + 0.92),
                                 rib_w, half_w * 1.55, rib_h, 1.5))
    return mesh.join(*parts)


def tapered_pan(x0, x1, hw0, hw1, z_top, depth, sump_w, sump_x, seg=4):
    """A sump: a wide rail at the block face falling into a narrow keel.

    The oil has to end up somewhere the pickup can reach it under braking, so
    a real dry-sump pan is a shallow tray with a deep local well, not a
    rectangular tank bolted to the bottom of the engine.
    """
    rings = []
    n_st = 9
    for i in range(n_st):
        f = i / (n_st - 1)
        x = x0 + (x1 - x0) * f
        hw = hw0 + (hw1 - hw0) * f
        # the well is deepest around sump_x
        d = depth * (0.42 + 0.58 * math.exp(-((x - sump_x) / (sump_w)) ** 2))
        ring = []
        for k in range(16):
            a = 2 * math.pi * k / 16
            ca, sa = math.cos(a), math.sin(a)
            p = 2.0 / 3.0
            y = hw * math.copysign(abs(ca) ** p, ca)
            z = (d / 2) * math.copysign(abs(sa) ** p, sa)
            ring.append((x, y, z_top - d / 2 + z))
        rings.append(ring)
    return _loft_closed(rings)


def fairing(path, chord, thickness=0.30, n_sec=14):
    """Sweep a streamlined section along a path.

    Anything exposed to the airstream on a fast vehicle is a teardrop, not a
    tube: a cylinder has roughly ten times the drag of a streamlined section
    of the same thickness, and it sheds a wake that ruins whatever is behind
    it.
    """
    import airfoil
    sect = airfoil.section_points(n_sec, thickness, 0.0)
    n = len(sect)
    verts = []
    for p in path:
        for (u, v) in sect:
            verts.append((p[0] + (u - 0.35) * chord, p[1], p[2] + v * chord))
    faces = []
    for j in range(len(path) - 1):
        a, b = j * n, (j + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((a + i, a + i2, b + i2, b + i))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = (len(path) - 1) * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def louvre_bank(x0, x1, y, z0, z1, n, blade_l, blade_h, t=2.0, cant=22.0):
    """A bank of angled blades over an exit duct."""
    parts = []
    for i in range(n):
        f = (i + 0.5) / n
        x = x0 + (x1 - x0) * f
        z = z0 + (z1 - z0) * f
        v, fc = rounded_box(0.0, 0.0, 0.0, blade_l, t, blade_h, t * 0.45)
        a = math.radians(cant)
        ca, sa = math.cos(a), math.sin(a)
        v = [(px * ca - pz * sa + x, py + y, px * sa + pz * ca + z)
             for (px, py, pz) in v]
        parts.append((v, fc))
    return mesh.join(*parts)


def core(cx, cy, cz, sx, sy, sz, n_tubes=18, n_fins=34):
    """A heat exchanger core: tubes with fin packs between them, in a frame.

    A radiator drawn as a solid block is the single laziest part on a car.
    """
    parts = [rounded_box(cx, cy, cz, sx, sy, sz, 4.0)]
    for i in range(n_tubes):
        f = (i + 0.5) / n_tubes
        parts.append(rounded_box(cx, cy, cz - sz / 2 + sz * f,
                                 sx * 1.01, sy * 0.96, sz / n_tubes * 0.42, 1.0))
    for i in range(n_fins):
        f = (i + 0.5) / n_fins
        parts.append(rounded_box(cx - sx / 2 + sx * f, cy, cz,
                                 sx / n_fins * 0.3, sy * 0.9, sz * 0.94, 0.6))
    return mesh.join(*parts)
