"""Fuselage: superellipse loft, shelled skin, bulkheads."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["fuse_sections"]
NST = spec.RES["fuse_stations"]
_INTAKE_MODULE = None
CHIN_ROLL = 25.0  # belongs in spec.py


def build():
    out = {}
    out.update(_skin())
    out.update(_bulkheads())
    from parts import common as pc
    pc.add_cut(out, "fuselage_skin", canopy_aperture())
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


# --------------------------------------------------------------------------

def chin_profile(x):
    """How far the lower surface is pushed down at station x, and how far out.

    The duct is slung under the forebody and the body has to close round it
    the way an F-16's belly does from the inlet back to the wing root. Before
    this the duct's outer wall hung in open air below the skin -- worst at
    station 86, where 73 % of the wall was outside the body -- so the inlet
    led to nothing and everything that reads the section carried the error.

    Returns (depth, width_gain), both zero outside the chin run. Depth is
    however far the keel must go for the base section to clear the duct's
    lowest point by chin_clear, clamped at zero where the base body already
    clears it; width_gain is the same for the duct's half-width. Both roll
    on and off with a cosine over the first and last CHIN_ROLL of the run so
    the belly has no crease, and the section itself weights them round to the
    keel so the chin fades out along the sides instead of ending at a rail.
    """
    global _INTAKE_MODULE
    if _INTAKE_MODULE is None:
        from parts import intake
        _INTAKE_MODULE = intake
    I = spec.INTAKE
    x0, x1 = I["chin_x0"], I["chin_x1"]
    if x < x0 or x > x1:
        return 0.0, 0.0
    if x - x0 < CHIN_ROLL:
        roll = 0.5 * (1.0 - math.cos(math.pi * (x - x0) / CHIN_ROLL))
    elif x1 - x < CHIN_ROLL:
        roll = 0.5 * (1.0 - math.cos(math.pi * (x1 - x) / CHIN_ROLL))
    else:
        roll = 1.0
    dw, dh, dzc = _INTAKE_MODULE.duct_section(x)
    duct_bottom = dzc - dh
    w, h, zc, _ = station_at(x)
    # Both of these had the wrong sign or the wrong units and the chin came
    # out zero everywhere, so the duct still hung in open air with the code
    # apparently in place. z is up, so the keel is ABOVE the duct's bottom and
    # the depth wanted is keel minus duct, not duct minus keel: at station 86
    # the keel is at -19.95 and the duct bottom at -33.60, so the body has to
    # come down 15.65 and the reversed form asked for -11.65 and clamped to 0.
    drop = (zc - h) - (dzc - dh) + I["chin_clear"]
    depth = max(drop, 0.0) * roll
    # and the widening is applied as `y *= 1 + gain`, so gain is a fraction of
    # the half-width, not a number of millimetres.
    gain = (dw - w) / w * roll if dw > w else 0.0
    return depth, gain


def _chin_weight(angle):
    """How much of the chin a ring point at `angle` picks up.

    Full at the keel and gone by chin_halfarc either side of it, on a cosine,
    so the fairing dies out along the belly instead of ending in a crease.
    """
    d = abs(math.degrees(angle) % 360.0 - 270.0)
    if d >= spec.INTAKE["chin_halfarc"]:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * d / spec.INTAKE["chin_halfarc"]))


def section_ring(x, inset=0.0, segments=SEG):
    """One superellipse section, unioned with the intake duct beneath it.

    The exponent is what gives a fighter its flat-sided fuselage: n = 2 is a
    plain ellipse, n ~ 3 reads as slab-sided with rounded corners.

    Under the forebody the section also has to contain the duct, and pushing
    the keel down by a cosine-weighted depth does not do it. The duct at
    station 86 is 40 mm across a 53 mm body but sits 26 mm lower, so its
    widest points are out beyond the body's own surface at that height, where
    a keel-weighted push has already faded to half strength. It got the wall
    from 73 % outside to 33 % outside and no further, because the shape being
    asked for is not a deeper section, it is a wider one low down.

    So the chin is a union, not an offset: along every ray from the section
    centre, the surface is whichever is further out, the body or the duct's
    outer wall plus its clearance. Containment then holds by construction at
    every angle rather than being approximated at one of them, and the roll
    and the keel weighting only decide how quickly the union fades back to
    the plain section fore and aft.
    """
    w, h, zc, n = station_at(x)
    w = max(w - inset, 0.05)
    h = max(h - inset, 0.05)
    p = 2.0 / n
    env = _chin_envelope(x, inset)
    ring = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        y = w * math.copysign(abs(ca) ** p, ca)
        z = h * math.copysign(abs(sa) ** p, sa)
        if env is not None:
            # No angular weighting. The union already only moves the surface
            # where the duct is outside it, which is the lower flanks and the
            # keel and nowhere else; weighting it by distance from the keel as
            # well throttled it to 29 % exactly where it was needed and left a
            # third of the duct wall outside the body. The only fade is
            # fore-and-aft, which is what `roll` is.
            r_body = math.hypot(y, z)
            r_duct = _ray_to_section(ca, sa, zc, env[:4])
            if r_duct > r_body > 0.0:
                f = 1.0 + env[4] * (r_duct / r_body - 1.0)
                y *= f
                z *= f
        ring.append((x, y, zc + z))
    return ring


def _chin_envelope(x, inset=0.0):
    """The duct's outer wall plus clearance at station x, or None.

    Returned as (half_width, half_height, z_centre, exponent, roll) so the ray
    test below can ask the same question of it that it asks of the body, and
    the caller knows how far into the chin run it is. `inset`
    comes off it as well, so the shell's inner surface follows the chin and
    the skin keeps its thickness along it instead of closing to nothing.
    """
    I = spec.INTAKE
    if x < I["chin_x0"] or x > I["chin_x1"]:
        return None
    global _INTAKE_MODULE
    if _INTAKE_MODULE is None:
        from parts import intake
        _INTAKE_MODULE = intake
    x0, x1 = I["chin_x0"], I["chin_x1"]
    if x - x0 < CHIN_ROLL:
        roll = 0.5 * (1.0 - math.cos(math.pi * (x - x0) / CHIN_ROLL))
    elif x1 - x < CHIN_ROLL:
        roll = 0.5 * (1.0 - math.cos(math.pi * (x1 - x) / CHIN_ROLL))
    else:
        roll = 1.0
    dw, dh, dzc = _INTAKE_MODULE.duct_section(x)
    c = I["chin_clear"]
    return (max(dw + c - inset, 0.05), max(dh + c - inset, 0.05), dzc, 2.4,
            roll)


def _ray_to_section(ca, sa, zc, env):
    """How far a ray from the body's section centre reaches the envelope.

    The subtlety that cost a wrong answer: the body's section centre is not
    inside the duct. At station 86 the body is centred at z +1.95 and the duct
    at z -24, so a ray fired from the body's centre starts OUTSIDE the duct
    envelope, crosses into it, and crosses out again. A plain bisection
    assuming "inside at t = 0, outside at t = big" therefore reported zero at
    every angle and the union never moved anything at all.

    What is wanted is the FAR crossing -- where the ray leaves the duct --
    because that is how far out the skin has to be to contain it. So: march
    out coarsely to find a sample inside, keep marching to find the sample
    after it that is outside, then bisect between those two.
    """
    ew, eh, ezc, en = env
    dz = zc - ezc
    reach = ew + eh + abs(dz) + 2.0

    def outside(t):
        yy = t * ca
        zz = dz + t * sa
        return (abs(yy / ew) ** en + abs(zz / eh) ** en) >= 1.0

    n = 48
    step = reach / n
    t_in = None
    for k in range(1, n + 1):
        if not outside(k * step):
            t_in = k * step
            break
    if t_in is None:
        return 0.0                       # the ray misses the duct entirely
    t_out = None
    k = int(t_in / step) + 1
    while k <= n:
        if outside(k * step):
            t_out = k * step
            break
        k += 1
    if t_out is None:
        return 0.0
    lo, hi = t_out - step, t_out
    for _ in range(26):
        mid = 0.5 * (lo + hi)
        if outside(mid):
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


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
        # Finer than the formers. The duct cuts the cockpit bulkhead down to
        # an arch over the intake -- which is what it has to be, the duct owns
        # the whole lower section there -- and at the formers' resolution the
        # arch that survived was 140 vertices.
        seg = SEG * 2
        ring_f = section_ring(x - t / 2, inset=spec.FUSELAGE_SKIN, segments=seg)
        ring_a = section_ring(x + t / 2, inset=spec.FUSELAGE_SKIN, segments=seg)
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
    # the duct runs through the frames it passes; see intake.duct_solid
    from parts import intake as _intake
    cutter = None
    for (name, x, t) in spec.BULKHEADS:
        if spec.INTAKE["x_throat"] - 4 <= x <= spec.INTAKE["x_duct_end"] + 4:
            if cutter is None:
                cutter = _intake.duct_solid()
            pc.add_cut(out, name, cutter)
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
