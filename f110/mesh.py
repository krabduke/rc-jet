"""
Pure-Python mesh primitives. No bpy.

Everything returns (verts, faces). The engine axis is +X; a profile is a list
of (x, r) points in the meridional plane, revolved about that axis.

Closed profiles produce watertight solids, which is what casings, flanges,
discs and shafts want. Open profiles produce shells with optional end caps,
which is what the spinner and flowpath walls want.
"""

import math


# --------------------------------------------------------------------------
# Surfaces of revolution
# --------------------------------------------------------------------------

def _area2(loop):
    """Twice the signed area of a closed (x, r) loop: positive anticlockwise."""
    a2 = 0.0
    for i in range(len(loop)):
        x0, r0 = loop[i]
        x1, r1 = loop[(i + 1) % len(loop)]
        a2 += x0 * r1 - x1 * r0
    return a2


def _outward(loop, faces):
    """Wind every face so the normals point out of the solid.

    The faces below are written for an anticlockwise meridional loop. A loop
    written the other way round came out inside-out -- a solid whose normals
    all point in. Blender repairs that at build time, so nothing on screen
    showed it, but every audit that asks whether a point is inside a part
    counts crossings by which way the face turns, and an inside-out piece
    scores a point inside it as outside. Two in five closed pieces across
    the four models were built that way, which made the intersection audit
    blind inside them.
    """
    if _area2(loop) < 0:
        return [tuple(reversed(f)) for f in faces]
    return faces


def revolve_closed(profile, segments=96, phase=0.0, sweep=None):
    """Revolve a CLOSED meridional loop [(x, r), ...] into a watertight solid.

    The loop must not cross the axis. Winding is preserved, so order the loop
    counter-clockwise in (x, r) for outward normals.
    """
    full = sweep is None
    sweep = 2.0 * math.pi if full else sweep
    n = len(profile)
    rings = segments if full else segments + 1

    verts = []
    for k in range(rings):
        a = phase + sweep * k / segments
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in profile:
            verts.append((x, r * ca, r * sa))

    faces = []
    for k in range(segments if full else segments):
        k2 = (k + 1) % rings if full else k + 1
        b0, b1 = k * n, k2 * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))

    if not full:
        faces.append(tuple(range(n - 1, -1, -1)))
        base = (rings - 1) * n
        faces.append(tuple(range(base, base + n)))
    return verts, _outward(profile, faces)


def revolve_open(profile, segments=96, cap_start=False, cap_end=False, phase=0.0):
    """Revolve an OPEN meridional polyline into a shell. Caps close the ends
    with a fan to the axis (use where the profile meets r=0, e.g. a nose cone)."""
    n = len(profile)
    verts = []
    for k in range(segments):
        a = phase + 2.0 * math.pi * k / segments
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in profile:
            verts.append((x, r * ca, r * sa))

    faces = []
    for k in range(segments):
        k2 = (k + 1) % segments
        b0, b1 = k * n, k2 * n
        for i in range(n - 1):
            faces.append((b0 + i, b0 + i + 1, b1 + i + 1, b1 + i))

    if cap_start:
        c = len(verts)
        verts.append((profile[0][0], 0.0, 0.0))
        for k in range(segments):
            k2 = (k + 1) % segments
            faces.append((c, k * n, k2 * n))
    if cap_end:
        c = len(verts)
        verts.append((profile[-1][0], 0.0, 0.0))
        for k in range(segments):
            k2 = (k + 1) % segments
            faces.append((c, k2 * n + n - 1, k * n + n - 1))
    # closed along the axis, which is where the caps close it
    loop = list(profile) + [(profile[-1][0], 0.0), (profile[0][0], 0.0)]
    return verts, _outward(loop, faces)


def revolve_ring(profile, segments=96, phase=0.0):
    """Revolve a closed meridional loop that does NOT touch the axis.

    This is the shape almost every flange, boss, union and clamp on an engine
    actually is: a section with a hole down the middle. The obvious way to
    build one -- revolve_open on the loop with cap_start and cap_end -- is
    wrong, and quietly so. Those caps are fans to the AXIS, which is right for
    a profile that ends at r = 0 and closes a nose cone, and for one that ends
    at r = 21 it lays a flat disc straight across the bore. The part comes out
    a solid puck, and in a render it is a blank circle with no feature on it
    at all: the mouth of the tailpipe, the hole in every bolt boss, the well
    the spark plug drops into.

    The loop's winding is fixed here rather than at each call site, from the
    sign of its area in (x, r), so a profile written in whichever direction
    reads best still comes out with its normals pointing outward.
    """
    a2 = 0.0
    for i in range(len(profile)):
        x0, r0 = profile[i]
        x1, r1 = profile[(i + 1) % len(profile)]
        a2 += x0 * r1 - x1 * r0
    return revolve_closed(list(profile) if a2 > 0 else list(reversed(profile)),
                          segments, phase)


# --------------------------------------------------------------------------
# Convenience solids
# --------------------------------------------------------------------------

def tube(x0, x1, r_in, r_out, segments=96):
    """Annular tube -- the workhorse for casings, ducts and shafts."""
    return revolve_closed([(x0, r_in), (x1, r_in), (x1, r_out), (x0, r_out)], segments)


def cone_tube(x0, x1, r_in0, r_out0, r_in1, r_out1, segments=96):
    """Tube whose inner and outer radii vary linearly -- tapered casings."""
    return revolve_closed(
        [(x0, r_in0), (x1, r_in1), (x1, r_out1), (x0, r_out0)], segments)


def cylinder(x0, x1, r, segments=32):
    return revolve_open([(x0, 0.001), (x0, r), (x1, r), (x1, 0.001)],
                        segments, cap_start=True, cap_end=True)


def ring_torus(x, r_centre, r_tube, segments=64, tube_segments=14):
    """A torus lying in the YZ plane at station x -- seal rings, manifolds."""
    prof = []
    for i in range(tube_segments):
        a = 2.0 * math.pi * i / tube_segments
        prof.append((x + r_tube * math.cos(a), r_centre + r_tube * math.sin(a)))
    return revolve_closed(prof, segments)


def box(cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    v = [(cx - hx, cy - hy, cz - hz), (cx + hx, cy - hy, cz - hz),
         (cx + hx, cy + hy, cz - hz), (cx - hx, cy + hy, cz - hz),
         (cx - hx, cy - hy, cz + hz), (cx + hx, cy - hy, cz + hz),
         (cx + hx, cy + hy, cz + hz), (cx - hx, cy + hy, cz + hz)]
    f = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
         (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return v, f


# --------------------------------------------------------------------------
# Swept pipes -- fuel lines, oil lines, harnesses, tower shafts
# --------------------------------------------------------------------------

def _normalise(v):
    l = math.sqrt(sum(c * c for c in v))
    return tuple(c / l for c in v) if l > 1e-9 else (1.0, 0.0, 0.0)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def fillet_path(path, radii, bend):
    """Round every corner of a polyline with an arc of radius `bend`.

    Each corner is replaced by a circular arc tangent to both legs, as far
    along each leg as the bend needs but never more than half of it, so two
    corners never fight over one leg and the ends stay where they are. The
    radii list is carried along, blended across each arc from the radius
    where it leaves one leg to the radius where it joins the next -- giving
    the whole arc the corner's radius put a step at each end of it, and a
    step on the inside of a bend is a fold.
    """
    if len(path) < 3 or bend <= 0.0:
        return list(path), list(radii)
    out, rad = [tuple(path[0])], [radii[0]]
    for i in range(1, len(path) - 1):
        a, p, b = path[i - 1], path[i], path[i + 1]
        u = _normalise(tuple(p[k] - a[k] for k in range(3)))
        w = _normalise(tuple(b[k] - p[k] for k in range(3)))
        c = max(-1.0, min(1.0, sum(u[k] * w[k] for k in range(3))))
        turn = math.acos(c)
        if turn < 1e-3:
            out.append(tuple(p)); rad.append(radii[i])
            continue
        la = math.dist(a, p) * (0.5 if i > 1 else 0.9)
        lb = math.dist(p, b) * (0.5 if i < len(path) - 2 else 0.9)
        t = min(bend * math.tan(turn / 2.0), la, lb)
        axis = _normalise(_cross(u, w))
        start = tuple(p[k] - u[k] * t for k in range(3))
        r_eff = t / math.tan(turn / 2.0)
        # the arc's centre is r_eff from the start, on the inside of the turn
        inward = _cross(axis, u)
        centre = tuple(start[k] + inward[k] * r_eff for k in range(3))
        n = max(2, int(math.ceil(turn / (math.pi / 24.0))))
        fa, fb = t / math.dist(a, p), t / math.dist(p, b)
        r0 = radii[i] + (radii[i - 1] - radii[i]) * fa
        r1 = radii[i] + (radii[i + 1] - radii[i]) * fb
        for j in range(n + 1):
            ang = turn * j / n
            d = tuple(start[k] - centre[k] for k in range(3))
            ca, sa = math.cos(ang), math.sin(ang)
            kd = sum(axis[k] * d[k] for k in range(3))
            kx = _cross(axis, d)
            q = tuple(centre[k] + d[k] * ca + kx[k] * sa + axis[k] * kd * (1 - ca)
                      for k in range(3))
            out.append(q); rad.append(r0 + (r1 - r0) * j / n)
    out.append(tuple(path[-1])); rad.append(radii[-1])
    return out, rad


def pipe(path, radius, segments=16, caps=True, subdiv=1, bend=None):
    """Sweep a circular section along a 3D polyline using parallel transport,
    so the tube does not twist around corners.

    `radius` may be a number, or a list with one entry per path point, which
    is how a tapered primary or a bellmouthed runner gets made -- a real pipe
    on an engine is almost never a constant diameter.

    Every corner is filleted with an arc of radius `bend`, by default one and
    a half times the pipe's largest radius. A corner left as one vertex is a
    ring in the bisecting plane at the pipe's own radius -- a pinch -- and
    where the turn is tighter than the pipe is fat the rings either side
    cross it and the tube folds through itself. A pipe that is meant to have
    a sharp corner passes bend=0.

    `subdiv` inserts extra rings between the given points so a swept curve is
    smooth rather than faceted.
    """
    if len(path) >= 3:
        rl = (list(radius) if isinstance(radius, (list, tuple))
              else [radius] * len(path))
        b = 1.5 * max(rl) if bend is None else bend
        path, rl = fillet_path(path, rl, b)
        radius = rl if isinstance(radius, (list, tuple)) else radius
    if len(path) < 2:
        return [], []
    # drop repeated points: two fillets meeting mid-leg share one
    keep = [0] + [i for i in range(1, len(path))
                  if math.dist(path[i], path[i - 1]) > 1e-6]
    path = [path[i] for i in keep]
    if isinstance(radius, (list, tuple)):
        radius = [radius[i] for i in keep]
    if len(path) < 2:
        return [], []

    def _dir(a, b):
        return _normalise(tuple(b[k] - a[k] for k in range(3)))

    # a section at a vertex is square to the bisector of the legs either
    # side of it, and the sections subdivision adds between two vertices
    # swing steadily from one vertex's to the next. Taking every added
    # section square to its own chord instead stepped the tilt at each
    # vertex, and on a bend that is tight for the pipe's girth a step is
    # enough to put one ring's inside edge behind its neighbour's.
    tangents = []
    for i in range(len(path)):
        if i == 0:
            t = _dir(path[0], path[1])
        elif i == len(path) - 1:
            t = _dir(path[-2], path[-1])
        else:
            u, w = _dir(path[i - 1], path[i]), _dir(path[i], path[i + 1])
            t = _normalise(tuple(u[k] + w[k] for k in range(3)))
        tangents.append(t)

    if subdiv > 1:
        dense, rad, tan = [], [], []
        radii = radius if isinstance(radius, (list, tuple)) else None
        for i in range(len(path) - 1):
            for k in range(subdiv):
                t = k / subdiv
                dense.append(tuple(path[i][j] + (path[i + 1][j] - path[i][j]) * t
                                   for j in range(3)))
                tan.append(_normalise(tuple(
                    tangents[i][j] + (tangents[i + 1][j] - tangents[i][j]) * t
                    for j in range(3))))
                if radii:
                    rad.append(radii[i] + (radii[i + 1] - radii[i]) * t)
        dense.append(tuple(path[-1]))
        tan.append(tangents[-1])
        if radii:
            rad.append(radii[-1])
        path, tangents = dense, tan
        radius = rad if radii else radius

    seed = (0.0, 0.0, 1.0)
    if abs(sum(a * b for a, b in zip(seed, tangents[0]))) > 0.9:
        seed = (0.0, 1.0, 0.0)
    normal = _normalise(_cross(tangents[0], seed))

    radii = (radius if isinstance(radius, (list, tuple))
             else [radius] * len(path))
    verts = []
    for i, p in enumerate(path):
        t = tangents[i]
        # re-project the carried normal onto this section's plane
        d = sum(a * b for a, b in zip(normal, t))
        normal = _normalise(tuple(normal[k] - d * t[k] for k in range(3)))
        binormal = _cross(t, normal)
        rr = radii[min(i, len(radii) - 1)]
        for s in range(segments):
            a = 2.0 * math.pi * s / segments
            ca, sa = math.cos(a) * rr, math.sin(a) * rr
            verts.append(tuple(p[k] + normal[k] * ca + binormal[k] * sa
                               for k in range(3)))

    faces = []
    for i in range(len(path) - 1):
        b0, b1 = i * segments, (i + 1) * segments
        for s in range(segments):
            s2 = (s + 1) % segments
            faces.append((b0 + s, b0 + s2, b1 + s2, b1 + s))
    if caps:
        faces.append(tuple(range(segments - 1, -1, -1)))
        base = (len(path) - 1) * segments
        faces.append(tuple(range(base, base + segments)))
    return verts, faces


# --------------------------------------------------------------------------
# Transforms and combination
# --------------------------------------------------------------------------

def rot_x(verts, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    return [(x, y * ca - z * sa, y * sa + z * ca) for (x, y, z) in verts]


def rot_z(verts, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    return [(x * ca - y * sa, x * sa + y * ca, z) for (x, y, z) in verts]


def translate(verts, dx=0.0, dy=0.0, dz=0.0):
    return [(x + dx, y + dy, z + dz) for (x, y, z) in verts]


def scale(verts, s):
    return [(x * s, y * s, z * s) for (x, y, z) in verts]


def join(*parts):
    """Merge several (verts, faces) pairs into one, re-indexing as it goes."""
    verts, faces = [], []
    for (v, f) in parts:
        off = len(verts)
        verts.extend(v)
        faces.extend(tuple(i + off for i in face) for face in f)
    return verts, faces


def replicate(verts, faces, count, phase=0.0):
    """Rotationally replicate about the +X axis."""
    out_v, out_f = [], []
    n = len(verts)
    for k in range(count):
        a = phase + 2.0 * math.pi * k / count
        out_v.extend(rot_x(verts, a))
        off = k * n
        out_f.extend(tuple(i + off for i in f) for f in faces)
    return out_v, out_f


def at_clock(verts, radius, angle_deg):
    """Move a part built about the origin out to a clock position on the casing."""
    a = math.radians(angle_deg)
    return translate(verts, 0.0, radius * math.cos(a), radius * math.sin(a))


# --------------------------------------------------------------------------
# Hardware details
# --------------------------------------------------------------------------

def bolt_ring(x, radius, count, head_r=9.0, head_h=7.0, segments=12):
    """A ring of hex-ish bolt heads standing proud of a flange face."""
    one_v, one_f = revolve_open(
        [(x, 0.001), (x, head_r), (x + head_h, head_r * 0.92), (x + head_h, 0.001)],
        segments, cap_start=True, cap_end=True)
    one_v = translate(one_v, 0.0, radius, 0.0)
    return replicate(one_v, one_f, count)


def flange(x, r_in, r_out, thickness, n_bolts, bolt_r=8.0):
    """A bolted casing joint: the raised ring plus its bolt circle."""
    ring = tube(x - thickness / 2, x + thickness / 2, r_in, r_out, 96)
    pitch = (r_in + r_out) / 2 + (r_out - r_in) * 0.18
    bolts = bolt_ring(x + thickness / 2, pitch, n_bolts, bolt_r, thickness * 0.55)
    return join(ring, bolts)


def perforate_ring(x, radius, count, hole_r, depth=6.0, segments=8):
    """Small cylinders on a circle -- used as cutters for cooling/screech holes,
    or directly as rivet detail where a boolean would be too expensive."""
    one_v, one_f = cylinder(-depth, depth, hole_r, segments)
    one_v = [(z + x, y + radius, xx) for (xx, y, z) in one_v]
    return replicate(one_v, one_f, count)


def bbox(verts):
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))
