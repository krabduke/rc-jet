"""
Airfoil section generation and spanwise lofting.

Pure Python -- no bpy. Produces (verts, faces) where verts is a list of
(x, y, z) tuples in engine coordinates (axis = +X, radius in the YZ plane)
and faces is a list of vertex-index tuples.

One blade is generated per row and then rotationally replicated, so expensive
per-blade detail (cooling holes, fillets) is paid for once per row rather than
once per blade.
"""

import math

STACK = 0.40   # sections are stacked about 40% true chord (typical CG stacking)


def naca_thickness(xc, tc):
    """NACA 4-digit half-thickness at chord fraction xc for thickness ratio tc.
    Final coefficient is -0.1036 rather than -0.1015 to close the trailing edge."""
    if xc < 0.0:
        xc = 0.0
    return 5.0 * tc * (
        0.2969 * math.sqrt(xc)
        - 0.1260 * xc
        - 0.3516 * xc * xc
        + 0.2843 * xc ** 3
        - 0.1036 * xc ** 4
    )


def camber_line(xc, mc):
    """Parabolic camber line with maximum camber mc at mid-chord.
    Returns (y_camber, dy/dx)."""
    y = 4.0 * mc * xc * (1.0 - xc)
    dy = 4.0 * mc * (1.0 - 2.0 * xc)
    return y, dy


def section_points(n_pts, tc, mc, le_radius_boost=1.0):
    """A closed airfoil contour as (u, v) pairs in chord-normalised coordinates.

    Walks trailing edge -> upper surface -> leading edge -> lower surface -> back.
    Cosine spacing clusters points at the leading and trailing edges where
    curvature is highest. n_pts is the total point count of the closed loop.
    """
    n_half = n_pts // 2 + 1
    xs = [0.5 * (1.0 - math.cos(math.pi * i / (n_half - 1))) for i in range(n_half)]

    upper, lower = [], []
    for xc in xs:
        yt = naca_thickness(xc, tc)
        # Blunt, rounded leading edge -- turbine sections need a fat nose.
        if xc < 0.05 and le_radius_boost != 1.0:
            yt *= 1.0 + (le_radius_boost - 1.0) * (1.0 - xc / 0.05)
        yc, dyc = camber_line(xc, mc)
        th = math.atan(dyc)
        st, ct = math.sin(th), math.cos(th)
        upper.append((xc - yt * st, yc + yt * ct))
        lower.append((xc + yt * st, yc - yt * ct))

    # TE -> LE along the upper surface, then LE -> TE along the lower.
    # Drop the duplicated LE and TE points so the loop closes cleanly.
    loop = list(reversed(upper)) + lower[1:-1]
    return loop


def _lerp(a, b, f):
    return a + (b - a) * f


def blade_mesh(row, n_chord=28, n_span=9, cap_hub=True, cap_tip=True):
    """Loft one blade for a BladeRow. Returns (verts, faces).

    The section at spanwise fraction f is placed on a radius that varies along
    the chord from the row's leading-edge radius to its trailing-edge radius,
    so the blade follows the real annulus flare rather than sitting on a
    cylinder. Stagger is interpolated from hub to tip, which is the twist.
    """
    le_boost = 1.8 if row.thickness > 0.10 else 1.0
    sect = section_points(n_chord, row.thickness, row.camber, le_boost)
    n_sec = len(sect)

    verts = []
    for j in range(n_span):
        f = j / (n_span - 1)
        r_le = _lerp(row.r_hub_le, row.r_tip_le, f)
        r_te = _lerp(row.r_hub_te, row.r_tip_te, f)
        gamma = math.radians(_lerp(row.twist_hub, row.twist_tip, f))
        cg, sg = math.cos(gamma), math.sin(gamma)
        axial_chord = row.chord
        true_chord = axial_chord / max(abs(cg), 0.25)
        lean = math.radians(row.lean) * f

        for (u, v) in sect:
            du = (u - STACK) * true_chord
            dv = v * true_chord
            dx = du * cg - dv * sg
            dt = du * sg + dv * cg
            r = _lerp(r_le, r_te, min(max(u, 0.0), 1.0))
            x = row.x + axial_chord * STACK + dx
            phi = dt / r + lean
            verts.append((x, r * math.cos(phi), r * math.sin(phi)))

    faces = []
    for j in range(n_span - 1):
        a = j * n_sec
        b = (j + 1) * n_sec
        for i in range(n_sec):
            i2 = (i + 1) % n_sec
            faces.append((a + i, a + i2, b + i2, b + i))

    if cap_hub:
        faces.append(tuple(range(n_sec - 1, -1, -1)))
    if cap_tip:
        base = (n_span - 1) * n_sec
        faces.append(tuple(range(base, base + n_sec)))

    return verts, faces


def replicate_rotational(verts, faces, count, phase=0.0):
    """Copy a mesh `count` times, evenly spaced about the +X axis."""
    out_v, out_f = [], []
    n = len(verts)
    for k in range(count):
        a = phase + 2.0 * math.pi * k / count
        ca, sa = math.cos(a), math.sin(a)
        off = k * n
        for (x, y, z) in verts:
            out_v.append((x, y * ca - z * sa, y * sa + z * ca))
        for f in faces:
            out_f.append(tuple(i + off for i in f))
    return out_v, out_f


def _sector(prof, dphi, n_seg):
    """Sweep a closed meridional profile through an arc and close both ends.

    A sector of an annulus is one closed surface: the profile swept round,
    plus a cap at each angular end. The platforms and shrouds here used to be
    written as a box per angular step, which leaves a wall between every pair
    of steps -- interior faces inside the solid -- and no caps on the ends at
    all. A blade platform came out with 38 edges used once instead of twice,
    and every blade row in the engine was built on one.
    """
    verts, faces = [], []
    n = len(prof)
    for k in range(n_seg + 1):
        a = -dphi / 2 + dphi * k / n_seg
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in prof:
            verts.append((x, r * ca, r * sa))
    for k in range(n_seg):
        b0, b1 = k * n, (k + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = n_seg * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def blade_platform(row, width_frac=1.0, height=9.0, n_seg=6, reach=None):
    """The root platform the blade sits on -- one tangential sector of the
    hub flowpath, spanning this blade's share of the circumference.

    `reach` is its axial extent (x0, x1); by default a tenth of a chord past
    the airfoil each way. Callers that know the neighbouring rows pass
    spec.row_reach so the platform stops short of them."""
    r0 = min(row.r_hub_le, row.r_hub_te)
    dphi = 2.0 * math.pi / row.count * 0.97 * width_frac
    x0, x1 = reach or (row.x - row.chord * 0.10, row.x + row.chord * 1.10)
    return _sector([(x0, r0 - height), (x1, r0 - height), (x1, r0), (x0, r0)],
                   dphi, n_seg)


def tip_shroud(row, thickness=6.0, standoff=4.0, reach=None):
    """Interlocking tip shroud for shrouded turbine rows: a short annular
    sector sitting on the blade tip. `reach` as for blade_platform."""
    # It sits ON the tip: from a millimetre into the lower end of it. Built
    # from the higher end, it floated a millimetre over the rest of the tip,
    # joined to its blade by nothing. Its outer face, which sets the running
    # clearance to the case, is where it always was.
    r_top = max(row.r_tip_le, row.r_tip_te)
    r0 = min(row.r_tip_le, row.r_tip_te) - 1.0
    dphi = 2.0 * math.pi / row.count * 0.99
    x0, x1 = reach or (row.x - row.chord * 0.06, row.x + row.chord * 1.06)
    r1 = r_top + standoff + thickness
    return _sector([(x0, r0), (x1, r0), (x1, r1), (x0, r1)], dphi, 5)


def cooling_hole_positions(row, n_rows=3, n_per_row=9):
    """Film-cooling hole centres on one blade: a leading-edge showerhead plus
    pressure-side rows. Returns (point, direction) pairs in engine coordinates,
    for the caller to cut with."""
    out = []
    sect = section_points(60, row.thickness, row.camber, 1.8)
    n_sec = len(sect)
    # chordwise fractions: LE showerhead, then two downstream rows
    targets = [0.02, 0.18, 0.40][:n_rows]
    for t in targets:
        # find the section point nearest this chord fraction on the pressure side
        best_i, best_d = 0, 1e9
        for i in range(n_sec // 2, n_sec):
            d = abs(sect[i][0] - t)
            if d < best_d:
                best_d, best_i = d, i
        u, v = sect[best_i]
        nx = sect[(best_i + 1) % n_sec][0] - sect[best_i - 1][0]
        ny = sect[(best_i + 1) % n_sec][1] - sect[best_i - 1][1]
        ln = math.hypot(nx, ny) or 1.0
        normal = (ny / ln, -nx / ln)
        for k in range(n_per_row):
            f = 0.10 + 0.80 * k / max(n_per_row - 1, 1)
            r_le = _lerp(row.r_hub_le, row.r_tip_le, f)
            r_te = _lerp(row.r_hub_te, row.r_tip_te, f)
            gamma = math.radians(_lerp(row.twist_hub, row.twist_tip, f))
            cg, sg = math.cos(gamma), math.sin(gamma)
            true_chord = row.chord / max(abs(cg), 0.25)
            du = (u - STACK) * true_chord
            dv = v * true_chord
            dx = du * cg - dv * sg
            dt = du * sg + dv * cg
            r = _lerp(r_le, r_te, u)
            x = row.x + row.chord * STACK + dx
            phi = dt / r
            p = (x, r * math.cos(phi), r * math.sin(phi))
            ndx = normal[0] * cg - normal[1] * sg
            ndt = normal[0] * sg + normal[1] * cg
            d = (ndx, math.cos(phi) * 0.0 - math.sin(phi) * ndt, math.cos(phi) * ndt)
            dl = math.sqrt(sum(c * c for c in d)) or 1.0
            out.append((p, tuple(c / dl for c in d)))
    return out
