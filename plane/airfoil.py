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


def blade_platform(row, width_frac=1.0, height=9.0, n_seg=6):
    """The root platform the blade sits on -- one tangential sector of the
    hub flowpath, spanning this blade's share of the circumference."""
    r0 = min(row.r_hub_le, row.r_hub_te)
    dphi = 2.0 * math.pi / row.count * 0.97 * width_frac
    x0 = row.x - row.chord * 0.10
    x1 = row.x + row.chord * 1.10

    verts, faces = [], []
    for xi in (x0, x1):
        for k in range(n_seg + 1):
            a = -dphi / 2 + dphi * k / n_seg
            for r in (r0 - height, r0):
                verts.append((xi, r * math.cos(a), r * math.sin(a)))
    per_x = (n_seg + 1) * 2
    for k in range(n_seg):
        for side in (0, 1):
            i0 = side + k * 2
            a0, a1 = i0, i0 + 2
            b0, b1 = i0 + per_x, i0 + 2 + per_x
            if side == 0:
                faces.append((a0, a1, b1, b0))
            else:
                faces.append((a0, b0, b1, a1))
    # inner and outer bands
    for k in range(n_seg):
        i = k * 2
        faces.append((i, i + per_x, i + per_x + 1, i + 1))
        j = i + 2
        faces.append((j, j + 1, j + per_x + 1, j + per_x))
    return verts, faces


def tip_shroud(row, thickness=6.0, standoff=4.0):
    """Interlocking tip shroud for shrouded turbine rows: a short annular
    sector sitting on the blade tip."""
    r0 = max(row.r_tip_le, row.r_tip_te)
    dphi = 2.0 * math.pi / row.count * 0.99
    x0 = row.x - row.chord * 0.06
    x1 = row.x + row.chord * 1.06
    verts, faces = [], []
    n_seg = 5
    for xi in (x0, x1):
        for k in range(n_seg + 1):
            a = -dphi / 2 + dphi * k / n_seg
            for r in (r0, r0 + standoff + thickness):
                verts.append((xi, r * math.cos(a), r * math.sin(a)))
    per_x = (n_seg + 1) * 2
    for k in range(n_seg):
        i = k * 2
        faces.append((i, i + 2, i + 3, i + 1))
        faces.append((i + per_x, i + per_x + 1, i + per_x + 3, i + per_x + 2))
        faces.append((i, i + 1, i + per_x + 1, i + per_x))
        faces.append((i + 2, i + per_x + 2, i + per_x + 3, i + 3))
        faces.append((i + 1, i + 3, i + per_x + 3, i + per_x + 1))
        faces.append((i, i + per_x, i + per_x + 2, i + 2))
    return verts, faces


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
