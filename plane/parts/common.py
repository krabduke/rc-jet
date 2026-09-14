"""Shared lifting-surface builder used by the wing and both tails."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import airfoil
import mesh


def section_arc(n_pts, tc, mc, u0=0.0, u1=1.0):
    """Closed contour of an airfoil between chord fractions u0 and u1.

    Restricting the range is how control surfaces get cut out: the wing is
    built 0 -> hinge and the flaperon hinge -> 1, each closed off with a
    straight face at the cut, which is what the real hinge line looks like.
    """
    n = max(n_pts // 2, 6)
    us = [u0 + (u1 - u0) * 0.5 * (1 - math.cos(math.pi * i / (n - 1)))
          for i in range(n)]
    upper, lower = [], []
    for u in us:
        yt = airfoil.naca_thickness(u, tc)
        yc, dyc = airfoil.camber_line(u, mc)
        th = math.atan(dyc)
        st, ct = math.sin(th), math.cos(th)
        upper.append((u - yt * st, yc + yt * ct))
        lower.append((u + yt * st, yc - yt * ct))
    # TE/cut -> upper -> LE/cut -> lower -> back
    loop = list(reversed(upper)) + lower[1:-1]
    return loop


def planform_at(table, f):
    """Leading-edge station and chord at span fraction f, from a table.

    Linear between entries. A table beats a root chord and a sweep angle
    because a real delta's leading edge is a curve -- highly swept at the
    root, unsweeping through the mid span, nearly straight at the tip -- and
    two numbers cannot say that.
    """
    if f <= table[0][0]:
        return table[0][1], table[0][2]
    if f >= table[-1][0]:
        return table[-1][1], table[-1][2]
    for i in range(len(table) - 1):
        f0, x0, c0 = table[i]
        f1, x1, c1 = table[i + 1]
        if f0 <= f <= f1:
            t = (f - f0) / (f1 - f0)
            return x0 + (x1 - x0) * t, c0 + (c1 - c0) * t
    return table[-1][1], table[-1][2]


def panel(root_le, root_chord, tip_chord, semi_span, sweep_le,
          dihedral=0.0, thickness=0.06, camber=0.0,
          twist_root=0.0, twist_tip=0.0, u0=0.0, u1=1.0,
          n_span=12, n_chord=40, pivot=0.25, vertical=False,
          mirror=False, planform=None, thickness_tip=None, tip_cap=0,
          span0=0.0):
    """Loft one lifting surface. Span runs +y, or +z when vertical.

    Twist is applied about `pivot` chord so washout does not also move the
    leading edge, which is what you want for a real wing.

    `planform` overrides the linear root-to-tip taper with a station table,
    which is how the wing gets a curved leading edge.

    `thickness_tip` tapers the section: a wing is thick at the root because
    that is where the spar has to be deep, and thin at the tip because that
    is where thickness is only drag. Constant thickness root to tip is the
    clearest sign a wing was extruded rather than designed.

    `span0` starts the loft part-way out, so a wing can begin at the fuselage
    side instead of at the centreline. Lofted to the centreline the panels run
    through the fuselage, through the tanks in it and through each other, and
    the root rib ends up inside the body. The carry-through that actually
    exists is the spar.

    `tip_cap` closes the tip with that many rings of a rounded cap instead of
    a flat rib. A flat-cut tip is the single thing that makes a lofted wing
    read as blocky -- real tips are closed over, and they shed a tip vortex
    off a radius rather than off a square edge.
    """
    sect = section_arc(n_chord, thickness, camber, u0, u1)
    n_sec = len(sect)
    t_ratio = (1.0 if thickness_tip is None
               else thickness_tip / max(thickness, 1e-6))

    def ring(f, s, v_scale=1.0, c_scale=1.0, s_extra=0.0):
        if planform is not None:
            x_le, chord = planform_at(planform, f)
        else:
            chord = root_chord + (tip_chord - root_chord) * f
            x_le = root_le[0] + s * math.tan(math.radians(sweep_le))
        # shrink about the quarter-chord so a capped tip rakes in, not back
        x_le += chord * pivot * (1.0 - c_scale)
        chord *= c_scale
        tw = math.radians(twist_root + (twist_tip - twist_root) * f)
        ct, st = math.cos(tw), math.sin(tw)
        rise = s * math.tan(math.radians(dihedral))
        tv = (1.0 + (t_ratio - 1.0) * f) * v_scale
        out = []
        for (u, v) in sect:
            du = (u - pivot) * chord
            dv = v * chord * tv
            dx = du * ct - dv * st
            dn = du * st + dv * ct     # offset along the section normal
            px = x_le + pivot * chord + dx
            if vertical:
                # span runs +z, so the aerofoil's thickness must run +/-y.
                # Putting both into z collapses the fin into a flat sheet.
                out.append((px, root_le[1] + dn, root_le[2] + s + s_extra))
            else:
                y = root_le[1] + (-(s + s_extra) if mirror else s + s_extra)
                out.append((px, y, root_le[2] + rise + dn))
        return out

    # The cap rolls over INSIDE the tip station -- semi-span is measured to
    # the tip, so a cap that bulged past it would make the wing wider than it
    # is specified to be, which is exactly what it did.
    bulge = 0.0
    if tip_cap:
        _, tip_c = (planform_at(planform, 1.0) if planform is not None
                    else (0.0, tip_chord))
        bulge = min(abs(tip_c * (thickness if thickness_tip is None
                                 else thickness_tip) * 0.72),
                    abs(semi_span) * 0.06)
        bulge = math.copysign(bulge, semi_span or 1.0)

    verts = []
    for j in range(n_span):
        f = span0 + (1.0 - span0) * j / (n_span - 1)
        verts.extend(ring(f, (semi_span - bulge) * f))

    if tip_cap:
        # a quarter-round over the tip: the section collapses towards its own
        # chord line as the span runs out to the tip station
        for k in range(1, tip_cap + 1):
            a = (math.pi / 2) * k / tip_cap
            verts.extend(ring(1.0, semi_span - bulge,
                              v_scale=max(math.cos(a), 0.09),
                              c_scale=1.0 - 0.16 * (1.0 - math.cos(a)),
                              s_extra=bulge * math.sin(a)))
        n_span += tip_cap

    # Winding handedness depends on span axis and mirroring, and the rules
    # interact badly once surfaces are also rotated to a deflection. These are
    # all closed manifolds, so assemble.py recalculates outward normals in
    # Blender instead of trusting a per-case rule here.
    flip = mirror

    faces = []
    for j in range(n_span - 1):
        a, b = j * n_sec, (j + 1) * n_sec
        for i in range(n_sec):
            i2 = (i + 1) % n_sec
            if flip:
                faces.append((a + i, b + i, b + i2, a + i2))
            else:
                faces.append((a + i, a + i2, b + i2, b + i))
    root_ring = tuple(range(n_sec - 1, -1, -1))
    tip_base = (n_span - 1) * n_sec
    tip_ring = tuple(range(tip_base, tip_base + n_sec))
    if flip:
        faces.append(tuple(reversed(root_ring)))
        faces.append(tuple(reversed(tip_ring)))
    else:
        faces.append(root_ring)
        faces.append(tip_ring)
    return verts, faces


def hinged_panel(deflect_deg, hinge_x, hinge_z, **kw):
    """A control surface, rotated about its hinge line by the given angle."""
    v, f = panel(**kw)
    a = math.radians(deflect_deg)
    ca, sa = math.cos(a), math.sin(a)
    out = []
    for (x, y, z) in v:
        dx, dz = x - hinge_x, z - hinge_z
        out.append((hinge_x + dx * ca - dz * sa, y, hinge_z + dx * sa + dz * ca))
    return out, f


def local_chord(root_chord, tip_chord, f):
    """Chord at span fraction f. Reads the wing's planform table when there is
    one, so every part placed on the wing agrees with the wing's own shape."""
    if getattr(spec, "WING_PLANFORM", None) and root_chord == spec.WING["root_chord"]:
        return planform_at(spec.WING_PLANFORM, f)[1]
    return root_chord + (tip_chord - root_chord) * f


def le_x_at(root_le_x, semi_span, sweep_le, f):
    if (getattr(spec, "WING_PLANFORM", None)
            and abs(root_le_x - spec.WING["x_root_le"]) < 1e-6):
        return planform_at(spec.WING_PLANFORM, f)[0]
    return root_le_x + semi_span * f * math.tan(math.radians(sweep_le))


def surface_z(W, f, u, upper=True):
    """Height of a wing's surface at span fraction f, chord fraction u.

    Details that sit on a wing -- fences, vortex generators, seams -- have to
    start at the skin. Guessing an offset from the mean line puts them either
    buried or floating, and the error grows towards the tip where the section
    is thinnest.
    """
    import airfoil
    chord = local_chord(W["root_chord"], W["tip_chord"], f)
    yt = airfoil.naca_thickness(u, W["thickness"])
    yc, _ = airfoil.camber_line(u, W.get("camber", 0.0))
    v = (yc + yt) if upper else (yc - yt)
    return W["z_root"] + v * chord


def surface_z_frac(sect, u, upper=True):
    """The section's v at chord fraction u, on the requested surface.

    `section_arc` returns the loop going round the aerofoil, so picking a
    surface means filtering on the sign of v -- which is what a rib needs in
    order to know how deep its web has to be at a given station.
    """
    best, bv = None, 0.0
    for (su, sv) in sect:
        if (sv >= 0.0) != upper:
            continue
        d = abs(su - u)
        if best is None or d < best:
            best, bv = d, sv
    return bv


def shrink_ring(ring, zc, k):
    return [(x, y * k, zc + (z - zc) * k) for (x, y, z) in ring]



def lightened_ring(front, back, zc, k_bore, k_hub, n_holes, web_frac=0.34):
    """A former web with a ring of lightening holes cut out of it.

    A former is defined by what is taken out of it. These were solid annular
    plates with a single central bore -- flat, two x-stations, no other
    feature -- so the web between the bore and the skin said nothing at all.

    Built as a rim out at the skin, a hub ring round the bore, and `n_holes`
    spokes bridging the two. The gaps between the spokes are the lightening
    holes, and because both rings follow the fuselage section the holes come
    out shaped like the frame instead of as circles stamped through a disc.
    """
    n = len(front)
    hub_f, hub_b = shrink_ring(front, zc, k_hub), shrink_ring(back, zc, k_hub)
    bore_f, bore_b = shrink_ring(front, zc, k_bore), shrink_ring(back, zc, k_bore)

    verts, faces = [], []

    def closed_band(of, ob, if_, ib):
        """A full annular band between an outer and an inner ring."""
        base = len(verts)
        verts.extend(of); verts.extend(ob); verts.extend(if_); verts.extend(ib)
        a, b, c, d = base, base + n, base + 2 * n, base + 3 * n
        for s in range(n):
            s2 = (s + 1) % n
            faces.append((a + s, a + s2, c + s2, c + s))     # front face
            faces.append((b + s, d + s, d + s2, b + s2))     # back face
            faces.append((a + s, b + s, b + s2, a + s2))     # outer edge
            faces.append((c + s, c + s2, d + s2, d + s))     # inner edge

    # the rim: from the skin in to the hub ring. Everything inboard of the hub
    # is open except where a spoke crosses it.
    closed_band(front, back, hub_f, hub_b)

    span = max(2, int(n * web_frac / max(n_holes, 1)))
    for h in range(n_holes):
        s0 = int(n * h / n_holes)
        idx = [(s0 + j) % n for j in range(span + 1)]
        m = len(idx)
        base = len(verts)
        verts.extend([hub_f[i] for i in idx])
        verts.extend([hub_b[i] for i in idx])
        verts.extend([bore_f[i] for i in idx])
        verts.extend([bore_b[i] for i in idx])
        hf, hb, bf, bb = base, base + m, base + 2 * m, base + 3 * m
        for j in range(m - 1):
            faces.append((hf + j, hf + j + 1, bf + j + 1, bf + j))   # front
            faces.append((hb + j, bb + j, bb + j + 1, hb + j + 1))   # back
            faces.append((bf + j, bf + j + 1, bb + j + 1, bb + j))   # bore edge
            faces.append((hf + j + 1, hf + j, hb + j, hb + j + 1))   # hub edge
        # the two radial cut faces that make this a spoke and not a band
        faces.append((hf, bf, bb, hb))
        e = m - 1
        faces.append((bf + e, hf + e, hb + e, bb + e))
    return verts, faces



