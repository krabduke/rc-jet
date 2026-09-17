"""Built-up internal structure: formers, longerons, stringers, ribs, hinges.

An RC airframe is a stressed skin over a frame. Modelling the frame means the
cutaway shows something real, and it is also what the STL set needs if anyone
wants to actually cut these parts.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common, fuselage as fus

ST = spec.STRUCTURE
W = spec.WING
V = spec.VTAIL
H = spec.HTAIL
FL = spec.FLAPERON
SEG = spec.RES["fuse_sections"]


def build():
    out = {}
    out.update(_formers())
    out.update(_longerons())
    out.update(_stringers())
    out.update(_wing_ribs())
    out.update(_rear_spar())
    out.update(_fin_ribs())
    out.update(_hinges())
    # These write INTO out rather than returning a dict to update it with:
    # a former can be cut by more than one thing -- the canopy aperture and
    # the intake duct both cross former 4 -- and update() would keep only the
    # last one. common.add_cut joins them.
    _cockpit_cutouts(out)
    _tail_cutouts(out)
    return out


def _cockpit_cutouts(built):
    """Open the frame where the cockpit is.

    A former is a ring and a stringer runs the length of the body, so both
    cross the canopy aperture -- and with the skin now cut, they crossed it in
    plain sight, six ply hoops standing across an open cockpit. On a built-up
    airframe they are cut away there and the tub is bonded to what is left of
    them, which is why the tub has a flange. Hand the same prism the skin is
    cut with to every frame member that reaches into it.
    """
    aperture = fus.canopy_aperture()
    K = spec.COCKPIT
    for name, (verts, _f) in list(built.items()):
        if not name.startswith(("former_", "longeron", "stringer")):
            continue
        # only the ones that actually reach into the hole; the boolean is
        # exact and slow, and most of the frame is nowhere near the cockpit
        if any(K["x_front"] <= x <= K["x_rear"]
               and abs(y) <= K["half_width"]
               and z > spec.CANOPY["z_base"] - 1.0 for (x, y, z) in verts):
            common.add_cut(built, name, aperture)
    return built


def _tail_cutouts(built):
    """Open the frame where the stabilator shaft and its arm run.

    The shaft crosses a former and the arm on its inboard end sweeps forward
    through a stringer. Both have to be cut away for the tail to move at all,
    and a ply former with a nine-millimetre lever swinging through it is not a
    detail anyone can leave to the renderer.
    """
    px, pz = spec.stab_pivot()
    tip = spec.stab_horn_tip(1.0)
    x0, x1 = tip[0] - 9.0, px + 8.0
    z0, z1 = pz - 4.8, pz + 4.8
    # One cutter carrying both sides. A dict has one entry per part, so
    # writing cut:former_14 once per side leaves only the second: the
    # right-hand frame was opened and the left-hand frame was not.
    y0, y1 = 4.0, spec.HTAIL["root_y"] + 8.0
    boxes = [mesh.box(0.5 * (x0 + x1), sgn * 0.5 * (y0 + y1), 0.5 * (z0 + z1),
                      x1 - x0, y1 - y0, z1 - z0) for sgn in (-1.0, 1.0)]
    cutter = mesh.join(*boxes)
    for name, (verts, _f) in list(built.items()):
        if not name.startswith(("former_", "longeron", "stringer")):
            continue
        if any(x0 <= x <= x1 and y0 <= abs(y) <= y1 and z0 <= z <= z1
               for (x, y, z) in verts):
            common.add_cut(built, name, cutter)
    return built


# --------------------------------------------------------------------------
# Fuselage frame
# --------------------------------------------------------------------------

def _ring_prism(front, back, hole_front, hole_back):
    """Close two annular rings into a solid plate with a hole through it."""
    n = len(front)
    verts = front + back + hole_front + hole_back
    o_f, o_b, h_f, h_b = 0, n, 2 * n, 3 * n
    faces = []
    for s in range(n):
        s2 = (s + 1) % n
        faces.append((o_f + s, o_f + s2, h_f + s2, h_f + s))   # front annulus
        faces.append((o_b + s, h_b + s, h_b + s2, o_b + s2))   # back annulus
        faces.append((o_f + s, o_b + s, o_b + s2, o_f + s2))   # outer rim
        faces.append((h_f + s, h_f + s2, h_b + s2, h_b + s))   # bore
    return verts, faces


def _shrink(ring, zc, k):
    return [(x, y * k, zc + (z - zc) * k) for (x, y, z) in ring]


def _formers():
    """A light former every 20-40 mm, each with a big lightening bore.

    These carry no landing or spar load -- that is what the ply bulkheads are
    for -- they just hold the skin's section between them.
    """
    out = {}
    t = ST["former_t"]
    k = ST["former_hole"]
    for i, x in enumerate(ST["former_x"], start=1):
        w, h, zc, n = fus.station_at(x)
        f = fus.section_ring(x - t / 2, inset=spec.FUSELAGE_SKIN)
        b = fus.section_ring(x + t / 2, inset=spec.FUSELAGE_SKIN)
        # rim out at the skin, hub round the bore, and six lightening holes
        # between them -- the web was solid before, which is the one thing a
        # former never is
        out[f"former_{i:02d}"] = common.lightened_ring(f, b, zc, k, k + 0.16, 6)
    # the duct runs through the formers it passes; see intake.duct_solid
    from parts import intake as _intake
    cutter = None
    for i, x in enumerate(ST["former_x"], start=1):
        if spec.INTAKE["x_throat"] - 4 <= x <= spec.INTAKE["x_duct_end"] + 4:
            if cutter is None:
                cutter = _intake.duct_solid()
            common.add_cut(out, f"former_{i:02d}", cutter)
    return out


def _skin_path(angle_deg, standoff, x0=None, x1=None, n=40,
               half_depth=0.0):
    """Follow the inside of the skin at a fixed clock angle."""
    # Start aft of the point where the section is smaller than the stringer.
    # At x = 6 the fuselage is 3 mm across and a 2.6 mm longeron cannot be
    # inside it, standoff or not; the nose is solid there anyway.
    xa = spec.FUSELAGE[0][0] + 28.0 if x0 is None else x0
    xb = spec.FUSELAGE[-1][0] - 4.0 if x1 is None else x1
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    path = []
    for i in range(n):
        x = xa + (xb - xa) * i / (n - 1)
        w, h, zc, expn = fus.station_at(x)
        p = 2.0 / expn
        w_in = max(w - spec.FUSELAGE_SKIN - standoff, 0.6)
        h_in = max(h - spec.FUSELAGE_SKIN - standoff, 0.6)
        y = w_in * math.copysign(abs(ca) ** p, ca)
        z = h_in * math.copysign(abs(sa) ** p, sa)
        # Then take up the slack towards the skin if the duct is under it.
        #
        # A stringer stands off the skin by its own depth, and along the
        # bottom of the fuselage the duct's outer wall is within a few
        # millimetres of the skin -- so the four lower stringers ran inside the
        # intake for most of their length. There is room for them between the
        # two; there was never room for them at their nominal standoff. Pull
        # them out towards the skin until they are clear, and no further than
        # the skin itself.
        # The room the member has: the skin's inner face, less its own half
        # depth. Tested BEFORE each step, not after -- checking afterwards
        # lets it overshoot by a whole step and the stringers came out a
        # millimetre through the skin.
        # The room the member has. Two things the obvious version gets wrong:
        #
        #  - the limit is the section's own superellipse, not a box round it.
        #    A member at 225 degrees is at the corner, where |y| and |z| are
        #    both well inside their axis limits and the point is outside the
        #    skin anyway. Both stringers that broke through were at a corner.
        #  - the swept profile is carried along the path tangent, so a member
        #    whose path point is at x 91 has vertices at x 89, where the nose
        #    is already narrower. Measure at the narrowest station it reaches.
        w_n, h_n, n_n = w, h, expn
        for dx in (-half_depth * 3.0, 0.0, half_depth * 3.0):
            w2, h2, _zc2, n2 = fus.station_at(x + dx)
            if w2 < w_n:
                w_n, h_n, n_n = w2, h2, n2
        w_lim = max(w_n - spec.FUSELAGE_SKIN - half_depth * 1.6, 0.4)
        h_lim = max(h_n - spec.FUSELAGE_SKIN - half_depth * 1.6, 0.4)

        def fits_skin(yy, zz):
            return (abs(yy / w_lim) ** n_n
                    + abs(zz / h_lim) ** n_n) <= 1.0
        for _ in range(14):
            if not _in_duct_wall((x, y, zc + z)):
                break
            # away from the DUCT's axis, not the fuselage's. The duct sits low
            # in the forward fuselage, so pushing radially outward from the
            # body's own centre drives a lower stringer along the duct rather
            # than off it.
            dy, dz = _duct_out((x, y, zc + z))
            if not fits_skin(y + dy, z + dz):
                break
            y += dy
            z += dz
        path.append((x, y, zc + z))
    # Where there is still no room, the member stops.
    #
    # Along the bottom of the forward fuselage the duct's outer wall is
    # against the inside of the skin and a stringer cannot be between them at
    # any standoff. A builder does not force one through the intake; the lower
    # stringers are interrupted by the duct and pick up again behind it. Keep
    # the longest run that is clear.
    clear = [not _in_duct_wall(q) for q in path]
    best, run, start = (0, 0), 0, 0
    for i, ok in enumerate(clear + [False]):
        if ok:
            if run == 0:
                start = i
            run += 1
        else:
            if run > best[1] - best[0]:
                best = (start, i)
            run = 0
    return path[best[0]:best[1]] if best[1] - best[0] >= 2 else path


def _duct_out(p, step=1.2):
    """A step away from the duct's axis, in the plane of its section."""
    from parts import intake as _intake
    _w, _h, zc = _intake.duct_section(p[0])
    dy, dz = p[1], p[2] - zc
    m = math.hypot(dy, dz)
    if m < 1e-6:
        return 0.0, step
    return dy / m * step, dz / m * step


def _in_duct_wall(p, margin=1.0):
    """Is this point in the intake duct or its wall?"""
    from parts import intake as _intake
    x, y, z = p
    I = spec.INTAKE
    if not (I["x_throat"] - 2.0 <= x <= I["x_duct_end"] + 2.0):
        return False
    w, h, zc = _intake.duct_section(x)
    _, _, _, n = _intake.duct_bore(x)
    w += margin
    h += margin
    return (abs(y / w) ** n + abs((z - zc) / h) ** n) <= 1.0


def _bay_start():
    """Where the engine bay begins, which is as far aft as the frame can go.

    The tailcone is full of engine. At x = 410 the thrust tube's outer wall is
    at radius 14.1 and the inside of the skin at 19.2, so the annulus left for
    structure is five millimetres -- and a longeron is a strip laid on EDGE
    against the skin, 8.3 mm deep, because that is what makes it stiff in
    bending. It does not fit, and it was not made to: the longerons and
    stringers ran the full 434 mm of the aeroplane, four millimetres inside
    the jet pipe and out through the nozzle's external flaps.

    They stop at the firewall now. Aft of it the tailcone is a moulded shell
    on its formers and the tail bulkhead, which is how a turbine model is
    actually built -- there is nowhere else for the structure to be.
    """
    for name, x, _t in spec.BULKHEADS:
        if name == "bhd_firewall":
            return x + 6.0
    return spec.ENGINE_X


def _longerons():
    """Four main longerons at the section corners, where a slab-sided body is
    stiffest, running from the nose back to the engine firewall."""
    out = {}
    r = ST["longeron_r"]
    # The lower pair used to sit at 222 and 318 -- the bottom corners, which
    # in the forward fuselage is exactly where the intake duct is. Moved out
    # towards the widest point of the section, past the duct's y = 20.6, so
    # they run around it instead of through it.
    for i, ang in enumerate((42.0, 138.0, 205.0, 335.0), start=1):
        # a longeron is a rectangular strip, laid on edge against the skin --
        # a round rod of the same area would be half as stiff in bending and
        # would give the skin a line contact to be glued to instead of a face
        # stood off by the section's half-diagonal, not its old radius: a
        # rectangle 3.2r across corners will not fit in a 1r gap
        out[f"longeron_{i}"] = shapes.swept_profile(
            _skin_path(ang, r * 1.80, x1=_bay_start(),
                       half_depth=r * 0.62),
            shapes.rounded_polygon(
                [(-r * 1.6, -r * 0.62), (r * 1.6, -r * 0.62),
                 (r * 1.6, r * 0.62), (-r * 1.6, r * 0.62)],
                r * 0.30, seg=3), subdiv=2)
    return out


def _stringers():
    """A dozen thin stringers between the longerons, so the skin has something
    to sit on between frames.

    They stop at the firewall with the longerons, and for the same reason: the
    tailcone is full of engine. See _bay_start.
    """
    r = ST["stringer_r"]
    parts = []
    n = ST["n_stringers"]
    for i in range(n):
        ang = 360.0 * i / n + 15.0
        parts.append(shapes.swept_profile(
            _skin_path(ang, r * 1.50, x1=_bay_start(), n=28,
                       half_depth=r * 0.55),
            shapes.rounded_polygon(
                [(-r * 1.3, -r * 0.55), (r * 1.3, -r * 0.55),
                 (r * 1.3, r * 0.55), (-r * 1.3, r * 0.55)],
                r * 0.28, seg=3), subdiv=2))
    return {f"stringer_{i + 1:02d}": m for i, m in enumerate(parts)}


# --------------------------------------------------------------------------
# Wing
# --------------------------------------------------------------------------

def _inset(sect, chord, inset, u0, u1):
    """Pull a section in by `inset` mm all round, so the rib fits under the
    skin instead of breaking through it."""
    du = inset / chord
    span = max(u1 - u0 - 2 * du, 1e-3)
    out = []
    for (u, v) in sect:
        uu = u0 + du + (u - u0) / max(u1 - u0, 1e-6) * span
        mag = max(abs(v) * chord - inset, 0.0)
        out.append((uu, math.copysign(mag / chord, v) if v else 0.0))
    return out


def _wing_section(f, u0, u1, n_pts=48):
    """The wing's own lofted section at span fraction f, between u0 and u1.

    The skin is lofted by `common.panel` from `common.section_arc` with the
    section's v scaled by a thickness taper and the whole section rotated
    about the quarter chord by the washout twist. A rib has to come from the
    same function and go through the same two transforms, or it is a
    different aerofoil from the one over it: the old rib read `surface_z`,
    which ignores the twist, and at the tip the two disagree by more than the
    skin is thick -- which is exactly what made the ribs read as tan stripes
    standing through a closed wing.
    """
    chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
    x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
    tw = math.radians(-W["washout"] * f)
    t_ratio = W["thickness_tip"] / max(W["thickness"], 1e-6)
    tv = 1.0 + (t_ratio - 1.0) * f
    sect = common.section_arc(n_pts, W["thickness"], W["camber"], u0, u1)
    ct, st = math.cos(tw), math.sin(tw)
    pivot = 0.25

    def place(u, v, yy):
        du = (u - pivot) * chord
        dv = v * chord * tv
        return (x_le + pivot * chord + du * ct - dv * st,
                yy, W["z_root"] + du * st + dv * ct)

    return sect, place


def _rib(y, f, t, u1):
    """One wing rib: cap strips top and bottom, webs between them, and a
    doubler where each spar passes through.

    The old rib was the aerofoil with one enormous hole scaled out of the
    middle of it, which leaves a thin closed ring -- a shape that would buckle
    the first time the skin loaded it and which nobody cuts. A real light rib
    is a truss: full-depth cap strips carrying the bending, vertical webs
    carrying the shear between them, and local doublers at the spars. It is
    also what makes a cutaway of this aeroplane worth looking at.

    The contour comes from `_wing_section`, the same section function the
    skin is lofted from, pulled in by the rib inset all round so the caps sit
    strictly under the skin even where washout tips the section.
    """
    chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
    sect, place = _wing_section(f, 0.0, u1)
    sect = _inset(sect, chord, ST["rib_inset"], 0.0, u1)

    # the cap strips: the section, and the same section drawn inwards by the
    # strip width, clamped so the two never cross near the trailing edge
    cap = ST["rib_inset"] / max(chord, 1e-6)
    inner = []
    for (u, v) in sect:
        keep = 0.22 * abs(v)
        dv = min(cap, max(abs(v) - keep, 0.0))
        uu = u + (cap * 0.8 if u < 0.06 else (-cap * 0.8 if u > u1 - 0.06
                                              else 0.0))
        inner.append((uu, v - math.copysign(dv, v or 1.0)))

    parts = [_ring_prism([place(u, v, y - t / 2) for (u, v) in sect],
                         [place(u, v, y + t / 2) for (u, v) in sect],
                         [place(u, v, y - t / 2) for (u, v) in inner],
                         [place(u, v, y + t / 2) for (u, v) in inner])]

    # shear webs between the caps, and a diagonal across the biggest bay
    def web(u0, u1_, wt):
        top = common.surface_z_frac(sect, (u0 + u1_) / 2, upper=True)
        bot = common.surface_z_frac(sect, (u0 + u1_) / 2, upper=False)
        a = place((u0 + u1_) / 2, (top + bot) / 2, y)
        h = abs(top - bot) * chord
        return shapes.rounded_box(a[0], a[1], a[2], (u1_ - u0) * chord,
                                  t * 0.92, max(h - 2 * ST["rib_inset"], 1.0),
                                  min(1.2, t * 0.4), seg=5)

    for (u0, u1_) in ((0.10, 0.145), (0.34, 0.385), (0.52, 0.565),
                      (u1 - 0.10, u1 - 0.055)):
        if u1_ < u1 - 0.01:
            parts.append(web(u0, u1_, t))
    # doublers where the two spars pass through
    for frac in (0.30, ST["rear_spar_frac"]):
        if frac >= u1 - 0.02:
            continue
        top = common.surface_z_frac(sect, frac, upper=True)
        bot = common.surface_z_frac(sect, frac, upper=False)
        a = place(frac, (top + bot) / 2, y)
        parts.append(shapes.rounded_box(
            a[0], a[1], a[2], chord * 0.075, t * 2.6,
            abs(top - bot) * chord * 0.80, min(1.4, t), seg=5))
    return mesh.join(*parts)


def _wing_ribs():
    """Ribs at even span stations.

    Every rib stops at the same chord fraction as the wing skin does. The wing
    is trimmed at the flaperon hinge line for its whole span, so a full-chord
    rib inboard of the flaperon would hang out behind the trailing edge.
    """
    out = {}
    n = ST["n_wing_ribs"]
    t = ST["rib_t"]
    u1 = 1.0 - FL["chord_frac"]
    # Ribs start where the wing does. They used to run from 6 % of semi-span,
    # which is inside the fuselage now that the panels begin at its side --
    # the first two hung in the body and the second one was in a fuel tank.
    from parts import wing as _wing
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        for i, f in enumerate(_wing._rib_stations(*range(1, n + 1))):
            y = sgn * W["semi_span"] * f
            out[f"rib_{side}_{i + 1:02d}"] = _rib(y, f, t, u1)
    return out


def _rear_spar():
    """A second, lighter spar at 70% chord -- this is the flaperon hinge
    backing, which the front spar is too far forward to provide."""
    frac = ST["rear_spar_frac"]
    tip = ST["rear_spar_span"]
    # rooted where the wing panels are, outboard of the intake duct -- see
    # wing._spar for why nothing crosses the fuselage at z -6
    from parts import wing as _wing
    f0 = _wing._root_span0() + 1.0 / W["semi_span"]
    # Two paths, one per panel. A single path from -tip to +tip would step
    # straight across the fuselage from y -23 to +23 at its middle point --
    # through the engine's spinner, which is exactly where it went.
    paths = []
    for sgn in (-1.0, 1.0):
        pts = []
        for i in range(6):
            af = f0 + (tip - f0) * i / 5
            chord = common.local_chord(W["root_chord"], W["tip_chord"], af)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], af)
            pts.append((x_le + chord * frac, sgn * W["semi_span"] * af,
                        W["z_root"]))
        paths.append(pts)
    # the hinge backing is a D-section: flat aft where the hinges screw into
    # it, round forward where it takes the bending
    r = ST["rear_spar_r"]
    sect = shapes.rounded_polygon(
        [(-r * 0.9, -r), (r * 1.5, -r), (r * 1.5, r), (-r * 0.9, r)],
        [r * 0.85, r * 0.22, r * 0.22, r * 0.85], seg=6)
    return {"spar_rear": mesh.join(
        *[shapes.swept_profile(p, sect, subdiv=3) for p in paths])}


def _fin_ribs():
    """Ribs in the vertical fin, ahead of the rudder hinge.

    A fin rib lies in a horizontal plane, so its plate thickness runs in z and
    the aerofoil's thickness runs in y -- the opposite of a wing rib.
    """
    out = {}
    n = ST["n_fin_ribs"]
    t = ST["rib_t"]
    k = ST["rib_hole"]
    u1 = 1.0 - V["rudder_chord"]
    sect0 = common.section_arc(44, V["thickness"], 0.0, 0.0, u1)
    cu = 0.5 * u1
    for i in range(n):
        f = 0.10 + 0.74 * i / (n - 1)
        chord = common.local_chord(V["root_chord"], V["tip_chord"], f)
        x_le = V["x_root_le"] + V["height"] * f * math.tan(
            math.radians(V["sweep_le"]))
        z = V["z_root"] + V["height"] * f
        sect = _inset(sect0, chord, ST["rib_inset"], 0.0, u1)
        hole = [(cu + (u - cu) * k, v * k) for (u, v) in sect]

        def ring(pts, dz):
            return [(x_le + u * chord, v * chord, z + dz) for (u, v) in pts]

        rib = [_ring_prism(ring(sect, -t / 2), ring(sect, t / 2),
                           ring(hole, -t / 2), ring(hole, t / 2))]
        # the doubler where the fin spar passes through, and a web aft of it
        for (frac, wid) in ((0.32, 0.10), (0.66, 0.06)):
            if frac >= u1 - 0.02:
                continue
            top = common.surface_z_frac(sect, frac, upper=True)
            bot = common.surface_z_frac(sect, frac, upper=False)
            rib.append(shapes.rounded_box(
                x_le + frac * chord, (top + bot) / 2 * chord, z,
                chord * wid, abs(top - bot) * chord * 0.78, t * 2.2,
                min(0.9, t), seg=5))
        out[f"fin_rib_{i + 1}"] = mesh.join(*rib)
    return out

# --------------------------------------------------------------------------
# Hinges
# --------------------------------------------------------------------------

def _piano_hinge(p0, p1, r, knuckles):
    """A hinge pin with knuckles alternating along it."""
    parts = [mesh.pipe([p0, p1], r * 0.45, 18, subdiv=3)]
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ang = math.atan2(dy, dx)
    for k in range(knuckles):
        f = (k + 0.5) / knuckles
        cx = p0[0] + dx * f
        cy = p0[1] + dy * f
        cz = p0[2] + (p1[2] - p0[2]) * f
        v, kf = mesh.tube(-1.1, 1.1, r * 0.45, r, 10)
        v = mesh.rot_z(v, ang)          # knuckle axis follows the hinge line
        parts.append(([(x + cx, y + cy, z + cz) for (x, y, z) in v], kf))
    return mesh.join(*parts)


def _hinges():
    """Hinge lines on every moving surface."""
    out = {}
    r = ST["hinge_r"]
    kn = ST["hinge_knuckles"]
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        pts = []
        for f in (FL["span_in"], FL["span_out"]):
            chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], f)
            pts.append((x_le + chord * (1 - FL["chord_frac"]),
                        sgn * W["semi_span"] * f, W["z_root"]))
        out[f"hinge_flaperon_{side}"] = _piano_hinge(pts[0], pts[1], r, kn)

    x_h = V["x_root_le"] + V["root_chord"] * (1 - V["rudder_chord"])
    x_t = (V["x_root_le"] + V["height"] * math.tan(math.radians(V["sweep_le"]))
           + V["tip_chord"] * (1 - V["rudder_chord"]))
    out["hinge_rudder"] = _piano_hinge((x_h, 0.0, 30.0),
                                       (x_t, 0.0, 30.0 + V["height"] * 0.9),
                                       r, 7)
    return out
