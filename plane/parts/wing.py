"""Cropped-delta wing: panels, flaperons, leading-edge strakes, carbon spar."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common

W = spec.WING
FL = spec.FLAPERON
NS = spec.RES["wing_stations"]
NC = spec.RES["airfoil_pts"]
LE_HINGE_U = 0.15
LE_SPAN_IN = 0.25
LE_SPAN_OUT = 0.95
LE_CRUISE_DEG = 1.0


def build():
    out = {}
    out.update(_panels())
    out.update(_flaperons())
    out.update(_strakes())
    out.update(_spar())
    out.update(_leading_edge_flaps())
    return out


def _lower_z(x, y):
    """Height of the wing's lower skin under a point in plan."""
    fr = min(0.999, abs(y) / W["semi_span"])
    chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
    x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], fr)
    u = min(1.0, max(0.0, (x - x_le) / chord))
    return common.surface_z(W, fr, u, upper=False)


def _rounded_rect(cx, cy, sx, sy, r, seg=4):
    """A rectangle in plan with radiused corners, going round once."""
    out = []
    for (ox, oy, a0) in (( 1, 1, 0.0), (-1, 1, math.pi / 2),
                         (-1, -1, math.pi), ( 1, -1, -math.pi / 2)):
        px = cx + ox * (sx / 2 - r)
        py = cy + oy * (sy / 2 - r)
        for k in range(seg + 1):
            a = a0 + math.pi / 2 * k / seg
            out.append((px + r * math.cos(a), py + r * math.sin(a)))
    return out


def _rib_stations(*which):
    """Span fractions of the numbered wing ribs, counting from one.

    Kept in step with structure._wing_ribs by construction rather than by
    coincidence: anything that has to land on a rib asks for the rib.
    """
    n = spec.STRUCTURE["n_wing_ribs"]
    f0 = _root_span0() + 0.02
    return [f0 + (0.96 - f0) * (i - 1) / (n - 1) for i in which]


def _hinge_u():
    return 1.0 - FL["chord_frac"]


def _root_span0():
    """Span fraction at which the wing panel starts: the fuselage side.

    The panels used to run to y = 0, so both wings passed through the
    fuselage, through the saddle tanks in it and through each other. The
    carry-through that exists is spar_carbon, which does span the body.
    Reference wing area is unchanged -- that is the gross planform, and it
    always included the covered centre section.
    """
    from parts import fuselage as fus
    x_mid = W["x_root_le"] + W["root_chord"] * 0.5
    w, h, zc, n = fus.station_at(x_mid)
    return min(0.45, max(0.0, (w - 2.0) / W["semi_span"]))


def _panels():
    """Main wing, trimmed at the flaperon hinge line."""
    out = {}
    for side, mir in (("l", True), ("r", False)):
        v, f = common.panel(
            root_le=(W["x_root_le"], 0.0, W["z_root"]),
            root_chord=W["root_chord"], tip_chord=W["tip_chord"],
            semi_span=W["semi_span"], sweep_le=W["sweep_le"],
            dihedral=W["dihedral"], thickness=W["thickness"],
            planform=spec.WING_PLANFORM,
            thickness_tip=W["thickness_tip"], tip_cap=7,
            camber=W["camber"], twist_root=W["incidence"],
            twist_tip=W["incidence"] - W["washout"],
            u0=LE_HINGE_U, u1=_hinge_u(), n_span=NS, n_chord=NC, mirror=mir,
            span0=_root_span0())
        out[f"wing_{side}"] = (v, f)
    return out


def _le_span0():
    """Span fraction at which the leading-edge flap starts.

    Not the wing's. The flap is a chord AHEAD of the wing, where the body is
    wider, and the strake fills the root leading edge as a fillet on it --
    `_strakes` runs its outboard edge to the section's own half width plus
    13 mm. Started at the wing's root station the flap's root end was inside
    the fuselage: the lengthwise seam, two stringers and the first rib were
    all within it.
    """
    from parts import fuselage as fus
    x0 = W["x_root_le"] - W["le_root_ext"]
    x1 = W["x_root_le"] + W["root_chord"] * LE_HINGE_U
    out = 0.0
    for i in range(17):
        w, _h, _zc, _n = fus.station_at(x0 + (x1 - x0) * i / 16.0)
        out = max(out, w * 0.96 + 14.0)
    return min(0.45, max(_root_span0(), out / W["semi_span"]))


def _leading_edge_flaps():
    out = {}
    for side, mir in (("l", True), ("r", False)):
        v, f = common.panel(
            root_le=(W["x_root_le"], 0.0, W["z_root"]),
            root_chord=W["root_chord"], tip_chord=W["tip_chord"],
            semi_span=W["semi_span"], sweep_le=W["sweep_le"],
            dihedral=W["dihedral"], thickness=W["thickness"],
            planform=spec.WING_PLANFORM,
            thickness_tip=W["thickness_tip"], tip_cap=7,
            camber=W["camber"], twist_root=W["incidence"],
            twist_tip=W["incidence"] - W["washout"],
            u0=0.0, u1=LE_HINGE_U, n_span=NS, n_chord=NC, mirror=mir,
            span0=_le_span0())
        out[f"leading_edge_flap_{side}"] = (v, f)
    return out


def flaperon_hinge(side):
    """The flaperon's hinge line, as its inboard and outboard points: the one
    line the surface turns about, the piano hinge is built on, and the viewer
    pivots it on."""
    sgn = -1.0 if side == "l" else 1.0
    pts = []
    for f in (FL["span_in"], FL["span_out"]):
        chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
        pts.append((x_le + chord * _hinge_u(), sgn * W["semi_span"] * f,
                    W["z_root"]))
    return tuple(pts)


def _flaperons():
    """One flaperon per side, spanning the outer wing and shown deflected."""
    out = {}
    hu = _hinge_u()
    f_in, f_out = FL["span_in"], FL["span_out"]
    for side, mir in (("l", True), ("r", False)):
        # the surface occupies a sub-span, so start it at the inboard station
        c_in = common.local_chord(W["root_chord"], W["tip_chord"], f_in)
        c_out = common.local_chord(W["root_chord"], W["tip_chord"], f_out)
        x_in = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f_in)
        span = W["semi_span"] * (f_out - f_in)
        sweep = math.degrees(math.atan(
            (common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f_out)
             - x_in) / span))
        hinge_x = x_in + hu * c_in
        v, f = common.hinged_panel(
            FL["deflect"] * (1 if mir else -1),   # differential, as ailerons
            hinge_x, W["z_root"], axis=flaperon_hinge(side),
            root_le=(x_in, W["semi_span"] * f_in * (-1 if mir else 1), W["z_root"]),
            root_chord=c_in, tip_chord=c_out, semi_span=span, sweep_le=sweep,
            dihedral=W["dihedral"], thickness=W["thickness"] * 0.80,
            thickness_tip=W["thickness_tip"] * 0.92, camber=W["camber"],
            u0=hu + FL["gap"] / c_in, u1=1.0,
            n_span=14, n_chord=NC, mirror=mir)
        out[f"flaperon_{side}"] = (v, f)
    return out


def _strakes():
    """Leading-edge root extensions. On a delta this size they are what keeps
    the nose-up vortex attached instead of letting the root stall first."""
    out = {}
    ext = W["le_root_ext"]
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        x0 = W["x_root_le"] - ext
        x1 = W["x_root_le"] + 46.0
        # inboard edge follows the fuselage surface so the strake is a fillet
        # on the body, not a plate floating beside it
        from parts import fuselage as fus
        pts = []
        n = 14
        for i in range(n + 1):
            t = i / n
            x = x0 + (x1 - x0) * t
            w, h, zc, _ = fus.station_at(x)
            y_in = w * 0.96
            y_out = y_in + 13.0 * math.sin(math.pi * min(t * 1.05, 1.0)) ** 0.7
            pts.append((x, y_in, y_out))
        # A strake exists to shed a strong, stable vortex over the wing root
        # at high alpha. That needs a sharp leading edge and a section that
        # thickens inboard into the body -- as a constant-thickness plate with
        # a blunt edge all round it shed nothing and stalled with the root.
        rings = []
        n_c = 13
        for (x, y_in, y_out) in pts:
            span = max(y_out - y_in, 0.1)
            ring = []
            for k in range(n_c):
                g = k / (n_c - 1)
                yy = y_in + span * g
                # thickest at the body, tapering to a knife at the tip, and
                # drooped outboard so it turns the flow down over the wing
                th = 2.6 * (1.0 - g) ** 1.25 + 0.22
                droop = -1.9 * g ** 1.9
                ring.append((yy, th, droop))
            loop = []
            for (yy, th, dz) in ring:
                loop.append((x, sgn * yy, W["z_root"] + dz + th / 2))
            for (yy, th, dz) in reversed(ring[:-1]):
                loop.append((x, sgn * yy, W["z_root"] + dz - th / 2))
            rings.append(loop)
        m = len(rings[0])
        verts = [v for r in rings for v in r]
        faces = []
        for i in range(len(rings) - 1):
            a, b = i * m, (i + 1) * m
            for k in range(m):
                k2 = (k + 1) % m
                faces.append((a + k, a + k2, b + k2, b + k))
        faces.append(tuple(range(m - 1, -1, -1)))
        base = (len(rings) - 1) * m
        faces.append(tuple(range(base, base + m)))
        out[f"wing_strake_{side}"] = (verts, faces)
    return out


def _spar():
    """Carbon tube through the wing, swept to follow a constant chord line.

    A straight spanwise spar does not work on a 40-degree delta: at 30 % of the
    root chord it leaves the planform by y = 66 mm and the rest hangs in free
    air. Following the same chord fraction out to the tip keeps it buried in
    the section the whole way, which is also where the bending material wants
    to be.
    """
    half = spec.SPAR["span"] / 2
    n = 10
    # Two stub spars, not one tube through the middle.
    #
    # The duct is 41 mm across the fuselage at every station the wing attaches
    # at, and the wing sits at z -6, which is the middle of it. A spanwise tube
    # at that height ran straight down the intake -- 18 % of the spar was in
    # the airflow. There is no height it can pass at either: 12 mm over the
    # duct at x 256 and 6 mm under it, against a wing whose mean line is 22 mm
    # below the one and 16 above the other.
    #
    # So each panel carries its own spar, rooted at the fuselage side where
    # the wing panels themselves begin, and the two panels are joined through
    # the fuselage frames rather than through the air the engine breathes.
    # That is how a nose-intake aeroplane is built -- an F-16 has no wing
    # carry-through either, for exactly this reason.
    # where the panels themselves begin -- outboard of the duct AND of the
    # saddle tanks either side of it
    y0 = _root_span0() * W["semi_span"] + 1.0
    paths = []
    for sgn in (-1.0, 1.0):
        pts = []
        for i in range(n + 1):
            t = i / n
            y = y0 + (half - y0) * t
            f = y / W["semi_span"]
            chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
            x = x_le + chord * spec.SPAR["x_frac"]
            # sit on the section mid-line, allowing for washout
            z = W["z_root"] + (W["incidence"] - W["washout"] * f) * 0.0
            pts.append((x, sgn * y, z))
        paths.append(mesh.pipe(pts, spec.SPAR["outer_r"],
                               spec.RES["small_revolve"], caps=True))
    return {"spar_carbon": mesh.join(*paths)}


def pivots():
    """Flaperon hinge lines.

    The flaperon rotates about the hinge it is cut at, so the viewer can move
    it the way the servo does and feed the same angle to the aero solver.
    Without a pivot it would swing about the nose of the aircraft.
    """
    out = {}
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        p0, p1 = flaperon_hinge(side)
        d = [p1[i] - p0[i] for i in range(3)]
        n = math.sqrt(sum(c * c for c in d))
        axis = tuple(c / n for c in d)
        if axis[1] < 0:
            axis = tuple(-c for c in axis)
        out[f"flaperon_{side}"] = (p0, axis, sgn, "hinge")
    return out
