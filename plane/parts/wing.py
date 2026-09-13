"""Cropped-delta wing: panels, flaperons, leading-edge strakes, carbon spar."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import common

W = spec.WING
FL = spec.FLAPERON
NS = spec.RES["wing_stations"]
NC = spec.RES["airfoil_pts"]


def build():
    out = {}
    out.update(_panels())
    out.update(_flaperons())
    out.update(_strakes())
    out.update(_spar())
    return out


def _hinge_u():
    return 1.0 - FL["chord_frac"]


def _panels():
    """Main wing, trimmed at the flaperon hinge line."""
    out = {}
    for side, mir in (("l", True), ("r", False)):
        v, f = common.panel(
            root_le=(W["x_root_le"], 0.0, W["z_root"]),
            root_chord=W["root_chord"], tip_chord=W["tip_chord"],
            semi_span=W["semi_span"], sweep_le=W["sweep_le"],
            dihedral=W["dihedral"], thickness=W["thickness"],
            camber=W["camber"], twist_root=W["incidence"],
            twist_tip=W["incidence"] - W["washout"],
            u0=0.0, u1=_hinge_u(), n_span=NS, n_chord=NC, mirror=mir)
        out[f"wing_{side}"] = (v, f)
    return out


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
            hinge_x, W["z_root"],
            root_le=(x_in, W["semi_span"] * f_in * (-1 if mir else 1), W["z_root"]),
            root_chord=c_in, tip_chord=c_out, semi_span=span, sweep_le=sweep,
            dihedral=W["dihedral"], thickness=W["thickness"], camber=W["camber"],
            u0=hu + FL["gap"] / c_in, u1=1.0,
            n_span=6, n_chord=NC, mirror=mir)
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
        verts, faces = [], []
        for (x, y_in, y_out) in pts:
            verts.append((x, sgn * y_out, W["z_root"] + 1.2))
            verts.append((x, sgn * y_out, W["z_root"] - 1.2))
            verts.append((x, sgn * y_in, W["z_root"] + 1.2))
            verts.append((x, sgn * y_in, W["z_root"] - 1.2))
        for i in range(len(pts) - 1):
            a, b = i * 4, (i + 1) * 4
            faces.append((a, b, b + 1, a + 1))
            faces.append((a + 2, a + 3, b + 3, b + 2))
            faces.append((a, a + 2, b + 2, b))
            faces.append((a + 1, b + 1, b + 3, a + 3))
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
    paths = []
    for sgn in (-1.0, 1.0):
        pts = []
        for i in range(n + 1):
            t = i / n
            y = half * t
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
    hu = _hinge_u()
    out = {}
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        f_in = FL["span_in"]
        c_in = common.local_chord(W["root_chord"], W["tip_chord"], f_in)
        x_in = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"],
                              f_in)
        out[f"flaperon_{side}"] = ((x_in + hu * c_in,
                                    sgn * W["semi_span"] * f_in,
                                    W["z_root"]), (0.0, 1.0, 0.0), sgn, "hinge")
    return out
