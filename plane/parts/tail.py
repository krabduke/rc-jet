"""Tail surfaces: all-moving stabilators, swept fin with rudder, ventral fins."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import common

H, V, VN = spec.HTAIL, spec.VTAIL, spec.VENTRAL
NC = spec.RES["airfoil_pts"]


def build():
    out = {}
    out.update(_stabilators())
    out.update(_fin())
    out.update(_ventrals())
    return out


def _stabilators():
    """All-moving tailplanes -- no separate elevator, the whole surface pivots
    about its 25 % chord, which is how a fighter this size gets pitch authority
    behind a delta wing."""
    out = {}
    pivot_x = H["x_root_le"] + H["root_chord"] * 0.25
    for side, mir in (("l", True), ("r", False)):
        v, f = common.hinged_panel(
            H["deflect"], pivot_x, H["z_root"],
            root_le=(H["x_root_le"], 0.0, H["z_root"]),
            root_chord=H["root_chord"], tip_chord=H["tip_chord"],
            semi_span=H["semi_span"], sweep_le=H["sweep_le"],
            dihedral=H["anhedral"], thickness=H["thickness"], camber=0.0,
            n_span=8, n_chord=NC, pivot=0.25, mirror=mir)
        out[f"stabilator_{side}"] = (v, f)
    return out


def _fin():
    """Vertical fin plus a hinged rudder."""
    out = {}
    ru = 1.0 - V["rudder_chord"]
    out["vtail_fin"] = common.panel(
        root_le=(V["x_root_le"], 0.0, 24.0),
        root_chord=V["root_chord"], tip_chord=V["tip_chord"],
        semi_span=V["height"], sweep_le=V["sweep_le"],
        thickness=V["thickness"], camber=0.0,
        u0=0.0, u1=ru, n_span=9, n_chord=NC, vertical=True)

    h_span = V["height"] * V["rudder_span"]
    c_root = V["root_chord"]
    c_tip = c_root + (V["tip_chord"] - c_root) * V["rudder_span"]
    hinge_x = V["x_root_le"] + ru * c_root
    v, f = common.panel(
        root_le=(V["x_root_le"], 0.0, 24.0),
        root_chord=c_root, tip_chord=c_tip, semi_span=h_span,
        sweep_le=V["sweep_le"], thickness=V["thickness"], camber=0.0,
        u0=ru + 0.030, u1=1.0, n_span=6, n_chord=NC, vertical=True)
    # rudder deflects about the vertical hinge, so rotate in the x-y plane
    a = math.radians(V["deflect"])
    ca, sa = math.cos(a), math.sin(a)
    v = [(hinge_x + (x - hinge_x) * ca - y * sa,
          (x - hinge_x) * sa + y * ca, z) for (x, y, z) in v]
    out["rudder"] = (v, f)
    return out


def _ventrals():
    """Small canted fins under the aft fuselage. They buy back the yaw
    stability the fin loses at high angle of attack behind a delta."""
    out = {}
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        x0, c, d = VN["x_le"], VN["chord"], VN["depth"]
        sw = math.tan(math.radians(VN["sweep"]))
        t = VN["thickness"] / 2
        # trapezoid in the x-z plane, extruded in y, then canted outboard
        prof = [(x0, -8.0), (x0 + c, -8.0),
                (x0 + c - 6.0, -8.0 - d), (x0 + sw * d, -8.0 - d)]
        verts, faces = [], []
        for (x, z) in prof:
            verts.append((x, sgn * t, z))
        for (x, z) in prof:
            verts.append((x, -sgn * t, z))
        n = len(prof)
        faces.append(tuple(range(n)))
        faces.append(tuple(range(2 * n - 1, n - 1, -1)))
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((i, i2, n + i2, n + i))
        ang = math.radians(VN["cant"]) * sgn
        ca, sa = math.cos(ang), math.sin(ang)
        z_hinge = -8.0
        verts = [(x, y * ca - (z - z_hinge) * sa + sgn * 14.0,
                  z_hinge + y * sa + (z - z_hinge) * ca) for (x, y, z) in verts]
        out[f"ventral_{side}"] = (verts, faces)
    return out
