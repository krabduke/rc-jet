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
            dihedral=H["anhedral"], thickness=H["thickness"],
            thickness_tip=H["thickness_tip"], tip_cap=6, camber=0.0,
            # Start at the fuselage side, not on the centreline. Rooted at
            # y = 0 the two stabilators were the same surface twice, sharing
            # the whole of their root thickness with each other and with the
            # tailpipe running between them.
            n_span=18, n_chord=NC, pivot=0.25, mirror=mir,
            span0=H["root_y"] / H["semi_span"])
        out[f"stabilator_{side}"] = (v, f)
    return out


def _fin():
    """Vertical fin plus a hinged rudder."""
    out = {}
    ru = 1.0 - V["rudder_chord"]
    out["vtail_fin"] = common.panel(
        root_le=(V["x_root_le"], 0.0, V["z_root"]),
        root_chord=V["root_chord"], tip_chord=V["tip_chord"],
        semi_span=V["height"], sweep_le=V["sweep_le"],
        thickness=V["thickness"], thickness_tip=V["thickness_tip"],
        tip_cap=6, camber=0.0,
        u0=0.0, u1=ru, n_span=20, n_chord=NC, vertical=True)

    h_span = V["height"] * V["rudder_span"]
    c_root = V["root_chord"]
    c_tip = c_root + (V["tip_chord"] - c_root) * V["rudder_span"]
    hinge_x = V["x_root_le"] + ru * c_root
    v, f = common.panel(
        root_le=(V["x_root_le"], 0.0, V["z_root"]),
        root_chord=c_root, tip_chord=c_tip, semi_span=h_span,
        sweep_le=V["sweep_le"], thickness=V["thickness"] * 0.84,
        thickness_tip=V["thickness_tip"] * 0.92, camber=0.0,
        u0=ru + 0.030, u1=1.0, n_span=16, n_chord=NC, vertical=True)
    # rudder deflects about the vertical hinge, so rotate in the x-y plane
    a = math.radians(V["deflect"])
    ca, sa = math.cos(a), math.sin(a)
    v = [(hinge_x + (x - hinge_x) * ca - y * sa,
          (x - hinge_x) * sa + y * ca, z) for (x, y, z) in v]
    out["rudder"] = (v, f)
    return out


def _ventrals():
    """Small canted fins under the aft fuselage. They buy back the yaw
    stability the fin loses at high angle of attack behind a delta.

    These were four-point trapezoids extruded in y: eight vertices each, flat,
    with a knife edge all the way round. A ventral is a lifting surface --
    that is the only reason to carry one -- so it gets a section, and it is
    cambered outboard because it is canted and only ever has to work one way.
    """
    out = {}
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        x0, c, d = VN["x_le"], VN["chord"], VN["depth"]
        sw = math.tan(math.radians(VN["sweep"]))
        # loft downward from the fuselage: span runs -z, so build it vertical
        # and then cant it outboard about its root
        v, f = common.panel(
            root_le=(x0, 0.0, -8.0), root_chord=c, tip_chord=c * 0.62,
            semi_span=-d, sweep_le=-VN["sweep"],
            thickness=VN["thickness"] / c * 1.9,
            thickness_tip=VN["thickness"] / c * 1.2,
            camber=0.03 * sgn, tip_cap=5,
            n_span=12, n_chord=NC, vertical=True)
        ang = math.radians(VN["cant"]) * sgn
        ca, sa = math.cos(ang), math.sin(ang)
        z_hinge = -8.0
        v = [(x, y * ca - (z - z_hinge) * sa + sgn * 14.0,
              z_hinge + y * sa + (z - z_hinge) * ca) for (x, y, z) in v]
        out[f"ventral_{side}"] = (v, f)
    return out


def pivots():
    """Hinge lines for the moving tail surfaces.

    The stabilators are all-moving, so each pivots about its own quarter-chord
    on a spanwise axis. The rudder hinges about a vertical axis at the fin's
    rudder cut. These are the same points the geometry is deflected about, so
    the modelled position and a viewer-driven one agree.
    """
    out = {}
    pivot_x = H["x_root_le"] + H["root_chord"] * 0.25
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        out[f"stabilator_{side}"] = ((pivot_x, 0.0, H["z_root"]),
                                     (0.0, 1.0, 0.0), sgn, "hinge")
    ru = 1.0 - V["rudder_chord"]
    out["rudder"] = ((V["x_root_le"] + ru * V["root_chord"], 0.0, V["z_root"]),
                     (0.0, 0.0, 1.0), 1.0, "hinge")
    return out
