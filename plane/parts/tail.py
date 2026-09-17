"""Full-size tail with symmetric sections, radiused tips and static wicks.

The fin-tip ECM antenna housing is a dielectric fairing, not a metal cap.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common

H, V, VN = spec.HTAIL, spec.VTAIL, spec.VENTRAL
NC = spec.RES["airfoil_pts"]


def build():
    out = {}
    out.update(_stabilators())
    out.update(_pivots_hw())
    out.update(_fin())
    out.update(_ventrals())
    return out


def _pivots_hw():
    """How the stabilators are carried and driven, full size.

    Each surface turns on a steel trunnion shaft carried in two spherical
    rolling bearings seated in a fuselage frame. A full-size machine carries
    its all-moving tail on bearings sized for the hinge moment of an
    eleven-tonne surface, so the bearing pitch is set by structure, not by
    whatever fits between thrust tube and skin.

    It is driven by an integrated hydraulic servo-actuator, one per surface,
    mounted inside the fuselage: body, ram, manifold block, and the two
    pressure lines running forward to the utility system. Nothing protrudes
    through the skin. A full-size aeroplane cannot use the model's horn and
    clevis: a model servo can sit anywhere and its pushrod needs three
    millimetres of access, so the link went through the pocket between the
    thrust tube and the skin -- 4.8 mm tall, 9 mm wide, which is why the arm
    lay fore-and-aft. An F110's actuators live behind sealed skin panels and
    drive the trunnion directly, so there is no external linkage at all.
    """
    out = {}
    px, pz = spec.stab_pivot()
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        parts = []
        y_root = sgn * H["root_y"]
        # trunnion shaft: from forward bearing out into the root rib, sized
        # for a full-size hinge moment, not a 3 mm model wire
        shank_r = H["thickness"] * H["root_chord"] * 0.42
        parts.append(mesh.pipe(
            [(px, sgn * 4.0, pz), (px, y_root + sgn * 7.0, pz)], shank_r, 16))
        # two spherical bearings, one either side of the tailpipe, pressed
        # into bores in a fuselage frame
        for yb in (sgn * 4.6, sgn * 11.0):
            bv, bf = mesh.revolve_ring(
                [(0.0, shank_r * 1.12), (0.0, shank_r * 1.85),
                 (shank_r * 1.55, shank_r * 1.85),
                 (shank_r * 1.55, shank_r * 1.12)], 20)
            parts.append(([(pz_ + px, px_ + yb, py_ + pz)
                           for (px_, py_, pz_) in bv], bf))
        # the fuselage frame web the bearings sit in, spanning the tailpipe
        frv, frf = mesh.box(px + H["root_chord"] * 0.30, sgn * 8.0, pz,
                            H["root_chord"] * 0.72, 2.4, H["root_chord"] * 0.34)
        parts.append((frv, frf))
        out[f"stab_pivot_{side}"] = mesh.join(*parts)
        out.update(_stab_servo(side, sgn, px, pz))
    return out


def _stab_servo(side, sgn, px, pz):
    """Integrated hydraulic servo-actuator driving one stabilator.

    Body, ram, manifold block, and the two pressure lines, all inside the
    fuselage. The ram picks up on the trunnion shaft inboard of the frame, so
    the actuator torques the surface through its own pivot rather than through
    a lever out on the shaft -- no horn, no clevis, nothing through the skin.
    """
    parts = []
    y = sgn * (H["root_y"] - 3.0)
    # actuator body: barrel of the hydraulic cylinder, trunnion-mounted to a
    # lug off the fuselage frame
    r_b = H["root_chord"] * 0.012
    bvv, bf = mesh.revolve_ring(
        [(0.0, r_b), (0.0, r_b * 1.28), (r_b * 3.4, r_b * 1.28),
         (r_b * 3.4, r_b)], 20)
    parts.append(([(px_ + px + r_b * 0.6, px_ * sgn + y, py_ + pz - r_b * 1.7)
                   for (px_, py_, pz_) in bvv], bf))
    # ram: rod out forward to pick up on the trunnion shaft
    parts.append(mesh.pipe(
        [(px + r_b * 3.4, y, pz - r_b * 1.7), (px + r_b * 0.9, y, pz - r_b * 1.7)],
        r_b * 0.42, 12))
    # manifold block between the pressure lines and the cylinder ports
    mv, mf = mesh.box(px + r_b * 1.6, y, pz - r_b * 2.6,
                      r_b * 1.4, r_b * 1.6, r_b * 1.4)
    parts.append((mv, mf))
    # two pressure lines running forward into the fuselage
    for dy in (-r_b * 0.9, r_b * 0.9):
        parts.append(mesh.pipe(
            [(px + r_b * 1.6, y + sgn * dy, pz - r_b * 2.6),
             (px - H["root_chord"] * 0.32, y + sgn * dy, pz - r_b * 2.9)],
            r_b * 0.22, 10))
    return {f"stab_servo_{side}": mesh.join(*parts)}


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
    """Vertical fin plus a hinged rudder.

    Full size, the rudder is driven by a hydraulic servo-actuator buried in
    the fin root -- there is no pushrod out through the skin and no horn on
    the rudder span, because anything proud of a full-size contour is drag
    and a FOD/ice hazard. The fin tip carries an ECM/antenna fairing, which
    is why the tip chord is as long as it is: the avionics inside need the
    volume.
    """
    out = {}
    ru = 1.0 - V["rudder_chord"]
    out["vtail_fin"] = common.panel(
        root_le=(V["x_root_le"], 0.0, V["z_root"]),
        root_chord=V["root_chord"], tip_chord=V["tip_chord"],
        semi_span=V["height"], sweep_le=V["sweep_le"],
        thickness=V["thickness"], thickness_tip=V["thickness_tip"],
        tip_cap=6, camber=0.0,
        u0=0.0, u1=ru, n_span=20, n_chord=NC, vertical=True)
    out.update(_rudder_servo(ru))
    out.update(_fin_tip_fairing(ru))

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


def _rudder_servo(ru):
    """Hydraulic servo-actuator for the rudder, in the fin root.

    Entirely inside the fin contour: a cylinder above the fin's lower rudder
    cut pushing forward on a crank off the rudder's forward extension. A
    full-size rudder is driven from inside its root fairing because the hinge
    line is buried in the fin/fuselage junction, and no fitting may stand
    proud of the contour -- a leak weeps into a bay, not over the skin.
    """
    parts = []
    c_root = V["root_chord"]
    # sit on the fin centreline, low on the fin, ahead of the hinge
    hinge_x = V["x_root_le"] + ru * c_root
    z = V["z_root"] + V["height"] * 0.12
    r_b = c_root * 0.05
    yc = 0.0
    # actuator body: barrel of the hydraulic cylinder
    bvv, bf = mesh.revolve_ring(
        [(0.0, r_b), (0.0, r_b * 1.25), (r_b * 3.2, r_b * 1.25),
         (r_b * 3.4, r_b)], 20)
    parts.append(([(px_ + hinge_x - r_b * 3.6, px_ * 1.0 + yc, py_ + z)
                   for (px_, py_, pz_) in bvv], bf))
    # ram aft onto a crank on the rudder's forward balance extension.
    #
    # It used to stop at hinge_x - 0.5 r_b, which is forward of the hinge and
    # therefore forward of the rudder: the actuator drove nothing. The rudder
    # panel starts 0.03 chord aft of the hinge line, which is 0.6 r_b, so the
    # rod has to reach past that to land on it.
    parts.append(mesh.pipe(
        [(hinge_x - r_b * 3.2, yc, z), (hinge_x + r_b * 1.4, yc, z)],
        r_b * 0.42, 12))
    # manifold block between the pressure lines and the cylinder ports
    mv, mf = mesh.box(hinge_x - r_b * 3.0, yc, z - r_b * 2.4,
                      r_b * 1.3, r_b * 1.5, r_b * 1.3)
    parts.append((mv, mf))
    # pressure lines forward into the fuselage spine, inside the root
    for dy in (-r_b * 0.85, r_b * 0.85):
        parts.append(mesh.pipe(
            [(hinge_x - r_b * 3.0, yc + dy, z - r_b * 2.4),
             (V["x_root_le"] - c_root * 0.18, yc, V["z_root"] + 2.0)],
            r_b * 0.2, 10))
    return {"rudder_servo": mesh.join(*parts)}


def _fin_tip_fairing(ru):
    """ECM/antenna fairing on the fin tip.

    Fin-tip volume is free volume: it is above the boundary layer the fin
    works in and costs nothing the fin did not already pay, which is why
    emitter packages go there. The teardrop section is the shape the
    fairing has to be, not a choice -- a cylinder of the same thickness
    would spoil the fin's tip drag for the sake of housing a transceiver.
    """
    tip_chord = V["tip_chord"]
    # leading edge of the fin tip
    x_le = V["x_root_le"] + V["height"] * math.tan(
        math.radians(V["sweep_le"]))
    z_tip = V["z_root"] + V["height"]
    path = [(x_le + tip_chord * 0.55, 0.0, z_tip + 0.6),
            (x_le + tip_chord * 0.60, 0.0, z_tip + 1.1),
            (x_le + tip_chord * 0.60, 0.0, z_tip + 1.5)]
    v, f = shapes.fairing(path, tip_chord * 0.52, thickness=0.34, n_sec=12)
    # tuck it down onto the fin tip
    v = [(x, y, z - 0.4) for (x, y, z) in v]
    return {"fin_tip_ecm_fairing": (v, f)}


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
            camber=0.0, tip_cap=10,
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
