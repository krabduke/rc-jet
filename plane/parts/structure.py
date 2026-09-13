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
    return out


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
        out[f"former_{i:02d}"] = _ring_prism(f, b, _shrink(f, zc, k),
                                             _shrink(b, zc, k))
    return out


def _skin_path(angle_deg, standoff, x0=None, x1=None, n=40):
    """Follow the inside of the skin at a fixed clock angle."""
    xa = spec.FUSELAGE[0][0] + 6.0 if x0 is None else x0
    xb = spec.FUSELAGE[-1][0] - 4.0 if x1 is None else x1
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    path = []
    for i in range(n):
        x = xa + (xb - xa) * i / (n - 1)
        w, h, zc, expn = fus.station_at(x)
        p = 2.0 / expn
        w = max(w - spec.FUSELAGE_SKIN - standoff, 0.6)
        h = max(h - spec.FUSELAGE_SKIN - standoff, 0.6)
        y = w * math.copysign(abs(ca) ** p, ca)
        z = h * math.copysign(abs(sa) ** p, sa)
        path.append((x, y, zc + z))
    return path


def _longerons():
    """Four main longerons at the section corners, where a slab-sided body is
    stiffest, running nose to tail."""
    out = {}
    r = ST["longeron_r"]
    for i, ang in enumerate((42.0, 138.0, 222.0, 318.0), start=1):
        out[f"longeron_{i}"] = mesh.pipe(_skin_path(ang, r), r, 6)
    return out


def _stringers():
    """A dozen thin stringers between the longerons, so the skin has something
    to sit on between frames. One object -- they are never handled singly."""
    r = ST["stringer_r"]
    parts = []
    n = ST["n_stringers"]
    for i in range(n):
        ang = 360.0 * i / n + 15.0
        parts.append(mesh.pipe(_skin_path(ang, r, n=28), r, 4))
    return {"stringers": mesh.join(*parts)}


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


def _rib(y, f, t, u1):
    """One wing rib: the local aerofoil, extruded in span, lightened."""
    chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
    x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
    tw = math.radians(-W["washout"] * f)
    sect = common.section_arc(28, W["thickness"], W["camber"], 0.0, u1)
    sect = _inset(sect, chord, ST["rib_inset"], 0.0, u1)
    ct, st = math.cos(tw), math.sin(tw)
    pivot = 0.25

    def place(u, v, yy):
        du = (u - pivot) * chord
        dv = v * chord
        return (x_le + pivot * chord + du * ct - dv * st,
                yy, W["z_root"] + du * st + dv * ct)

    front = [place(u, v, y - t / 2) for (u, v) in sect]
    back = [place(u, v, y + t / 2) for (u, v) in sect]
    # the cut-out is the same aerofoil shrunk about the half-chord: cap strips
    # stay full width top and bottom, which is how a real rib is lightened
    cu, cv = 0.5 * u1, 0.0
    k = ST["rib_hole"]
    hf = [place(cu + (u - cu) * k, cv + (v - cv) * k, y - t / 2)
          for (u, v) in sect]
    hb = [place(cu + (u - cu) * k, cv + (v - cv) * k, y + t / 2)
          for (u, v) in sect]
    return _ring_prism(front, back, hf, hb)


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
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        for i in range(n):
            f = 0.06 + 0.90 * i / (n - 1)
            y = sgn * W["semi_span"] * f
            out[f"rib_{side}_{i + 1:02d}"] = _rib(y, f, t, u1)
    return out


def _rear_spar():
    """A second, lighter spar at 70% chord -- this is the flaperon hinge
    backing, which the front spar is too far forward to provide."""
    frac = ST["rear_spar_frac"]
    tip = ST["rear_spar_span"]
    pts = []
    for i in range(9):
        f = tip * (-1.0 + 2.0 * i / 8)
        af = abs(f)
        chord = common.local_chord(W["root_chord"], W["tip_chord"], af)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], af)
        pts.append((x_le + chord * frac, W["semi_span"] * f, W["z_root"]))
    return {"spar_rear": mesh.pipe(pts, ST["rear_spar_r"], 8)}


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
    sect0 = common.section_arc(22, V["thickness"], 0.0, 0.0, u1)
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

        out[f"fin_rib_{i + 1}"] = _ring_prism(
            ring(sect, -t / 2), ring(sect, t / 2),
            ring(hole, -t / 2), ring(hole, t / 2))
    return out

# --------------------------------------------------------------------------
# Hinges
# --------------------------------------------------------------------------

def _piano_hinge(p0, p1, r, knuckles):
    """A hinge pin with knuckles alternating along it."""
    parts = [mesh.pipe([p0, p1], r * 0.45, 8)]
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
