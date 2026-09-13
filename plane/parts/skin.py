"""Skin relief: panel seams, fastener rows, and the doubler plates.

Full-size aircraft are assembled from panels, and the seams and fastener rows
are most of what the eye reads as "aircraft" at a distance. They are modelled
as raised relief on the skin rather than cut into it, which keeps the shell
watertight for printing while still catching a highlight in the viewer.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common, fuselage as fus

SD = spec.SKIN_DETAIL
W = spec.WING
V = spec.VTAIL
SEG = spec.RES["fuse_sections"]


def build():
    out = {}
    out.update(_circumferential_seams())
    out.update(_lengthwise_seams())
    out.update(_rivets())
    out.update(_screws())
    out.update(_wing_seams())
    out.update(_doublers())
    return out


# --------------------------------------------------------------------------

def _proud_ring(x, h, segments=SEG):
    """The skin section at x, pushed out by h. Both scaled together, so the
    raised strip follows the section's shape instead of bulging at the sides."""
    w, hh, zc, n = fus.station_at(x)
    p = 2.0 / n
    ring = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        y = (w + h) * math.copysign(abs(ca) ** p, ca)
        z = (hh + h) * math.copysign(abs(sa) ** p, sa)
        ring.append((x, y, zc + z))
    return ring


def _band(x, width, h):
    """A raised band around the body: skin ring, out, along, back in."""
    half = width / 2
    inner_f = fus.section_ring(x - half)
    inner_b = fus.section_ring(x + half)
    outer_f = _proud_ring(x - half, h)
    outer_b = _proud_ring(x + half, h)
    verts = inner_f + inner_b + outer_f + outer_b
    n = SEG
    i_f, i_b, o_f, o_b = 0, n, 2 * n, 3 * n
    faces = []
    for s in range(n):
        s2 = (s + 1) % n
        faces.append((i_f + s, i_f + s2, o_f + s2, o_f + s))   # front wall
        faces.append((o_b + s, o_b + s2, i_b + s2, i_b + s))   # rear wall
        faces.append((o_f + s, o_f + s2, o_b + s2, o_b + s))   # crown
    return verts, faces


def _circumferential_seams():
    """One band per production joint, where the fuselage sections butt."""
    out = {}
    for i, x in enumerate(SD["seam_x"], start=1):
        out[f"seam_ring_{i:02d}"] = _band(x, SD["seam_w"], SD["seam_h"])
    return out


def _lengthwise_seams():
    """Longitudinal seams over the longerons, top and bottom, both sides."""
    parts = []
    h, wdt = SD["seam_h"], SD["seam_w"]
    x0 = spec.FUSELAGE[1][0]
    x1 = spec.FUSELAGE[-2][0]
    for ang in (0.0, 90.0, 180.0, 270.0):
        a = math.radians(ang)
        ca, sa = math.cos(a), math.sin(a)
        path = []
        for i in range(36):
            x = x0 + (x1 - x0) * i / 35
            w, hh, zc, n = fus.station_at(x)
            p = 2.0 / n
            y = (w + h * 0.5) * math.copysign(abs(ca) ** p, ca)
            z = (hh + h * 0.5) * math.copysign(abs(sa) ** p, sa)
            path.append((x, y, zc + z))
        parts.append(mesh.pipe(path, wdt * 0.5, 4))
    return {"seam_lengthwise": mesh.join(*parts)}


def _dome(cx, cy, cz, r, segments=8, rings=3):
    """A low hemisphere -- one rivet head."""
    verts, faces = [], []
    for j in range(rings + 1):
        phi = (math.pi / 2) * j / rings
        rr = r * math.cos(phi)
        zz = r * 0.55 * math.sin(phi)
        for i in range(segments):
            a = 2 * math.pi * i / segments
            verts.append((cx + rr * math.cos(a), cy + rr * math.sin(a), cz + zz))
    for j in range(rings):
        a0, b0 = j * segments, (j + 1) * segments
        for i in range(segments):
            i2 = (i + 1) % segments
            faces.append((a0 + i, a0 + i2, b0 + i2, b0 + i))
    faces.append(tuple(range(rings * segments, (rings + 1) * segments)))
    faces.append(tuple(reversed(range(segments))))
    return verts, faces


def _rivets():
    """Rivet rows flanking every circumferential seam.

    Two rows per joint, as on a real lap joint -- one each side of the seam.
    """
    parts = []
    n = SD["rivets_per_ring"]
    r = SD["rivet_r"]
    for x in SD["seam_x"]:
        for dx in (-SD["seam_w"] - 1.6, SD["seam_w"] + 1.6):
            ring = _proud_ring(x + dx, r * 0.3, segments=n)
            w, hh, zc, expn = fus.station_at(x + dx)
            for (px, py, pz) in ring:
                # orient the head outward by offsetting along the radius
                d = math.hypot(py, pz - zc) or 1.0
                v, f = _dome(0.0, 0.0, 0.0, r)
                # rotate so +z of the dome points along the outward radius
                cy, cz = py / d, (pz - zc) / d
                rot = []
                for (vx, vy, vz) in v:
                    rot.append((px + vx,
                                py + vy * cz + vz * cy,
                                pz - vy * cy + vz * cz))
                parts.append((rot, f))
    return {"rivets": mesh.join(*parts)}


def _screws():
    """Countersunk fasteners round the edge of every access panel.

    Placed on the panel's own conformal outline, so the screw row hugs the
    body the way the panel does.
    """
    parts = []
    r = SD["screw_r"]
    per = max(SD["screws_per_panel"] // 2, 3)
    for (_, x0, x1, a0, a1, h) in SD["panels"]:
        for i in range(per):
            f = i / (per - 1)
            for a in (a0, a1):
                cx, cy, cz = fus.surface_point(x0 + (x1 - x0) * f, a, h)
                parts.append(_dome(cx, cy, cz, r, 6, 2))
            for x in (x0, x1):
                cx, cy, cz = fus.surface_point(x, a0 + (a1 - a0) * f, h)
                parts.append(_dome(cx, cy, cz, r, 6, 2))
    return {"panel_screws": mesh.join(*parts)}


def _wing_seams():
    """Seam over each wing spar, and the skin joint at the strake."""
    parts = []
    h = SD["seam_h"]
    for frac in (spec.SPAR["x_frac"], spec.STRUCTURE["rear_spar_frac"]):
        for sgn in (-1.0, 1.0):
            path = []
            for i in range(10):
                f = 0.04 + 0.92 * i / 9
                chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
                x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                      W["sweep_le"], f)
                t = W["thickness"] * chord * 0.5
                path.append((x_le + chord * frac, sgn * W["semi_span"] * f,
                             W["z_root"] + t + h))
            parts.append(mesh.pipe(path, SD["seam_w"] * 0.4, 4))
    return {"wing_seams": mesh.join(*parts)}


def _doublers():
    """Reinforcing doubler plates: gear bay, spar carry-through, tail joint."""
    parts = []
    for (x, lx, ly) in ((spec.GEAR["main_x"], 70.0, 104.0),
                        (spec.BULKHEADS[2][1], 44.0, 58.0),
                        (V["x_root_le"] + 12.0, 62.0, 26.0)):
        w, hh, zc, _ = fus.station_at(x)
        parts.append(shapes.rounded_box(x, 0.0, zc - hh - 0.4, lx, min(ly, w * 1.9), 1.0))
    return {"doublers": mesh.join(*parts)}
