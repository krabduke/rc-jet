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

# these full-size relief dimensions belong in spec.py: seams 6 mm wide,
# rivets 4.6 mm across and 0.33 mm proud, straps 66 mm wide and 2 mm proud,
# doublers 1.5 mm proud; seam relief is only 0.165 mm.
SD = dict(spec.SKIN_DETAIL, seam_w=0.18, seam_h=0.005, screw_r=0.07)
RIVET_HEIGHT = 0.01
STRAP_WIDTH = 2.0
STRAP_HEIGHT = 0.06
DOUBLER_HEIGHT = 0.045
W = spec.WING
V = spec.VTAIL
SEG = spec.RES["fuse_sections"]


def build():
    out = {}
    out.update(_circumferential_seams())
    out.update(_lengthwise_seams())
    out.update(_screws())
    out.update(_wing_seams())
    out.update(_doublers())
    return out


# --------------------------------------------------------------------------

def _proud_ring(x, h, segments=SEG):
    """The skin section at x, pushed out by h. Both scaled together, so the
    raised strip follows the section's shape instead of bulging at the sides."""
    return fus.section_ring(x, inset=-h, segments=segments)


def _surface_point(x, angle, h=0.0):
    """Sample the live skin ring so belly details inherit the inlet fairing."""
    ring = _proud_ring(x, h)
    pos = (angle % 360.0) * len(ring) / 360.0
    i = int(pos)
    f = pos - i
    a, b = ring[i], ring[(i + 1) % len(ring)]
    return tuple(a[k] + f * (b[k] - a[k]) for k in range(3))


def _swept_band(x, half, h, profile):
    """A relief band round the body, swept from a section profile.

    `profile` is a list of (dx, dr) in multiples of the half-width and the
    standoff: dx walks forward to aft across the joint, dr lifts the strip
    off the skin. Both ends sit at dr = 0 so the band dies into the skin
    instead of ending on a wall.
    """
    # The profile is walked as a closed loop: out along the relief and back
    # along the skin, so the band is a watertight ring whose underside lies
    # on the surface. Left open at the two ends it was a tube with two holes
    # in it, and a ray fired at it could come out through either.
    loop = list(profile) + [(dx, 0.0) for (dx, _dr) in
                            reversed(profile[1:-1])]
    rings = [_proud_ring(x + dx * half, dr * h) for (dx, dr) in loop]
    verts = [v for r in rings for v in r]
    n = SEG
    faces = []
    for k in range(len(rings)):
        a0, b0 = k * n, ((k + 1) % len(rings)) * n
        for s in range(n):
            s2 = (s + 1) % n
            faces.append((a0 + s, a0 + s2, b0 + s2, b0 + s))
    return verts, faces


# A moulded composite joint is not a flat band stuck on the skin. The two
# shells butt together, the bond line itself is a shallow groove, and a strap
# is laid over it and faired in -- so the section is a low shelf, a proud
# strap either side, and the groove down the middle. The section splits (at
# the bulkheads, where the fuselage comes apart for transport) get a wider
# strap and a deeper groove than the panel joints.

_JOINT_MINOR = [
    (-1.70, 0.00), (-1.50, 0.30), (-0.55, 0.34), (-0.40, 0.95),
    (-0.16, 1.00), (-0.07, 0.40), (0.07, 0.40), (0.16, 1.00),
    (0.40, 0.95), (0.55, 0.34), (1.50, 0.30), (1.70, 0.00),
]

_JOINT_MAJOR = [
    (-2.60, 0.00), (-2.38, 0.24), (-1.15, 0.30), (-0.92, 0.88),
    (-0.66, 1.00), (-0.26, 1.00), (-0.13, 0.34), (0.13, 0.34),
    (0.26, 1.00), (0.66, 1.00), (0.92, 0.88), (1.15, 0.30),
    (2.38, 0.24), (2.60, 0.00),
]


def _circumferential_seams():
    """One band per production joint, where the fuselage sections butt."""
    out = {}
    splits = [b[1] for b in spec.BULKHEADS]
    for i, x in enumerate(SD["seam_x"], start=1):
        major = any(abs(x - xb) < 4.0 for xb in splits)
        out[f"seam_ring_{i:02d}"] = _swept_band(
            x, (STRAP_WIDTH / 5.2 if major else SD["seam_w"] / 3.4),
            STRAP_HEIGHT if major else SD["seam_h"],
            _JOINT_MAJOR if major else _JOINT_MINOR)
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
        zz = RIVET_HEIGHT * math.sin(phi)
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


# There were 720 rivet heads here, in two rows flanking every circumferential
# seam. This airframe is a moulded composite shell: its production joints are
# bonded, not riveted, and the only fasteners on it are the countersunk screws
# round the access hatches -- which are modelled, in `_panel_screws`. Rows of
# rivets on a bonded joint are a scale-model convention borrowed from sheet
# metal aircraft, and on this one they were 720 objects asserting something
# untrue about how it is made. `verify.py` now fails if they come back.

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
            for i in range(22):
                f = 0.04 + 0.92 * i / 21
                chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
                x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                      W["sweep_le"], f)
                t = W["thickness"] * chord * 0.5
                path.append((x_le + chord * frac, sgn * W["semi_span"] * f,
                             W["z_root"] + t + h))
            # a moulded seam over a spar is a low rounded ridge, not a
            # four-sided rod laid on the skin
            parts.append(shapes.swept_profile(
                path, shapes.rounded_polygon(
                    [(-SD["seam_w"] * 0.5, -h * 0.5),
                     (SD["seam_w"] * 0.5, -h * 0.5),
                     (SD["seam_w"] * 0.34, h * 0.5),
                     (-SD["seam_w"] * 0.34, h * 0.5)],
                    h * 0.42, seg=4)))
    return {"wing_seams": mesh.join(*parts)}


def _ply_pad(x0, x1, a0, a1, h, nx=18, na=20, edge=0.22, plies=3):
    """A doubler laminated onto the skin, with the plies dropping off.

    A composite doubler is a stack of cloth plies, each one smaller than the
    last, so the edge is a staircase that fairs the load into the skin rather
    than a wall that concentrates it. The pad follows the section, so it sits
    down on the surface everywhere.
    """
    def thickness(fx, fa):
        d = min(fx, 1.0 - fx, fa, 1.0 - fa) / edge
        if d >= 1.0:
            return h
        step = math.ceil(max(d, 0.0) * plies) / plies
        return h * max(step, 1.0 / (plies + 2))

    inner, outer = [], []
    for i in range(nx):
        fx = i / (nx - 1)
        x = x0 + (x1 - x0) * fx
        for j in range(na):
            fa = j / (na - 1)
            a = a0 + (a1 - a0) * fa
            inner.append(fus.surface_point(x, a, 0.0))
            outer.append(fus.surface_point(x, a, thickness(fx, fa)))
    verts = inner + outer
    off = len(inner)
    faces = []
    for i in range(nx - 1):
        for j in range(na - 1):
            k = i * na + j
            faces.append((k, k + 1, k + na + 1, k + na))
            faces.append((off + k, off + k + na, off + k + na + 1, off + k + 1))
    for i in range(nx - 1):
        for j in (0, na - 1):
            k = i * na + j
            if j == 0:
                faces.append((k, k + na, off + k + na, off + k))
            else:
                faces.append((k + na, k, off + k, off + k + na))
    for j in range(na - 1):
        for i in (0, nx - 1):
            k = i * na + j
            if i == 0:
                faces.append((k + 1, k, off + k, off + k + 1))
            else:
                faces.append((k, k + 1, off + k + 1, off + k))
    return verts, faces


def _doublers():
    """Reinforcing doublers: gear bay, spar carry-through, fin root.

    Each one is where a point load goes into the shell -- the gear legs, the
    wing carry-through, the fin post -- so each is laid over that spot on the
    skin, not floated under the belly on a bounding box.
    """
    pads = [
        # (x0, x1, angle from, angle to, plies)
        (spec.GEAR["main_x"] - 34.0, spec.GEAR["main_x"] + 34.0, 228.0, 312.0, 4),
        (spec.BULKHEADS[2][1] - 20.0, spec.BULKHEADS[2][1] + 26.0, 236.0, 304.0, 3),
        (V["x_root_le"] - 4.0, V["x_root_le"] + 30.0, 66.0, 114.0, 3),
    ]
    parts = [_ply_pad(x0, x1, a0, a1, spec.FUSELAGE_SKIN * 0.75, plies=n)
             for (x0, x1, a0, a1, n) in pads]
    return {"doublers": mesh.join(*parts)}
