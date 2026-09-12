"""Shared builders used by more than one part module."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import airfoil


def build_row(row, platform=True):
    """A complete blade row: one lofted airfoil plus its platform (and tip
    shroud where the row is shrouded), replicated around the annulus.

    Rotor rows get a hub platform; stator rows get an outer band, because that
    is how each is actually carried -- rotors off the disc, stators off the case.
    """
    bv, bf = airfoil.blade_mesh(row,
                                spec.RES["airfoil_chord_pts"],
                                spec.RES["airfoil_span_pts"])
    parts = [(bv, bf)]

    if platform:
        if row.rotor:
            h = max(4.0, min(12.0, (row.r_tip_le - row.r_hub_le) * 0.06))
            parts.append(airfoil.blade_platform(row, height=h))
        else:
            parts.append(_outer_band(row))
            parts.append(_inner_shroud(row))

    if row.shrouded:
        parts.append(airfoil.tip_shroud(row))

    one_v, one_f = mesh.join(*parts)
    return mesh.replicate(one_v, one_f, row.count)


def _outer_band(row, height=10.0, n_seg=6):
    """Stator outer band -- the sector of casing ring this vane hangs from."""
    r0 = max(row.r_tip_le, row.r_tip_te)
    return _sector_band(row, r0, r0 + height, n_seg)


def _inner_shroud(row, height=8.0, n_seg=6):
    """Stator inner shroud, carrying the interstage air seal."""
    r1 = min(row.r_hub_le, row.r_hub_te)
    return _sector_band(row, r1 - height, r1, n_seg)


def _sector_band(row, r_lo, r_hi, n_seg=6):
    dphi = 2.0 * math.pi / row.count * 0.98
    x0 = row.x - row.chord * 0.12
    x1 = row.x + row.chord * 1.12
    prof = [(x0, r_lo), (x1, r_lo), (x1, r_hi), (x0, r_hi)]
    verts, faces = [], []
    n = len(prof)
    for k in range(n_seg + 1):
        a = -dphi / 2 + dphi * k / n_seg
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in prof:
            verts.append((x, r * ca, r * sa))
    for k in range(n_seg):
        b0, b1 = k * n, (k + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = n_seg * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def disc(x_centre, r_bore, r_rim, width_rim, width_web=None, width_bore=None,
         segments=96):
    """A turbomachinery disc: thick bore, thin web, thicker rim.

    The classic hourglass meridional section -- mass at the bore to carry hoop
    stress, mass at the rim to carry the blades, as little as possible between.
    """
    width_web = width_web if width_web is not None else width_rim * 0.42
    width_bore = width_bore if width_bore is not None else width_rim * 1.25
    r_web0 = r_bore + (r_rim - r_bore) * 0.22
    r_web1 = r_bore + (r_rim - r_bore) * 0.80

    hb, hw, hr = width_bore / 2, width_web / 2, width_rim / 2
    x = x_centre
    prof = [
        (x - hb, r_bore), (x + hb, r_bore),
        (x + hw, r_web0), (x + hw, r_web1),
        (x + hr, r_rim),  (x - hr, r_rim),
        (x - hw, r_web1), (x - hw, r_web0),
    ]
    return mesh.revolve_closed(prof, segments)


def labyrinth_seal(x0, x1, r, n_fins=4, fin_h=5.0, fin_w=2.5, segments=64):
    """Knife-edge air seal -- a land with a row of thin fins."""
    parts = [mesh.tube(x0, x1, r - 4.0, r, segments)]
    if n_fins > 1:
        for i in range(n_fins):
            fx = x0 + (x1 - x0) * (i + 0.5) / n_fins
            parts.append(mesh.tube(fx - fin_w / 2, fx + fin_w / 2,
                                   r, r + fin_h, segments))
    return mesh.join(*parts)
