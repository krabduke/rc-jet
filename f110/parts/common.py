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


def _inner_shroud(row, height=8.0, n_seg=6, clearance=3.0):
    """Stator inner shroud, carrying the interstage air seal.

    A stator's inner shroud runs just above the rotating drum on a labyrinth
    seal. Hung a fixed height below the flowpath line it goes straight into
    the drum instead, so the lower edge is clamped to clear whatever the drum
    is doing at this station.
    """
    r1 = min(row.r_hub_le, row.r_hub_te)
    lo = r1 - height
    floor = hpc_drum_radius_max(row.x - row.chord * 0.12,
                                row.x + row.chord * 1.12)
    if floor is not None:
        lo = max(lo, floor + clearance)
    if lo >= r1 - 0.5:
        lo = r1 - 0.5
    return _sector_band(row, lo, r1, n_seg)


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


def shell_profile(x0, x1, r0, r1, wall, ribs=True, n_rib=None):
    """Meridional loop for a casing can or a flowpath shell.

    A casing is not a tube. It is a rolled and machined can with a thick
    bolting land at each end, a wall that steps where the pressure does, and
    hoop stiffeners in between to keep it round under case load. Built as a
    straight taper it reads as turned bar stock -- which is exactly what every
    shell in this engine was: two x-stations each, no feature anywhere along.

    The inner surface stays exactly on the taper, because it is the flowpath
    and the blade tip clearances are set from it. Everything added is outside.

    `ribs=False` for a shell whose outer surface is itself an aerodynamic
    surface -- the bypass duct inner wall, the inlet case -- where hoop
    stiffeners would sit in the airflow instead of behind it.

    Stiffener height is held under 4 mm on purpose: the bolted access panels
    in accessories.py sit on the nominal outer radius with a 5 mm lip, and a
    rib taller than that lip would stand through them.
    """
    L = x1 - x0
    r_in = lambda x: r0 + (r1 - r0) * (x - x0) / L
    land = wall * 0.85
    rib = min(4.0, wall * 0.30)
    step = wall * 0.12

    outer = [(0.000, land), (0.030, land), (0.055, step)]
    if ribs:
        n = n_rib if n_rib is not None else max(3, int(abs(L) / 320.0))
        for i in range(n):
            f = 0.14 + (0.72 * (i + 0.5) / n)
            base = step if f < 0.45 else 0.0
            outer += [(f - 0.013, base), (f - 0.009, base + rib),
                      (f + 0.009, base + rib), (f + 0.013, base)]
    else:
        outer += [(0.45, step), (0.47, 0.0)]
    outer += [(0.945, 0.0), (0.970, land), (1.000, land)]
    outer.sort()

    prof = [(x0, r_in(x0)), (x1, r_in(x1))]
    for (f, extra) in reversed(outer):
        x = x0 + L * f
        prof.append((x, r_in(x) + wall + extra))
    return prof


def shaft_profile(x0, x1, r_in, r_out, journals=(), flange_at=None):
    """Meridional loop for a spool shaft.

    A spool is not a plain tube. It carries a bearing journal wherever a
    bearing sits, a spline at each drive end, a curvic coupling flange where
    it bolts to a disc, and a bore that steps to keep the section where the
    torque is. Both shafts in this engine were single `mesh.tube` calls: two
    x-stations, constant wall, nothing to say which end drove what.

    `journals` are fractions along the shaft that carry a raised bearing land.
    """
    L = x1 - x0
    spl = (r_out - r_in) * 0.34          # spline land height
    jr = (r_out - r_in) * 0.26           # journal land height

    outer = [(0.000, spl), (0.022, spl), (0.030, 0.0)]
    for f in journals:
        outer += [(f - 0.020, 0.0), (f - 0.015, jr),
                  (f + 0.015, jr), (f + 0.020, 0.0)]
    if flange_at is not None:
        outer += [(flange_at - 0.010, 0.0), (flange_at - 0.006, spl * 2.1),
                  (flange_at + 0.006, spl * 2.1), (flange_at + 0.010, 0.0)]
    # the section steps down aft of mid-span, where the torque has been taken
    outer += [(0.56, 0.0), (0.60, -(r_out - r_in) * 0.13),
              (0.965, -(r_out - r_in) * 0.13), (0.970, spl), (1.000, spl)]
    outer.sort()

    # bore steps with it, so the wall stays roughly constant
    inner = [(0.000, 0.0), (0.58, 0.0), (0.62, -(r_out - r_in) * 0.10),
             (1.000, -(r_out - r_in) * 0.10)]

    prof = [(x0 + L * f, r_in + d) for (f, d) in inner]
    for (f, extra) in reversed(outer):
        prof.append((x0 + L * f, r_out + extra))
    return prof


def platform_h(row):
    """Height of the platform a rotor blade stands on."""
    return max(4.0, min(12.0, (row.r_tip_le - row.r_hub_le) * 0.06))


def hpc_drum_profile():
    """(x, r) along the HP compressor drum's outer surface.

    One definition, used by rotating.py to build the drum and by the stator
    inner shrouds to clear it. When the two carried their own idea of where
    the drum was, the shrouds hung a fixed 8 mm below the flowpath hub line
    and the drum sat just below the same line -- so every HPC stator ran
    through the rotating drum, by up to 23 mm.
    """
    rotors = [r for r in spec.HPC_ROWS if r.rotor]
    # The drum surface stops at the compressor exit. The last rotor's segment
    # ran 1.25 chords past it to 1583, and the diffuser begins at 1540 and the
    # combustor dome at 1556 -- so the drum ran into both.
    aft = spec.STATION["hpc_exit"] - 4.0
    prof = []
    for row in rotors:
        rim = min(row.r_hub_le, row.r_hub_te) - platform_h(row)
        prof.append((min(row.x - row.chord * 0.25, aft), rim))
        prof.append((min(row.x + row.chord * 1.25, aft), rim))
    return prof


def hpc_drum_radius_max(x0, x1):
    """Greatest drum radius anywhere between two stations.

    Taking the radius at the row's centre is not enough: the drum steps up
    between one rotor and the next, so across a 58 mm stator shroud it can
    climb 10 mm. Clamping on the centre value left the shroud 6 mm inside the
    drum at its aft end.
    """
    prof = hpc_drum_profile()
    if x1 < prof[0][0] or x0 > prof[-1][0]:
        return None
    best = None
    for x in (x0, x1):
        r = hpc_drum_radius_at(min(max(x, prof[0][0]), prof[-1][0]))
        if r is not None:
            best = r if best is None else max(best, r)
    for (px, pr) in prof:
        if x0 <= px <= x1:
            best = pr if best is None else max(best, pr)
    return best


def hpc_drum_radius_at(x):
    """Drum outer radius at station x, or None if x is off the drum."""
    prof = hpc_drum_profile()
    if x < prof[0][0] or x > prof[-1][0]:
        return None
    for (x0, r0), (x1, r1) in zip(prof, prof[1:]):
        if x0 <= x <= x1:
            if x1 - x0 < 1e-9:
                return r0
            return r0 + (r1 - r0) * (x - x0) / (x1 - x0)
    return prof[-1][1]
