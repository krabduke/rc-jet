"""HP and LP turbines: nozzle guide vanes, rotor blades, discs, shaft cones,
blade outer air seals, and real film-cooling holes in the hot rows.

Cooled rows are built as a single prototype blade, perforated with a boolean,
and only then arrayed around the disc -- so 27 holes are cut once instead of
72 times. ARRAYS tells assemble.py which prototypes to replicate and by how many.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import airfoil
from parts import common

SEG = spec.RES["revolve_segments"]
ARRAYS = {}


def build():
    ARRAYS.clear()
    out = {}
    out.update(_blade_rows())
    out.update(_discs())
    out.update(_seals_and_shrouds())
    return out


def _blade_rows():
    out = {}
    for row in spec.HPT_ROWS + spec.LPT_ROWS:
        if row.cooled:
            out.update(_cooled_row(row))
        else:
            out[f"blades_{row.name}"] = common.build_row(row)
    return out


def _cooled_row(row):
    """One prototype blade plus its cooling-hole cutters, to be arrayed after
    the boolean is applied."""
    name = f"blades_{row.name}"
    parts = [airfoil.blade_mesh(row,
                                spec.RES["airfoil_chord_pts"],
                                spec.RES["airfoil_span_pts"])]
    if row.rotor:
        parts.append(airfoil.blade_platform(row, height=9.0))
    else:
        parts.append(common._outer_band(row))
        parts.append(common._inner_shroud(row))
    if row.shrouded:
        parts.append(airfoil.tip_shroud(
            row, thickness=spec.SHROUD_THICKNESS,
            standoff=spec.SHROUD_STANDOFF))

    cutters = []
    for (p, d) in airfoil.cooling_hole_positions(row, n_rows=3, n_per_row=9):
        L = row.chord * 0.9
        path = [tuple(p[k] - d[k] * L for k in range(3)),
                tuple(p[k] + d[k] * L for k in range(3))]
        cutters.append(mesh.pipe(path, 0.9, 10))

    # trailing-edge ejection slots
    for k in range(7):
        f = 0.15 + 0.70 * k / 6
        r = airfoil._lerp(row.r_hub_te, row.r_tip_te, f)
        x = row.x + row.chord * 0.97
        sv, sf = mesh.box(x, r, 0.0, row.chord * 0.12,
                          (row.r_tip_te - row.r_hub_te) * 0.07, 1.8)
        cutters.append((sv, sf))

    ARRAYS[name] = row.count
    return {name: mesh.join(*parts), f"cut:{name}": mesh.join(*cutters)}


def _platform_h(row):
    return 9.0


def _discs():
    """Turbine discs. The HPT disc is the most highly stressed part in the
    engine -- deep bore, short web, heavy rim with fir-tree blade slots."""
    out = {}
    hp = spec.SHAFTS
    parts_hp, parts_lp = [], []

    hpt = spec.HPT_ROWS[1]
    rim = min(hpt.r_hub_le, hpt.r_hub_te) - _platform_h(hpt)
    xc = hpt.x + hpt.chord * 0.5
    parts_hp.append(common.disc(xc, hp["hp_outer_r"] + 22.0, rim,
                                hpt.chord * 1.05, hpt.chord * 0.5,
                                hpt.chord * 1.45))
    parts_hp.append(mesh.cone_tube(
        spec.STATION["combustor_exit"] - 60.0, xc - hpt.chord * 0.6,
        hp["hp_outer_r"], hp["hp_outer_r"] + 20.0,
        hp["hp_outer_r"] + 22.0, hp["hp_outer_r"] + 44.0, SEG))
    out["hpt_disc"] = mesh.join(*parts_hp)

    lp = spec.SHAFTS
    for i, row in enumerate([r for r in spec.LPT_ROWS if r.rotor]):
        rim = min(row.r_hub_le, row.r_hub_te) - _platform_h(row)
        xc = row.x + row.chord * 0.5
        parts_lp.append(common.disc(xc, lp["lp_outer_r"] + 34.0, rim,
                                    row.chord * 1.0, row.chord * 0.46,
                                    row.chord * 1.4))
    # drum linking the two LPT discs, and the cone down to the LP shaft
    a, b = [r for r in spec.LPT_ROWS if r.rotor]
    ra = min(a.r_hub_le, a.r_hub_te) - _platform_h(a)
    rb = min(b.r_hub_le, b.r_hub_te) - _platform_h(b)
    parts_lp.append(mesh.cone_tube(a.x + a.chord, b.x,
                                   ra - 24.0, ra, rb - 24.0, rb, SEG))
    parts_lp.append(mesh.cone_tube(
        b.x + b.chord * 1.2, spec.STATION["lpt_exit"] + 90.0,
        rb - 24.0, rb, lp["lp_outer_r"], lp["lp_outer_r"] + 26.0, SEG))
    out["lpt_disc_assembly"] = mesh.join(*parts_lp)
    return out


def _seals_and_shrouds():
    """Blade outer air seals (the shroud segments the rotor tips run against)
    and the interstage air seals between turbine stages."""
    out = {}
    boas = []
    for row in spec.HPT_ROWS + spec.LPT_ROWS:
        if not row.rotor:
            continue
        # The seal fills the space between the running tip and the casing
        # bore, so it has to follow the bore rather than be a straight tube:
        # as a tube of fixed radius it stood 16 mm into the turbine case.
        xa, xb = spec.row_outer_span(row)
        x0, x1 = xa - row.chord * 0.14, xb + row.chord * 0.14
        cas = spec.enclosing_casing(row.x + row.chord * 0.5,
                                    spec.row_outer_r(row, row.x))
        xs = [x0 + (x1 - x0) * i / 6.0 for i in range(7)]
        prof = [(x, spec.row_outer_r(row, x) + spec.TIP_RUB) for x in xs]
        prof += [(x, spec.casing_inner(cas, x) - 0.3) for x in reversed(xs)]
        boas.append(mesh.revolve_ring(prof, SEG))
    out["turbine_blade_outer_air_seals"] = mesh.join(*boas)

    seals = []
    rows = spec.HPT_ROWS + spec.LPT_ROWS
    for a, b in zip(rows, rows[1:]):
        xa, xb = a.x + a.chord * 1.25, b.x - b.chord * 0.25
        if xb <= xa + 2:
            continue
        r = min(a.r_hub_te, b.r_hub_le) - 16.0
        seals.append(common.labyrinth_seal(xa, xb, r, 3, 6.0, 3.0, SEG))
    out["turbine_interstage_seals"] = mesh.join(*seals)

    # the hot-section flowpath inner wall, between the disc rims
    out["turbine_inner_flowpath"] = mesh.revolve_closed(
        common.shell_profile(spec.STATION["hpt_inlet"] - 10.0,
                             spec.STATION["lpt_exit"] + 10.0,
                             296.0, 286.0, 10.0, ribs=False), segments=SEG)
    return out
