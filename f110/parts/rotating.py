"""Rotating assembly: spinner, fan and HPC rotors, discs, drum, shafts, bearings."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import common

SEG = spec.RES["revolve_segments"]


def build():
    out = {}
    out.update(_spinner())
    out.update(_fan_rotors())
    out.update(_hpc_rotor())
    out.update(_shafts())
    out.update(_bearings())
    return out


# --------------------------------------------------------------------------

def _spinner():
    s = spec.SPINNER
    n = 30
    prof = []
    for i in range(n + 1):
        f = i / n
        x = s["x_nose"] + s["length"] * f
        r = s["tip_radius"] + (s["base_radius"] - s["tip_radius"]) * (f ** s["ogive_power"])
        prof.append((x, r))
    # short cylindrical skirt where it bolts to the fan disc
    prof.append((s["x_nose"] + s["length"] + 40.0, s["base_radius"]))
    v, f = mesh.revolve_open(prof, SEG, cap_start=True, cap_end=False)
    skirt = mesh.tube(s["x_nose"] + s["length"] + 40.0 - 1.0,
                      s["x_nose"] + s["length"] + 40.0,
                      s["base_radius"] - 14.0, s["base_radius"], SEG)
    return {"spinner": mesh.join((v, f), skirt)}


def _platform_h(row):
    return max(4.0, min(12.0, (row.r_tip_le - row.r_hub_le) * 0.06))


def _fan_rotors():
    """Three fan stages: blades, discs, and the conical arms tying them to the
    LP shaft. The spool is a drum-and-disc hybrid, as on the real engine."""
    out = {}
    rows = [r for r in spec.FAN_ROWS if r.rotor]
    lp_r = spec.SHAFTS["lp_outer_r"]
    disc_parts = []

    for row in rows:
        out[f"blades_{row.name}"] = common.build_row(row)
        rim = min(row.r_hub_le, row.r_hub_te) - _platform_h(row)
        xc = row.x + row.chord * 0.5
        bore = lp_r + 16.0
        disc_parts.append(common.disc(xc, bore, rim, row.chord * 0.85))

    # conical spacer arms between adjacent discs, carrying the interstage seals
    for a, b in zip(rows, rows[1:]):
        xa = a.x + a.chord * 1.0
        xb = b.x
        ra = min(a.r_hub_le, a.r_hub_te) - _platform_h(a)
        rb = min(b.r_hub_le, b.r_hub_te) - _platform_h(b)
        disc_parts.append(mesh.cone_tube(xa, xb, ra - 26.0, ra, rb - 26.0, rb, SEG))

    # stub shaft forward of stage 1 to the No.1 bearing
    r0 = min(rows[0].r_hub_le, rows[0].r_hub_te) - _platform_h(rows[0])
    disc_parts.append(mesh.cone_tube(spec.STATION["igv"] - 30.0, rows[0].x,
                                     lp_r, lp_r + 22.0, r0 - 26.0, r0, SEG))
    out["fan_disc_assembly"] = mesh.join(*disc_parts)

    # interstage labyrinth seals on the spool
    seals = []
    for a, b in zip(rows, rows[1:]):
        xa, xb = a.x + a.chord * 1.15, b.x - b.chord * 0.15
        r = min(a.r_hub_te, b.r_hub_le) - _platform_h(a) - 4.0
        if xb > xa:
            seals.append(common.labyrinth_seal(xa, xb, r, 4, 6.0, 3.0, SEG))
    if seals:
        out["fan_interstage_seals"] = mesh.join(*seals)
    return out


def _hpc_rotor():
    """Nine-stage HP compressor. A welded drum rotor rather than stacked discs:
    the hub line rises only 19 mm across the whole machine, so a drum is both
    lighter and what GE actually uses here."""
    out = {}
    rotors = [r for r in spec.HPC_ROWS if r.rotor]
    for row in rotors:
        out[f"blades_{row.name}"] = common.build_row(row)

    hp_r = spec.SHAFTS["hp_outer_r"]
    bore = hp_r + 15.0
    prof_outer = []
    for row in rotors:
        rim = min(row.r_hub_le, row.r_hub_te) - _platform_h(row)
        prof_outer.append((row.x - row.chord * 0.25, rim))
        prof_outer.append((row.x + row.chord * 1.25, rim))

    x_front = rotors[0].x - rotors[0].chord * 0.9
    x_rear = rotors[-1].x + rotors[-1].chord * 1.9

    # A drum is a drum: a shell under the blade platforms, 26 mm thick. It was
    # modelled solid all the way down to the shaft bore, which left nothing
    # for the cones to do -- and so the front cone came out 0.6 mm long and
    # 340 mm across, a washer with a cone's name on it.
    wall = 26.0
    inner = [(x, r - wall) for (x, r) in reversed(prof_outer)]
    rim0 = prof_outer[0][1]
    rim1 = prof_outer[-1][1]
    prof = ([(x_front, rim0)] + prof_outer + [(x_rear, rim1),
             (x_rear, rim1 - wall)] + inner + [(x_front, rim0 - wall)])
    out["hpc_drum"] = mesh.revolve_closed(prof, SEG)

    # The cones are the conical webs that actually carry the drum on the
    # shaft: each one runs from the drum's inner surface down and away to the
    # shaft OD at its bearing, which is a 180 mm radial drop over about the
    # same axially. That is a cone.
    # Both webs slope aft-and-inboard from the drum bore to the shaft, over
    # 150 mm for a 157 mm radial drop -- a 46 degree cone. The No.3 thrust
    # bearing then sits on the stub of shaft that cantilevers forward of the
    # front web, which is where it goes.
    web = 14.0
    run = 150.0
    # The drive cones carry the whole HP compressor torque into the shaft, so
    # they are not plain sheet: each has a thickened rim land where it bolts to
    # the drum, a thickened hub land at the shaft, a scalloped balance flange,
    # and a web that thins between the two. Modelled as cone_tube they were
    # two x-stations of constant thickness.
    def _cone(xa, xb, ra, rb, ta, tb, flange_at):
        L = xb - xa
        r_at = lambda f: ra + (rb - ra) * f
        t_at = lambda f: ta + (tb - ta) * f
        outer = [(0.000, t_at(0.0) * 1.9), (0.055, t_at(0.055) * 1.9),
                 (0.085, t_at(0.085))]
        outer += [(flange_at - 0.035, t_at(flange_at) * 1.0),
                  (flange_at - 0.020, t_at(flange_at) * 2.4),
                  (flange_at + 0.020, t_at(flange_at) * 2.4),
                  (flange_at + 0.035, t_at(flange_at) * 1.0)]
        outer += [(0.915, t_at(0.915)), (0.945, t_at(0.945) * 1.9),
                  (1.000, t_at(1.0) * 1.9)]
        outer.sort()
        prof = [(xa, r_at(0.0)), (xb, r_at(1.0))]
        for (f, t) in reversed(outer):
            prof.append((xa + L * f, r_at(f) + t))
        return mesh.revolve_closed(prof, segments=SEG)

    out["hpc_front_cone"] = _cone(
        x_front, x_front + run, rim0 - wall - web, hp_r, web, web, 0.42)
    out["hpc_rear_cone"] = _cone(
        x_rear - run, x_rear, hp_r, rim1 - wall - web, web, web, 0.58)

    seals = []
    for a, b in zip(rotors, rotors[1:]):
        xa, xb = a.x + a.chord * 1.3, b.x - b.chord * 0.3
        if xb > xa + 3:
            r = min(a.r_hub_te, b.r_hub_le) - _platform_h(a) - 3.0
            seals.append(common.labyrinth_seal(xa, xb, r, 3, 4.0, 2.0, SEG))
    out["hpc_interstage_seals"] = mesh.join(*seals)
    return out


def _shafts():
    s = spec.SHAFTS
    # journal fractions follow where the bearings actually sit on each spool
    return {
        "shaft_lp": mesh.revolve_closed(
            common.shaft_profile(s["lp_x0"], s["lp_x1"],
                                 s["lp_inner_r"], s["lp_outer_r"],
                                 journals=(0.09, 0.26, 0.93),
                                 flange_at=0.80), segments=72),
        "shaft_hp": mesh.revolve_closed(
            common.shaft_profile(s["hp_x0"], s["hp_x1"],
                                 s["hp_inner_r"], s["hp_outer_r"],
                                 journals=(0.07, 0.88),
                                 flange_at=0.20), segments=72),
    }


def _bearings():
    """Five main bearings. Each is an inner race, an outer race and its rolling
    elements -- balls for the thrust bearings, rollers for the rest."""
    out = {}
    for (name, x, r_shaft, r_house, kind) in spec.BEARINGS:
        race_w = 46.0 if kind == "ball" else 38.0
        inner = mesh.tube(x - race_w / 2, x + race_w / 2,
                          r_shaft, r_shaft + 16.0, 48)
        outer = mesh.tube(x - race_w / 2, x + race_w / 2,
                          r_house - 16.0, r_house, 48)
        r_mid = (r_shaft + 16.0 + r_house - 16.0) / 2
        gap = (r_house - 16.0) - (r_shaft + 16.0)
        er = max(gap * 0.5, 3.0)
        n_el = max(10, int(2.0 * math.pi * r_mid / (er * 2.6)))

        if kind == "ball":
            el_v, el_f = mesh.revolve_open(
                [(-er, 0.001)] + [(-er * math.cos(math.pi * i / 10),
                                   er * math.sin(math.pi * i / 10))
                                  for i in range(1, 10)] + [(er, 0.001)],
                14, cap_start=True, cap_end=True)
        else:
            el_v, el_f = mesh.cylinder(-race_w * 0.34, race_w * 0.34, er, 14)
        el_v = mesh.translate(el_v, x, r_mid, 0.0)
        elements = mesh.replicate(el_v, el_f, n_el)

        cage = mesh.tube(x - race_w * 0.36, x + race_w * 0.36,
                         r_mid - er * 0.25, r_mid + er * 0.25, 48)
        out[name] = mesh.join(inner, outer, elements, cage)
    return out
