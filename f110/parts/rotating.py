"""Rotating assembly: spinner, fan and HPC rotors, discs, drum, shafts, bearings."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
from parts import common

SEG = spec.RES["revolve_segments"]


# --------------------------------------------------------------------------
# Fir-tree blade root
# --------------------------------------------------------------------------

def _fir_tree_half():
    """Left-half profile of a 2-lobe fir-tree root, in (fraction_of_half_width,
    fraction_of_depth) coordinates, going from platform face (top centre) down
    the left side to the root tip (bottom centre)."""
    return [
        (0.00, 0.00),
        (0.12, 0.00),
        (0.12, -0.06),
        (0.38, -0.10),
        (0.38, -0.18),
        (0.10, -0.22),
        (0.08, -0.24),
        (0.34, -0.28),
        (0.34, -0.38),
        (0.06, -0.42),
        (0.04, -0.58),
        (0.00, -0.70),
    ]


def _fir_tree_one(row, platform_h):
    """(verts, faces) for the fir-tree root of one blade.

    The root is a prismatic solid swept axially under the blade platform. It
    extends from just inside the leading edge to just beyond the trailing edge
    and radially inward from the platform floor for ~2.5 platform heights into
    the disc dovetail slot."""
    r0 = min(row.r_hub_le, row.r_hub_te)
    r_top = r0 - platform_h
    depth = platform_h * 2.5
    half_angle = 2.0 * math.pi / row.count * 0.18

    half = _fir_tree_half()
    # Build full closed profile: left side down + right side back up
    full = [(w, d) for (w, d) in half]
    full += [(-w, d) for (w, d) in reversed(half[:-1])]

    x_stations = [
        row.x - row.chord * 0.08,
        row.x + row.chord * 0.08,
        row.x + row.chord * 0.92,
        row.x + row.chord * 1.08,
    ]

    verts = []
    n_pts = len(full)
    for x in x_stations:
        for (fw, fd) in full:
            ang = fw * half_angle
            r = r_top + fd * depth
            ca, sa = math.cos(ang), math.sin(ang)
            verts.append((x, r * ca, r * sa))

    faces = []
    for s in range(len(x_stations) - 1):
        b0, b1 = s * n_pts, (s + 1) * n_pts
        for i in range(n_pts):
            i2 = (i + 1) % n_pts
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
    # close the ends so the root is watertight
    faces.append(tuple(range(n_pts - 1, -1, -1)))
    base = (len(x_stations) - 1) * n_pts
    faces.append(tuple(range(base, base + n_pts)))
    return verts, faces


# --------------------------------------------------------------------------


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
    # A shell with a wall, closed at the aft rim.
    #
    # Left open at the back its rim was an edge with one face on it all the
    # way round, so the spinner bounded no volume. Closed with a fan to the
    # axis instead it became a solid cone, and a solid cone at the front of an
    # engine contains the fan disc, the LP shaft and the front bearing sump --
    # which the intersection audit then reported, correctly. A spinner is a
    # 2 mm moulding bolted to the disc.
    wall = 2.0
    loop = list(prof) + [(x, max(r - wall, 0.8)) for (x, r) in reversed(prof)]
    v, f = mesh.revolve_ring(loop, SEG)
    skirt = mesh.tube(s["x_nose"] + s["length"] + 40.0 - 1.0,
                      s["x_nose"] + s["length"] + 40.0,
                      s["base_radius"] - 14.0, s["base_radius"], SEG)
    return {"spinner": mesh.join((v, f), skirt)}


def _platform_h(row):
    return max(4.0, min(12.0, (row.r_tip_le - row.r_hub_le) * 0.06))


def _slotted_disc(x_centre, r_bore, r_rim, width_rim, n_blades, row, platform_h,
                  segments=96):
    """A disc whose rim carries an integral dovetail slot and a balance land.

    The inner bore and web are a full revolve (no cutouts). The rim is built
    per-blade so every blade slot has a dovetail-shaped cut in the rim. A
    raised balance land sits between and outboard of each slot."""
    width_web = width_rim * 0.42
    width_bore = width_rim * 1.25
    r_web0 = r_bore + (r_rim - r_bore) * 0.22
    r_web1 = r_bore + (r_rim - r_bore) * 0.80

    x = x_centre
    hb, hw, hr = width_bore / 2, width_web / 2, width_rim / 2
    core_prof = [
        (x - hb, r_bore), (x + hb, r_bore),
        (x + hw, r_web0), (x + hw, r_web1),
    ]
    core_v, core_f = mesh.revolve_closed(core_prof, segments)

    # each angular segment is one blade's share of the rim, with its dovetail
    dphi = 2.0 * math.pi / n_blades
    slot_h = platform_h * 2.5
    half_angle = dphi * 0.21
    n_ax = 6

    rim_x = [x - hr, x - hr + 0.1, x + hr - 0.1, x + hr]

    half_raw = _fir_tree_half()
    slot_faces = [(w, -d) for (w, d) in half_raw]
    slot_faces += [(-w, -d) for (w, d) in reversed(half_raw[:-1])]

    rim_verts, rim_faces = [], []
    n_sf = len(slot_faces)

    # build each blade's rim segment
    for k in range(n_blades):
        phi0 = -dphi / 2 + dphi * k
        # inner wall of the rim (constant radius r_web1)
        for ii in range(n_ax):
            f = ii / (n_ax - 1)
            xp = x - hr + (2.0 * hr) * f
            ang = -half_angle
            ca, sa = math.cos(phi0 + ang), math.sin(phi0 + ang)
            rim_verts.append((xp, r_web1 * ca, r_web1 * sa))
            ang = half_angle
            ca, sa = math.cos(phi0 + ang), math.sin(phi0 + ang)
            rim_verts.append((xp, r_web1 * ca, r_web1 * sa))
        # outer wall with dovetail slot
        for ii in range(n_ax):
            f = ii / (n_ax - 1)
            xp = x - hr + (2.0 * hr) * f
            for (fw, fd) in slot_faces:
                ang = fw * half_angle
                r = r_rim + fd * slot_h
                ca, sa = math.cos(phi0 + ang), math.sin(phi0 + ang)
                rim_verts.append((xp, r * ca, r * sa))

    # indices
    stride = 2          # inner wall: 2 verts per axial station
    outer_stride = n_sf  # outer wall: n_sf verts per axial station
    tot_inner = n_blades * stride * n_ax
    tot_seg = stride * n_ax + outer_stride * n_ax

    for k in range(n_blades):
        base = k * tot_seg
        inner0 = base
        outer0 = base + stride * n_ax
        # inner wall faces
        for ii in range(n_ax - 1):
            i0 = inner0 + ii * stride
            i1 = inner0 + (ii + 1) * stride
            rim_faces.append((i0, i0 + 1, i1 + 1, i1))
        # close inner wall ends
        rim_faces.append((inner0,
                          inner0 + stride * (n_ax - 1),
                          inner0 + stride * (n_ax - 1) + 1,
                          inner0 + 1))
        # outer dovetail wall faces
        for ii in range(n_ax - 1):
            o0 = outer0 + ii * outer_stride
            o1 = outer0 + (ii + 1) * outer_stride
            for jj in range(n_sf):
                j2 = (jj + 1) % n_sf
                rim_faces.append((o0 + jj, o0 + j2, o1 + j2, o1 + jj))
        # close outer wall ends
        rim_faces.append(tuple(outer0 + jj for jj in range(n_sf - 1, -1, -1)))
        rim_faces.append(tuple(outer0 + (n_ax - 1) * outer_stride + jj
                               for jj in range(n_sf)))
        # connect inner to outer: two abutment quads per axial station
        for ii in range(n_ax):
            i_base = inner0 + ii * stride
            o_base = outer0 + ii * outer_stride
            # left abutment: inner[0] -> outer[0]
            rim_faces.append((i_base, o_base,
                              o_base + outer_stride * (1 if ii < n_ax - 1 else 1 - n_ax),
                              i_base + stride * (1 if ii < n_ax - 1 else -stride * (n_ax - 1))))
            # right abutment: inner[1] -> outer[-1]
            nxt = (ii + 1) if ii < n_ax - 1 else 0
            rim_faces.append((i_base + 1,
                              o_base + outer_stride * (1 if ii < n_ax - 1 else 1 - n_ax) + n_sf - 1,
                              o_base + n_sf - 1,
                              i_base + stride * (1 if ii < n_ax - 1 else -stride * (n_ax - 1)) + 1))

    # balance lands: raised rings on the rim face between dovetail slots
    land_h = platform_h * 0.45
    land_w = width_rim * 0.30
    land_lx = x - land_w / 2
    land_rx = x + land_w / 2
    land_verts, land_faces = [], []
    for k in range(n_blades):
        phi0 = -dphi / 2 + dphi * k
        ang = 0.9 * (dphi / 2)  # land angular half-width, slot fraction
        for ii in range(n_ax // 2 + 1):
            f = ii / (n_ax // 2)
            xp = land_lx + (land_rx - land_lx) * f
            ca, sa = math.cos(phi0 + ang), math.sin(phi0 + ang)
            land_verts.append((xp, (r_rim + land_h) * ca, (r_rim + land_h) * sa))
            ca, sa = math.cos(phi0 - ang), math.sin(phi0 - ang)
            land_verts.append((xp, (r_rim + land_h) * ca, (r_rim + land_h) * sa))
    n_land_ax = n_ax // 2 + 1
    land_per = 2 * n_land_ax
    for k in range(n_blades):
        b = k * land_per
        for ii in range(n_land_ax - 1):
            i0 = b + ii * 2
            i1 = b + (ii + 1) * 2
            land_faces.append((i0, i0 + 1, i1 + 1, i1))
        land_faces.append((b, b + 2 * (n_land_ax - 1),
                           b + 2 * (n_land_ax - 1) + 1, b + 1))

    return mesh.join((core_v, core_f), (rim_verts, rim_faces),
                     (land_verts, land_faces))


def _fan_rotors():
    """Three fan stages: blades, discs, and the conical arms tying them to the
    LP shaft. The spool is a drum-and-disc hybrid, as on the real engine."""
    out = {}
    rows = [r for r in spec.FAN_ROWS if r.rotor]
    lp_r = spec.SHAFTS["lp_outer_r"]
    disc_parts = []

    for row in rows:
        ph = _platform_h(row)
        rim = min(row.r_hub_le, row.r_hub_te) - ph
        xc = row.x + row.chord * 0.5
        bore = lp_r + 16.0
        w_rim = row.chord * 0.85

        bv, bf = common.build_row(row)
        root_v, root_f = _fir_tree_one(row, ph)
        roots_v, roots_f = mesh.replicate(root_v, root_f, row.count)
        out[f"blades_{row.name}"] = mesh.join((bv, bf), (roots_v, roots_f))
        disc_parts.append(_slotted_disc(xc, bore, rim, w_rim, row.count,
                                        row, ph, SEG))

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
        ph = _platform_h(row)
        bv, bf = common.build_row(row)
        root_v, root_f = _fir_tree_one(row, ph)
        roots_v, roots_f = mesh.replicate(root_v, root_f, row.count)
        out[f"blades_{row.name}"] = mesh.join((bv, bf), (roots_v, roots_f))

    hp_r = spec.SHAFTS["hp_outer_r"]
    bore = hp_r + 15.0
    # one definition of where the drum surface is, shared with the stator
    # inner shrouds in common.py so the two cannot drift into each other
    prof_outer = common.hpc_drum_profile()

    x_front = rotors[0].x - rotors[0].chord * 0.9
    # Stop at the compressor exit. Running 1.9 chords past the last rotor put
    # the drum's aft end at 1599, and the diffuser starts at 1540 and the
    # combustor dome at 1556 -- so the drum ran 59 mm into both of them.
    x_rear = min(rotors[-1].x + rotors[-1].chord * 1.9,
                 spec.STATION["hpc_exit"] - 4.0)

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
