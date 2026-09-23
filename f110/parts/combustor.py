"""Annular combustor: dump diffuser, inner and outer liners with cooling rings
and dilution holes, dome and swirlers, 20 fuel nozzles, 2 igniters.

Objects whose key starts with "cut:" are boolean cutters, applied by
assemble.py to the named target and then discarded.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
C = spec.COMBUSTOR


def build():
    out = {}
    out.update(_diffuser())
    out.update(_liners())
    out.update(_dome())
    out.update(_fuel_system())
    out.update(_igniters())
    return out


# How far the dome's cowl stands forward of its bulkhead, and the gap between
# the diffuser's exit and the cowl's nose that the air dumps across.
COWL_DEPTH = 32.0
DUMP_GAP = 8.0
# The pins the dome hangs from, out to the combustor case.
N_DOME_PINS = 10


def _diffuser():
    """Dump diffuser: the compressor exit guide vanes discharge into a
    sudden-expansion cavity that feeds the liner and the cooling annuli."""
    # from the compressor's real exit to where the dump has always ended;
    # diffuser_len was measured from a station 62 mm too far forward
    #
    # It ends a dump gap short of the dome's cowl. Its walls used to diverge
    # on to x 1640, to r 300 and 430 -- through the cowl and the bulkhead
    # and onto the liners, so the diffuser was what held them up and the
    # air it slowed had nowhere to dump into.
    x0 = spec.STATION["hpc_exit"]
    x1 = C["x_front"] - 3.0 - COWL_DEPTH - DUMP_GAP
    inner = mesh.cone_tube(x0, x1, 352.0, 358.0, 350.0, 356.0, SEG)
    outer = mesh.cone_tube(x0, x1, 364.0, 372.0, 368.0, 376.0, SEG)
    struts = []
    for k in range(24):
        a = 2 * math.pi * k / 24
        v, f = mesh.box((x0 + x1) / 2, 362.0, 0.0, (x1 - x0) * 0.7, 12.0, 9.0)
        struts.append((mesh.rot_x(v, a), f))
    return {"diffuser": mesh.join(inner, outer, *struts)}


def _liner_profile(r_front, r_exit, outer):
    """Meridional profile of one liner wall, with the stepped cooling rings
    that feed a film of compressor air down the inside of the wall."""
    x0, x1 = C["x_front"], C["x_exit"]
    n = C["n_cooling_rings"]
    step = 5.0 if outer else -5.0
    pts = []
    for i in range(n + 1):
        f = i / n
        x = x0 + (x1 - x0) * f
        r = r_front + (r_exit - r_front) * f
        pts.append((x, r))
        if i < n:
            pts.append((x + 6.0, r + step))
            pts.append((x + 12.0, r))
    return pts


def _liners():
    """Inner and outer liner walls. Each is a thin sheet, so it is built as a
    closed profile 3 mm thick and then perforated with dilution holes."""
    out = {}
    t = 3.0
    for tag, r_f, r_e, is_outer in (
            ("outer", C["liner_outer_r"], 396.0, True),
            ("inner", C["liner_inner_r"], 314.0, False)):
        prof = _liner_profile(r_f, r_e, is_outer)
        back = [(x, r - t) if is_outer else (x, r + t) for (x, r) in reversed(prof)]
        name = f"combustor_liner_{tag}"
        out[name] = mesh.revolve_closed(prof + back, SEG)

        # two staggered rows of dilution holes per wall
        cutters = []
        for row_i, xf in enumerate((0.30, 0.52)):
            xd = C["x_front"] + (C["x_exit"] - C["x_front"]) * xf
            rd = r_f + (r_e - r_f) * xf
            n_holes = C["n_dilution_holes"] - row_i * 8
            phase = math.pi / n_holes if row_i else 0.0
            cv, cf = mesh.cylinder(-26.0, 26.0, C["dilution_hole_r"], 12)
            cv = mesh.rot_z(cv, math.pi / 2)          # point radially
            cv = mesh.translate(cv, xd, rd, 0.0)
            cutters.append(mesh.replicate(cv, cf, n_holes, phase))
        out[f"cut:{name}"] = mesh.join(*cutters)
    return out


def _dome():
    """Combustor dome: the rounded head that closes the front of the annulus,
    carrying one swirler per fuel nozzle."""
    x = C["x_front"]
    ro, ri = C["dome_r_outer"], C["dome_r_inner"]
    rm = (ro + ri) / 2
    half = (ro - ri) / 2

    # It was a solid half-round from r 308 to 420, reaching neither liner,
    # with the twenty swirler cups sunk 42 mm into it. A combustor head is
    # sheet metal: a bulkhead across the annulus from liner to liner with a
    # hole for each swirler, and a cowl in front of it that splits the
    # diffuser's flow -- through the slot in its nose to the swirlers, round
    # its lips to the passages outside the liners.
    t = 3.0
    r_lo = C["liner_inner_r"] + t           # the liners' inside faces
    r_hi = C["liner_outer_r"] - t
    plate = mesh.revolve_closed([(x - t, r_lo), (x - t, r_hi),
                                 (x, r_hi), (x, r_lo)], SEG)
    parts = [plate]
    slot = math.asin(min(0.9, C["swirler_r"] * 0.7 / half))
    for a0, a1 in ((0.0, math.pi / 2 - slot), (math.pi / 2 + slot, math.pi)):
        n = 12
        arc = [a0 + (a1 - a0) * i / n for i in range(n + 1)]
        outer = [(x - t - COWL_DEPTH * math.sin(q), rm + half * math.cos(q))
                 for q in arc]
        inner = [(x - t - (COWL_DEPTH - t) * math.sin(q),
                  rm + (half - t) * math.cos(q)) for q in reversed(arc)]
        parts.append(mesh.revolve_closed(outer + inner, SEG))
    # and it hangs from the case on radial pins through the outer passage,
    # which is what carries it and both liners
    # -- off the cowl's outer lip just ahead of the bulkhead, clear of the
    # liner's front edge
    x_pin = x - t - 7.0
    r_case = spec.casing_inner("casing_combustor", x_pin)
    pv, pf = mesh.cylinder(rm + half - 7.0, r_case - 0.2, 6.0, 14)
    pv = mesh.rot_z(pv, math.pi / 2)            # point radially
    pv = mesh.translate(pv, x_pin, 0.0, 0.0)
    parts.append(mesh.replicate(pv, pf, N_DOME_PINS, math.pi / N_DOME_PINS))
    dome = mesh.join(*parts)
    holes = []
    for k in range(C["n_fuel_nozzles"]):
        # the cup's own outside diameter where it passes the bulkhead
        hv, hf = mesh.cylinder(x - 10.0, x + 6.0, C["swirler_r"] * 1.15, 20)
        hv = [(px, py + C["nozzle_r"], pz) for (px, py, pz) in hv]
        holes.append((mesh.rot_x(hv, 2 * math.pi * k / C["n_fuel_nozzles"]), hf))

    swirl = []
    for k in range(C["n_fuel_nozzles"]):
        a = 2 * math.pi * k / C["n_fuel_nozzles"]
        # a ring: the profile is a closed loop, so revolving it open leaves
        # the cup's two rims as edges with one face on them
        cup_v, cup_f = mesh.revolve_ring(
            [(x - 16.0, C["swirler_r"] * 0.45), (x - 16.0, C["swirler_r"]),
             (x + 16.0, C["swirler_r"] * 1.22), (x + 16.0, C["swirler_r"] * 0.7)],
            20)
        cup_v = mesh.translate(cup_v, 0.0, C["nozzle_r"], 0.0)
        # swirl vanes inside the cup
        vanes = []
        for j in range(10):
            b = 2 * math.pi * j / 10
            bv, bf = mesh.box(x, C["nozzle_r"], 0.0, 22.0, C["swirler_r"] * 0.5, 3.0)
            bv = [(px, (py - C["nozzle_r"]) * math.cos(b) - pz * math.sin(b)
                   + C["nozzle_r"],
                   (py - C["nozzle_r"]) * math.sin(b) + pz * math.cos(b))
                  for (px, py, pz) in bv]
            vanes.append((bv, bf))
        ov, of = mesh.join((cup_v, cup_f), *vanes)
        swirl.append((mesh.rot_x(ov, a), of))

    # and the cowl is pierced where each nozzle's stem comes through it
    sv, sf = mesh.pipe(nozzle_path(), 12.5, 14, bend=NOZZLE_BEND)
    for k in range(C["n_fuel_nozzles"]):
        holes.append((mesh.rot_x(sv, 2 * math.pi * k / C["n_fuel_nozzles"]), sf))
    return {"combustor_dome": dome, "cut:combustor_dome": mesh.join(*holes),
            "combustor_swirlers": mesh.join(*swirl)}


NOZZLE_BOSS_R = 520.0
# the stem's bends, shared with the hole the dome's cowl has for it
NOZZLE_BEND = 16.5


def nozzle_path():
    """Centreline of one fuel nozzle's stem, in from its boss on the case
    and turning aft into its swirler, on the +y axis."""
    x = C["x_front"] - 4.0
    return [(x - 40.0, NOZZLE_BOSS_R, 0.0), (x - 40.0, 430.0, 0.0),
            (x - 30.0, 392.0, 0.0), (x - 8.0, C["nozzle_r"] + 6.0, 0.0),
            (x + 6.0, C["nozzle_r"], 0.0)]


def _fuel_system():
    """Twenty fuel nozzles entering radially through the combustor casing,
    each turning aft to spray into its swirler, plus the supply manifold."""
    out = {}
    x = C["x_front"] - 4.0
    r_out = NOZZLE_BOSS_R
    nozzles = []

    stem = mesh.pipe(nozzle_path(), 11.0, 14, bend=NOZZLE_BEND)
    # a ring, so the orifice is a hole: capping it fanned a disc to the axis
    # and every nozzle came out blanked off at the tip
    tip = mesh.revolve_ring(
        [(x + 4.0, 2.0), (x + 4.0, C["nozzle_tip_r"]),
         (x + 22.0, C["nozzle_tip_r"] * 0.75), (x + 22.0, 2.0)], 18)
    tip_v = mesh.translate(tip[0], 0.0, C["nozzle_r"], 0.0)
    boss_v, boss_f = mesh.cylinder(-18.0, 18.0, 26.0, 18)
    boss_v = mesh.rot_z(boss_v, math.pi / 2)
    boss_v = mesh.translate(boss_v, x - 40.0, r_out - 4.0, 0.0)

    one_v, one_f = mesh.join(stem, (tip_v, tip[1]), (boss_v, boss_f))
    for k in range(C["n_fuel_nozzles"]):
        a = 2 * math.pi * k / C["n_fuel_nozzles"]
        nozzles.append((mesh.rot_x(one_v, a), one_f))
    out["fuel_nozzles"] = mesh.join(*nozzles)

    # On the plane the stems enter at, which is what a manifold is for: the
    # twenty stems tee off it. At x - 62 it sat 22 mm upstream of them, on
    # top of the compressor's aft flange, feeding nothing.
    #
    # (r_out + 6 put the ring at 513-539 and the bypass casing wall is at
    # 528-537 here, so it is inboard of that, in the gap between the
    # compressor casing and the duct.)
    out["fuel_manifold"] = mesh.ring_torus(x - 40.0, r_out - 22.0, 13.0, SEG, 14)
    return out


def _igniters():
    ig = []
    x = C["x_front"] + 70.0
    for ang in C["igniter_angles"]:
        path = [(x, 530.0, 0.0), (x, C["liner_outer_r"] + 4.0, 0.0)]
        body = mesh.pipe(path, 17.0, 14)
        cap_v, cap_f = mesh.cylinder(-26.0, 0.0, 30.0, 16)
        cap_v = mesh.rot_z(cap_v, math.pi / 2)
        cap_v = mesh.translate(cap_v, x, 556.0, 0.0)
        v, f = mesh.join(body, (cap_v, cap_f))
        ig.append((mesh.rot_x(v, math.radians(ang)), f))
    return {"igniters": mesh.join(*ig)}
