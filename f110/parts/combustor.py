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


def _diffuser():
    """Dump diffuser: the compressor exit guide vanes discharge into a
    sudden-expansion cavity that feeds the liner and the cooling annuli."""
    # from the compressor's real exit to where the dump has always ended;
    # diffuser_len was measured from a station 62 mm too far forward
    x0 = spec.STATION["hpc_exit"]
    x1 = spec.STATION["diffuser_exit"]
    inner = mesh.cone_tube(x0, x1, 352.0, 358.0, 300.0, 308.0, SEG)
    outer = mesh.cone_tube(x0, x1, 364.0, 372.0, 420.0, 430.0, SEG)
    struts = []
    for k in range(24):
        a = 2 * math.pi * k / 24
        v, f = mesh.box((x0 + x1) / 2, 362.0, 0.0, C["diffuser_len"] * 0.7, 12.0, 9.0)
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

    prof = []
    for i in range(17):
        a = math.pi / 2 + math.pi * i / 16
        prof.append((x - half * math.cos(a - math.pi / 2) * 1.15,
                     rm + half * math.sin(a - math.pi / 2)))
    prof = [(x - half * math.sin(math.pi * i / 16) * 1.15,
             rm - half * math.cos(math.pi * i / 16)) for i in range(17)]
    back = [(x + 26.0, ro), (x + 26.0, ri)]
    dome = mesh.revolve_closed(prof + back, SEG)

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

    return {"combustor_dome": dome, "combustor_swirlers": mesh.join(*swirl)}


def _fuel_system():
    """Twenty fuel nozzles entering radially through the combustor casing,
    each turning aft to spray into its swirler, plus the supply manifold."""
    out = {}
    x = C["x_front"] - 4.0
    r_out = 520.0
    nozzles = []

    path = [(x - 40.0, r_out, 0.0), (x - 40.0, 430.0, 0.0),
            (x - 30.0, 392.0, 0.0), (x - 8.0, C["nozzle_r"] + 6.0, 0.0),
            (x + 6.0, C["nozzle_r"], 0.0)]
    stem = mesh.pipe(path, 11.0, 14)
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
