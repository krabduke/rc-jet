"""Accessory drive and external plumbing: the gearbox, the tower shaft that
drives it off the HP spool, fuel and oil lines, and the electrical harnesses."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
A = dict(spec.ACCESSORIES, gearbox_x=290.0, gearbox_r=660.0,
         gearbox_height=110.0)


def build():
    out = {}
    out.update(_gearbox())
    out.update(_towershaft())
    out.update(_plumbing())
    out.update(_case_detail())
    out.update(_systems())
    out.update(_accessory_drives())
    return out


def _casing_outer(x):
    """Outer radius of the casing that owns station x (its own wall, not the
    global max) so an access panel sits on the skin it belongs to."""
    for (name, x0, x1, r0, r1, wall) in spec.CASINGS:
        if x0 <= x <= x1:
            return spec.casing_inner(name, x) + wall
    return 300.0


def _case_detail():
    """Raised, bolted access panels and compressor bleed pipes on the casings.

    A bare turned cylinder is the clearest tell that an engine was modelled as
    a shell rather than built up. Real casings carry inspection covers for the
    bleed and fuel systems, and external ducting, and they are bolted on with a
    visible fastener line. Each panel is a shallow blister -- a capped sector of
    the casing -- ringed with bolt heads, and the bleed pipes are run at clock
    positions clear of the fuel and oil lines already routed.
    """
    out = {}
    # (casing x-centre, clock-centre deg, x-span, angular-span deg)
    panels = [
        (300.0, 78.0, 150.0, 26.0),      # fan case, top
        (300.0, -102.0, 150.0, 26.0),    # fan case, underside opposite the AGB
        (1150.0, 92.0, 220.0, 30.0),     # HP-compressor casing, top
        (1360.0, 92.0, 200.0, 28.0),
        (1800.0, 60.0, 150.0, 24.0),     # combustor fuel-access panel
        (1800.0, -60.0, 150.0, 24.0),
        (2250.0, 90.0, 180.0, 26.0),     # turbine casing, top
        (900.0, -92.0, 200.0, 28.0),     # bypass cowl, bottom
        (1650.0, -92.0, 220.0, 30.0),
    ]
    covers = []
    bolts = []
    for (xc, clock, dx, da) in panels:
        r = _casing_outer(xc)
        x0, x1 = xc - dx / 2.0, xc + dx / 2.0
        a0 = clock - da / 2.0
        # The panel stands clear of the casing's hoop stiffeners, which are up
        # to 4 mm proud of the nominal outer radius and run underneath it.
        lip = 7.0
        pv, pf = mesh.revolve_closed(
            [(x0, r - 1.0), (x1, r - 1.0), (x1, r + lip), (x0, r + lip)],
            segments=10, phase=math.radians(a0), sweep=math.radians(da))
        covers.append((pv, pf))
        # a bolt head at each corner and mid-edge of the panel border
        for bx in (x0 + dx * 0.12, xc, x1 - dx * 0.12):
            for bt in (a0 + 2.0, clock, a0 + da - 2.0):
                p = _at(r + lip, bt, bx)
                # The head sits on the panel, not sunk through it into the
                # casing wall: starting 3 mm in put 22 % of the bolt geometry
                # inside casing_fan.
                bv, bf = mesh.cylinder(-1.5, 6.0, 6.0, 6)
                bv = mesh.rot_z(bv, math.pi / 2)        # axis now +Y
                bv = mesh.rot_x(bv, math.radians(bt))   # swing to clock angle
                bv = mesh.translate(bv, p[0], p[1], p[2])
                bolts.append((bv, bf))
    out["access_panels"] = mesh.join(*covers)
    out["panel_bolts"] = mesh.join(*bolts)

    # The fan cowl door: the big bolted panel you open to get at the fan, on
    # the upper-left of the fan case. It is much larger than the accessory
    # panels and it has real hinge fittings forward and quick-access latches
    # aft -- the two things that make it read as a DOOR and not just a patch.
    door = []
    # aft of the fan containment wrap (x 26..250), which is a full ring round
    # this same stretch of case -- the door was sitting inside it
    dx0, dx1 = 286.0, 760.0
    dc, dspan = 118.0, 44.0
    lip = 7.0

    def rx(x):
        return _casing_outer(x)

    # The door in section: a skin with a thickness, sunk flush into the casing
    # at both ends and standing proud of it in between, with a stiffening bead
    # inboard of each end and a slight crown along the middle. A cowl door is
    # a formed panel on a seal land, not a curved tile, and the section is
    # where that reads -- from outside you see the step down to the seal and
    # the two beads catch the light along their length.
    xs = [dx0, dx0 + 16.0, dx0 + 34.0, dx0 + 96.0, dx0 + 124.0,
          0.5 * (dx0 + dx1),
          dx1 - 124.0, dx1 - 96.0, dx1 - 34.0, dx1 - 16.0, dx1]
    outer = [rx(dx0) - 1.0, rx(dx0 + 16.0) + 2.0, rx(dx0 + 34.0) + lip,
             rx(dx0 + 96.0) + lip + 1.8, rx(dx0 + 124.0) + lip,
             rx(0.5 * (dx0 + dx1)) + lip + 0.9,
             rx(dx1 - 124.0) + lip, rx(dx1 - 96.0) + lip + 1.8,
             rx(dx1 - 34.0) + lip, rx(dx1 - 16.0) + 2.0, rx(dx1) - 1.0]
    # inner skin: forward to aft along the underside, then back up the outside
    prof = [(dx0, rx(dx0) - 3.0), (dx1, rx(dx1) - 3.0)]
    prof += [(x, r) for x, r in zip(reversed(xs), reversed(outer))]
    dv, df = mesh.revolve_closed(
        prof, segments=44, phase=math.radians(dc - dspan / 2.0),
        sweep=math.radians(dspan))
    door.append((dv, df))

    hw = []

    def radial(verts, ang):
        """Take an x-axis solid and stand its axis up along the radius at
        clock angle `ang`, which is what a latch or a boss wants."""
        return mesh.rot_x(mesh.rot_z(verts, math.pi / 2), math.radians(ang))

    # Hinge line down one longitudinal edge. The knuckles are collinear and
    # their pin runs fore and aft along that edge -- that is the axis the door
    # actually swings about. (Pins set crosswise would lock it shut.)
    ha = dc - dspan / 2.0 + 3.0
    for hf in (0.12, 0.42, 0.72):
        hx = dx0 + (dx1 - dx0) * hf
        rh = _casing_outer(hx) + lip
        p0 = _at(rh, ha, hx - 18.0)
        # barrel, on the hinge axis
        bv, bf = mesh.tube(hx - 18.0, hx + 18.0, 3.6, 8.0, 18)
        hw.append((mesh.translate(bv, 0.0, p0[1], p0[2]), bf))
        # the pin through it, standing out either end
        pv, pf = mesh.cylinder(hx - 26.0, hx + 26.0, 3.4, 14)
        hw.append((mesh.translate(pv, 0.0, p0[1], p0[2]), pf))
        # the strap that carries the barrel down onto the casing, swept round
        # the casing rather than cut straight through it
        path = [_at(_casing_outer(hx) + lip * 0.5, ha - 2.0 - 7.0 * k, hx)
                for k in range(4)]
        sv, sf = mesh.pipe(path, 6.0, segments=12)
        hw.append((sv, sf))

    # Quarter-turn latches down the other edge. A latch turns about the radius,
    # so it is a dished cup recessed into the skin with a slotted stud in it.
    la = dc + dspan / 2.0 - 3.0
    for lf in (0.2, 0.5, 0.8):
        lx = dx0 + (dx1 - dx0) * lf
        pl = _at(_casing_outer(lx) + lip, la, lx)
        cup = [(-4.0, 4.0), (-4.0, 13.0), (0.0, 15.0), (3.0, 15.0),
               (3.0, 12.0), (-1.0, 10.5), (-1.0, 4.0)]
        cv, cf = mesh.revolve_closed(cup, segments=20)
        cv = radial(cv, la)
        hw.append((mesh.translate(cv, pl[0], pl[1], pl[2]), cf))
        st, stf = mesh.cylinder(-1.5, 4.5, 6.5, 16)
        st = radial(st, la)
        hw.append((mesh.translate(st, pl[0], pl[1], pl[2]), stf))
        # the screwdriver slot across the stud head
        sl, slf = mesh.box(0.0, 0.0, 0.0, 2.4, 11.0, 2.0)
        sl = radial(sl, la)
        hw.append((mesh.translate(sl, pl[0], pl[1], pl[2] + 0.0), slf))

    out["fan_cowl_door"] = mesh.join(*door)
    out["fan_door_hardware"] = mesh.join(*hw)

    # compressor bleed pipes: heavier than the fuel lines, run high and to the
    # sides where they do not clash with the plumbing already on the centreline
    bleed = []
    for (clock, x0, x1, rr) in ((40.0, 1560.0, 820.0, 30.0),
                                (-40.0, 1560.0, 820.0, 30.0),
                                (140.0, 1500.0, 980.0, 26.0)):
        pts = []
        n = 10
        for i in range(n + 1):
            x = x0 + (x1 - x0) * i / n
            # sag the pipe off the casing at mid-span so it reads as a run, not
            # a line stuck to the surface
            bow = 34.0 * math.sin(math.pi * i / n)
            pts.append(_at(_casing_outer(x), clock, x, extra_r=18.0 + bow))
        bleed.append(mesh.pipe(pts, rr, 14))
        # a couple of saddle clamps holding it to the casing
        for i in (2, 7):
            p = pts[i]
            cv, cf = mesh.tube(-rr * 1.2, rr * 1.2, rr, rr * 1.35, 12)
            cv = mesh.rot_z(cv, math.pi / 2)
            ang = math.atan2(p[2], p[1])
            cv = mesh.rot_x(cv, ang)
            cv = mesh.translate(cv, p[0], p[1], p[2])
            bleed.append((cv, cf))
        # the offtake stub through the bypass duct onto the compressor case.
        # `_casing_radius` returns the BYPASS case at these stations, so the
        # pipes lay 33 mm outside the HPC case they are meant to tap.
        # x1 is the FORWARD end of the run and x0 the aft one, so the stubs
        # go off x1: taken off x0 they land at 1600 and 1730, in the
        # combustor dome and the fuel nozzles.
        for xb in (x1 + 40.0, x1 + 170.0):
            r_out = _casing_radius(xb) + 18.0
            bleed.append(mesh.pipe(
                [_at(r_out, clock, xb),
                 # into the casing's wall, not 26 mm proud of its bore. The
                 # wall is 14 mm thick, so +26 put the end of the offtake
                 # 12 mm outside the case it is supposed to be tapping.
                 _at(spec.casing_inner("casing_hpc", xb) + 6.0, clock, xb)],
                rr * 0.9, 12))
    out["bleed_pipes"] = mesh.join(*bleed)
    return out


def _accessory_drives():
    cx = A["gearbox_x"] + A["gearbox_len"] / 2.0
    center = _at(A["gearbox_r"], A["gearbox_angle"], cx)
    out = {}
    mounts = []
    for dx in (-0.36, 0.36):
        x = cx + dx * A["gearbox_len"]
        for dy in (-0.32, 0.32):
            p = (x, center[1] + dy * A["gearbox_width"],
                 center[2] + A["gearbox_height"] / 2.0 - 4.0)
            clock = math.degrees(math.atan2(p[2], p[1]))
            q = _at(_casing_radius(x) + 8.0, clock, x)
            mounts.extend([_case_boss(x, clock, radius=23.0),
                           _fitting(p, q, 16.0)])
    out["gearbox_mounts"] = mesh.join(*mounts)
    for name, fraction, radius, length in (
            ("generator_1", -0.28, 52.0, 154.0),
            ("generator_2", 0.25, 52.0, 154.0),
            ("hydraulic_pump_1", -0.25, 34.0, 106.0),
            ("hydraulic_pump_2", 0.25, 34.0, 106.0),
            ("oil_pressure_pump", -0.12, 29.0, 86.0),
            ("oil_scavenge_pump", 0.18, 34.0, 96.0)):
        x = cx + fraction * A["gearbox_len"]
        if name.startswith("generator"):
            base = (x, center[1] + A["gearbox_width"] / 2.0 - 3.0, center[2])
            axis = (0.0, 1.0, 0.0)
        else:
            y = center[1] + (0.24 if name.startswith("oil") else -0.24) * A["gearbox_width"]
            base = (x, y, center[2] - A["gearbox_height"] / 2.0 + 3.0)
            axis = (0.0, 0.0, -1.0)
        def point(t):
            return tuple(base[k] + axis[k] * t for k in range(3))
        # body, drive-pad flange, and the hardware that says what it is:
        # cooling fins on a generator, inlet and outlet unions on a pump
        pieces = _bolted_pad(base, axis, radius + 12.0)
        pieces.append(mesh.pipe([point(10.0), point(length)], radius,
                                segments=28))
        pieces.append(_fitting(point(length - 4.0), point(length + 12.0),
                               radius * 0.55))
        u, v = _frame(axis)
        if name.startswith("generator"):
            # a fin stack, and the terminal box the feeders land on
            for k in range(7):
                t = 26.0 + k * 9.5
                pieces.append(_fitting(point(t), point(t + 4.0), radius + 5.0,
                                       28))
            box = tuple(point(length * 0.62)[j] + u[j] * (radius + 2.0)
                        for j in range(3))
            end = tuple(box[j] + u[j] * 22.0 for j in range(3))
            pieces.append(_fitting(box, end, 15.0, 16))
            pieces.append(_fitting(end, tuple(end[j] + u[j] * 5.0
                                              for j in range(3)), 18.0, 16))
        else:
            # suction and pressure unions, one each side, with their caps
            for side, t in ((u, 0.34), (v, 0.62)):
                at = point(length * t)
                p0 = tuple(at[j] + side[j] * (radius - 2.0) for j in range(3))
                p1 = tuple(at[j] + side[j] * (radius + 26.0) for j in range(3))
                pieces.append(_fitting(p0, p1, 9.5, 16))
                p2 = tuple(at[j] + side[j] * (radius + 32.0) for j in range(3))
                pieces.append(_fitting(p1, p2, 12.5, 16))
        out[name] = mesh.join(*pieces)
    return out


def _fitting(p0, p1, radius, segments=20):
    """A short cylindrical boss between two points.

    20 sides, not 6. Every boss, flange, fin and case pad on this engine
    goes through here, and at 6 they were hexagons -- which is why seven of
    the accessories sat at 96 to 120 vertices against a floor of 200.
    """
    return mesh.pipe([p0, p1], radius, segments=segments)


def _frame(axis):
    """Two unit vectors across `axis`, for putting bolts round a flange."""
    up = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = mesh._normalise(mesh._cross(axis, up))
    return u, mesh._normalise(mesh._cross(axis, u))


def _bolted_pad(base, axis, r_flange, n_bolt=8):
    """The flange an accessory is clamped to its drive pad by."""
    def point(t):
        return tuple(base[k] + axis[k] * t for k in range(3))
    u, v = _frame(axis)
    pieces = [_fitting(point(0.0), point(14.0), r_flange, 24)]
    for k in range(n_bolt):
        a = 2.0 * math.pi * k / n_bolt
        off = tuple((u[j] * math.cos(a) + v[j] * math.sin(a)) * (r_flange - 6.0)
                    for j in range(3))
        p0 = tuple(point(12.0)[j] + off[j] for j in range(3))
        p1 = tuple(point(20.0)[j] + off[j] for j in range(3))
        pieces.append(_fitting(p0, p1, 3.4, 8))
    return pieces


def _case_boss(x, clock, height=14.0, radius=15.0):
    r = _casing_radius(x)
    return _fitting(_at(r - 2.0, clock, x),
                    _at(r + height, clock, x), radius)


def _supported_line(path, radius, clock=None, spacing=3):
    pieces = [mesh.pipe(path, radius, segments=12)]
    for i in range(1, len(path) - 1, spacing):
        p = path[i]
        x = p[0]
        r = math.hypot(p[1], p[2])
        angle = math.degrees(math.atan2(p[2], p[1]))
        pieces.append(_case_boss(x, angle, radius=radius + 8.0))
        pieces.append(_fitting(_at(_casing_radius(x) + 10.0, angle, x),
                               _at(r - radius + 2.0, angle, x), 5.0))
        cv, cf = mesh.tube(-5.0, 5.0, radius * 0.94, radius + 3.0, 16)
        tangent = tuple(path[i + 1][k] - path[i - 1][k] for k in range(3))
        length = math.sqrt(sum(t * t for t in tangent))
        ux, uy, uz = (t / length for t in tangent)
        yaw = math.atan2(uy, ux)
        pitch = math.atan2(uz, math.hypot(ux, uy))
        cv = [(t * math.cos(pitch) - v * math.sin(pitch), u,
               t * math.sin(pitch) + v * math.cos(pitch)) for t, u, v in cv]
        cv = mesh.rot_z(cv, yaw)
        pieces.append((mesh.translate(cv, *p), cf))
    for p, q in ((path[0], path[1]), (path[-1], path[-2])):
        length = math.dist(p, q)
        end = tuple(p[k] + (q[k] - p[k]) * min(14.0 / length, 0.4)
                    for k in range(3))
        pieces.append(_fitting(p, end, radius + 4.0))
    return pieces


def _case_line(clock, x0, x1, standoff, radius, start=None, end=None):
    path = _route(clock, x0, x1, standoff,
                  n=max(4, int(abs(x1 - x0) / 80.0)))
    path.insert(0, start if start is not None else _at(_casing_radius(x0), clock, x0))
    path.append(end if end is not None else _at(_casing_radius(x1), clock, x1))
    pieces = _supported_line(path, radius)
    for x, endpoint in ((x0, start), (x1, end)):
        if endpoint is None:
            pieces.append(_case_boss(x, clock, radius=radius + 9.0))
    return pieces


def _at(radius, clock_deg, x, extra_r=0.0):
    a = math.radians(clock_deg)
    r = radius + extra_r
    return (x, r * math.cos(a), r * math.sin(a))


def _gearbox():
    """Airframe-mounted-style accessory gearbox slung under the fan case,
    with the pump and generator pads that hang off it."""
    cx = A["gearbox_x"] + A["gearbox_len"] / 2
    ang = A["gearbox_angle"]
    cy = A["gearbox_r"] * math.cos(math.radians(ang))
    cz = A["gearbox_r"] * math.sin(math.radians(ang))

    body = mesh.box(cx, cy, cz, A["gearbox_len"],
                    A["gearbox_width"], A["gearbox_height"])
    parts = [body]

    # accessory pads: fuel pump, oil filter, starter/generator. Three
    # near-identical fat drums read as a garlic bulb; a real AGB carries
    # visibly different accessories, so each gets its own silhouette -- a
    # hex-bodied pump with two pipe bosses, a banded filter canister, and a
    # finned generator.
    #
    # Every accessory is modelled in the pad's own frame and mapped into the
    # engine exactly once, by `place`. Building them in engine coordinates and
    # rotating each piece into position is how a pad ends up growing back
    # through the gearbox it hangs off, or sitting at the engine centreline
    # because one piece was never given its axial station.
    pads = [(-0.30, 74.0, 132.0), (0.02, 62.0, 104.0), (0.34, 86.0, 150.0)]
    for k, (fx, pr, pl) in enumerate(pads):
        px = cx + A["gearbox_len"] * fx
        yb = cy - A["gearbox_width"] / 2

        def place(v, px=px, yb=yb):
            """Pad-local (t, u, w) -> engine (x, y, z).

            t runs outboard along the pad axis, away from the gearbox face;
            u lies along the engine; w is vertical. mesh.cylinder, mesh.tube
            and mesh.revolve_closed all produce solids along their own +x with
            the section in (y, z), which is exactly (t, u, w)."""
            return [(px + u, yb - t, cz + w) for (t, u, w) in v]

        if k == 0:
            # fuel pump: octagonal body stepping down to a delivery neck
            body = [(0.0, 1.0), (pl, 1.0), (pl, pr * 0.30),
                    (pl * 0.72, pr * 0.30), (pl * 0.72, pr * 0.52),
                    (0.0, pr * 0.52)]
            pv, pf = mesh.revolve_closed(body, segments=8)
            parts.append((place(pv), pf))
            # inlet and outlet bosses, out of the side of the body
            for sgn in (-1, 1):
                bv, bf = mesh.cylinder(0.0, pr * 0.62, pr * 0.26, 12)
                bv = mesh.rot_z(bv, math.pi / 2)     # pad axis -> along u
                bv = [(t + pl * 0.36, u * sgn, w) for (t, u, w) in bv]
                parts.append((place(bv), bf))
        elif k == 1:
            # oil filter: a canister with a domed end and two retaining bands
            body = [(0.0, 1.0), (pl, 1.0), (pl, pr * 0.40),
                    (pl * 0.93, pr * 0.78), (pl * 0.82, pr), (0.0, pr)]
            pv, pf = mesh.revolve_closed(body, segments=20)
            parts.append((place(pv), pf))
            for t0 in (pl * 0.24, pl * 0.55):
                bv, bf = mesh.tube(t0, t0 + 9.0, pr * 0.99, pr * 1.10, 20)
                parts.append((place(bv), bf))
        else:
            # starter/generator: a drum with circumferential cooling fins and
            # a splined drive stub on the end
            dv, df = mesh.cylinder(0.0, pl * 0.88, pr, 24)
            parts.append((place(dv), df))
            for fi in range(7):
                t0 = pl * (0.10 + 0.105 * fi)
                fv, ff = mesh.tube(t0, t0 + 7.0, pr, pr * 1.16, 24)
                parts.append((place(fv), ff))
            sv, sf = mesh.cylinder(pl * 0.88, pl, pr * 0.34, 16)
            parts.append((place(sv), sf))

        # mounting flange, on the gearbox face
        fv, ff = mesh.cylinder(0.0, 14.0, pr * 1.22, 20)
        parts.append((place(fv), ff))

    return {"gearbox": mesh.join(*parts)}


def _towershaft():
    """Radial drive shaft: takes power off the HP spool through a bevel gear
    and carries it down to the gearbox. This is also the starting path -- the
    starter drives back up it to crank the HP spool."""
    ang = A["towershaft_angle"]
    p0 = _at(A["towershaft_r0"], ang, A["towershaft_x"])
    p1 = _at(A["towershaft_r1"], ang, A["towershaft_x"])
    shaft = mesh.pipe([p0, p1], A["towershaft_r"], 16)

    # bevel gear at the inboard end, housing at the outboard end
    gv, gf = mesh.revolve_open(
        [(0.0, 0.001), (0.0, 62.0), (34.0, 40.0), (34.0, 0.001)],
        24, cap_start=True, cap_end=True)
    gv = mesh.rot_z(gv, -math.pi / 2)
    gv = mesh.translate(gv, *p0)

    hv, hf = mesh.cylinder(0.0, 120.0, 54.0, 20)
    hv = mesh.rot_z(hv, -math.pi / 2)
    hv = mesh.translate(hv, p1[0], p1[1], p1[2])
    hv = [(x, y + 60.0 * math.cos(math.radians(ang)),
           z + 60.0 * math.sin(math.radians(ang))) for (x, y, z) in hv]

    return {"towershaft": mesh.join(shaft, (gv, gf), (hv, hf))}


def _casing_radius(x):
    """Outer radius of whatever casing is at station x -- so external lines can
    be routed to lie on the engine instead of floating beside it."""
    best = 300.0
    for (name, x0, x1, r0, r1, wall) in spec.CASINGS:
        if x0 <= x <= x1:
            best = max(best, spec.casing_inner(name, x) + wall)
    return best


def _route(clock_deg, x_start, x_end, standoff, n=16):
    return [_at(_casing_radius(x_start + (x_end - x_start) * i / n),
                clock_deg, x_start + (x_end - x_start) * i / n, standoff)
            for i in range(n + 1)]


def _plumbing():
    """External lines. Routed along the casing at realistic clock positions:
    fuel high on the sides, oil returning to the gearbox low down, electrical
    harnesses clipped along the top."""
    out = {}

    S = spec.STATION

    # ---- fuel ------------------------------------------------------------
    #
    # `fuel = []` and then `mesh.join(*fuel)`, which is a part with zero
    # vertices. The engine shipped a `fuel_lines` object that was not there:
    # nothing in a render, nothing for any audit to measure, and a fuel
    # manifold at station 1576 fed by nothing at all.
    #
    # The run is the real one: out of the fuel/oil heat exchanger low on the
    # right, aft along the bypass case, then in through the duct to the
    # manifold on the combustor's outer liner.
    fuel = []
    x_man = (spec.STATION["combustor_inlet"] + 40.0
             if "combustor_inlet" in S else 1576.0)
    for clock in (-30.0, -46.0):
        path = _route(clock, 560.0, x_man, 15.0)
        fuel.append(mesh.pipe(path, A["oil_line_r"] * 0.78, 12))
        # the drop through the bypass duct onto the manifold
        r_out = _casing_radius(x_man) + 15.0
        fuel.append(mesh.pipe(
            [_at(r_out, clock, x_man), _at(498.0, clock, x_man)],
            A["oil_line_r"] * 0.78, 12))
    out["fuel_lines"] = mesh.join(*fuel)

    # ---- oil -------------------------------------------------------------
    #
    # The run used to start at the fan frame and end at the turbine frame,
    # which left the tank 794 mm away, both pumps 187 and 105, the heat
    # exchanger 201 and the bearing sumps -- the only things in the engine
    # that oil is for -- 236. Six parts of one circuit, none of them joined.
    #
    # Forward to the pumps at station 465, a transfer hose round the case to
    # the tank at 1 o'clock, and a drop inboard through each frame to the
    # sump it feeds.
    oil = []
    for i, clock in enumerate((-84.0, -96.0, -70.0)):
        # 68 mm of standoff, not 20. The fan case's rear flange stands 32 mm
        # proud at station 569-603, and the variable-vane actuation is a full
        # ring out to r 651 over the whole compressor; the run went through
        # both of them.
        path = _route(clock, 440.0, S["turbine_frame"], 68.0 + i * 13.0)
        oil.append(mesh.pipe(path, A["oil_line_r"], 12))
    # out to the two pumps, which stand off the gearbox at about r 760
    for (xp, clock) in ((465.0, -86.0), (600.0, -88.0)):
        r0 = _casing_radius(xp) + 68.0
        oil.append(mesh.pipe([_at(r0, clock, xp), _at(770.0, clock, xp)],
                             A["oil_line_r"], 12))
    # and round the case to the tank, which sits at 1-2 o'clock
    arc = [_at(_casing_radius(500.0) + 46.0, -80.0 + t * 110.0 / 12.0,
               500.0) for t in range(13)]
    oil.append(mesh.pipe(arc, A["oil_line_r"], 12))
    oil.append(mesh.pipe([_at(_casing_radius(500.0) + 46.0, 30.0, 500.0),
                          _at(660.0, 30.0, 500.0)], A["oil_line_r"], 12))
    # in through each frame to the sumps
    # The line delivers to the turbine frame and stops there.
    #
    # The sumps sit at r 107-229, inside the turbine's own flowpath, so
    # nothing external can reach them radially -- on a real engine the oil
    # goes in through the frame's hollow struts, and that is the path this
    # models. Dropping the pipe all the way to r 190 put it through the LPT
    # disc assembly, which is the one thing back there that turns.
    for (xf, clock) in ((S["turbine_frame"] - 60.0, -74.0),):
        r0 = _casing_radius(xf) + 68.0
        oil.append(mesh.pipe(
            [_at(r0, clock, xf),
             _at(spec.casing_inner("casing_turbine", xf) + 12.0, clock, xf)],
            A["oil_line_r"] * 0.8, 10))
    out["oil_lines"] = mesh.join(*oil)

    # ---- electrical ------------------------------------------------------
    harness = []
    # Three trunks side by side at one standoff, not stacked on top of each
    # other at 26, 38 and 50. A loom is clipped round the casing, not piled
    # up off it: at 50 plus a clamp box the outer trunk stood 66 mm proud of
    # the augmentor case, and in the airframe that vendors this engine it was
    # the widest thing on the installation -- 0.1 mm from the fuselage's
    # inner surface, where the aircraft asks for half a millimetre. They are
    # 12 to 14 degrees apart in clock, which is 120 mm of separation at this
    # radius.
    for i, clock in enumerate((70.0, 84.0, 96.0)):
        path = _route(clock, spec.STATION["fan_face"] + 60.0,
                      spec.STATION["augmentor_exit"] - 120.0, 26.0)
        harness.append(mesh.pipe(path, A["harness_r"], 10))
        for j in range(0, len(path), 4):
            bv, bf = mesh.box(path[j][0], path[j][1], path[j][2], 16.0, 20.0, 20.0)
            harness.append((bv, bf))
    # and the branch down to the control unit, which sits at 7-8 o'clock on
    # the fan case. The trunks run along the top, so the FADEC was 292 mm
    # from the nearest wire on an engine whose every schedule it sets.
    brc = [_at(_casing_radius(430.0) + 26.0, 84.0 + t * 56.0 / 10.0, 430.0)
           for t in range(11)]
    harness.append(mesh.pipe(brc, A["harness_r"] * 0.85, 10))
    harness.append(mesh.pipe(
        [_at(_casing_radius(430.0) + 26.0, 140.0, 430.0),
         _at(735.0, 139.0, 430.0)], A["harness_r"] * 0.85, 10))
    out["harnesses"] = mesh.join(*harness)

    return out


def _systems():
    """The line-replaceable hardware bolted to the outside of the engine.

    An engine is not a bare gas path with some pipes on it. Everything here is
    a unit a technician removes with hand tools, and every one of them was
    missing: the oil tank the whole lubrication system draws from, the fuel/oil
    heat exchanger that cools it, the control that schedules fuel and nozzle
    area, the exciters that fire the igniters already modelled, the probes the
    control reads, and the mounts that actually carry the engine's thrust into
    the airframe.
    """
    out = {}
    S = spec.STATION

    def radial(v, ang):
        """Stand an x-axis solid up along the radius at a clock angle."""
        return mesh.rot_x(mesh.rot_z(v, math.pi / 2), math.radians(ang))

    def strap(x, ang, r_out, r_in, width, span=54.0):
        """A mounting strap clamping a can down onto the casing.

        It used to sweep 192 degrees -- half the circumference of the engine
        for a strap over a 190 mm can. Every unit's strap therefore reached
        round into every other unit's, which is why the oil tank and the heat
        exchanger, mounted 76 degrees apart, had 97 per cent of their shared
        vertices inside each other.
        """
        return mesh.revolve_closed(
            [(x - width / 2, r_in), (x + width / 2, r_in),
             (x + width / 2, r_out), (x - width / 2, r_out)],
            segments=16, phase=math.radians(ang - span / 2),
            sweep=math.radians(span))

    # A line-replaceable unit is a can lying ALONG the engine, clamped down on
    # the casing. Each of these was built as a revolve running along x and then
    # rotated so that its length pointed outward instead -- so a 350 mm oil
    # tank became 350 mm of standoff, and the four of them reached 1,070 to
    # 1,247 mm from the axis on a casing whose outer radius is 600. They stood
    # half a metre proud, joined to the engine by nothing but their straps,
    # which is why the bounding-box attachment test still passed them.
    #
    # `mount` keeps the can's axis along x and moves it out along the radius
    # only as far as its own radius, so it sits on the skin it is bolted to.
    def mount(profile, ang, rr, xa, xb, seg=28):
        rc = _casing_outer(0.5 * (xa + xb))
        c = _at(rc + rr + 4.0, ang, 0.0)
        v, f = mesh.revolve_closed(profile, segments=seg)
        return (mesh.translate(v, 0.0, c[1], c[2]), f), rc + rr + 4.0

    # ---- oil tank: the reservoir the whole lubrication system draws from ----
    ang = 38.0
    xt0, xt1 = 300.0, 640.0
    rr = 96.0
    tank, r_ctr = mount(
        [(xt0, 8.0), (xt0 + 14.0, rr), (xt1 - 14.0, rr), (xt1, 8.0),
         (xt1 - 20.0, rr * 0.81), (xt0 + 20.0, rr * 0.81)], ang, rr, xt0, xt1)
    pieces = [tank]
    # filler neck and cap, standing off the top of the tank
    fv, ff = mesh.revolve_closed(
        [(0.0, 6.0), (26.0, 6.0), (26.0, 30.0), (34.0, 30.0), (34.0, 34.0),
         (20.0, 34.0), (20.0, 24.0), (0.0, 24.0)], segments=18)
    fv = radial(fv, ang)
    fp = _at(r_ctr + rr - 6.0, ang, xt0 + 90.0)
    pieces.append((mesh.translate(fv, fp[0], fp[1], fp[2]), ff))
    for sx in (xt0 + 70.0, xt1 - 70.0):
        sv, sf = strap(sx, ang, r_ctr + rr * 0.30,
                       _casing_outer(sx) + 2.0, 26.0)
        pieces.append((sv, sf))
    out["oil_tank"] = mesh.join(*pieces)

    # ---- fuel/oil heat exchanger: a tube-and-shell cooler -------------------
    ang = -38.0
    xh0, xh1 = 316.0, 596.0
    rh = 62.0
    cooler, rh_ctr = mount(
        [(xh0, 10.0), (xh0 + 10.0, rh), (xh1 - 10.0, rh), (xh1, 10.0),
         (xh1 - 18.0, rh * 0.81), (xh0 + 18.0, rh * 0.81)], ang, rh, xh0, xh1,
        seg=24)
    pieces = [cooler]
    # inlet and outlet unions, out of each end cap along the engine
    for ex, sgn in ((xh0 - 6.0, -1.0), (xh1 + 6.0, 1.0)):
        for off in (-0.30, 0.30):
            uv, uf = mesh.revolve_closed(
                [(0.0, 5.0), (22.0, 5.0), (22.0, 15.0), (30.0, 15.0),
                 (30.0, 11.0), (12.0, 11.0), (12.0, 13.0), (0.0, 13.0)],
                segments=14)
            up = _at(rh_ctr, ang + off * 26.0, ex)
            uv = radial(uv, ang + off * 26.0)
            pieces.append((mesh.translate(uv, up[0], up[1], up[2]), uf))
    for sx in (xh0 + 52.0, xh1 - 52.0):
        sv, sf = strap(sx, ang, rh_ctr + rh * 0.30,
                       _casing_outer(sx) + 2.0, 20.0)
        pieces.append((sv, sf))
    out["heat_exchanger"] = mesh.join(*pieces)

    # ---- engine control (AFTC): schedules fuel and nozzle area -------------
    ang = 150.0
    xc0, xc1 = 306.0, 566.0
    rc_ = 74.0
    ctl, rc_ctr = mount(
        [(xc0, 10.0), (xc0 + 12.0, rc_), (xc1 - 12.0, rc_), (xc1, 10.0),
         (xc1 - 22.0, rc_ * 0.81), (xc0 + 22.0, rc_ * 0.81)], ang, rc_,
        xc0, xc1, seg=8)
    pieces = [ctl]
    # the connector bank down one face -- this is how it is recognised
    for i in range(5):
        kv, kf = mesh.revolve_closed(
            [(0.0, 7.0), (18.0, 7.0), (18.0, 19.0), (26.0, 19.0),
             (26.0, 15.0), (10.0, 15.0), (10.0, 17.0), (0.0, 17.0)],
            segments=12)
        kx = xc0 + 34.0 + i * 46.0
        kp = _at(rc_ctr + rc_ * 0.72, ang - 13.0, kx)
        kv = radial(kv, ang - 13.0)
        pieces.append((mesh.translate(kv, kp[0], kp[1], kp[2]), kf))
    for sx in (xc0 + 44.0, xc1 - 44.0):
        sv, sf = strap(sx, ang, rc_ctr + rc_ * 0.30,
                       _casing_outer(sx) + 2.0, 18.0)
        pieces.append((sv, sf))
    out["engine_control"] = mesh.join(*pieces)

    # ---- ignition exciters, and the leads down to the igniters -------------
    pieces = []
    for sgn in (-1.0, 1.0):
        ea = 118.0 * sgn
        ex0, ex1 = 470.0, 610.0
        re = 44.0
        box, re_ctr = mount(
            [(ex0, 8.0), (ex0 + 10.0, re), (ex1 - 10.0, re), (ex1, 8.0),
             (ex1 - 16.0, re * 0.77), (ex0 + 16.0, re * 0.77)], ea, re,
            ex0, ex1, seg=16)
        pieces.append(box)
        # The screened lead running aft to the igniter plug -- and round to
        # where the plug actually is. The drift was 0.004 deg/mm, so the two
        # leads finished at 123 and -113 o'clock while the igniters sit at
        # 40 and -140: an ignition system whose leads ran the length of the
        # engine and stopped 200 mm from the plugs they fire.
        ig_clock = 40.0 if sgn > 0 else -140.0
        drift = (ig_clock - ea) / (1760.0 - ex1)
        path = [_at(_casing_outer(x) + 30.0, ea + (x - ex1) * drift, x)
                for x in (ex1 - 20.0, 900.0, 1300.0, 1640.0, 1760.0)]
        pieces.append(mesh.pipe(path, 11.0, segments=12))
        # and the drop inboard through the bypass duct onto the plug
        pieces.append(mesh.pipe(
            [_at(_casing_outer(1720.0) + 30.0, ig_clock, 1720.0),
             _at(520.0, ig_clock, 1700.0)], 11.0, segments=12))
        sv, sf = strap(ex0 + 40.0, ea, re_ctr + re * 0.30,
                       _casing_outer(ex0 + 40.0) + 2.0, 16.0)
        pieces.append((sv, sf))
    out["ignition_exciters"] = mesh.join(*pieces)

    # ---- variable bleed valve doors at the fan discharge -------------------
    # The VBVs dump fan air overboard to keep the HP compressor out of surge
    # at low speed. Twelve hinged doors round the bypass wall, each with its
    # own bellcrank onto a unison ring.
    # Ten doors, not twelve, and the unison ring is an arc.
    #
    # The accessory gearbox hangs at clock -90 and is 280 mm wide on a 560 mm
    # radius, so it subtends about 28 degrees at the bottom of the fan case
    # and reaches out to r 765. The two doors at 255 and 285 were inside it,
    # and so was the unison ring, which was a full torus at r 656 -- right
    # through the casing the whole accessory drive bolts to. A VBV ring is
    # interrupted where the gearbox is on every engine that has one there.
    pieces = []
    xv = 690.0
    rv = _casing_outer(xv)
    half_door = 11.0

    # What is already bolted to this part of the case, as (clock, half
    # angle). The accessory gearbox hangs straight down and is 280 mm wide
    # on a 660 mm radius; the two forward engine mount pads stand 96 mm off
    # the casing at 55 and 125 and are 76 mm across, which is only about
    # four degrees each. A door ring and a unison ring both have to go round
    # them -- the ring was a full torus at r 656 straight through the
    # gearbox the whole accessory drive bolts to, and two doors were inside
    # it with two more inside the mount pads.
    blocked = [(A["gearbox_angle"] % 360.0,
                math.degrees(math.atan2(A["gearbox_width"] / 2.0,
                                        A["gearbox_r"]))),
               (55.0, 4.0), (125.0, 4.0)]

    def clear(a, extra):
        return all(abs(((a - c + 180.0) % 360.0) - 180.0) >= hw + extra
                   for (c, hw) in blocked)

    for i in range(12):
        a = 360.0 * i / 12.0 + 15.0
        if not clear(a, half_door):
            continue
        dv, df = mesh.revolve_closed(
            [(xv - 44.0, rv - 3.0), (xv + 44.0, rv - 3.0),
             (xv + 40.0, rv + 11.0), (xv - 40.0, rv + 9.0)],
            segments=6, phase=math.radians(a - half_door),
            sweep=math.radians(2.0 * half_door))
        pieces.append((dv, df))
        # bellcrank from the door up to the unison ring
        p0 = _at(rv + 10.0, a, xv + 36.0)
        p1 = _at(rv + 34.0, a + 3.0, xv + 66.0)
        pieces.append(mesh.pipe([p0, p1], 5.0, segments=8))

    # the unison ring, as the arcs between those obstructions
    ring = [(xv + 70.0 + 7.0 * math.cos(t), rv + 34.0 + 7.0 * math.sin(t))
            for t in [2.0 * math.pi * k / 12 for k in range(12)]]
    cuts = sorted((c - hw - 6.0, c + hw + 6.0) for (c, hw) in blocked)
    for (_, end), (start, _) in zip(cuts, cuts[1:] + [(cuts[0][0] + 360.0, 0)]):
        span = start - end
        if span <= 2.0:
            continue
        pieces.append(mesh.revolve_closed(
            ring, segments=max(8, int(span / 5.0)),
            phase=math.radians(end), sweep=math.radians(span)))
    out["vbv_doors"] = mesh.join(*pieces)

    # ---- borescope ports: how the compressor is inspected on wing ----------
    pieces = []
    for (bx, ang) in spec.BORESCOPE:
        r = _casing_outer(bx)
        # a raised boss with a captive threaded plug in it
        bv, bf = mesh.revolve_closed(
            [(-6.0, 6.0), (10.0, 6.0), (10.0, 20.0), (16.0, 20.0),
             (16.0, 26.0), (-6.0, 26.0)], segments=16)
        bv = mesh.rot_x(mesh.rot_z(bv, math.pi / 2), math.radians(ang))
        pt = _at(r, ang, bx)
        pieces.append((mesh.translate(bv, pt[0], pt[1], pt[2]), bf))
    out["borescope_ports"] = mesh.join(*pieces)

    # ---- T5 thermocouple rake ring round the low turbine -------------------
    pieces = []
    xr = S["lpt_exit"] + 40.0
    rr = _casing_outer(xr)
    for i in range(8):
        a = 360.0 * i / 8.0 + 22.5
        p0 = _at(rr - 26.0, a, xr)
        p1 = _at(rr + 18.0, a, xr)
        pieces.append(mesh.pipe([p0, p1], 7.0, segments=10))
        hb, hf = mesh.revolve_closed(
            [(-5.0, 5.0), (9.0, 5.0), (9.0, 15.0), (14.0, 15.0),
             (14.0, 19.0), (-5.0, 19.0)], segments=12)
        hb = mesh.rot_x(mesh.rot_z(hb, math.pi / 2), math.radians(a))
        pieces.append((mesh.translate(hb, p1[0], p1[1], p1[2]), hf))
    # the harness ring collecting all eight leads
    hv, hf = mesh.ring_torus(xr + 26.0, rr + 16.0, 8.0, 64, 10)
    pieces.append((hv, hf))
    out["t5_harness"] = mesh.join(*pieces)

    # ---- anti-ice: hot air taken off the compressor and run to the inlet ----
    pieces = []
    ang = -150.0
    path = [_at(_casing_outer(x) + 34.0, ang, x)
            for x in (1180.0, 900.0, 620.0, 300.0, 40.0, -180.0, -330.0)]
    pieces.append(mesh.pipe(path, 22.0, segments=16))
    # the shutoff valve in the run, and the manifold ring at the lip
    vv, vf = mesh.revolve_closed(
        [(-30.0, 12.0), (30.0, 12.0), (30.0, 40.0), (16.0, 46.0),
         (-16.0, 46.0), (-30.0, 40.0)], segments=18)
    vv = mesh.rot_x(mesh.rot_z(vv, math.pi / 2), math.radians(ang))
    vp = _at(_casing_outer(760.0) + 34.0, ang, 760.0)
    pieces.append((mesh.translate(vv, vp[0], vp[1], vp[2]), vf))
    # inside the inlet bore, feeding the lip -- at r=578 with a 15 mm tube it
    # ran straight through the inlet case, whose bore is at 586
    mv, mf = mesh.ring_torus(-352.0, 556.0, 12.0, 64, 10)
    pieces.append((mv, mf))
    out["antiice_duct"] = mesh.join(*pieces)

    # ---- inlet probes: what the control actually measures ------------------
    pieces = []
    for (a, ln) in ((70.0, 96.0), (-70.0, 96.0), (170.0, 74.0)):
        base = _at(586.0, a, -300.0)
        tip = _at(586.0 - ln, a, -300.0)
        pieces.append(mesh.pipe([base, tip], 9.0, segments=12))
        hv, hf = mesh.revolve_closed(
            [(-6.0, 6.0), (8.0, 6.0), (8.0, 18.0), (14.0, 18.0),
             (14.0, 24.0), (-6.0, 24.0)], segments=14)
        hv = mesh.rot_x(mesh.rot_z(hv, math.pi / 2), math.radians(a))
        pieces.append((mesh.translate(hv, base[0], base[1], base[2]), hf))
    out["inlet_probes"] = mesh.join(*pieces)

    # ---- fan containment wrap: the case is only half the story -------------
    # over the fan disc plane only: run aft to x=330 it sat under the oil tank
    # and the cowl door, both of which mount on the same stretch of case
    # The variable-vane actuation carries unison rings out at r 632-635 over
    # x -100..0 and x 200..300, with a clear band between them. The wrap ran
    # to 250 and went straight through the aft ring.
    xc0, xc1 = 20.0, 190.0
    layers = []
    for i, (f0, f1) in enumerate(((0.0, 1.0), (0.08, 0.92), (0.16, 0.84))):
        a0 = xc0 + (xc1 - xc0) * f0
        a1 = xc0 + (xc1 - xc0) * f1
        t = 9.0 - i * 1.0
        r = _casing_outer(0.5 * (a0 + a1)) + 2.0 + i * 8.0
        layers.append(mesh.revolve_closed(
            [(a0, r), (a1, r), (a1, r + t), (a0, r + t)], segments=72))
    out["fan_containment"] = mesh.join(*layers)

    # ---- rear mount: the front trunnions cannot carry thrust alone ---------
    # Two side links into the turbine rear frame take thrust and side load;
    # the front trunnions take vertical only. Without these the engine was
    # hanging off one station.
    pieces = []
    xm = S["turbine_frame"]
    rm = _casing_outer(xm)
    for sgn in (-1.0, 1.0):
        a = 34.0 * sgn
        p0 = _at(rm + 6.0, a, xm - 30.0)
        p1 = _at(rm + 128.0, a * 0.42, xm + 96.0)
        pieces.append(mesh.pipe([p0, p1], 19.0, segments=14))
        # clevis at each end of the link
        for pt in (p0, p1):
            cv, cf = mesh.revolve_closed(
                [(-16.0, 10.0), (16.0, 10.0), (16.0, 30.0), (8.0, 34.0),
                 (-8.0, 34.0), (-16.0, 30.0)], segments=16)
            cv = mesh.rot_x(mesh.rot_z(cv, math.pi / 2), math.radians(a))
            pieces.append((mesh.translate(cv, pt[0], pt[1], pt[2]), cf))
        # the pad it bolts to on the frame
        bv, bf = mesh.revolve_closed(
            [(xm - 54.0, rm - 2.0), (xm + 6.0, rm - 2.0),
             (xm + 6.0, rm + 16.0), (xm - 54.0, rm + 16.0)],
            segments=10, phase=math.radians(a - 13.0), sweep=math.radians(26.0))
        pieces.append((bv, bf))
    out["mount_links_rear"] = mesh.join(*pieces)

    # ---- turbine case cooling: impingement rings that hold tip clearance ---
    pieces = []
    for xr in (S["hpt_inlet"] + 40.0, S["hpt_exit"] + 20.0,
               S["lpt_inlet"] + 40.0, S["lpt_exit"] - 60.0):
        r = _casing_outer(xr)
        mv, mf = mesh.ring_torus(xr, r + 26.0, 13.0, 56, 10)
        pieces.append((mv, mf))
        # the spray tubes standing the ring off the case
        for i in range(10):
            a = 360.0 * i / 10.0 + 18.0
            pieces.append(mesh.pipe(
                [_at(r + 2.0, a, xr), _at(r + 26.0, a, xr)], 5.0, segments=8))
    # the supply riser feeding all four rings from the compressor bleed
    riser = [_at(_casing_outer(x) + 40.0, 8.0, x)
             for x in (1500.0, 1800.0, 2050.0, 2200.0, 2360.0)]
    pieces.append(mesh.pipe(riser, 15.0, segments=12))
    # and the delivery tubes into the turbine case. The manifold ran round
    # the outside at r 497 with the case at 490, so the cooling air it
    # collects had nowhere to go.
    for clock in (-60.0, 60.0, 180.0):
        for xd in (2080.0, 2300.0):
            pieces.append(mesh.pipe(
                [_at(520.0, clock, xd),
                 _at(spec.casing_inner("casing_turbine", xd) + 4.0, clock, xd)],
                13.0, 10))
    out["turbine_cooling_manifold"] = mesh.join(*pieces)

    # ---- bearing sumps: the bearings were modelled without housings --------
    def _scavenge_end(name, r_house, bx):
        """(radius, clock, x) where a sump's scavenge line ends.

        Every line used to run 60 mm straight out and 30 mm aft, whatever was
        there. No.1's went out through the nose it sits in, and No.3's aft
        into the compressor's front cone -- a static pipe through a rotor.
        Each now ends in the frame that carries its sump."""
        if name == "brg_1_lp_thrust":
            # into the centre-body's skin, inside the guide vanes' hub
            c = spec.CENTRE_BODY
            return c["hub_radius"] - c["wall"] - 2.0, -90.0, bx + 30.0
        if name == "brg_3_hp_thrust":
            # forward into the fan frame hub, away from the HPC front cone
            return r_house + 74.0, -90.0, bx - 30.0
        return r_house + 74.0, -90.0, bx + 30.0

    pieces = []
    for (name, bx, r_shaft, r_house, kind) in spec.BEARINGS:
        # the sealed can around the bearing, with its scavenge boss
        # +11, not +3: the shafts now carry raised bearing journals (5.7 mm on
        # the LP spool, 8.3 on the HP), so a housing bore 3 mm off the nominal
        # shaft diameter closed on the journal it is supposed to run on
        # No.4 sits right behind the HP turbine disc, whose bore reached 30 mm
        # into a sump that ran 54 mm ahead of the bearing; its can starts
        # just ahead of the race instead.
        # And No.2, the LP bearing just ahead of the HP spool, ran 54 mm aft
        # and closed round the first 14 mm of the HP shaft; it stops 4 mm
        # short of it.
        fore = 22.0 if name == "brg_4_hp_roller" else 54.0
        aft = 54.0
        if name == "brg_2_lp_roller":
            aft = spec.SHAFTS["hp_x0"] - 4.0 - bx
        hv, hf = mesh.revolve_closed(
            [(bx - fore, r_shaft + 11.0), (bx + aft, r_shaft + 11.0),
             (bx + aft, r_house + 4.0), (bx + aft - 14.0, r_house + 16.0),
             (bx - fore + 14.0, r_house + 16.0), (bx - fore, r_house + 4.0)],
            segments=40)
        pieces.append((hv, hf))
        # scavenge line off the bottom of the sump, out to the static
        # structure the sump hangs from
        p0 = _at(r_house + 14.0, -90.0, bx)
        p1 = _at(*_scavenge_end(name, r_house, bx))
        pieces.append(mesh.pipe([p0, p1], 8.0, segments=10))
    out["bearing_sumps"] = mesh.join(*pieces)

    return out
