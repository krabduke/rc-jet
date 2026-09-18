"""The systems this airframe needs and did not have.

An RC jet is not a shell with an engine in it. It has a fuel system, a
retract system, a cooling path for the electronics, an antenna installation
that actually works, and a cockpit somebody looked at. None of that was
modelled, and several of the things that were modelled were joined into single
meshes that cannot be inspected or counted.

Everything here is placed off the real fuselage section, so nothing floats.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common, fuselage as fus, intake

W = spec.WING
V = spec.VTAIL
H = spec.HTAIL
G = spec.GEAR
FL = spec.FLAPERON


def inside(x, fy, fz, clear=2.0):
    """A point at a fraction of the fuselage section, guaranteed inside it.

    fy and fz run -1 to 1 across the section. `clear` is the gap left to the
    skin. Placing hardware by typing coordinates is how twenty-six parts ended
    up poking through the fuselage; this makes that impossible, because the
    section is read at the part's own station rather than assumed.
    """
    w, h, zc, n = fus.station_at(x)
    w = max(w - spec.FUSELAGE_SKIN - clear, 0.5)
    h = max(h - spec.FUSELAGE_SKIN - clear, 0.5)
    # scale so the superellipse test is satisfied for any (fy, fz) in the unit
    # square, not just on the axes
    t = (abs(fy) ** n + abs(fz) ** n) ** (1.0 / n)
    k = 1.0 / max(t, 1.0)
    y = w * fy * k
    return (x, y, clear_duct(x, y, zc + h * fz * k, 0.0, clear))


def duct_roof(x, y):
    """Top of the intake duct's outer wall at this station and this y.

    Infinity if the duct is not there, and -inf if it reaches no further out
    than |y| -- in which case nothing has to move for it.
    """
    I = spec.INTAKE
    if not (I["x_throat"] - 2.0 <= x <= I["x_duct_end"] + 2.0):
        return float("-inf")
    w, h, zc = intake.duct_section(x)
    _, _, _, n = intake.duct_bore(x)
    t = abs(y) / w
    if t >= 1.0:
        return float("-inf")
    return zc + h * (1.0 - t ** n) ** (1.0 / n)


def clear_duct(x, y, z, half_y, half_z, gap=2.0):
    """Lift a box until it is out of the air the engine breathes.

    inside() and fits() have always known where the skin is and never where
    the duct is -- and on a nose-intake model the duct is what decides where
    there is room: 39 mm across a 62 mm fuselage. So everything placed through
    them that happened to land low ended up in the airflow. The avionics tray
    was 87 % inside it, both tail servos two thirds, all three gear door
    actuators, the flight pack's strap, the nose steering link along nearly
    its whole length.

    A box is checked at the y where the duct is tallest under it, which is the
    edge of the box nearest the centreline, because the duct's section is
    widest on its own axis.
    """
    y_near = 0.0 if abs(y) <= half_y else (abs(y) - half_y)
    roof = duct_roof(x, y_near)
    if roof == float("-inf"):
        return z
    need = roof + gap + half_z
    return max(z, need)


def fits(x, fy, fz, half_y, half_z, clear=2.0):
    """The centre for a box of the given half-extents, clear of the skin and
    clear of the intake duct."""
    w, h, zc, n = fus.station_at(x)
    w = max(w - spec.FUSELAGE_SKIN - clear, 0.5)
    h = max(h - spec.FUSELAGE_SKIN - clear, 0.5)
    y = max(-(w - half_y), min(w - half_y, w * fy))
    z = zc + max(-(h - half_z), min(h - half_z, h * fz))
    z = clear_duct(x, y, z, half_y, half_z)
    return (x, y, z)


def build():
    out = {}
    out.update(_fuel_system())
    out.update(_retracts())
    out.update(_cooling())
    out.update(_engine_bay())
    return out


# --------------------------------------------------------------------------

def _fuel_system():
    """A turbine burns kerosene, so there is a tank, a collector, a pump, a
    filter and the lines between them. It is most of the aircraft's
    consumable mass.

    This was written and then never called: `build()` listed the retracts,
    the cooling and the engine bay and nothing else, so the aeroplane carried
    no fuel at all -- 3.3 tonnes of it in the mass budget and not one litre
    of tankage in the geometry. The structure audit had been asking for a
    `fuel_tank` the whole time and reporting "expected 1, found 0".
    """
    out = {}
    x0, x1 = 196.0, 282.0

    # Two saddle tanks, one each side of the intake duct.
    #
    # This was a single lofted bladder on the centreline, 20 mm of half-width
    # and 14 of half-height at its widest -- which is inside the duct, whose
    # section here is y +/-20.6 and nearly the full height of the fuselage.
    # The fuel was in the airflow. What is actually free at these stations is
    # a slot about 10 mm wide between the duct wall and the skin on each side,
    # so that is where the fuel goes: the arrangement a nose-intake model of
    # this size has to use.
    DUCT_Y = 21.6                      # duct half-width plus a wall
    tank = []
    for side in (-1.0, 1.0):
        rings = []
        n_st = 22
        for i in range(n_st):
            f = i / (n_st - 1)
            x = x0 + (x1 - x0) * f
            w, h, zc, n = fus.station_at(x)
            dome = math.sin(math.pi * min(1.0, 0.06 + 0.94 * f)) ** 0.30
            y_in = DUCT_Y
            # station_at gives the OUTER half-width, so the skin and a
            # bonding clearance come off before the tank wall
            y_out = max(y_in + 1.0, w - spec.FUSELAGE_SKIN - 3.5)
            hz = (h - spec.FUSELAGE_SKIN * 2 - 6.0) * 0.50 * dome
            cy = side * (y_in + y_out) / 2.0
            hy = max(0.6, (y_out - y_in) / 2.0 * dome)
            # The fuselage is a superellipse, so the section pulls in at the
            # corners: a slot that reaches full width at full height is
            # outside the skin even though both extents look legal on their
            # own. Each point is clamped to the width the section actually
            # has at its own height.
            iw = w - spec.FUSELAGE_SKIN - 1.6
            ih = h - spec.FUSELAGE_SKIN - 1.6
            ring = []
            for k in range(26):
                ang = 2 * math.pi * k / 26
                ca, sa = math.cos(ang), math.sin(ang)
                zz = zc - 1.0 + hz * sa
                t = min(1.0, abs(zz - zc) / max(ih, 1e-6))
                y_lim = iw * max(0.0, 1.0 - t ** n) ** (1.0 / n)
                yy = cy + hy * ca
                if abs(yy) > y_lim:
                    yy = math.copysign(max(y_in + 0.5, y_lim), yy)
                ring.append((x, yy, zz))
            rings.append(ring)
        tank.append(_loft(rings))
        # filler on the outboard shoulder of each saddle
        # filler and vent standpipes, on the inboard shoulder of each saddle.
        # On the outboard shoulder their collar reached y = 29.7 where the
        # section only allows 28.9, so they were the part poking through.
        for (fx, r_) in ((x0 + 18.0, 3.0), (x1 - 14.0, 2.4)):
            w, h, zc, n = fus.station_at(fx)
            sv, sf = mesh.revolve_closed(
                [(0.0, 0.0), (10.0, 0.0), (10.0, r_), (8.0, r_ * 1.35),
                 (6.0, r_ * 1.35), (6.0, r_), (0.0, r_)], 18)
            tank.append(([(pz + fx, py + side * (DUCT_Y + 1.5),
                           px + zc + 5.0)
                          for (px, py, pz) in sv], sf))
    out["fuel_tank"] = mesh.join(*tank)

    # A hopper's job is to be the one place the pump never sees air, so it is
    # domed at both ends, has a standpipe vent out of the top and takes its
    # feed from the very bottom.
    #
    # It lives INSIDE the tank -- which is what a hopper is on an
    # installation this size: a baffled compartment with the clunk in it.
    # As a separate 37 mm bottle standing on end there was nowhere aft of
    # the tank for it to go that was not already duct, firewall or engine,
    # and it was in all three at once. The saddle it sits in is seven
    # millimetres wide, so it is a slim cylinder lying along the tank, not
    # a bottle.
    k = 0.36
    hop = [mesh.revolve_closed(
        [(x, r * k) for (x, r) in
         [(0.0, 0.0), (2.0, 0.0), (3.4, 5.2), (5.0, 8.0), (7.0, 9.0),
          (19.0, 9.0), (21.0, 8.0), (22.6, 5.2), (24.0, 0.0),
          (26.0, 0.0), (24.6, 5.6), (22.6, 8.4), (20.0, 8.0),
          (6.0, 8.0), (3.4, 8.4), (1.4, 5.6)]], 26)]
    # the standpipe vent out of the top and the feed union at the bottom,
    # which are the two things that make it a hopper and not a can
    hop.append(mesh.revolve_closed(
        [(9.0, 0.0), (9.0, 1.0), (15.0, 1.0), (15.0, 0.0)], 12))
    hv, hf = mesh.join(*hop)
    tank_mid = (x0 + x1) / 2
    hx, hy, hz = tank_mid - 10.0, -(DUCT_Y + 3.2), 0.0
    out["fuel_hopper"] = ([(px + hx - 13.0, py + hy, pz + hz)
                           for (px, py, pz) in hv], hf)
    px_, py_, pz_, pl, pw, ph = spec.equipment("fuel_pump")
    out["fuel_pump"] = shapes.rounded_box(px_, py_, pz_, pl, pw, ph, 3.0)
    # a filter is a clear bowl with a threaded cap at each end and the mesh
    # element visible inside it -- the element is the reason it exists
    out["fuel_filter"] = mesh.join(
        mesh.revolve_closed(
            [(0.0, 0.0), (22.0, 0.0), (22.0, 3.6), (20.0, 4.4),
             (18.0, 6.0), (4.0, 6.0), (2.0, 4.4), (0.0, 3.6)], 28),
        mesh.revolve_closed(
            [(3.0, 4.2), (19.0, 4.2), (19.0, 5.2), (3.0, 5.2)], 24),
        mesh.revolve_closed(
            [(-4.0, 0.0), (0.0, 0.0), (0.0, 2.6), (-2.4, 3.0),
             (-4.0, 3.0)], 16),
        mesh.revolve_closed(
            [(22.0, 0.0), (26.0, 0.0), (26.0, 3.0), (24.4, 3.0),
             (22.0, 2.6)], 16))
    # lying along the bay above the duct, not standing in it
    fx, fy_, fz_ = spec.equipment("fuel_filter")[:3]
    out["fuel_filter"] = ([(px + fx - 11.0, py + fy_, pz + fz_)
                           for (px, py, pz) in out["fuel_filter"][0]],
                          out["fuel_filter"][1])
    # tank -> filter -> pump -> the engine's fuel inlet, which is a union on
    # the OUTSIDE of the casing. The last run used to end on the engine's
    # centreline 24 mm inside the inlet flange, which put the fuel line
    # through the fan disc and down the LP shaft.
    fil = spec.equipment("fuel_filter")
    pmp = spec.equipment("fuel_pump")
    inlet = (spec.ENGINE_X + 16.0, 0.0, 22.0)
    out["fuel_lines"] = mesh.join(
        # hopper -> filter, forward along the tank's own saddle
        mesh.pipe([(hx - 4.0, hy, hz + 2.0),
                   (210.0, hy + 2.0, 6.0),
                   (fil[0] - 14.0, fil[1], fil[2])], 1.8, 16, subdiv=3),
        # filter -> pump, round the FRONT of the ECU rather than through it
        mesh.pipe([(fil[0] - 14.0, fil[1], fil[2]),
                   (149.0, fil[1] * 0.5, fil[2]),
                   (149.0, pmp[1] * 0.5, pmp[2]),
                   (pmp[0] - 12.0, pmp[1], pmp[2])], 1.8, 16, subdiv=3),
        # pump -> the engine's fuel union, over the tank and under the skin.
        # At z 25 the line's own wall reached 26.8, which is 0.24 mm outside
        # the skin's inner surface at that station.
        mesh.pipe([(pmp[0] + 12.0, pmp[1], pmp[2]),
                   (205.0, 19.0, 22.0), (250.0, 13.5, 22.0),
                   (292.0, 2.0, 24.0), inlet], 1.8, 16, subdiv=3))
    return out


def _retracts():
    """Electric retract units and their doors' actuators. The gear folds
    forward into the nose and inboard into the wing root, which is the only
    place a 300 mm span leaves for it."""
    out = {}
    rx_, ry_, rz_, rl, rw, rh = spec.equipment("retract_nose")
    out["retract_nose"] = shapes.rounded_box(rx_, ry_, rz_, rl, rw, rh, 3.0)
    # In the wing, where the leg is.
    #
    # `fits` places a box as a fraction of the FUSELAGE section, and at 40 %
    # of half width that is y 1 to 21 -- inside the intake duct, whose outer
    # wall at this station is at y 20.6. A retract is a mechanism with a motor
    # and a gearbox in it and it was sitting in the air path, driving a leg
    # whose trunnion is at y 44, twenty-three millimetres outboard of it.
    #
    # The wing is 18.5 mm thick at the gear station and the leg is already
    # there. The unit goes on the wing's own mean line, just inboard of the
    # trunnion, which is what the docstring above has always said it does.
    # The main legs' retract jacks and all four door actuators belong to
    # gear.py.
    #
    # They were built here as well, and `retract_main_l` and
    # `gear_main_retract_actuator_l` came out 0.19 units apart -- two jacks
    # in the same place on the same leg, one of them a 36x20x14 box from
    # when this was a model with electric retract units in it. gear.py's set
    # is the one that is a mechanism: a jack, a downlock, a drag stay, a side
    # stay and a trailing link per leg, with its own door hinges, latches and
    # actuators. Nothing here could tell, because every audit was asking
    # whether parts got in each other's way and two coincident actuators
    # read as one assembly.
    return out


def _avionics():
    """ECU, receiver battery, kill switch and the data link. A turbine needs
    its own controller and its own power, separate from the flight pack --
    losing the receiver should not stop the fuel pump mid-flameout."""
    out = {}
    # every box here comes out of spec.EQUIPMENT, which is the one place
    # the bay is laid out and the only one that knows where the duct is
    for name, r in (("turbine_ecu", 3.0), ("ecu_battery", 2.0),
                    ("kill_switch", 2.0),
                    ("receiver", 1.6), ("rx_battery", 3.0),
                    ("lipo_3s_900", 3.0)):
        x, y, z, l, w, h = spec.equipment(name)
        out[name] = shapes.rounded_box(x, y, z, l, w, h, r)
    # the tray has to fit the narrowest station it spans, not the widest
    w_min = min(fus.station_at(x)[0] for x in (214.0, 256.0, 298.0))
    tw = (w_min - spec.FUSELAGE_SKIN - 3.0) * 1.5
    out["avionics_tray"] = shapes.rounded_box(
        *fits(256.0, 0.0, 0.05, tw / 2, 2.0), 84.0, tw, 3.0, 6.0)
    return out


def _cooling():
    """A NACA inlet feeding the electronics bay, and its exit.

    A flush NACA duct is how you take air aboard without paying full scoop
    drag, which is why every fast model has them and no slow one does.
    """
    out = {}
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        # A NACA duct is a ramp that diverges in plan while it deepens. Built
        # as two rows lofted into a flat sheet it was a slot, and a slot
        # ingests the boundary layer it is supposed to spill. It sits in the
        # flank, so its width runs vertically and its depth cuts inboard.
        w, h, zc, n = fus.station_at(253.0)
        out[f"naca_inlet_{side}"] = shapes.naca_duct(
            236.0, 270.0, across=zc + 10.0, surface=sgn * (w - 1.0),
            width=20.0, depth=7.0, n=22, axis="y", sgn=sgn)
        # the exit: a flush louvred vent, not a block
        prof = shapes.panel_outline(
            [(298.0, 9.0), (316.0, 11.0), (316.0, 23.0), (298.0, 21.0)],
            subdiv=6)
        vent = shapes.shaped_panel(prof, sgn * 22.0, 3.0, rim_seg=4)
        out[f"cooling_exit_{side}"] = mesh.join(
            vent, shapes.louvre_bank(300.0, 314.0, sgn * 23.5, 11.0, 21.0,
                                     3, 12.0, 3.0, t=1.1, cant=24.0))
    return out


def _cockpit():
    """What is actually under an RC jet's canopy: the flight pack and the
    receiver, on a tray you can reach.

    There is no pilot, no seat and no HUD. This aircraft is flown from the
    ground, and modelling a person in it was a mistake -- the canopy is an
    access hatch, and the volume under it is the most useful dry, protected,
    reachable space on the airframe. Putting the flight battery there is also
    what sets the centre of gravity.
    """
    out = {}
    C = spec.CANOPY
    cx = (C["x_front"] + C["x_rear"]) / 2

    # The tray the flight pack straps to. It sits ON the duct's upper wall,
    # which is what the bay floor actually is this far forward -- it used to
    # be set from the section and ended up 2 mm inside the airflow.
    lx, ly, lz, ll, lw, lh = spec.equipment("lipo_3s_900")
    floor = intake.duct_top(lx + ll / 2) - 1.0
    out["access_tray"] = shapes.rounded_box(
        lx, 0.0, floor - 1.5, ll + 26.0, lw + 18.0, 3.0, 6.0)
    # the ply tray the receiver and its pack sit on, aft of the bulkhead
    # `rx_mount` used to be a second mounting block for the receiver pack,
    # placed under a flight pack that has since moved. The receiver sits on
    # the avionics tray with everything else, and a part whose only job was
    # to hold something that is no longer above it is not a part.
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        # on the canopy's own sill, not out at the fuselage's widest point:
        # at 0.72 of the section they were down in the equipment bay
        out[f"canopy_latch_{side}"] = shapes.rounded_box(
            *fits(C["x_rear"] - 12.0, sgn * 0.56, 0.83, 4.0, 3.0),
            14.0, 7.0, 6.0, 1.6)
    return out


def _engine_bay():
    """The mount, the firewall it bolts to, and the bypass slots that keep the
    tailcone cool. A turbine's casing runs hot enough to melt foam."""
    out = {}
    x = spec.ENGINE_X
    # The ring takes the whole thrust load into the firewall, so it is a
    # flanged collar with a boss at every bolt, not a plain washer.
    # The flange has to stay inside the skin: the fuselage is 28.1 mm in
    # half-width at this station and 1.2 mm of that is skin, so the outer
    # diameter stops at 25.
    ring = [mesh.revolve_closed(
        [(0.0, 19.0), (5.0, 19.0), (5.0, 21.0), (9.0, 21.0), (9.0, 23.6),
         (7.0, 25.0), (2.0, 25.0), (0.0, 23.6)], 40)]
    for k in range(8):
        a = 2 * math.pi * k / 8
        bv, bf = mesh.revolve_closed(
            [(0.0, 0.0), (6.0, 0.0), (6.0, 1.9), (4.4, 2.6),
             (0.0, 2.6)], 12)
        ring.append(([(px + 2.0, py + math.cos(a) * 22.2,
                       pz + math.sin(a) * 22.2) for (px, py, pz) in bv], bf))
    out["mount_ring"] = mesh.join(*ring)
    out["mount_ring"] = ([(px + x + 6.0, py, pz + spec.ENGINE_Z)
                                 for (px, py, pz) in out["mount_ring"][0]],
                                out["mount_ring"][1])
    rails = []
    for sgn in (-1.0, 1.0):
        p0 = inside(x + 6.0, sgn * 0.62, -0.52, 4.0)
        # +104, not +120: at 120 the rails ran aft to station 420 and the
        # nozzle actuators start at 414, so the rail the engine slides on
        # ended inside the mechanism that moves the nozzle.
        p1 = inside(x + 104.0, sgn * 0.62, -0.52, 4.0)
        # the rail the engine slides on, with a saddle clamp at each end and
        # two standoffs holding it off the skin
        rails.append(mesh.pipe([p0, p1], 2.6, 20, subdiv=4))
        for f in (0.06, 0.94):
            pc = tuple(p0[k] + (p1[k] - p0[k]) * f for k in range(3))
            cv, cf = mesh.revolve_closed(
                [(-3.6, 2.4), (3.6, 2.4), (3.6, 5.2), (2.6, 6.0),
                 (-2.6, 6.0), (-3.6, 5.2)], 22)
            rails.append((shapes.orient(cv, pc,
                                        tuple(p1[k] - p0[k] for k in range(3))),
                          cf))
        for f in (0.22, 0.72):
            pc = tuple(p0[k] + (p1[k] - p0[k]) * f for k in range(3))
            rails.append(mesh.pipe(
                [pc, (pc[0], pc[1] + sgn * 5.0, pc[2] - 5.0)], 1.8, 14))
    out["mount_rails"] = mesh.join(*rails)
    slots = []
    for i in range(6):
        f = (i + 0.5) / 6
        xx = x + 40.0 + 90.0 * f
        w, h, zc, n = fus.station_at(xx)
        for sgn in (-1.0, 1.0):
            slots.append(shapes.rounded_box(xx, sgn * (w - 1.0), zc + 6.0,
                                            12.0, 2.5, 5.0, 1.0))
    out["bypass_slots"] = mesh.join(*slots)
    return out


def _aerials():
    """Two receiver antennas at right angles, which is how diversity works,
    plus the GPS puck and the telemetry sensor."""
    out = {}
    out["antenna_a"] = shapes.whip_antenna((250.0, 12.0, 18.0),
                                          (262.0, 34.0, 26.0), 0.9)
    out["antenna_b"] = shapes.whip_antenna((250.0, -12.0, 18.0),
                                          (250.0, -18.0, 44.0), 0.9)
    # a GPS module is a ceramic patch under a domed radome on a base plate
    # One unit, not two. A separate GPS puck and a separate telemetry box
    # were two 19 mm boxes fighting for the same 16 mm of crown; every
    # receiver of this class has the GPS and the telemetry on one board.
    gx, gy, gz, gl, gw, gh = spec.equipment("telemetry_gps")
    dome = mesh.revolve_closed(
        [(0.0, 0.0), (1.6, 0.0), (1.6, 8.6), (3.4, 8.6), (4.6, 7.6),
         (5.2, 5.4), (5.4, 0.0)], 30)
    out["telemetry_gps"] = mesh.join(
        shapes.rounded_box(gx, gy, gz, gl, gw, gh, 2.0),
        ([(px + gx, py + gy, pz + gz + gh / 2) for (pz, py, px) in dome[0]],
         dome[1]))
    return out


# --------------------------------------------------------------------------

def _loft(rings):
    n = len(rings[0])
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n):
            j2 = (j + 1) % n
            faces.append((a + j, a + j2, b + j2, b + j))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = (len(rings) - 1) * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def _loft_open(rows, t):
    """Two-point rows given thickness along y."""
    nr = len(rows)
    lo, hi = [], []
    for r in rows:
        for p in r:
            s = 1.0 if p[1] >= 0 else -1.0
            lo.append((p[0], p[1] - s * t / 2, p[2]))
            hi.append((p[0], p[1] + s * t / 2, p[2]))
    verts = lo + hi
    o = len(lo)
    faces = []
    for i in range(nr - 1):
        a, b = i * 2, (i + 1) * 2
        faces.append((a, a + 1, b + 1, b))
        faces.append((o + a, o + b, o + b + 1, o + a + 1))
        faces.append((a, b, o + b, o + a))
        faces.append((a + 1, o + a + 1, o + b + 1, b + 1))
    faces.append((0, o, o + 1, 1))
    e = (nr - 1) * 2
    faces.append((e, e + 1, o + e + 1, o + e))
    return verts, faces
