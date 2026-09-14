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
from parts import common, fuselage as fus

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
    return (x, w * fy * k, zc + h * fz * k)


def fits(x, fy, fz, half_y, half_z, clear=2.0):
    """The centre for a box of the given half-extents, pulled in so its
    corners clear the skin too."""
    w, h, zc, n = fus.station_at(x)
    w = max(w - spec.FUSELAGE_SKIN - clear, 0.5)
    h = max(h - spec.FUSELAGE_SKIN - clear, 0.5)
    y = max(-(w - half_y), min(w - half_y, w * fy))
    z = zc + max(-(h - half_z), min(h - half_z, h * fz))
    return (x, y, z)


def build():
    out = {}
    out.update(_fuel_system())
    out.update(_retracts())
    out.update(_avionics())
    out.update(_cooling())
    out.update(_cockpit())
    out.update(_engine_bay())
    out.update(_aerials())
    return out


# --------------------------------------------------------------------------

def _fuel_system():
    """A turbine burns kerosene, so there is a tank, a hopper, a pump, a
    filter and the lines between them. An electric ducted fan would not need
    this; a turbine very much does, and it is most of the aircraft's
    consumable mass.
    """
    out = {}
    x0, x1 = 196.0, 292.0

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
    out["fuel_hopper"] = mesh.join(
        mesh.revolve_closed(
            [(0.0, 0.0), (2.0, 0.0), (3.4, 5.2), (5.0, 8.0), (7.0, 9.0),
             (19.0, 9.0), (21.0, 8.0), (22.6, 5.2), (24.0, 0.0),
             (26.0, 0.0), (24.6, 5.6), (22.6, 8.4), (20.0, 8.0),
             (6.0, 8.0), (3.4, 8.4), (1.4, 5.6)], 30),
        mesh.revolve_closed(
            [(26.0, 0.0), (32.0, 0.0), (32.0, 2.4), (30.0, 3.0),
             (28.0, 3.0), (26.0, 2.4)], 16),
        mesh.revolve_closed(
            [(-5.0, 0.0), (0.0, 0.0), (0.0, 2.4), (-3.0, 2.8),
             (-5.0, 2.8)], 16))
    # The lathe runs along its own +x, and the remap below sends that to z --
    # so the part is 26 mm TALL, not 26 mm long, and its half-extent in z is
    # 13 mm about a centre 13 mm above the placed point. Placing it as if it
    # were 9 mm in every direction pushed it through the top of the fuselage.
    hx, hy, hz = fits(300.0, 0.45, 0.0, 9.0, 13.0, 3.0)
    out["fuel_hopper"] = ([(pz + hx, py + hy, px + hz - 13.0)
                           for (px, py, pz) in out["fuel_hopper"][0]],
                          out["fuel_hopper"][1])
    out["fuel_pump"] = shapes.rounded_box(
        *fits(302.0, -0.45, -0.30, 7.0, 7.0), 26.0, 14.0, 14.0, 3.0)
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
    fx, fy_, fz_ = fits(296.0, -0.55, 0.0, 6.0, 11.0, 4.0)
    out["fuel_filter"] = ([(pz + fx, py + fy_, px + fz_ - 11.0)
                           for (px, py, pz) in out["fuel_filter"][0]],
                          out["fuel_filter"][1])
    a = inside(292.0, 0.0, -0.25, 3.0)
    b = inside(300.0, 0.45, 0.30, 4.0)
    c = inside(302.0, -0.45, -0.30, 4.0)
    d = inside(296.0, -0.55, 0.30, 5.0)
    out["fuel_lines"] = mesh.join(
        mesh.pipe([a, b], 1.8, 16, subdiv=3),
        mesh.pipe([b, c], 1.8, 16, subdiv=3),
        mesh.pipe([c, d], 1.8, 16, subdiv=3),
        mesh.pipe([d, (spec.ENGINE_X + 24.0, 0.0, 0.0)], 1.8, 16, subdiv=3))
    return out


def _retracts():
    """Electric retract units and their doors' actuators. The gear folds
    forward into the nose and inboard into the wing root, which is the only
    place a 300 mm span leaves for it."""
    out = {}
    out["retract_nose"] = shapes.rounded_box(
        *fits(G["nose_x"] + 12.0, 0.0, -0.45, 9.0, 8.0), 34.0, 18.0, 16.0, 3.0)
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        out[f"retract_main_{side}"] = shapes.rounded_box(
            *fits(G["main_x"] + 10.0, sgn * 0.50, -0.40, 10.0, 8.0),
            36.0, 20.0, 16.0, 3.0)
        out[f"gear_door_actuator_{side}"] = shapes.linear_actuator(
            inside(G["main_x"] - 16.0, sgn * 0.45, -0.35, 4.0),
            inside(G["main_x"] + 6.0, sgn * 0.62, -0.55, 4.0), 2.0)
    out["gear_door_actuator_n"] = shapes.linear_actuator(
        inside(G["nose_x"] - 14.0, 0.40, -0.40, 4.0),
        inside(G["nose_x"] + 4.0, 0.52, -0.58, 4.0), 2.0)
    return out


def _avionics():
    """ECU, receiver battery, kill switch and the data link. A turbine needs
    its own controller and its own power, separate from the flight pack --
    losing the receiver should not stop the fuel pump mid-flameout."""
    out = {}
    out["turbine_ecu"] = shapes.rounded_box(
        *fits(268.0, 0.48, 0.34, 12.0, 6.0), 42.0, 24.0, 12.0, 3.0)
    # on the crown, above the duct. The side slots beside the duct are about
    # ten millimetres wide and the fuel is in them; there is nowhere else at
    # this station for a pack this size to go.
    out["ecu_battery"] = shapes.rounded_box(
        *fits(258.0, 0.0, 0.95, 13.0, 4.0), 38.0, 26.0, 8.0, 2.0)
    out["kill_switch"] = shapes.rounded_box(
        *fits(236.0, 0.60, 0.46, 5.0, 4.0), 16.0, 10.0, 8.0, 2.0)
    out["data_link"] = shapes.rounded_box(
        *fits(246.0, -0.60, 0.42, 7.0, 4.0), 22.0, 14.0, 8.0, 2.0)
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

    out["access_tray"] = shapes.rounded_box(
        *fits(cx, 0.0, -0.35, 30.0, 2.0), 96.0, 56.0, 3.0, 6.0)
    out["rx_battery"] = shapes.rounded_box(
        *fits(cx - 26.0, 0.0, 0.10, 16.0, 8.0), 46.0, 30.0, 14.0, 4.0)
    # under the flight pack, which now sits high because the intake duct has
    # the bottom of the section this far forward
    out["rx_mount"] = shapes.rounded_box(
        *fits(cx + 24.0, 0.0, -0.62, 13.0, 6.0), 30.0, 24.0, 10.0, 3.0)
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        out[f"canopy_latch_{side}"] = shapes.rounded_box(
            *fits(C["x_rear"] - 12.0, sgn * 0.72, 0.45, 4.0, 3.0),
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
        p1 = inside(x + 120.0, sgn * 0.62, -0.52, 4.0)
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
    gx, gy, gz = fits(214.0, 0.0, 0.62, 9.0, 3.0)
    dome = mesh.revolve_closed(
        [(0.0, 0.0), (1.6, 0.0), (1.6, 8.6), (3.4, 8.6), (4.6, 7.6),
         (5.2, 5.4), (5.4, 0.0)], 30)
    out["gps_puck"] = mesh.join(
        ([(px + gx, py + gy, pz + gz - 2.4) for (pz, py, px) in dome[0]],
         dome[1]),
        shapes.rounded_box(gx, gy, gz - 3.0, 19.0, 19.0, 1.8, 2.0, seg=5))
    out["telemetry_sensor"] = shapes.rounded_box(
        *fits(276.0, 0.56, 0.40, 4.0, 3.0), 14.0, 8.0, 6.0, 2.0)
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
