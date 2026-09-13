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
    rings = []
    for i in range(7):
        f = i / 6
        x = x0 + (x1 - x0) * f
        w, h, zc, n = fus.station_at(x)
        ring = []
        for k in range(14):
            a = 2 * math.pi * k / 14
            ring.append((x, (w - 7.0) * 0.80 * math.cos(a),
                         zc + (h - 7.0) * 0.62 * math.sin(a) - 2.0))
        rings.append(ring)
    out["fuel_tank"] = _loft(rings)

    out["fuel_hopper"] = mesh.revolve_open(
        [(0.0, 0.0), (0.0, 9.0), (26.0, 9.0), (26.0, 0.0)], 12,
        cap_start=True, cap_end=True)
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
    out["fuel_filter"] = mesh.revolve_open(
        [(0.0, 0.0), (0.0, 6.0), (22.0, 6.0), (22.0, 0.0)], 10,
        cap_start=True, cap_end=True)
    fx, fy_, fz_ = fits(296.0, -0.55, 0.0, 6.0, 11.0, 4.0)
    out["fuel_filter"] = ([(pz + fx, py + fy_, px + fz_ - 11.0)
                           for (px, py, pz) in out["fuel_filter"][0]],
                          out["fuel_filter"][1])
    a = inside(292.0, 0.0, -0.25, 3.0)
    b = inside(300.0, 0.45, 0.30, 4.0)
    c = inside(302.0, -0.45, -0.30, 4.0)
    d = inside(296.0, -0.55, 0.30, 5.0)
    out["fuel_lines"] = mesh.join(
        mesh.pipe([a, b], 1.8, 6), mesh.pipe([b, c], 1.8, 6),
        mesh.pipe([c, d], 1.8, 6),
        mesh.pipe([d, (spec.ENGINE_X + 24.0, 0.0, 0.0)], 1.8, 6))
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
        out[f"gear_door_actuator_{side}"] = mesh.pipe(
            [inside(G["main_x"] - 16.0, sgn * 0.45, -0.35, 4.0),
             inside(G["main_x"] + 6.0, sgn * 0.62, -0.55, 4.0)], 2.0, 6)
    out["gear_door_actuator_n"] = mesh.pipe(
        [inside(G["nose_x"] - 14.0, 0.40, -0.40, 4.0),
         inside(G["nose_x"] + 4.0, 0.52, -0.58, 4.0)], 2.0, 6)
    return out


def _avionics():
    """ECU, receiver battery, kill switch and the data link. A turbine needs
    its own controller and its own power, separate from the flight pack --
    losing the receiver should not stop the fuel pump mid-flameout."""
    out = {}
    out["turbine_ecu"] = shapes.rounded_box(
        *fits(268.0, 0.48, 0.34, 12.0, 6.0), 42.0, 24.0, 12.0, 3.0)
    out["ecu_battery"] = shapes.rounded_box(
        *fits(258.0, -0.48, 0.34, 10.0, 6.0), 38.0, 20.0, 12.0, 3.0)
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
        rows = []
        for i in range(6):
            f = i / 5
            x = 236.0 + 34.0 * f
            w, h, zc, n = fus.station_at(x)
            half = 10.0 * f
            depth = 6.0 * f
            rows.append([(x, sgn * (w - depth) - sgn * 0.0, zc + 10.0 - half),
                         (x, sgn * (w - depth), zc + 10.0 + half)])
        out[f"naca_inlet_{side}"] = _loft_open(rows, 1.2)
        out[f"cooling_exit_{side}"] = shapes.rounded_box(
            306.0, sgn * 22.0, 16.0, 18.0, 4.0, 12.0, 2.0)
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
    out["rx_mount"] = shapes.rounded_box(
        *fits(cx + 24.0, 0.0, 0.10, 13.0, 6.0), 30.0, 24.0, 10.0, 3.0)
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
    out["mount_ring"] = mesh.revolve_open(
        [(0.0, 19.0), (0.0, 24.0), (5.0, 24.0), (5.0, 19.0)], 16,
        cap_start=True, cap_end=True)
    out["mount_ring"] = ([(px + x + 6.0, py, pz + spec.ENGINE_Z)
                                 for (px, py, pz) in out["mount_ring"][0]],
                                out["mount_ring"][1])
    rails = []
    for sgn in (-1.0, 1.0):
        rails.append(mesh.pipe([inside(x + 6.0, sgn * 0.62, -0.52, 4.0),
                                inside(x + 120.0, sgn * 0.62, -0.52, 4.0)],
                               2.6, 6))
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
    out["antenna_a"] = mesh.pipe([(250.0, 12.0, 18.0), (262.0, 34.0, 26.0)],
                                 0.9, 6)
    out["antenna_b"] = mesh.pipe([(250.0, -12.0, 18.0), (250.0, -18.0, 44.0)],
                                 0.9, 6)
    out["gps_puck"] = shapes.rounded_box(
        *fits(214.0, 0.0, 0.62, 9.0, 3.0), 18.0, 18.0, 5.0, 3.0)
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
