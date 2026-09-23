"""External fittings on the full-size agility demonstrator.

Pitot and antennas, vortex generators, navigation lights, gear doors,
wheel hubs, flush access panels and the tailpipe shroud remain outside.
Flight-control actuation belongs under the skin, not in the airstream.

There are no stores and no pylons. This aircraft is built for speed and roll
rate, and a pylon is a bluff body hanging in the flow that buys neither.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import common, fuselage as fus

W = spec.WING
H = spec.HTAIL
V = spec.VTAIL
G = spec.GEAR
FL = spec.FLAPERON

# belongs in spec.py (next to SKIN_DETAIL): how far flush access relief may
# stand proud of the skin. 0.15 units is 5 mm full size -- a panel line plus a
# fastener row, which is all an inspection panel shows on a real airframe.
SKIN_RELIEF = spec.SKIN_DETAIL["relief"]


def build():
    out = {}
    out.update(_probes())
    out.update(_vg_rows())   # wing fences deleted: see _vg_rows
    out.update(_lights())
    out.update(_gear_doors())
    out.update(_access_panels())
    out.update(_exhaust_petals())
    out.update(_speed_kit())
    return out


def _probes():
    """A pitot boom on the nose, plus the UHF blade aerial."""
    out = {}
    P = spec.PROBE
    tip, z = P["pitot_tip"], P["pitot_z"]
    probe = shapes.pitot_probe((tip + 34.0, 0.0, z), (1.0, 0.0, 0.0),
                               length=28.0, r=P["pitot_r"], mast=7.0)
    cone = mesh.revolve_closed(
        [(tip, 0.0), (tip + 10.0, 0.0), (tip + 10.0, P["fairing_r"]),
         (tip + 7.0, P["fairing_r"]), (tip + 3.0, P["fairing_r"] * 0.72),
         (tip + 0.6, P["fairing_r"] * 0.30)], 24)
    cone = ([(x, y, z + zz) for (x, y, zz) in cone[0]], cone[1])
    out["pitot"] = mesh.join(probe, cone)

    # A blade aerial is sized from the spine it sits on, so it follows the
    # skin rather than being picked by eye, and the whole blade is well under
    # the shark fin a full-size UHF fit would hang there.
    ants = []
    x_a = 262.0
    base = fus.surface_point(x_a, 90.0)
    blade = [(base[0] - 11.0, base[1], base[2]),
             (base[0] - 6.0, base[1], base[2] + 7.5),
             (base[0] + 9.0, base[1], base[2] + 7.5),
             (base[0] + 11.0, base[1], base[2])]
    # a blade aerial is a moulded fin with a base flange, so it gets a
    # section and a rolled top edge rather than being a flat card
    prof = shapes.panel_outline(
        [(blade[0][0], blade[0][2]), (blade[3][0], blade[3][2]),
         (blade[2][0], blade[2][2]), (blade[1][0], blade[1][2])], subdiv=7)
    ants.append(shapes.shaped_panel(prof, base[1], 1.9, rim_seg=5))
    ants.append(shapes.rounded_box(base[0], base[1], base[2] - 0.4,
                                   26.0, 5.0, 1.4, 1.2, seg=5))
    out["antennas"] = mesh.join(*ants)
    return out


def _plate(loop, thickness):
    """Extrude a closed polygon in y into a thin plate.

    The loop is given in the x-z plane; callers that need the plate at another
    clock angle rotate the result about x afterwards.
    """
    t = thickness / 2
    n = len(loop)
    verts = ([(x, y - t, z) for (x, y, z) in loop]
             + [(x, y + t, z) for (x, y, z) in loop])
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j, n + i))
    return verts, faces


_SKIN = {}


def _skin_z(name, x, y, upper=True):
    """Height of the built wing skin at (x, y), read off the mesh itself.

    `common.surface_z` computes this from the aerofoil and z_root, and leaves
    out the washout: the sections are twisted 1.5 degrees towards the tip, so
    the further out you go the further its answer is from the skin. Measured
    on the built wing, every vortex generator was standing 1.4 units clear of
    it at the root and 2.2 at the tip -- 24 tabs, none of them touching the
    aeroplane, all of them in a render nobody could read at that size.

    Reading the part that is actually built cannot drift. This does mean the
    wing has to be built before the detail that lands on it, which is why the
    module is imported here and not at the top.

    It is the surface under the point, off the triangle there, not the
    highest vertex within 6 units: that is the crest of the section ahead of
    or behind the point, and it stood the vortex generators on the outer wing
    up to 2 mm clear of the skin sloping away beneath them.
    """
    if name not in _SKIN:
        from parts import wing
        _SKIN[name] = wing.build()[name]
    verts, faces = _SKIN[name]
    best = None
    for q in faces:
        for k in range(1, len(q) - 1):
            a, b, c = verts[q[0]], verts[q[k]], verts[q[k + 1]]
            det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(det) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / det
            l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / det
            l3 = 1.0 - l1 - l2
            if min(l1, l2, l3) < -1e-9:
                continue
            z = l1 * a[2] + l2 * b[2] + l3 * c[2]
            if best is None or (z > best if upper else z < best):
                best = z
    return best


def _vg_rows():
    """Retain the paired vortex generators; the wing fences go.

    The old fences stood 3.0 m tall over a third of the chord at full size --
    a fence, not a fillet, and no delta this size carries them. Leading-edge
    vortices are fed at the root by the LERX strakes, which already exist, so
    the fences bought drag and said nothing. The generator pairs stay: on a
    delta the outer wing sheds first and the generators re-energise that
    boundary layer so the flaperon keeps working. Both are sized off the
    local chord, so they stay in proportion out to the tip instead of turning
    into scenery.
    """
    D = spec.WING_DETAIL
    out = {}

    vgs = []
    n = D["n_vg"]
    for sgn in (-1.0, 1.0):
        for k in range(n):
            fr = D["vg_from"] + (D["vg_to"] - D["vg_from"]) * k / (n - 1)
            y = sgn * W["semi_span"] * fr
            chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], fr)
            h = chord * D["vg_h"]
            x_vg = x_le + chord * D["vg_x"]
            z0 = _skin_z(f"wing_{'lr'[0 if sgn < 0 else 1]}", x_vg, y)
            if z0 is None:
                z0 = common.surface_z(W, fr, D["vg_x"])
            # A vortex generator is a swept triangular vane, tall at the
            # back and faired into the skin at the front: that geometry is
            # what rolls the flow up into a discrete vortex. A rectangular
            # tab just trips the boundary layer and adds drag.
            c = chord * D["vg_chord"]
            prof = shapes.panel_outline(
                [(-c / 2, 0.0), (c / 2, 0.0), (c * 0.34, h),
                 (c * 0.10, h * 0.94)], subdiv=9)
            v, f = shapes.shaped_panel(prof, 0.0, D["vg_t"], rim_seg=5)
            v = [(px, py, pz - h / 2) for (px, py, pz) in v]
            # alternate the yaw so the pair sheds counter-rotating vortices
            a = math.radians(D["vg_yaw"] * (1 if k % 2 else -1))
            ca, sa = math.cos(a), math.sin(a)
            # bedded 1 mm (full size) into the skin, so its straight foot
            # still bears on a surface that curves under it
            vgs.append(([(x_le + chord * D["vg_x"] + px * ca - py * sa,
                          y + px * sa + py * ca, z0 - 0.03 + h / 2 + pz)
                         for (px, py, pz) in v], f))
    # a vortex generator is a separately bonded tab; there are 24 of them
    half = len(vgs) // 2
    for i, m in enumerate(vgs):
        out[f"vg_{'lr'[i // half]}{i % half + 1}"] = m
    return out


def _lights():
    """Navigation lights: red to port, green to starboard, white at the tail."""
    out = {}
    for tag, sgn in (("port", -1.0), ("stbd", 1.0)):
        y = sgn * (W["semi_span"] - 2.0)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], 1.0)
        chord = W["tip_chord"]
        # on the tip face itself, where a wingtip light actually goes
        v, f = shapes.rounded_box(x_le + chord * 0.30, y, W["z_root"], 9.0, 4.0, 3.0)
        out[f"navlight_{tag}"] = (v, f)
    v, f = shapes.rounded_box(V["x_root_le"] + V["root_chord"] - 8.0, 0.0, 96.0,
                    10.0, 6.0, 8.0)
    out["navlight_tail"] = (v, f)
    return out


def _gear_doors():
    """A door per leg, hinged open, each one its own part.

    All three used to be a single object called `gear_doors`, which is a
    convenience rather than a description: they are three separate panels on
    three separate hinges that open at different times. Each now carries its
    hinge line and the actuator link that swings it, because a door with
    neither is a plate stuck to the side of a hole.
    """
    out = {}
    w, hh, zc, _ = fus.station_at(G["nose_x"])
    # 40 mm, centred a little aft: the leg is ahead of the intake throat now
    # and a 54 mm door reached x 53, which is the chin inlet's own lip.
    lip_x = spec.INTAKE["x_lip"] + 12.0
    door_l = 40.0
    door_x = max(G["nose_x"] + 4.0, lip_x + door_l / 2)
    # hinged at the skin line and hanging DOWN. At zc - hh + 8 the panel's top
    # edge stood 23 mm up inside the fuselage, through the boundary-layer
    # diverter that sits under the chin inlet.
    door_d = 30.0
    # At the skin line, not at y 14. A gear door is a panel in the side of the
    # well; at y 14 it was inside the intake, whose outer wall is at y 20.3 at
    # this station.
    w_d, hh_d, zc_d, _ = fus.station_at(door_x)
    # 2.5 mm off the skin, not 4: the hinge knuckles stand 1.5 mm inboard of
    # the panel and at 4 they were inside the inlet, whose outer wall is at
    # y 20.1 here. And the link that opens it reaches inboard, so on this door
    # it is short -- there is nothing but intake to reach towards.
    out["gear_door_n"] = _door(door_x, w_d - 2.5,
                               zc_d - hh_d - door_d / 2 + 3.0,
                               door_l, door_d, 1.0, link=1.5)
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        w, hh, zc, _ = fus.station_at(G["main_x"])
        out[f"gear_door_{side}"] = _door(
            G["main_x"], sgn * (G["main_y"] - 14.0),
            zc - hh * 0.55 - 12.0, 62.0, 34.0, sgn)
    return out


def _door(x, y, z, length, depth, sgn, link=6.0):
    """One door: the panel, a piano hinge down its inboard edge, and the rod
    that holds it open."""
    parts = [shapes.rounded_box(x, y, z, length, 3.0, depth, r=1.2)]
    # piano hinge: a knuckle every few millimetres along the top edge
    n = 7
    for k in range(n):
        f = (k + 0.5) / n
        hv, hf = mesh.revolve_closed(
            [(0.0, 0.0), (length / n * 0.62, 0.0),
             (length / n * 0.62, 1.5), (0.0, 1.5)], 12)
        parts.append(([(px + x - length / 2 + length * f,
                        pz + y - sgn * 1.5, py + z + depth / 2)
                       for (px, py, pz) in hv], hf))
    # the link from the door to the leg, which is what opens it. It stops
    # short of the intake duct: the mains are either side of it and the duct
    # is 21 mm wide at that station.
    parts.append(mesh.pipe(
        [(x - length * 0.22, y - sgn * 2.0, z + depth * 0.30),
         (x - length * 0.30, y - sgn * link, z + depth * 0.46)], 0.8, 10))
    return mesh.join(*parts)


# _wheel_hubs used to live here and built wheel_hub_n / _ml / _mr from
# spec.GEAR's nose_x and leg lengths. gear.py now builds the legs and their
# hubs together, so these were a second set computed from geometry that had
# stopped matching: wheel_hub_n came out at station 72 with the nose wheel it
# belonged to at station 40, and rendered as a spoked star hanging in the air
# under the forward fuselage, attached to nothing.


def _hub(x, y, z, r, half_w, spokes):
    """A wheel turns about the y axis, so the hub barrel runs in y and the
    spokes lie in the x-z plane. Rotating them about x instead puts them out
    sideways past the tyre, which is what the first version did.
    """
    parts = []
    bv, bf = mesh.cylinder(-half_w, half_w, r * 0.42, 14)
    parts.append(([(py + x, px + y, pz + z) for (px, py, pz) in bv], bf))
    for k in range(spokes):
        a = 2 * math.pi * k / spokes
        ca, sa = math.cos(a), math.sin(a)
        sv, sf = shapes.rounded_box(r * 0.62, 0.0, 0.0, r * 0.80, half_w * 1.1, 2.2)
        parts.append(([(x + px * ca - pz * sa, y + py,
                        z + px * sa + pz * ca) for (px, py, pz) in sv], sf))
    return mesh.join(*parts)


def _access_panels():
    """Access panels as flush relief, not raised hatches.

    The old panels were `surface_patch` shells standing 0.8-0.9 units off the
    skin -- 27-30 mm at full size, a crate on the spine rather than an
    inspection panel. A full-size access panel is flush: it reads as a panel
    line and a fastener row, nothing more. Emitted here as shallow relief
    whose top face sits just above the skin (0.15 units, 5 mm full size) and
    whose skirt runs under it, so the relief never floats; the build boolean
    later buries the skirt in the skin it follows.
    """
    out = {}
    t = SKIN_RELIEF
    for (name, x0, x1, a0, a1) in spec.SKIN_DETAIL["panels"]:
        out[f"panel_{name}"] = _panel_relief(x0, x1, a0, a1, t)
    return out


def _panel_relief(x0, x1, a0, a1, height, nx=6, na=6):
    """One flush panel: a shallow cap over the skin it covers.

    The same loft `fuselage.surface_patch` uses, driven by the same
    `surface_point` solve, with the outer ring 0.15 units out instead of a
    hatch-height standoff. The inner ring runs 0.6 units UNDER the skin so
    the relief is bonded to it rather than floating over it -- the build
    boolean subtracts the skin, and what survives is the proud face.
    """
    inner, outer = [], []
    for i in range(nx):
        x = x0 + (x1 - x0) * i / (nx - 1)
        for j in range(na):
            a = a0 + (a1 - a0) * j / (na - 1)
            inner.append(fus.surface_point(x, a, -0.6))
            outer.append(fus.surface_point(x, a, height))
    verts = inner + outer
    off = len(inner)
    faces = []
    for i in range(nx - 1):
        for j in range(na - 1):
            k = i * na + j
            faces.append((k, k + 1, k + na + 1, k + na))
            faces.append((off + k, off + k + na, off + k + na + 1, off + k + 1))
    for i in range(nx - 1):
        for j in (0, na - 1):
            k = i * na + j
            if j == 0:
                faces.append((k, k + na, off + k + na, off + k))
            else:
                faces.append((k + na, k, off + k, off + k + na))
    for j in range(na - 1):
        for i in (0, nx - 1):
            k = i * na + j
            if i == 0:
                faces.append((k + 1, k, off + k, off + k + 1))
            else:
                faces.append((k, k + 1, off + k + 1, off + k))
    return verts, faces


def _exhaust_petals():
    """The tailpipe shroud: the fairing between the fuselage skin and the
    engine's own nozzle, split into petals like the real thing.

    The engine already carries convergent and divergent flaps out to its exit
    plane, so this is a shroud around them, not a second nozzle.
    """
    T = spec.TAILPIPE
    # round the thrust line, which is the engine's nozzle's axis and the
    # fuselage's own centre here. It was built round z 0, a unit above
    # both, so the top petals were in the skin and the bottom four hung a
    # unit clear of it and of everything else.
    zc = spec.ENGINE_Z
    parts = []
    n = T["petals"]
    step = 2 * math.pi / n
    for k in range(n):
        a0 = k * step + step * 0.08
        a1 = (k + 1) * step - step * 0.08
        seg = []
        for (x, r) in ((T["x_front"], T["r_front"]),
                       (T["x_rear"], T["r_rear"])):
            for rr in (r - T["wall"], r):
                seg.append((x, rr, a0, a1))
        # Each tile follows the cone it sits on, so it is curved in section
        # and tapered along its length. Four corners made it a flat card
        # standing off a round tailpipe on its own edges.
        n_a, n_x = 7, 5
        rings = []
        for i in range(n_x):
            fx = i / (n_x - 1)
            x = T["x_front"] + (T["x_rear"] - T["x_front"]) * fx
            r_o = T["r_front"] + (T["r_rear"] - T["r_front"]) * fx
            r_i = r_o - T["wall"]
            loop = []
            for j in range(n_a):
                g = j / (n_a - 1)
                a = a0 + (a1 - a0) * g
                loop.append((x, r_o * math.cos(a), zc + r_o * math.sin(a)))
            for j in range(n_a - 1, -1, -1):
                g = j / (n_a - 1)
                a = a0 + (a1 - a0) * g
                loop.append((x, r_i * math.cos(a), zc + r_i * math.sin(a)))
            rings.append(loop)
        m = len(rings[0])
        verts = [v for r in rings for v in r]
        faces = []
        for i in range(n_x - 1):
            a, b = i * m, (i + 1) * m
            for k in range(m):
                k2 = (k + 1) % m
                faces.append((a + k, a + k2, b + k2, b + k))
        faces.append(tuple(range(m - 1, -1, -1)))
        base = (n_x - 1) * m
        faces.append(tuple(range(base, base + m)))
        parts.append((verts, faces))
    return {"tailpipe_shroud": mesh.join(*parts)}


def _speed_kit():
    """The parts that buy speed and roll rate, which is what this aircraft is
    for: a sharp-lipped intake, a boundary-layer diverter, leading-edge root
    extensions that keep the wing flying at high alpha, and a tailplane that
    is not shadowed by anything.
    """
    out = {}
    W = spec.WING
    I = spec.INTAKE

    # Boundary-layer diverter: the fuselage grows a sluggish layer of air and
    # feeding it to the engine costs thrust. A splitter plate stands the
    # intake off the skin so it swallows clean air.
    # It is a wedge in plan, pointed at the front so the boundary layer
    # splits cleanly and widening aft so what it splits off is pushed out
    # past the intake rather than allowed to close back in behind it.
    xl, zl = I["x_lip"], I["z_lip"] + 12.0
    hw = I["lip_width"] / 2 * 0.82
    prof = shapes.panel_outline(
        [(xl - 22.0, 0.0), (xl - 6.0, -hw * 0.52), (xl + 16.0, -hw * 0.92),
         (xl + 56.0, -hw), (xl + 56.0, hw), (xl + 16.0, hw * 0.92),
         (xl - 6.0, hw * 0.52)], subdiv=7)
    out["bl_diverter"] = shapes.shaped_panel(
        prof, zl, 2.6, rim_seg=5, axis="z",
        bow=lambda fx, fz: -1.1 * (1.0 - fx))

    # No lip ring here. intake.py builds the inlet lip as a rolled rim --
    # a closed section wrapping from the outer skin round to the duct wall,
    # which is what a cowl lip is. This was eighteen 1.5-unit pipes laid
    # round the same rim, and at full size they read as a ring of studs
    # sticking out of the mouth. Two lips, one of them wrong.

    # Vortex generators are already on the wing; these are the strakes that
    # start the LERX vortex at the nose.
    strakes = []
    for sgn in (-1.0, 1.0):
        pts = []
        for i in range(7):
            f = i / 6
            x = 96.0 + 96.0 * f
            w, hh, zc, _ = fus.station_at(x)
            pts.append((x, sgn * (w + 1.5 + 5.0 * f), zc + hh * 0.30))
        # A nose strake works by having a sharp edge to shed from. A round
        # rod of the same size is a drag item that sheds nothing.
        strakes.append(shapes.swept_profile(
            pts, shapes.rounded_polygon(
                [(-0.5, -2.4), (3.4, 0.0), (-0.5, 2.4)],
                [0.35, 0.12, 0.35], seg=5), subdiv=3))
    out["nose_strakes"] = mesh.join(*strakes)
    return out
