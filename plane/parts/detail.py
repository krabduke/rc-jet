"""Airframe detail: the parts that make a model look like an aircraft.

Control horns and linkages, pitot and antennas, wing fences and vortex
generators, navigation lights, gear doors, wheel hubs, access panels, the
tailpipe shroud and static dischargers.

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


def build():
    out = {}
    out.update(_control_horns())
    out.update(_probes())
    out.update(_fences())
    out.update(_lights())
    out.update(_gear_doors())
    out.update(_wheel_hubs())
    out.update(_access_panels())
    out.update(_exhaust_petals())
    out.update(_dischargers())
    out.update(_cockpit())
    out.update(_speed_kit())
    out.update(_linkages())
    return out


def _control_horns():
    """A horn and clevis on every moving surface, where the pushrod meets it."""
    horns, clevises = [], []

    def horn(x, y, z, h=None):
        h = spec.WING_DETAIL["horn_h"] if h is None else h
        sgn_h = 1.0 if h >= 0 else -1.0
        # A horn is a moulded arm with a row of holes in it: the hole you use
        # sets the throw, and the base flange is what bonds it to the surface.
        arm = shapes.panel_outline(
            [(x - 3.4, z), (x + 3.4, z),
             (x + 2.4, z + h * 0.62), (x + 1.8, z + h),
             (x - 1.8, z + h), (x - 2.6, z + h * 0.62)], subdiv=4)
        parts = [shapes.shaped_panel(arm, y, 2.2, rim_seg=3)]
        for k in (0.52, 0.74, 0.94):
            hv, hf = mesh.revolve_closed(
                [(-1.6, 0.62), (1.6, 0.62), (1.6, 1.05), (-1.6, 1.05)], 12)
            parts.append(([(pz + x, px + y, py + z + h * k)
                           for (px, py, pz) in hv], hf))
        parts.append(shapes.rounded_box(x, y, z + sgn_h * 0.8, 11.0, 7.0,
                                        1.6, 1.4, seg=4))
        horns.append(mesh.join(*parts))
        clevises.append(shapes.clevis(
            (x, y, z + h), (0.0, 0.0, sgn_h), 1.9))

    f = (FL["span_in"] + FL["span_out"]) / 2
    for sgn in (-1.0, 1.0):
        y = sgn * W["semi_span"] * f
        chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
        # under the wing: a horn and rod on the upper surface sit in the
        # working flow and cost drag, which is the opposite of the brief
        horn(x_le + chord * (1 - FL["chord_frac"]) + 6.0, y,
             common.surface_z(W, f, 1 - FL["chord_frac"], upper=False),
             -spec.WING_DETAIL["horn_h"])

    for sgn in (-1.0, 1.0):
        horn(H["x_root_le"] + H["root_chord"] * 0.30, sgn * 18.0,
             H["z_root"] + 2.0)
    horn(V["x_root_le"] + V["root_chord"] * 0.78, 4.0, 44.0)
    tags = ["fl", "fr", "sl", "sr", "rud"]
    out2 = {}
    for i, m in enumerate(horns):
        out2[f"horn_{tags[i]}"] = m
    for i, m in enumerate(clevises):
        out2[f"clevis_{tags[i]}"] = m
    return out2


def _probes():
    """Pitot boom on the nose, plus UHF and GPS antennas."""
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

    # A 440 mm model carries a small blade aerial, not a shark fin -- sized
    # from the spine it sits on rather than picked by eye.
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
    # There used to be a GPS patch here as well, and a GPS module in
    # systems.py -- two GPS aerials on a 440 mm aeroplane with one receiver.
    # `gps_puck` is the one that survived; this is the UHF telemetry blade.
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


def _fences():
    """Wing fences and counter-rotating vortex generators.

    On a delta at high angle of attack the outer wing sheds first; the
    generators re-energise that boundary layer so the flaperon keeps working.
    Both are sized off the local chord, so they stay in proportion out to the
    tip instead of turning into scenery.
    """
    D = spec.WING_DETAIL
    out = {}
    fences = []
    for sgn in (-1.0, 1.0):
        for fr in D["fence_stations"]:
            y = sgn * W["semi_span"] * fr
            chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], fr)
            h = chord * D["fence_h"]
            z0 = common.surface_z(W, fr, 0.30)
            # A fence is cut to a profile: tall at its leading edge where the
            # spanwise flow it is stopping is strongest, faired away aft so it
            # does not stand in the flaperon's flow. A constant-height slab
            # does the second half of that job badly.
            c = chord * D["fence_chord"]
            x0 = x_le + chord * 0.30 - c / 2
            prof = shapes.panel_outline(
                [(x0, z0), (x0 + c, z0), (x0 + c, z0 + h * 0.30),
                 (x0 + c * 0.62, z0 + h * 0.86), (x0 + c * 0.24, z0 + h),
                 (x0 - c * 0.04, z0 + h * 0.52)], subdiv=8)
            fences.append(shapes.shaped_panel(
                prof, y, D["fence_t"], rim_seg=6,
                bow=lambda fx, fz, sgn=sgn: sgn * 2.4 * fx * fx))
    half = len(fences) // 2
    for i, m in enumerate(fences):
        out[f"wing_fence_{'lr'[i // half]}{i % half + 1}"] = m

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
            vgs.append(([(x_le + chord * D["vg_x"] + px * ca - py * sa,
                          y + px * sa + py * ca, z0 + h / 2 + pz)
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
    """A door per leg, hinged open."""
    doors = []
    w, hh, zc, _ = fus.station_at(G["nose_x"])
    nv, nf = shapes.rounded_box(G["nose_x"], 14.0, zc - hh + 8.0, 54.0, 3.0, 30.0)
    doors.append((nv, nf))
    for sgn in (-1.0, 1.0):
        w, hh, zc, _ = fus.station_at(G["main_x"])
        mv, mf = shapes.rounded_box(G["main_x"], sgn * (G["main_y"] - 14.0),
                          zc - hh * 0.55 - 12.0, 62.0, 3.0, 34.0)
        doors.append((mv, mf))
    return {"gear_doors": mesh.join(*doors)}


def _wheel_hubs():
    """Spoked hubs, so the wheels are not featureless discs."""
    G = spec.GEAR
    parts = []
    _, hh, zc, _ = fus.station_at(G["nose_x"])
    z_ax = (zc - hh + 1.0) - G["nose_leg"] * math.cos(math.radians(-6.0))
    x_ax = G["nose_x"] + G["nose_leg"] * math.sin(math.radians(-6.0))
    parts.append(_hub(x_ax, 0.0, z_ax, G["nose_wheel_r"] * 0.55,
                      G["nose_wheel_w"] * 0.42, 5))
    _, hh, zc, _ = fus.station_at(G["main_x"])
    z_top = zc - hh * 0.55
    z_ax = z_top - G["main_leg"] * math.cos(math.radians(4.0))
    x_ax = G["main_x"] + G["main_leg"] * math.sin(math.radians(4.0))
    for sgn in (-1.0, 1.0):
        parts.append(_hub(x_ax, sgn * G["main_y"], z_ax,
                          G["main_wheel_r"] * 0.55,
                          G["main_wheel_w"] * 0.42, 5))
    tags = ["n", "ml", "mr"]
    return {f"wheel_hub_{tags[i]}": m for i, m in enumerate(parts)}


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
    """Hatches and bay covers, each lying down on the skin it covers."""
    out = {}
    for (name, x0, x1, a0, a1, h) in spec.SKIN_DETAIL["panels"]:
        out[f"panel_{name}"] = fus.surface_patch(x0, x1, a0, a1, h)
    return out


def _exhaust_petals():
    """The tailpipe shroud: the fairing between the fuselage skin and the
    engine's own nozzle, split into petals like the real thing.

    The engine already carries convergent and divergent flaps out to its exit
    plane, so this is a shroud around them, not a second nozzle.
    """
    T = spec.TAILPIPE
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
                loop.append((x, r_o * math.cos(a), r_o * math.sin(a)))
            for j in range(n_a - 1, -1, -1):
                g = j / (n_a - 1)
                a = a0 + (a1 - a0) * g
                loop.append((x, r_i * math.cos(a), r_i * math.sin(a)))
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


def _dischargers():
    """Static wicks on the trailing edges."""
    parts = []
    for sgn in (-1.0, 1.0):
        for fr in (0.58, 0.80, 0.95):
            y = sgn * W["semi_span"] * fr
            chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], fr)
            # a wick is a base, a stub, and a bundle of filaments -- the
            # filaments are the part that actually bleeds the charge
            p0 = (x_le + chord * 0.74, y, W["z_root"])
            p1 = (x_le + chord * 0.74 + 8.0, y + sgn * 2.0,
                  W["z_root"] + 1.6)
            d = tuple(p1[k] - p0[k] for k in range(3))
            bv, bf = mesh.revolve_closed(
                [(0.0, 0.0), (2.2, 0.0), (2.2, 0.9), (1.4, 1.5),
                 (0.0, 1.5)], 14)
            parts.append((shapes.orient(bv, p0, d), bf))
            parts.append(mesh.pipe([p0, p1], [0.72, 0.34], 12, subdiv=2))
            for k in range(5):
                a = 2 * math.pi * k / 5
                tip2 = (p1[0] + d[0] * 0.34 + math.cos(a) * 0.9,
                        p1[1] + d[1] * 0.34 + math.sin(a) * 0.9,
                        p1[2] + d[2] * 0.34)
                parts.append(mesh.pipe([p1, tip2], 0.16, 6))
    return {"static_dischargers": mesh.join(*parts)}


def _cockpit():
    """Instrument coaming and a seat pan, visible through the canopy."""
    out = {}
    C = spec.CANOPY
    out["instrument_panel"] = shapes.rounded_box(C["x_front"] + 26.0, 0.0,
                                       C["z_base"] + 2.0, 12.0, 26.0,
                                       C["height"] * 0.60)
    out["seat_pan"] = shapes.rounded_box(C["x_front"] + 62.0, 0.0, C["z_base"] - 4.0,
                               34.0, 24.0, 6.0)
    # the canopy glass tops out at z_base + height; a taller seat goes
    # straight through it
    out["seat_back"] = shapes.rounded_box(C["x_front"] + 80.0, 0.0,
                                C["z_base"] + C["height"] * 0.30,
                                6.0, 24.0, C["height"] * 0.70)
    return out


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

    # Intake lip: a sharp, slightly drooped lip pays at speed and a rounded
    # one pays at low speed. This one is closer to sharp, because the brief
    # was maximum speed.
    lip = []
    for k in range(18):
        a = 2 * math.pi * k / 18
        cy = math.cos(a) * I["lip_width"] / 2
        cz = math.sin(a) * I["lip_height"] / 2 + I["z_lip"]
        lip.append(mesh.pipe([(I["x_lip"] - 3.0, cy * 0.97, cz * 0.97),
                              (I["x_lip"] + 5.0, cy, cz)], 1.5, 6))
    out["intake_lip_ring"] = mesh.join(*lip)

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


def _linkages():
    """Every control surface needs a pushrod from its horn to its servo, and
    a snake or a bellcrank where the run changes direction. Modelling the
    horns and not the rods is modelling half a control system.
    """
    out = {}
    W, FL, V, H = spec.WING, spec.FLAPERON, spec.VTAIL, spec.HTAIL
    rods = []
    # A rod runs from the servo's output arm to the surface's horn. It used to
    # run from the horn to a point in mid-air near the servo bay, while a
    # second complete set of rods in internals.py ran from the servos to the
    # hinge lines. Both existed; neither joined a servo to a horn.
    servo = {h[0]: h for h in spec.HARDWARE}

    def arm(tag, dz=0.0):
        s_ = servo[tag]
        return (s_[1] + s_[4] * 0.52, s_[2], s_[3] + s_[6] * 0.46 + dz)

    f = (FL["span_in"] + FL["span_out"]) / 2
    for sgn, tag in ((-1.0, "servo_ail_l"), (1.0, "servo_ail_r")):
        y = sgn * W["semi_span"] * f
        chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
        horn = (x_le + chord * (1 - FL["chord_frac"]) + 6.0, y,
                common.surface_z(W, f, 1 - FL["chord_frac"], upper=False)
                - spec.WING_DETAIL["horn_h"])
        rods.append(_rod(arm(tag), horn))
    rods.append(_rod(arm("servo_rudder"),
                     (V["x_root_le"] + V["root_chord"] * 0.78, 4.0, 44.0)))
    for sgn in (-1.0, 1.0):
        rods.append(_rod(arm("servo_stab"),
                         (H["x_root_le"] + H["root_chord"] * 0.30,
                          sgn * 18.0, H["z_root"] + 9.0)))
    out["pushrod_linkages"] = mesh.join(*rods)

    horns = []
    for sgn in (-1.0, 1.0):
        horns.append(shapes.rounded_box(298.0, sgn * 22.0, 8.0, 22.0, 4.0, 16.0, 1.5))
    out["bellcranks"] = mesh.join(*horns)
    return out


def _rod(p0, p1, r=1.3):
    """A pushrod: a threaded rod with a ball link swaged on each end.

    The links are the whole linkage -- they take out the misalignment between
    a surface swinging on its hinge line and a servo arm swinging on a
    different one. A bare cylinder between two points cannot articulate.
    """
    d = tuple(p1[k] - p0[k] for k in range(3))
    parts = [mesh.pipe([p0, p1], [r * 0.78, r * 0.78], 14, subdiv=3)]
    for (p, dirn) in ((p0, tuple(-c for c in d)), (p1, d)):
        bv, bf = mesh.revolve_closed(
            [(0.0, 0.0), (r * 1.1, 0.0), (r * 1.6, r * 1.1),
             (r * 1.1, r * 1.8), (0.0, r * 1.8), (-r * 0.9, r * 1.5),
             (-r * 1.4, r * 0.9), (-r * 2.8, r * 0.62),
             (-r * 4.0, r * 0.62)], 20)
        parts.append((shapes.orient([(-px, py, pz) for (px, py, pz) in bv],
                                    p, tuple(-c for c in dirn)), bf))
    return mesh.join(*parts)
