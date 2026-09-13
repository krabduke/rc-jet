"""Airframe detail: the parts that make a model look like an aircraft.

Control horns and linkages, pitot and antennas, wing fences and vortex
generators, navigation lights, gear doors, wheel hubs, access panels, exhaust
petals, stores pylons and static dischargers.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
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
    out.update(_pylons())
    out.update(_dischargers())
    out.update(_cockpit())
    return out


def _control_horns():
    """A horn and clevis on every moving surface, where the pushrod meets it."""
    horns, clevises = [], []

    def horn(x, y, z, h=spec.WING_DETAIL["horn_h"]):
        hv, hf = mesh.box(x, y, z + h / 2, 5.0, 2.4, h)
        horns.append((hv, hf))
        cv, cf = mesh.cylinder(-3.2, 3.2, 2.2, 8)
        cv = mesh.rot_z(cv, math.pi / 2)
        cv = [(px + x, py + y, pz + z + h) for (px, py, pz) in cv]
        clevises.append((cv, cf))

    f = (FL["span_in"] + FL["span_out"]) / 2
    for sgn in (-1.0, 1.0):
        y = sgn * W["semi_span"] * f
        chord = common.local_chord(W["root_chord"], W["tip_chord"], f)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], f)
        horn(x_le + chord * (1 - FL["chord_frac"]) + 6.0, y,
             common.surface_z(W, f, 1 - FL["chord_frac"]))

    for sgn in (-1.0, 1.0):
        horn(H["x_root_le"] + H["root_chord"] * 0.30, sgn * 18.0,
             H["z_root"] + 2.0)
    horn(V["x_root_le"] + V["root_chord"] * 0.78, 4.0, 44.0)
    return {"control_horns": mesh.join(*horns),
            "clevises": mesh.join(*clevises)}


def _probes():
    """Pitot boom on the nose, plus UHF and GPS antennas."""
    out = {}
    P = spec.PROBE
    tip, z = P["pitot_tip"], P["pitot_z"]
    boom = mesh.pipe([(tip + 6.0, 0.0, z), (tip + 34.0, 0.0, z)],
                     P["pitot_r"], 8)
    cone = mesh.revolve_open([(tip, 0.001), (tip + 6.0, P["fairing_r"]),
                              (tip + 10.0, P["fairing_r"])],
                             12, cap_start=True, cap_end=True)
    cone = ([(x, y, z + zz) for (x, y, zz) in cone[0]], cone[1])
    out["pitot"] = mesh.join(boom, cone)

    # A 440 mm model carries a small blade aerial, not a shark fin -- sized
    # from the spine it sits on rather than picked by eye.
    ants = []
    x_a = 262.0
    base = fus.surface_point(x_a, 90.0)
    blade = [(base[0] - 11.0, base[1], base[2]),
             (base[0] - 6.0, base[1], base[2] + 7.5),
             (base[0] + 9.0, base[1], base[2] + 7.5),
             (base[0] + 11.0, base[1], base[2])]
    ants.append(_plate(blade, 1.4))
    ants.append(fus.surface_patch(190.0, 212.0, 74.0, 106.0, 1.6))  # GPS patch
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
            fences.append(mesh.box(x_le + chord * 0.30, y, z0 + h / 2,
                                   chord * D["fence_chord"], D["fence_t"], h))
    out["wing_fences"] = mesh.join(*fences)

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
            v, f = mesh.box(0.0, 0.0, 0.0, chord * D["vg_chord"],
                            D["vg_t"], h)
            # alternate the yaw so the pair sheds counter-rotating vortices
            a = math.radians(D["vg_yaw"] * (1 if k % 2 else -1))
            ca, sa = math.cos(a), math.sin(a)
            vgs.append(([(x_le + chord * D["vg_x"] + px * ca - py * sa,
                          y + px * sa + py * ca, z0 + h / 2 + pz)
                         for (px, py, pz) in v], f))
    out["vortex_generators"] = mesh.join(*vgs)
    return out


def _lights():
    """Navigation lights: red to port, green to starboard, white at the tail."""
    out = {}
    for tag, sgn in (("port", -1.0), ("stbd", 1.0)):
        y = sgn * (W["semi_span"] - 2.0)
        x_le = common.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], 1.0)
        chord = W["tip_chord"]
        # on the tip face itself, where a wingtip light actually goes
        v, f = mesh.box(x_le + chord * 0.30, y, W["z_root"], 9.0, 4.0, 3.0)
        out[f"navlight_{tag}"] = (v, f)
    v, f = mesh.box(V["x_root_le"] + V["root_chord"] - 8.0, 0.0, 96.0,
                    10.0, 6.0, 8.0)
    out["navlight_tail"] = (v, f)
    return out


def _gear_doors():
    """A door per leg, hinged open."""
    doors = []
    w, hh, zc, _ = fus.station_at(G["nose_x"])
    nv, nf = mesh.box(G["nose_x"], 14.0, zc - hh + 8.0, 54.0, 3.0, 30.0)
    doors.append((nv, nf))
    for sgn in (-1.0, 1.0):
        w, hh, zc, _ = fus.station_at(G["main_x"])
        mv, mf = mesh.box(G["main_x"], sgn * (G["main_y"] - 14.0),
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
    return {"wheel_hubs": mesh.join(*parts)}


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
        sv, sf = mesh.box(r * 0.62, 0.0, 0.0, r * 0.80, half_w * 1.1, 2.2)
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
        verts, faces = [], []
        for (x, rr, aa0, aa1) in seg:
            for a in (aa0, aa1):
                verts.append((x, rr * math.cos(a), rr * math.sin(a)))
        # 8 corners: front-inner, front-outer, rear-inner, rear-outer
        fi, fo, ri, ro = 0, 2, 4, 6
        faces = [(fi, fi + 1, fo + 1, fo), (ro, ro + 1, ri + 1, ri),
                 (fi, fo, ro, ri), (fi + 1, ri + 1, ro + 1, fo + 1),
                 (fi, ri, ri + 1, fi + 1), (fo, fo + 1, ro + 1, ro)]
        parts.append((verts, faces))
    return {"tailpipe_shroud": mesh.join(*parts)}


def _pylons():
    """A pylon per station carrying a short-range missile.

    A bare cylinder under a wing reads as a dropped pipe; the ogive nose, tail
    fins and canards are what make it read as a weapon.
    """
    out = {}
    S = spec.STORES
    for sgn, side in ((-1.0, "l"), (1.0, "r")):
        for n, fr in enumerate(S["stations"], start=1):
            y = sgn * W["semi_span"] * fr
            chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], fr)
            # the nose leads the wing, as a rail-launched missile does
            x_mid = x_le - S["nose_lead"] + S["body_len"] / 2
            z_wing = W["z_root"] - W["thickness"] * chord * 0.5
            z_body = z_wing - S["pylon_h"] - S["body_r"]
            out[f"pylon_{side}{n}"] = mesh.box(
                x_mid, y, (z_wing + z_body) / 2, chord * 0.38, 5.0,
                z_wing - z_body)
            out[f"missile_{side}{n}"] = _missile(x_mid, y, z_body, S)
    return out


def _missile(x_mid, y, z, S):
    """Ogive nose, parallel body, boat-tail, four tail fins and four canards."""
    r = S["body_r"]
    L = S["body_len"]
    x0 = x_mid - L / 2
    nose, tail = S["nose_len"], S["tail_len"]
    prof = []
    for i in range(9):                      # tangent ogive
        f = i / 8
        prof.append((x0 + nose * f, r * math.sin(math.pi / 2 * f) ** 0.7))
    prof.append((x0 + L - tail, r))
    prof.append((x0 + L, r * 0.72))
    body = mesh.revolve_open(prof, 16, cap_start=True, cap_end=True)
    body = ([(px, py + y, pz + z) for (px, py, pz) in body[0]], body[1])
    parts = [body]

    def fin_set(x_c, span, chord, sweep):
        for k in range(S["n_fins"]):
            a = 2 * math.pi * k / S["n_fins"] + math.pi / 4
            loop = [(x_c - chord / 2, r * 0.9),
                    (x_c + chord / 2, r * 0.9),
                    (x_c + chord / 2 - sweep * 0.3, r + span),
                    (x_c - chord / 2 + sweep, r + span)]
            v, f = _plate([(px, 0.0, pr) for (px, pr) in loop], 1.1)
            ca, sa = math.cos(a), math.sin(a)
            parts.append(([(px, y + py * ca - pz * sa,
                            z + py * sa + pz * ca) for (px, py, pz) in v], f))

    fin_set(x0 + L - tail - S["fin_chord"] * 0.5, S["fin_span"],
            S["fin_chord"], 6.0)
    fin_set(x0 + nose + S["canard_chord"] * 0.6, S["canard_span"],
            S["canard_chord"], 4.0)
    return mesh.join(*parts)


def _dischargers():
    """Static wicks on the trailing edges."""
    parts = []
    for sgn in (-1.0, 1.0):
        for fr in (0.58, 0.80, 0.95):
            y = sgn * W["semi_span"] * fr
            chord = common.local_chord(W["root_chord"], W["tip_chord"], fr)
            x_le = common.le_x_at(W["x_root_le"], W["semi_span"],
                                  W["sweep_le"], fr)
            parts.append(mesh.pipe(
                [(x_le + chord * 0.74, y, W["z_root"]),
                 (x_le + chord * 0.74 + 8.0, y + sgn * 2.0,
                  W["z_root"] + 1.6)],
                0.6, 6))
    return {"static_dischargers": mesh.join(*parts)}


def _cockpit():
    """Instrument coaming and a seat pan, visible through the canopy."""
    out = {}
    C = spec.CANOPY
    out["instrument_panel"] = mesh.box(C["x_front"] + 26.0, 0.0,
                                       C["z_base"] + 2.0, 12.0, 26.0,
                                       C["height"] * 0.60)
    out["seat_pan"] = mesh.box(C["x_front"] + 62.0, 0.0, C["z_base"] - 4.0,
                               34.0, 24.0, 6.0)
    # the canopy glass tops out at z_base + height; a taller seat goes
    # straight through it
    out["seat_back"] = mesh.box(C["x_front"] + 80.0, 0.0,
                                C["z_base"] + C["height"] * 0.30,
                                6.0, 24.0, C["height"] * 0.70)
    return out
