"""The pressure tub extends below the sill to leave room for a seated pilot.

The former battery shelf forced a half figure and a shallow seat. The
full-size demonstrator instead needs a footwell above the inlet, while its
headbox and optics must still clear the unchanged canopy aperture.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes

# these full-size cockpit dimensions belong in spec.py.
K = dict(spec.COCKPIT, z_floor=6.0, depth=17.8, pedal_half_spacing=3.6)
C = spec.CANOPY
HUD = dict(half_width=4.6, half_height=2.3, thickness=0.35,
           spacing=1.8, cant=-26.0, frame_radius=0.32,
           projector_length=6.0, projector_depth=3.4)
SEAT = dict(restraint_radius=0.22, restraint_half_width=2.4,
            leg_radius=1.75, boot_length=4.8)


def build():
    out = {}
    out.update(_tub())
    out.update(_consoles())
    out.update(_panel())
    out.update(_hud())
    out.update(_seat())
    out.update(_controls())
    out.update(_pilot())
    return out


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _loft(rings, cap_first=True, cap_last=True):
    """Close a stack of equal-length rings into a solid."""
    n = len(rings[0])
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for s in range(n):
            s2 = (s + 1) % n
            faces.append((a + s, a + s2, b + s2, b + s))
    if cap_first:
        faces.append(tuple(range(n - 1, -1, -1)))
    if cap_last:
        base = (len(rings) - 1) * n
        faces.append(tuple(range(base, base + n)))
    return verts, faces


def _dome(rings, bottom, top):
    """Close a stack of rings onto a point at each end.

    A ring of eighteen coincident vertices is not a cap: every edge in it runs
    from a point to itself, so the mesh is left with a hole the renderer shows
    as the inside of the part. Two real apexes and two triangle fans.
    """
    n = len(rings[0])
    verts = [v for r in rings for v in r]
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for s in range(n):
            s2 = (s + 1) % n
            faces.append((a + s, a + s2, b + s2, b + s))
    lo, hi = len(verts), len(verts) + 1
    verts.extend([bottom, top])
    last = (len(rings) - 1) * n
    for s in range(n):
        s2 = (s + 1) % n
        faces.append((lo, s2, s))
        faces.append((hi, last + s, last + s2))
    return verts, faces


def _cant(verts, angle_deg, x0, z0):
    """Rotate about the +y axis through (x0, z0) -- the pitch a panel or a
    seat back is set at. Positive lays the top backwards."""
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [(x0 + (x - x0) * ca + (z - z0) * sa, y,
             z0 - (x - x0) * sa + (z - z0) * ca) for (x, y, z) in verts]


def _u_half(hw, top, floor, r, seg=5):
    """Down the right wall, round the floor, up the left wall."""
    out = [(hw, top)]
    for i in range(seg + 1):
        a = (math.pi / 2) * i / seg
        out.append((hw - r + r * math.cos(a), floor + r - r * math.sin(a)))
    for i in range(seg + 1):
        a = (math.pi / 2) * i / seg
        out.append((-hw + r - r * math.sin(a), floor + r - r * math.cos(a)))
    out.append((-hw, top))
    return out


def _rounded_rect(cx, cz, hx, hz, r, seg=5):
    """A rounded rectangle as an (x, z) outline for shapes.shaped_panel."""
    r = max(0.15, min(r, hx - 0.05, hz - 0.05))
    out = []
    for (sx, sz) in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        ox, oz = sx * (hx - r), sz * (hz - r)
        a0 = math.atan2(sz, sx) - math.pi / 4
        for i in range(seg + 1):
            a = a0 + (math.pi / 2) * i / seg
            out.append((cx + ox + r * math.cos(a), cz + oz + r * math.sin(a)))
    return out


# --------------------------------------------------------------------------
# the moulded tub
# --------------------------------------------------------------------------

def _tub():
    """The moulded tray the whole cockpit drops into.

    It is a thin-walled U swept down the fuselage: a floor with two side
    walls and nothing on top, because the top is the canopy. Sweeping a real
    section rather than boxing it means the wall has a thickness you can see
    at the coaming cut, which is where a moulding shows what it is.
    """
    n = 16
    path, scale = [], []
    for i in range(n):
        t = i / (n - 1)
        x = K["x_front"] + (K["x_rear"] - K["x_front"]) * t
        hw = K["half_width"] + (K["half_width_aft"] - K["half_width"]) * t
        path.append((x, 0.0, K["z_floor"]))
        scale.append((hw / K["half_width"], 1.0))

    w = K["wall"]
    outer = _u_half(K["half_width"], K["depth"], 0.0, 2.4)
    inner = _u_half(K["half_width"] - w, K["depth"], w, max(2.4 - w, 0.5))
    section = outer + list(reversed(inner))
    tub = shapes.swept_profile(path, section, scale, subdiv=2, caps=True)

    # A swept U is capped on its own cross-section, which leaves the tray open
    # at both ends -- and with the skin cut away above it you look straight
    # through the back of the cockpit into the equipment bay. Close it: a
    # footwell bulkhead at the front and the seat bulkhead at the back, which
    # is what a cockpit has anyway.
    walls = [tub]
    for (x, hw, t) in ((K["x_front"] + w / 2, K["half_width"], w),
                       (K["x_rear"] - w / 2, K["half_width_aft"], w)):
        walls.append(shapes.rounded_box(
            x, 0.0, K["z_floor"] + K["depth"] / 2 + w / 2,
            t, (hw - w) * 2, K["depth"] - w, r=0.3, seg=3))
    return {"cockpit_tub": mesh.join(*walls)}


def _consoles():
    """Side consoles: the shelves either side of the seat that carry the
    switches, and the reason a fighter cockpit is narrow at the hips."""
    out = {}
    x0, x1 = K["x_panel"] + 6.0, K["x_seat"] + 16.0
    z_bot = K["z_floor"] + K["wall"] + 0.1        # on the floor, not in it
    z_top = K["z_floor"] + K["depth"] + 0.7
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        y_in = sgn * K["seat_half_width"]
        y_out = sgn * (K["half_width"] - K["wall"] - 0.3)
        parts = [shapes.rounded_box(
            (x0 + x1) / 2, (y_in + y_out) / 2, (z_bot + z_top) / 2,
            x1 - x0, abs(y_out - y_in), z_top - z_bot, r=0.8, seg=4)]
        # switch blocks, canted inboard so the pilot can reach them
        for i in range(5):
            f = (i + 0.5) / 5
            parts.append(shapes.rounded_box(
                x0 + (x1 - x0) * f, (y_in + y_out) / 2 + sgn * 0.4,
                z_top + 0.5, (x1 - x0) / 5 * 0.56,
                abs(y_out - y_in) * 0.52, 1.0, r=0.3, seg=3))
        out[f"console_{side}"] = mesh.join(*parts)
    return out


# --------------------------------------------------------------------------
# instrument panel, coaming and head-up display
# --------------------------------------------------------------------------

def _panel():
    """A canted face with two display bezels and three round instruments,
    under a glareshield hood.

    The cant is what makes it read as a panel rather than a bulkhead: an
    instrument face is square to the pilot's eye line, not to the datum.
    """
    out = {}
    xp, hw = K["x_panel"], K["half_width"] - 1.6
    z0 = K["z_floor"] + K["depth"]
    z1 = K["z_panel_top"]
    hz = (z1 - z0) / 2
    zc = (z0 + z1) / 2

    face = shapes.shaped_panel(
        _rounded_rect(0.0, 0.0, hw, hz, 1.6, seg=7), 0.0, 1.5,
        rim_seg=6, axis="z")
    # shaped_panel with axis="z" lays the outline in x-y and the thickness in
    # z; turn it up so the outline is the panel face and the thickness is fore
    # and aft, then cant it back from the pilot.
    v = [(pz, px, py) for (px, py, pz) in face[0]]
    v = _cant(mesh.translate(v, xp, 0.0, zc), K["panel_cant"], xp, zc)
    parts = [(v, face[1])]

    inst = []
    for (dy, dz, w, h) in ((-5.6, 0.6, 4.4, 4.4), (5.6, 0.6, 4.4, 4.4)):
        inst.append(shapes.rounded_box(-0.9, dy, dz, 1.6, w, h, r=0.5, seg=3))
    for (dy, dz, r) in ((-0.0, 2.4, 1.5), (-3.0, -2.6, 1.3), (3.0, -2.6, 1.3)):
        rv, rf = mesh.revolve_closed(
            [(-0.9, 0.0), (0.6, 0.0), (0.6, r), (0.0, r * 1.06),
             (-0.9, r * 0.92)], 18)
        inst.append((mesh.translate(rv, 0.0, dy, dz), rf))
    iv, if_ = mesh.join(*inst)
    iv = _cant(mesh.translate(iv, xp, 0.0, zc), K["panel_cant"], xp, zc)
    out["instrument_panel"] = mesh.join(*parts)
    out["panel_instruments"] = (iv, if_)

    # the coaming: a padded hood over the top of the panel that keeps the sun
    # off the displays and stops the pilot's head finding the panel edge in a
    # hard landing. It is crowned across the span and falls away aft.
    rings = []
    for i in range(9):
        t = i / 8
        x = xp - 1.2 + 7.4 * t
        w = hw + 1.2 - 1.8 * t
        ring = []
        for j in range(13):
            u = j / 12 * 2 - 1                       # -1 .. 1 across the span
            ring.append((x, u * w,
                         z1 + 1.5 - 2.8 * t * t - 0.9 * u * u))
        rings.append(ring)
    out["coaming"] = _shell(rings, 0.9)
    return out


def _shell(rings, t):
    """Give an open lofted surface a thickness, so it has an edge."""
    n = len(rings[0])
    lower = [[(x, y, z - t) for (x, y, z) in r] for r in rings]
    verts = [v for r in rings for v in r] + [v for r in lower for v in r]
    off = len(rings) * n
    faces = []
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n - 1):
            faces.append((a + j, a + j + 1, b + j + 1, b + j))
            faces.append((off + a + j, off + b + j,
                          off + b + j + 1, off + a + j + 1))
    for i in range(len(rings) - 1):
        a, b = i * n, (i + 1) * n
        faces.append((a, b, off + b, off + a))
        faces.append((a + n - 1, off + a + n - 1, off + b + n - 1, b + n - 1))
    last = (len(rings) - 1) * n
    for j in range(n - 1):
        faces.append((j + 1, j, off + j, off + j + 1))
        faces.append((last + j, last + j + 1,
                      off + last + j + 1, off + last + j))
    return verts, faces


def _hud():
    """Combiner glass and its frame, ahead of the pilot's eye line.

    It has to stand clear of the coaming and clear of the canopy, which at
    this station is already coming down: the bubble's ceiling is what limits
    how tall the combiner can be.
    """
    out = {}
    x0, z0 = K["x_hud"], K["z_panel_top"]
    hw, hh = HUD["half_width"], HUD["half_height"]
    fr = HUD["frame_radius"]
    posts = []
    for i in range(2):
        x = x0 + i * HUD["spacing"]
        z = z0 + hh
        outline = _rounded_rect(0.0, 0.0, hw, hh, fr * 2, seg=7)
        gv, gf = shapes.shaped_panel(
            outline, 0.0, HUD["thickness"], rim_seg=6, axis="z")
        gv = [(x + pz, px, z + py) for px, py, pz in gv]
        out[f"hud_glass_combiner_{i + 1}"] = (
            _cant(gv, HUD["cant"], x, z), gf)
        rim = [(x, u, z + v) for u, v in outline]
        for j in range(len(rim)):
            posts.append(mesh.pipe(_cant(
                [rim[j], rim[(j + 1) % len(rim)]], HUD["cant"], x, z), fr, 8))
        for sgn in (-1.0, 1.0):
            bottom = _cant([(x, sgn * hw, z - hh)], HUD["cant"], x, z)[0]
            posts.append(mesh.pipe(
                [(x, sgn * hw, z0 - HUD["projector_depth"] / 2), bottom], fr, 10))
    out["hud_frame_combiner"] = mesh.join(*posts)
    out["hud_frame_projector"] = shapes.rounded_box(
        x0, 0.0, z0 - HUD["projector_depth"] / 2,
        HUD["projector_length"], hw * 2, HUD["projector_depth"], r=fr, seg=4)
    lv, lf = mesh.cylinder(0.0, HUD["thickness"], hw * 0.42, 28)
    out["hud_glass_projector_lens"] = (
        [(x0 + pz, py, z0 + px) for px, py, pz in lv], lf)
    return out


# --------------------------------------------------------------------------
# ejection seat
# --------------------------------------------------------------------------

def _seat():
    """A seat is a bucket on rails, not a cushion on a slab.

    The pan has raised sides the pilot's thighs sit between, the back is a
    reclined shell with its own side beams, and above it is the headbox --
    the parachute and drogue container that gives an ejection seat its
    square-shouldered silhouette. The headbox is the tallest thing in here,
    so the bubble decides how tall it can be, not the seat.
    """
    out = {}
    hw = K["seat_half_width"]
    xs = K["x_seat"]
    z_pan = K["z_floor"] + K["depth"] - 0.4

    # pan: a shallow bucket, wider at the front than under the hips, with a
    # rolled lip the thighs sit inside
    rings = []
    for i in range(13):
        t = i / 12
        x = xs + 16.0 * t
        w = hw * (1.0 - 0.10 * t)
        lip = 1.5 + 0.9 * t
        sec = [(-w, lip), (-w, -0.3), (-w, -0.8), (-w * 0.86, -1.1),
               (-w * 0.50, -1.25), (0.0, -1.3), (w * 0.50, -1.25),
               (w * 0.86, -1.1), (w, -0.8), (w, -0.3), (w, lip),
               (w * 0.94, lip * 0.80), (w * 0.90, lip * 0.62),
               (w * 0.80, 0.5), (w * 0.52, 0.1), (0.0, 0.0),
               (-w * 0.52, 0.1), (-w * 0.80, 0.5),
               (-w * 0.90, lip * 0.62), (-w * 0.94, lip * 0.80)]
        rings.append([(x, u, z_pan + v) for (u, v) in sec])
    out["seat_pan"] = _loft(rings)

    # back: a reclined shell with side beams, built upright then laid back
    x_hinge = xs + 15.0
    rings = []
    for i in range(14):
        t = i / 13
        z = z_pan + 13.2 * t
        w = hw * (1.0 - 0.06 * t)
        d = 2.6 - 0.7 * t
        # a shell: flat-ish cushion face, beams down each side, curved back
        sec = [(-w, d), (-w, 0.8), (-w, -0.4), (-w * 0.62, -0.9),
               (-w * 0.30, -1.08), (0.0, -1.1), (w * 0.30, -1.08),
               (w * 0.62, -0.9), (w, -0.4), (w, 0.8), (w, d),
               (w * 0.88, d - 0.5), (w * 0.72, d - 1.4),
               (w * 0.36, d - 1.8), (0.0, d - 1.9), (-w * 0.36, d - 1.8),
               (-w * 0.72, d - 1.4), (-w * 0.88, d - 0.5)]
        rings.append([(x_hinge + v, u, z) for (u, v) in sec])
    bv, bf = _loft(rings)
    out["seat_back"] = (_cant(bv, K["seat_recline"], x_hinge, z_pan), bf)

    # headbox: the parachute container above the shoulders, with the drogue
    # can stepped in on top of it
    box = shapes.rounded_box(
        0.0, 0.0, 0.0, 7.2, hw * 1.68, 4.4, r=0.9, seg=6)
    drogue = shapes.rounded_box(
        0.6, 0.0, 2.6, 5.0, hw * 1.20, 1.6, r=0.5, seg=5)
    # z_pan + 9.2, not 11.9. The recline takes the box aft as well as up, and
    # the canopy is tapering hard by then: at x 150 its crown is at z 37.65
    # and the headbox's aft-top corner was at 37.9, through the glass.
    hv = _cant(mesh.translate(box[0], x_hinge + 2.0, 0.0, z_pan + 9.2),
               K["seat_recline"], x_hinge, z_pan)
    out["seat_headbox"] = (hv, box[1])

    # rails: the seat runs up these when it fires, so they lean back with it
    rails = []
    for sgn in (-1.0, 1.0):
        rails.append(mesh.pipe(
            [(x_hinge + 3.4, sgn * hw * 0.82, z_pan - 1.0),
             (x_hinge + 4.6, sgn * hw * 0.82, z_pan + 4.0),
             (x_hinge + 5.8, sgn * hw * 0.82, z_pan + 7.0),
             (x_hinge + 6.6, sgn * hw * 0.82, z_pan + 8.8)], 0.7, 12))
    rails.append(mesh.pipe(
        [(x_hinge + 6.4, -hw * 0.82, z_pan + 8.2),
         (x_hinge + 6.4, hw * 0.82, z_pan + 8.2)], 0.5, 10))
    out["seat_rails"] = mesh.join(*rails)

    # harness: two shoulder straps over the back, a lap belt across the pan
    # and the negative-g strap between them, all meeting at one buckle --
    # which is the whole point of a five-point harness and the detail that
    # says the seat is occupied.
    def webbing(width, t=0.20, n=8):
        """A strap section: flat, with the edges rolled rather than cut."""
        sec = []
        for i in range(n):
            a = 2 * math.pi * i / n
            sec.append((width * math.cos(a) * 0.5,
                        t * math.sin(a) + t * 0.6 * math.copysign(
                            1.0, math.sin(a)) * abs(math.cos(a))))
        return sec

    buckle_x, buckle_z = xs + 9.6, z_pan + 2.0
    straps = []
    for sgn in (-1.0, 1.0):
        straps.append(shapes.swept_profile(
            [(x_hinge + 1.4, sgn * hw * 0.52, z_pan + 12.0),
             (x_hinge + 0.2, sgn * hw * 0.50, z_pan + 7.6),
             (xs + 13.0, sgn * hw * 0.44, z_pan + 3.2),
             (buckle_x + 1.4, sgn * 1.2, buckle_z + 0.4)],
            webbing(3.0), subdiv=4))
        straps.append(shapes.swept_profile(
            [(xs + 11.8, sgn * hw * 0.84, z_pan + 0.9),
             (xs + 10.6, sgn * hw * 0.44, z_pan + 1.6),
             (buckle_x + 1.0, sgn * 1.0, buckle_z - 0.2)],
            webbing(2.6), subdiv=4))
    straps.append(shapes.swept_profile(
        [(xs + 4.6, 0.0, z_pan + 0.2), (xs + 7.0, 0.0, z_pan + 0.9),
         (buckle_x, 0.0, buckle_z - 0.9)], webbing(2.2), subdiv=4))
    straps.append(shapes.rounded_box(buckle_x, 0.0, buckle_z,
                                     2.6, 2.4, 1.1, r=0.4, seg=4))
    out["seat_harness"] = mesh.join(*straps)

    # the handle between the knees: the one part of a seat everyone can name.
    # Sweeping a closed path leaves the two ends unjoined, so this is a torus
    # turned into the fore-and-aft plane rather than a pipe chasing its tail.
    tv, tf = mesh.ring_torus(0.0, 1.55, 0.38, 10, 6)
    tv = mesh.translate(mesh.rot_z(tv, math.pi / 2),
                        xs + 1.8, 0.0, z_pan + 2.9)
    out["ejection_handle"] = (tv, tf)
    out["seat_bucket"] = mesh.join(*[
        shapes.rounded_box(xs + hw, sgn * hw, (z_pan + K["z_floor"]) / 2,
                           hw * 2, K["wall"], z_pan - K["z_floor"],
                           r=K["wall"] / 3, seg=3)
        for sgn in (-1.0, 1.0)])
    dv = _cant(mesh.translate(drogue[0], x_hinge + 2.0, 0.0, z_pan + 9.2),
               K["seat_recline"], x_hinge, z_pan)
    out["seat_drogue_container"] = (dv, drogue[1])
    restraints = []
    for sgn in (-1.0, 1.0):
        y = sgn * hw * 0.52
        knee = xs - hw
        z = K["z_floor"] + K["depth"] * 0.58
        restraints.append(mesh.pipe(
            [(xs + hw, sgn * hw, z_pan - K["wall"]),
             (knee, y + sgn * SEAT["restraint_half_width"], z),
             (knee - K["wall"], y, z - SEAT["leg_radius"]),
             (knee, y - sgn * SEAT["restraint_half_width"], z),
             (xs, sgn * hw * 0.3, z_pan - K["wall"])],
            SEAT["restraint_radius"], 10))
    out["seat_harness_leg_restraints"] = mesh.join(*restraints)
    return out


# --------------------------------------------------------------------------
# what the pilot actually holds
# --------------------------------------------------------------------------

def _controls():
    """Side stick, throttle and rudder pedals.

    The stick is on the right console, not between the knees: that is the one
    thing everybody knows about this cockpit, and putting it on the
    centreline would make the whole tub read as the wrong aeroplane.
    """
    out = {}
    z_top = K["z_floor"] + K["depth"] + 1.2
    y_c = (K["seat_half_width"] + K["half_width"] - 1.2) / 2

    grip = [(0.0, 0.0), (0.0, 1.05), (1.2, 1.25), (3.6, 1.15),
            (5.0, 0.95), (5.6, 0.55), (5.8, 0.0)]
    gv, gf = mesh.revolve_closed(grip, 16)
    gv = [(pz + K["x_seat"] - 3.0, py + y_c, px + z_top)
          for (px, py, pz) in gv]
    out["control_stick"] = (_cant(gv, -8.0, K["x_seat"] - 3.0, z_top), gf)

    lever = shapes.rounded_box(K["x_seat"] - 8.0, -y_c, z_top + 1.3,
                               11.0, 2.2, 2.0, r=0.6, seg=3)
    knob = mesh.revolve_closed(
        [(0.0, 0.0), (0.0, 1.3), (2.4, 1.5), (3.4, 1.0), (3.6, 0.0)], 14)
    kv = [(pz + K["x_seat"] - 13.6, py - y_c, px + z_top + 1.3)
          for (px, py, pz) in knob[0]]
    out["throttle_lever"] = mesh.join(lever, (kv, knob[1]))

    pedals = []
    for sgn in (-1.0, 1.0):
        y = sgn * K["pedal_half_spacing"]
        pv, pf = shapes.rounded_box(0.0, 0.0, 0.0, 1.0, 3.2, 4.6, r=0.4, seg=3)
        pedals.append((_cant(mesh.translate(pv, K["x_pedals"], y,
                                            K["z_floor"] + 3.2), 28.0,
                             K["x_pedals"], K["z_floor"] + 3.2), pf))
        pedals.append(mesh.pipe(
            [(K["x_pedals"] + 1.0, y, K["z_floor"] + 1.0),
             (K["x_pedals"] + 9.0, y, K["z_floor"] + 0.8)], 0.5, 10))
    out["rudder_pedals"] = mesh.join(*pedals)
    out["rudder_pedals_rail"] = mesh.join(*[
        shapes.rounded_box(
            (K["x_pedals"] + K["x_seat"]) / 2, sgn * K["pedal_half_spacing"],
            K["z_floor"] + K["wall"], K["x_seat"] - K["x_pedals"],
            K["wall"], K["wall"], r=K["wall"] / 4, seg=3)
        for sgn in (-1.0, 1.0)])
    out["control_stick_hotas"] = mesh.join(*[
        shapes.rounded_box(K["x_seat"] - 3.0 + dx, y_c + dy, z_top + dz,
                           K["wall"], K["wall"], K["wall"],
                           r=K["wall"] / 4, seg=3)
        for dx, dy, dz in ((0.0, 0.0, 5.6), (-1.1, 0.0, 4.5), (0.0, -0.9, 4.9))])
    out["throttle_lever_hotas"] = shapes.rounded_box(
        K["x_seat"] - 13.6, -y_c + K["wall"], z_top + 3.8,
        K["wall"], K["wall"], K["wall"], r=K["wall"] / 4, seg=3)
    return out


# --------------------------------------------------------------------------
# the pilot
# --------------------------------------------------------------------------

def _pilot():
    """The seated pilot reaches the pedals rather than ending at the harness.

    The pressure tub now has a footwell instead of a battery shelf; bent
    knees and boots connect the existing upper body to the rudder controls.
    """
    out = {}
    hw = K["seat_half_width"]
    x_h = K["x_seat"] + 9.0          # head station
    z_pan = K["z_floor"] + K["depth"] - 0.4
    z_sh = z_pan + 5.2               # shoulder line
    z_head = z_pan + 10.4            # helmet centre, set by the bubble

    # torso: a chest 0.30 m deep and shoulders 0.55 m across at 1:34, pushed
    # back against the seat and cut off where the harness crosses it. The
    # section is not a circle -- a chest is flat at the back and rounded at
    # the front, which is what makes a figure read as a person from above.
    rings = []
    for (dz, w, d, lean) in ((0.0, 3.0, 2.2, 0.0), (1.2, 4.0, 2.8, 0.2),
                             (2.4, 4.9, 3.3, 0.4), (3.6, 5.6, 3.6, 0.6),
                             (4.7, 6.2, 3.7, 0.8), (5.6, 6.4, 3.5, 1.0),
                             (6.4, 5.2, 3.0, 1.1), (7.0, 3.2, 2.2, 1.2),
                             # and a neck, so the helmet sits on something
                             (7.6, 2.0, 1.7, 1.2), (8.6, 1.9, 1.6, 1.2)):
        z = z_pan + dz
        xc = x_h + 2.2 + lean
        ring = []
        for i in range(20):
            a = 2 * math.pi * i / 20
            ca = math.cos(a)
            # flat against the seat back, rounded across the front
            depth = d * (0.62 if ca > 0 else 1.0)
            ring.append((xc + depth * ca, w * math.sin(a), z))
        rings.append(ring)
    out["pilot_torso"] = _loft(rings)
    legs, boots = [], []
    for sgn in (-1.0, 1.0):
        y = sgn * hw * 0.52
        legs.append(mesh.pipe(
            [(x_h, y, z_pan + SEAT["leg_radius"]),
             (K["x_seat"] - hw, y, z_pan + SEAT["leg_radius"]),
             (K["x_pedals"] + SEAT["boot_length"], y,
              K["z_floor"] + SEAT["leg_radius"] * 2)], SEAT["leg_radius"], 16))
        boots.append(shapes.rounded_box(
            K["x_pedals"] + SEAT["boot_length"] / 2, y,
            K["z_floor"] + SEAT["leg_radius"], SEAT["boot_length"],
            SEAT["leg_radius"] * 1.7, SEAT["leg_radius"] * 1.5,
            r=SEAT["leg_radius"] / 3, seg=4))
    out["pilot_torso_legs"] = mesh.join(*legs)
    out["control_stick_boots"] = mesh.join(*boots)

    # arms: forward and inboard to the stick and the throttle. The pose is
    # what makes a figure look like a pilot rather than a passenger, and it
    # has to stay inside the tub -- an arm resting on the fuselage deck reads
    # as a crash, not a cockpit.
    arms = []
    y_c = K["seat_half_width"] + 1.6
    z_rest = K["z_floor"] + K["depth"]
    for (sgn, x_hand, z_hand) in ((1.0, K["x_seat"] - 2.4, z_rest + 4.6),
                                  (-1.0, K["x_seat"] - 9.0, z_rest + 3.4)):
        shoulder = (x_h + 2.4, sgn * 5.6, z_sh + 0.4)
        elbow = (K["x_seat"] + 5.0, sgn * y_c, z_sh - 2.6)
        wrist = (x_hand + 2.0, sgn * y_c * 0.88, z_hand + 0.6)
        for (p0, p1, r0, r1) in ((shoulder, elbow, 1.70, 1.35),
                                 (elbow, wrist, 1.35, 1.05)):
            mid = tuple((p0[k] + p1[k]) / 2 for k in range(3))
            arms.append(shapes.swept_profile(
                [p0, mid, p1],
                [(math.cos(2 * math.pi * i / 12),
                  math.sin(2 * math.pi * i / 12)) for i in range(12)],
                scale=[(r0, r0), ((r0 + r1) / 2, (r0 + r1) / 2), (r1, r1)],
                subdiv=3))
        # the glove closed round the grip, which is the whole point of the pose
        arms.append(shapes.rounded_box(x_hand, sgn * y_c * 0.88, z_hand,
                                       2.8, 2.0, 3.0, r=0.7, seg=4))
    out["pilot_arms"] = mesh.join(*arms)

    # helmet: a shell with a brow over the face, an ear cup at the side and a
    # drawn-out occiput -- not a ball. The brow is what makes the visor read
    # as fitted into an opening rather than stuck on the outside.
    r = K["helmet_r"]
    rings = []
    for i in range(12, 0, -1):          # bottom to top: a loft that is not
        p = math.pi * i / 13            # monotonic in z closes on itself
        rz, rr = math.cos(p), math.sin(p)
        ring = []
        for j in range(22):
            a = 2 * math.pi * j / 22
            ca, sa = math.cos(a), math.sin(a)
            fwd = max(0.0, -ca)                 # 1 at the face, 0 at the back
            aft = max(0.0, ca)
            # occiput drawn back, face cut in under the brow, ears bulged
            k = (1.0 + 0.17 * aft - 0.10 * fwd * max(0.0, 0.5 - rz)
                 + 0.06 * abs(sa) * (1.0 - abs(rz)))
            ring.append((x_h + r * rr * ca * k,
                         r * rr * sa * 0.94,
                         z_head + r * rz * 1.06))
        rings.append(ring)
    out["pilot_helmet"] = _dome(
        rings, (x_h + r * 0.28, 0.0, z_head - r * 1.12),
        (x_h, 0.0, z_head + r * 1.12))

    # visor: a curved band set into the face opening, standing just proud of
    # the shell the way a lowered visor does
    rings = []
    for i in range(7):
        p = math.pi * (0.40 + 0.24 * i / 6)
        rz, rr = math.cos(p), math.sin(p)
        for k in (0.99, 1.05):
            ring = []
            for j in range(17):
                a = math.pi + (-1.05 + 2.10 * j / 16)
                ca, sa = math.cos(a), math.sin(a)
                ring.append((x_h + r * rr * ca * 0.96 * k,
                             r * rr * sa * 0.94 * k,
                             z_head + r * rz * 1.06 * k))
            rings.append(ring)
    out["pilot_visor"] = _visor_shell(rings)

    # oxygen mask: a cup over the nose and mouth, narrowing to the chin, with
    # the bayonet fittings that clip it to the helmet and a corrugated hose
    # down to the regulator on the seat
    x_m, z_m = x_h - r * 0.74, z_head - r * 0.80
    rings = []
    for (dz, w, d) in ((2.0, 1.5, 0.7), (1.0, 2.1, 1.2), (0.0, 2.2, 1.5),
                       (-1.2, 1.9, 1.5), (-2.2, 1.3, 1.2), (-2.9, 0.7, 0.8)):
        ring = []
        for j in range(12):
            a = 2 * math.pi * j / 12
            ring.append((x_m - d * max(0.0, math.cos(a)) - d * 0.35,
                         w * math.sin(a), z_m + dz + d * 0.3 * math.cos(a)))
        rings.append(ring)
    parts = [_loft(rings)]
    for sgn in (-1.0, 1.0):
        parts.append(mesh.pipe(
            [(x_m + 0.4, sgn * 2.0, z_m + 0.6),
             (x_m + 1.8, sgn * 3.0, z_m + 0.9)], 0.45, 8))
    hose = [(x_m + 0.2, 1.4, z_m - 2.6)]
    for i in range(1, 8):
        t = i / 7
        hose.append((x_m + 0.6 + 4.2 * t * t, 1.6 + 3.2 * t,
                     z_m - 2.6 - (z_m - 2.6 - (z_pan + 2.4)) * t))
    parts.append(mesh.pipe(hose, 0.55, 10))
    out["pilot_mask"] = mesh.join(*parts)
    return out


def _visor_shell(rings):
    """Pair up the inner and outer bands into a closed curved plate."""
    inner = [rings[i] for i in range(0, len(rings), 2)]
    outer = [rings[i] for i in range(1, len(rings), 2)]
    n = len(inner[0])
    verts = [v for r in inner for v in r] + [v for r in outer for v in r]
    off = len(inner) * n
    faces = []
    for i in range(len(inner) - 1):
        a, b = i * n, (i + 1) * n
        for j in range(n - 1):
            faces.append((a + j, a + j + 1, b + j + 1, b + j))
            faces.append((off + a + j, off + b + j,
                          off + b + j + 1, off + a + j + 1))
        faces.append((a, b, off + b, off + a))
        faces.append((a + n - 1, off + a + n - 1, off + b + n - 1, b + n - 1))
    last = (len(inner) - 1) * n
    for j in range(n - 1):
        faces.append((j + 1, j, off + j, off + j + 1))
        faces.append((last + j, last + j + 1,
                      off + last + j + 1, off + last + j))
    return verts, faces
