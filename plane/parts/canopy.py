"""A fixed windscreen and rear-hinged bubble share the existing sill outline.

The transparency is uninterrupted aft of the arch; its seal and hardware
follow the same profile as the skin opening rather than inventing a wider tub.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import fuselage

C = spec.CANOPY
SEG = 40
# these canopy hardware dimensions belong in spec.py.
H = dict(glass=0.55, seal=0.22, hinge_x=164.0, hinge_span=5.0,
         hinge_radius=0.8, strut_radius=0.5, slider_radius=0.26,
         lock_length=2.0, lock_width=0.8, lock_height=0.7, cord_radius=0.10)


def build():
    out = {}
    out.update(_glass(C["x_front"], C["windscreen_x"], "canopy_windscreen_glass"))
    out.update(_glass(C["windscreen_x"], C["x_rear"], "canopy_glass"))
    out.update(_frame())
    out.update(_hardware())
    return out


def _profile(t):
    """Canopy half-width and height as a fraction of length. Peaks just behind
    the windscreen and fairs into the spine, which is the bubble shape.

    It lives in spec because the fuselage skin has to cut its aperture to the
    same outline, and two copies of a curve is how a canopy ends up landing
    half on skin and half on nothing.
    """
    return spec.canopy_profile(t)


def _glass(x0, x1, name):
    """Separate the fixed windscreen from the rear-hinged transparency.

    Both shells use the shared aperture profile rather than rescaling the
    bubble at the arch, which would open a gap above the cockpit sill.
    """
    n = 26
    rings = []
    for i in range(n):
        t = i / (n - 1)
        x = x0 + (x1 - x0) * t
        t = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
        w, h = _profile(t)
        _, _, zc, _ = fuselage.station_at(x)
        base = C["z_base"] + zc * 0.15
        ring = []
        # half-dome: only the part standing above the fuselage spine
        for s in range(SEG + 1):
            a = math.pi * s / SEG
            ring.append((x, w * math.cos(a), base + h * math.sin(a)))
        rings.append(ring)

    inner = []
    for r in rings:
        x = r[0][0]
        t = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
        w, h = _profile(t)
        base = r[0][2]
        thickness = min(H["glass"], w * 0.45, h * 0.45)
        row = []
        for s_ in range(SEG + 1):
            a = math.pi * s_ / SEG
            row.append((x, (w - thickness) * math.cos(a),
                        base + (h - thickness) * math.sin(a)))
        inner.append(row)

    per = SEG + 1
    verts = [v for r in rings for v in r] + [v for r in inner for v in r]
    off = n * per
    faces = []
    for i in range(n - 1):
        a, b = i * per, (i + 1) * per
        for s_ in range(SEG):
            faces.append((a + s_, a + s_ + 1, b + s_ + 1, b + s_))
            faces.append((off + a + s_, off + b + s_,
                          off + b + s_ + 1, off + a + s_ + 1))
        # the two sills, where the glass meets the fuselage
        for s_ in (0, SEG):
            if s_ == 0:
                faces.append((a, b, off + b, off + a))
            else:
                faces.append((a + s_, off + a + s_, off + b + s_, b + s_))
    # and the front and rear rims
    for i, flip in ((0, False), (n - 1, True)):
        a = i * per
        for s_ in range(SEG):
            q = (a + s_, a + s_ + 1, off + a + s_ + 1, off + a + s_)
            faces.append(q if flip else tuple(reversed(q)))
    return {name: (verts, faces)}


def _frame():
    """Windscreen bow and the canopy sill rails."""
    parts = []
    x = C["windscreen_x"]
    t = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
    w, h = _profile(t)
    _, _, zc, _ = fuselage.station_at(x)
    base = C["z_base"] + zc * 0.15
    bow = []
    for s in range(SEG + 1):
        a = math.pi * s / SEG
        bow.append((x, w * math.cos(a), base + h * math.sin(a)))
    parts.append(mesh.pipe(bow, C["frame"] * 0.7, 8, caps=True))

    for sgn in (-1.0, 1.0):
        rail = []
        for i in range(20):
            tt = i / 19
            xx = C["x_front"] + (C["x_rear"] - C["x_front"]) * tt
            ww, _ = _profile(tt)
            _, _, zz, _ = fuselage.station_at(xx)
            rail.append((xx, sgn * ww, C["z_base"] + zz * 0.15))
        parts.append(mesh.pipe(rail, C["frame"] * 0.55, 8, caps=True))

    return {"canopy_frame": mesh.join(*parts)}


def _sill(x, sgn):
    t = (x - C["x_front"]) / (C["x_rear"] - C["x_front"])
    w, _ = _profile(t)
    _, _, zc, _ = fuselage.station_at(x)
    return (x, sgn * w, C["z_base"] + zc * 0.15)


def _hardware():
    """Rear pivots and side locks carry the bubble without blocking the aperture.

    The seal follows the shared sill, and the breaker cord lies just inside
    the curved glass rather than crossing the pilot's head as a straight bar.
    """
    out = {}
    seals, locks, hinges, actuators, bows = [], [], [], [], []
    xa, xb = C["windscreen_x"], C["x_rear"]
    for sgn in (-1.0, 1.0):
        seals.append(mesh.pipe([
            (x, y, z - H["seal"])
            for x, y, z in [_sill(xa + (xb - xa) * i / SEG, sgn)
                            for i in range(SEG + 1)]], H["seal"], 8))
        for f in (0.15, 0.45, 0.72):
            x, y, z = _sill(xa + (xb - xa) * f, sgn)
            locks.append(shapes.rounded_box(
                x, y - sgn * H["lock_width"], z - H["lock_height"],
                H["lock_length"], H["lock_width"], H["lock_height"],
                r=H["seal"], seg=3))
        x, y, z = _sill(H["hinge_x"], sgn)
        hinges.append(mesh.pipe([(x, y - H["hinge_span"] / 2, z),
                                 (x, y + H["hinge_span"] / 2, z)],
                                H["hinge_radius"], 16))
        p0 = (x - H["hinge_span"] * 2, y, z - H["hinge_span"])
        p1 = _sill(x - H["hinge_span"], sgn)
        pm = tuple(p0[k] + (p1[k] - p0[k]) * 0.6 for k in range(3))
        actuators.append(mesh.pipe([p0, pm], H["strut_radius"], 12))
        actuators.append(mesh.pipe([pm, p1], H["slider_radius"], 12))
        front = [_sill(C["x_front"] + (xa - C["x_front"]) * i / SEG, sgn)
                 for i in range(SEG + 1)]
        bows.append(mesh.pipe(front, C["frame"] * 0.4, 10))
    w, h = _profile((xa - C["x_front"]) / (xb - C["x_front"]))
    base = _sill(xa, 1.0)[2]
    seals.append(mesh.pipe([
        (xa, (w - H["glass"]) * math.cos(math.pi * i / SEG),
         base + (h - H["glass"]) * math.sin(math.pi * i / SEG))
        for i in range(SEG + 1)], H["seal"], 8))
    rear = [_sill(xb, sgn) for sgn in (-1.0, 1.0)]
    seals.append(mesh.pipe(rear, H["seal"], 8))
    out["frame_canopy_seal"] = mesh.join(*seals)
    out["canopy_latch_locks"] = mesh.join(*locks)
    out["frame_canopy_rear_hinge"] = mesh.join(*hinges)
    out["strut_canopy_actuator"] = mesh.join(*actuators)
    out["frame_windscreen_bow"] = mesh.join(*bows)
    cords = []
    for f in (0.36, 0.57):
        x = C["x_front"] + (C["x_rear"] - C["x_front"]) * f
        w, h = _profile(f)
        base = _sill(x, 1.0)[2]
        path = []
        for i in range(SEG + 1):
            a = math.pi * i / SEG
            inset = H["glass"] + H["cord_radius"]
            path.append((x, (w - inset) * math.cos(a),
                         base + (h - inset) * math.sin(a)))
        cords.append(mesh.pipe(path, H["cord_radius"], 8))
    out["frame_canopy_breaker_cord"] = mesh.join(*cords)
    return out
