"""Fixed tricycle landing gear, modelled down."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes
from parts import fuselage

G = spec.GEAR
SEG = spec.RES["small_revolve"]


def build():
    out = {}
    out.update(_nose())
    out.update(_mains())
    return out


def _wheel(x, y, z, r, w):
    """A wheel with a rounded tread, lathed about the +y axis so it lies in the
    x-z plane and rolls the right way."""
    verts, faces = [], []
    nr, nt = 14, 22
    for i in range(nr):
        a = 2 * math.pi * i / nr
        rr = r * 0.72 + (r * 0.28) * math.sin(a)
        yy = y + (w / 2) * math.cos(a) * 0.85
        for j in range(nt):
            b = 2 * math.pi * j / nt
            verts.append((x + rr * math.cos(b), yy, z + rr * math.sin(b)))
    for i in range(nr):
        i2 = (i + 1) % nr
        for j in range(nt):
            j2 = (j + 1) % nt
            faces.append((i * nt + j, i * nt + j2, i2 * nt + j2, i2 * nt + j))
    return verts, faces


def _strut(x, y, z_top, length, r, rake=0.0):
    """An oleo leg, which is two tubes and not one.

    A landing gear leg has to absorb the landing: the sliding member runs
    inside the outer cylinder, so there are two diameters with a wiper seal
    between them, a trunnion at the top where it pivots into the bay, a torque
    link stopping the axle from castoring, and an axle boss at the bottom. A
    single capped cylinder is a peg.
    """
    a = math.radians(rake)
    sa, ca = math.sin(a), math.cos(a)

    def place(verts):
        # built with +x down the leg from the trunnion; rake it and drop it in
        return [(x + px * sa + pz, y + py, z_top - px * ca)
                for (px, py, pz) in verts]

    parts = []
    split = length * 0.52
    # outer cylinder with its wiper gland, then the sliding member
    parts.append((place(mesh.revolve_closed(
        [(0.0, 0.0), (split, 0.0), (split, r * 0.72), (split - 1.2, r * 1.06),
         (split - 3.0, r * 1.10), (4.0, r * 1.10), (1.5, r * 0.96),
         (0.0, r * 0.80)], 34)[0]), mesh.revolve_closed(
        [(0.0, 0.0), (split, 0.0), (split, r * 0.72), (split - 1.2, r * 1.06),
         (split - 3.0, r * 1.10), (4.0, r * 1.10), (1.5, r * 0.96),
         (0.0, r * 0.80)], 34)[1]))
    parts.append((place(mesh.revolve_closed(
        [(split - 4.0, 0.0), (length, 0.0), (length, r * 0.74),
         (split - 4.0, r * 0.74)], 30)[0]), mesh.revolve_closed(
        [(split - 4.0, 0.0), (length, 0.0), (length, r * 0.74),
         (split - 4.0, r * 0.74)], 30)[1]))
    # trunnion: the pivot the whole leg swings on
    tv, tf = mesh.revolve_closed(
        [(-r * 1.6, r * 0.5), (r * 1.6, r * 0.5), (r * 1.6, r * 1.5),
         (r * 1.2, r * 1.7), (-r * 1.2, r * 1.7), (-r * 1.6, r * 1.5)], 26)
    parts.append((place([(pz + 2.0, px, py) for (px, py, pz) in tv]), tf))
    # torque link: two arms with a knuckle, on the forward face
    for (p0, p1) in (((split - 10.0, 0.0, -r * 1.5),
                      (split + 3.0, 0.0, -r * 2.6)),
                     ((split + 3.0, 0.0, -r * 2.6),
                      (split + 18.0, 0.0, -r * 1.3))):
        seg = mesh.pipe([(p0[0], p0[1], p0[2]), (p1[0], p1[1], p1[2])],
                        r * 0.34, 14)
        parts.append((place(seg[0]), seg[1]))
    # axle boss
    av, af = mesh.revolve_closed(
        [(-r * 1.4, 0.0), (r * 1.4, 0.0), (r * 1.4, r * 0.6),
         (r * 1.0, r * 0.9), (-r * 1.0, r * 0.9), (-r * 1.4, r * 0.6)], 22)
    parts.append((place([(pz + length - r * 1.2, px, py)
                         for (px, py, pz) in av]), af))
    return mesh.join(*parts)


def _nose():
    w, hh, zc, _ = fuselage.station_at(G["nose_x"])
    z_top = zc - hh + 1.0
    leg = _strut(G["nose_x"], 0.0, z_top, G["nose_leg"], G["strut_r"], rake=-6.0)
    z_ax = z_top - G["nose_leg"] * math.cos(math.radians(-6.0))
    x_ax = G["nose_x"] + G["nose_leg"] * math.sin(math.radians(-6.0))
    wheel = _wheel(x_ax, 0.0, z_ax, G["nose_wheel_r"], G["nose_wheel_w"])
    return {"gear_nose_strut": leg, "wheel_nose": wheel}


def _mains():
    """One leg per side, named per side.

    They were one object called `gear_main_struts` and one called
    `wheel_main`, each holding both sides. That hides the thing the structure
    audit exists to check -- that a left part really is the mirror of its
    right -- and it means the viewer cannot retract one leg without the other.
    """
    out = {}
    w, hh, zc, _ = fuselage.station_at(G["main_x"])
    z_top = zc - hh * 0.55
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        y = sgn * G["main_y"]
        out[f"gear_main_{side}"] = _strut(
            G["main_x"], y, z_top, G["main_leg"], G["strut_r"], rake=4.0)
        z_ax = z_top - G["main_leg"] * math.cos(math.radians(4.0))
        x_ax = G["main_x"] + G["main_leg"] * math.sin(math.radians(4.0))
        out[f"wheel_main_{side}"] = _wheel(
            x_ax, y, z_ax, G["main_wheel_r"], G["main_wheel_w"])
        out[f"brake_{side}"] = _brake(x_ax, y, z_ax, sgn)
    return out


def _brake(x, y, z, sgn):
    """A disc and a caliper on each main wheel.

    A retract set this size is sold with brakes, and a model jet needs them:
    it lands at 60 km/h on whatever runway it has, and the nose leg steers
    rather than stops. The disc is drilled, the caliper has two pistons and
    a banjo where the line comes in.
    """
    r = G["main_wheel_r"] * 0.62
    parts = []
    dv, df = mesh.revolve_closed(
        [(0.0, r * 0.30), (0.0, r), (1.4, r), (1.4, r * 0.30)], 30)
    parts.append(([(pz + x, py + y - sgn * (G["main_wheel_w"] / 2 + 0.9),
                    px + z) for (px, py, pz) in dv], df))
    # the bell the disc bolts to
    bv, bf = mesh.revolve_closed(
        [(0.0, 1.6), (0.0, r * 0.34), (-3.2, r * 0.34), (-3.2, 1.6)], 22)
    parts.append(([(pz + x, py + y - sgn * (G["main_wheel_w"] / 2 + 0.9),
                    px + z) for (px, py, pz) in bv], bf))
    # drillings, which is what a disc this size has instead of vanes
    for k in range(10):
        a = 2 * math.pi * k / 10
        hv, hf = mesh.cylinder(0.0, 2.2, 0.55, 8)
        parts.append(([(pz + x + r * 0.66 * math.cos(a),
                        px + y - sgn * (G["main_wheel_w"] / 2 + 1.6),
                        py + z + r * 0.66 * math.sin(a))
                       for (px, py, pz) in hv], hf))
    # caliper astride the disc, forward of the axle
    cv, cf = shapes.rounded_box(
        x - r * 0.78, y - sgn * (G["main_wheel_w"] / 2 + 0.9), z + r * 0.30,
        5.0, 5.6, 8.0, r=1.0)
    parts.append((cv, cf))
    for dz in (-2.2, 2.2):
        pv, pf = mesh.revolve_closed(
            [(0.0, 0.0), (2.0, 0.0), (2.0, 1.5), (0.0, 1.5)], 12)
        parts.append(([(pz + x - r * 0.78,
                        px + y - sgn * (G["main_wheel_w"] / 2 + 3.4),
                        py + z + r * 0.30 + dz) for (px, py, pz) in pv], pf))
    parts.append(mesh.pipe(
        [(x - r * 0.78, y - sgn * (G["main_wheel_w"] / 2 + 0.9), z + r * 0.30 + 4.6),
         (x - r * 0.40, y - sgn * (G["main_wheel_w"] / 2 + 0.4), z + r * 0.92),
         (x + 2.0, y - sgn * 1.0, z + G["main_leg"] * 0.40)], 0.7, 10))
    return mesh.join(*parts)
