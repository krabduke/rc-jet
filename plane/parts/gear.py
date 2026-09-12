"""Fixed tricycle landing gear, modelled down."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
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
    a = math.radians(rake)
    x_bot = x + length * math.sin(a)
    path = [(x, y, z_top), (x_bot, y, z_top - length * math.cos(a))]
    return mesh.pipe(path, r, 10, caps=True)


def _nose():
    w, hh, zc, _ = fuselage.station_at(G["nose_x"])
    z_top = zc - hh + 1.0
    leg = _strut(G["nose_x"], 0.0, z_top, G["nose_leg"], G["strut_r"], rake=-6.0)
    z_ax = z_top - G["nose_leg"] * math.cos(math.radians(-6.0))
    x_ax = G["nose_x"] + G["nose_leg"] * math.sin(math.radians(-6.0))
    wheel = _wheel(x_ax, 0.0, z_ax, G["nose_wheel_r"], G["nose_wheel_w"])
    return {"gear_nose_strut": leg, "wheel_nose": wheel}


def _mains():
    out = {}
    w, hh, zc, _ = fuselage.station_at(G["main_x"])
    z_top = zc - hh * 0.55
    legs, wheels = [], []
    for sgn in (-1.0, 1.0):
        y = sgn * G["main_y"]
        legs.append(_strut(G["main_x"], y, z_top, G["main_leg"],
                           G["strut_r"], rake=4.0))
        z_ax = z_top - G["main_leg"] * math.cos(math.radians(4.0))
        x_ax = G["main_x"] + G["main_leg"] * math.sin(math.radians(4.0))
        wheels.append(_wheel(x_ax, y, z_ax, G["main_wheel_r"], G["main_wheel_w"]))
    out["gear_main_struts"] = mesh.join(*legs)
    out["wheel_main"] = mesh.join(*wheels)
    return out
