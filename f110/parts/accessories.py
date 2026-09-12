"""Accessory drive and external plumbing: the gearbox, the tower shaft that
drives it off the HP spool, fuel and oil lines, and the electrical harnesses."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
A = spec.ACCESSORIES


def build():
    out = {}
    out.update(_gearbox())
    out.update(_towershaft())
    out.update(_plumbing())
    return out


def _at(radius, clock_deg, x, extra_r=0.0):
    a = math.radians(clock_deg)
    r = radius + extra_r
    return (x, r * math.cos(a), r * math.sin(a))


def _gearbox():
    """Airframe-mounted-style accessory gearbox slung under the fan case,
    with the pump and generator pads that hang off it."""
    cx = A["gearbox_x"] + A["gearbox_len"] / 2
    ang = A["gearbox_angle"]
    cy = A["gearbox_r"] * math.cos(math.radians(ang))
    cz = A["gearbox_r"] * math.sin(math.radians(ang))

    body = mesh.box(cx, cy, cz, A["gearbox_len"],
                    A["gearbox_width"], A["gearbox_height"])
    parts = [body]

    # accessory pads: fuel pump, oil pump, starter/generator
    pads = [(-0.30, 96.0, 120.0), (0.02, 78.0, 96.0), (0.34, 110.0, 150.0)]
    for (fx, pr, pl) in pads:
        px = cx + A["gearbox_len"] * fx
        pv, pf = mesh.cylinder(0.0, pl, pr, 20)
        pv = mesh.rot_z(pv, math.pi / 2)          # axis now +Y
        pv = mesh.translate(pv, px, cy - A["gearbox_width"] / 2 - pl * 0.0, cz)
        pv = [(x, y - A["gearbox_width"] / 2, z) for (x, y, z) in pv]
        parts.append((pv, pf))
        # mounting flange
        fv, ff = mesh.cylinder(0.0, 14.0, pr * 1.22, 20)
        fv = mesh.rot_z(fv, math.pi / 2)
        fv = [(x + px, y - A["gearbox_width"] / 2 + cy, z + cz) for (x, y, z) in fv]
        parts.append((fv, ff))

    return {"gearbox": mesh.join(*parts)}


def _towershaft():
    """Radial drive shaft: takes power off the HP spool through a bevel gear
    and carries it down to the gearbox. This is also the starting path -- the
    starter drives back up it to crank the HP spool."""
    ang = A["towershaft_angle"]
    p0 = _at(A["towershaft_r0"], ang, A["towershaft_x"])
    p1 = _at(A["towershaft_r1"], ang, A["towershaft_x"])
    shaft = mesh.pipe([p0, p1], A["towershaft_r"], 16)

    # bevel gear at the inboard end, housing at the outboard end
    gv, gf = mesh.revolve_open(
        [(0.0, 0.001), (0.0, 62.0), (34.0, 40.0), (34.0, 0.001)],
        24, cap_start=True, cap_end=True)
    gv = mesh.rot_z(gv, -math.pi / 2)
    gv = mesh.translate(gv, *p0)

    hv, hf = mesh.cylinder(0.0, 120.0, 54.0, 20)
    hv = mesh.rot_z(hv, -math.pi / 2)
    hv = mesh.translate(hv, p1[0], p1[1], p1[2])
    hv = [(x, y + 60.0 * math.cos(math.radians(ang)),
           z + 60.0 * math.sin(math.radians(ang))) for (x, y, z) in hv]

    return {"towershaft": mesh.join(shaft, (gv, gf), (hv, hf))}


def _casing_radius(x):
    """Outer radius of whatever casing is at station x -- so external lines can
    be routed to lie on the engine instead of floating beside it."""
    best = 300.0
    for (_, x0, x1, r0, r1, wall) in spec.CASINGS:
        if x0 <= x <= x1:
            f = (x - x0) / max(x1 - x0, 1e-6)
            best = max(best, r0 + (r1 - r0) * f + wall)
    return best


def _route(clock_deg, x_start, x_end, standoff, n=16):
    return [_at(_casing_radius(x_start + (x_end - x_start) * i / n),
                clock_deg, x_start + (x_end - x_start) * i / n, standoff)
            for i in range(n + 1)]


def _plumbing():
    """External lines. Routed along the casing at realistic clock positions:
    fuel high on the sides, oil returning to the gearbox low down, electrical
    harnesses clipped along the top."""
    out = {}

    fuel = []
    for i, clock in enumerate((-58.0, -122.0, -74.0, -106.0)):
        x0 = A["gearbox_x"] + 60.0
        x1 = spec.COMBUSTOR["x_front"] - 40.0
        path = _route(clock, x0, x1, 34.0 + i * 15.0)
        fuel.append(mesh.pipe(path, A["fuel_line_r"], 12))
        for j in range(0, len(path), 5):      # support clamps
            cv, cf = mesh.cylinder(-9.0, 9.0, A["fuel_line_r"] * 1.7, 10)
            cv = mesh.rot_z(cv, math.pi / 2)
            ang = math.atan2(path[j][2], path[j][1])
            cv = mesh.rot_x(cv, ang)
            cv = mesh.translate(cv, path[j][0], path[j][1], path[j][2])
            fuel.append((cv, cf))
    out["fuel_lines"] = mesh.join(*fuel)

    oil = []
    for i, clock in enumerate((-84.0, -96.0, -70.0)):
        path = _route(clock, spec.STATION["fan_frame"] - 40.0,
                      spec.STATION["turbine_frame"], 20.0 + i * 13.0)
        oil.append(mesh.pipe(path, A["oil_line_r"], 12))
    out["oil_lines"] = mesh.join(*oil)

    harness = []
    for i, clock in enumerate((84.0, 96.0, 70.0)):
        path = _route(clock, spec.STATION["fan_face"] + 60.0,
                      spec.STATION["augmentor_exit"] - 120.0, 26.0 + i * 12.0)
        harness.append(mesh.pipe(path, A["harness_r"], 10))
        for j in range(0, len(path), 4):
            bv, bf = mesh.box(path[j][0], path[j][1], path[j][2], 16.0, 20.0, 20.0)
            harness.append((bv, bf))
    out["harnesses"] = mesh.join(*harness)
    return out
