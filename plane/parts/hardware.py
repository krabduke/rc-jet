"""The hardware the airframe was missing.

A thrust tube between the engine and the tailpipe, the joiner and bolts that
hold the wings on, output arms on the servos, straps over the batteries, the
steering link to the nose leg, and the exhaust cone in the tailpipe.

Most of this is not decoration on a model this size. The wings were carried by
a spar that passed through the fuselage with nothing clamping it; four servos
drove pushrods with no arms between them; and the engine exhausted into a
shroud with no tube joining the two.
"""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes


def build():
    out = {}
    out.update(_thrust_tube())
    out.update(_wing_joint())
    out.update(_servo_arms())
    out.update(_battery_straps())
    out.update(_nose_steering())
    return out


def _lathe(profile, cx, cy, cz, axis="x", seg=22):
    v, f = mesh.revolve_closed(list(profile), seg)
    if axis == "z":
        v = [(pz, py, px) for (px, py, pz) in v]
    elif axis == "y":
        v = [(pz, px, py) for (px, py, pz) in v]
    return ([(px + cx, py + cy, pz + cz) for (px, py, pz) in v], f)


def _thrust_tube():
    """Engine exhaust to tailpipe, and the cone inside it.

    The engine ended at the mount ring and the tailpipe shroud began 110 mm
    later with nothing between them, so the exhaust had nowhere to go.
    """
    out = {}
    x0, x1 = 318.0, 426.0
    parts = []
    # the tube, necking down towards the pipe
    parts.append(_lathe(
        [(x0, 14.4), (x0 + 12.0, 15.6), (x1 - 26.0, 13.2), (x1, 12.4),
         (x1, 13.6), (x1 - 26.0, 14.4), (x0 + 12.0, 16.8), (x0, 15.6)],
        0.0, 0.0, 0.0, axis="x", seg=28))
    # standoff mounts holding it off the rails, three stations round the clock
    for fx in (0.18, 0.52, 0.86):
        bx = x0 + (x1 - x0) * fx
        for k in range(3):
            a = 2 * math.pi * k / 3 + math.pi / 6
            p0 = (bx, 15.8 * math.cos(a), 15.8 * math.sin(a))
            p1 = (bx, 20.4 * math.cos(a), 20.4 * math.sin(a))
            parts.append(mesh.pipe([p0, p1], 1.5, segments=8))
        parts.append(_lathe([(-1.6, 15.8), (1.6, 15.8), (1.6, 17.4),
                             (-1.6, 17.4)], bx, 0.0, 0.0, axis="x", seg=24))
    out["thrust_tube"] = mesh.join(*parts)

    # the exhaust cone, on its spider inside the pipe
    parts = []
    parts.append(_lathe(
        [(424.0, 0.0), (424.0, 7.4), (427.0, 7.2), (430.0, 6.6),
         (433.0, 5.6), (436.0, 4.0), (438.5, 2.2), (440.0, 1.2),
         (440.0, 0.0)], 0.0, 0.0, 0.0, axis="x", seg=36))
    for k in range(4):
        a = 2 * math.pi * k / 4 + math.pi / 4
        parts.append(mesh.pipe(
            [(426.0, 7.0 * math.cos(a), 7.0 * math.sin(a)),
             (426.0, 15.4 * math.cos(a), 15.4 * math.sin(a))],
            1.3, segments=10))
    out["tailpipe_cone"] = mesh.join(*parts)
    return out


def _wing_joint():
    """Joiner tube through the fuselage, and a bolt through each wing root.

    The two wing panels met a spar that simply passed through; nothing held
    either of them on.
    """
    out = {}
    parts = []
    # the joiner: a sleeve round the carbon spar, bonded into the bulkheads
    parts.append(_lathe(
        [(-46.0, 2.4), (46.0, 2.4), (46.0, 4.6), (-46.0, 4.6)],
        256.0, 0.0, -6.0, axis="y", seg=22))
    for sy in (-1.0, 1.0):
        parts.append(_lathe(
            [(-3.0, 4.6), (3.0, 4.6), (3.0, 8.2), (-3.0, 8.2)],
            256.0, sy * 30.0, -6.0, axis="y", seg=20))
    out["wing_joiner"] = mesh.join(*parts)

    for tag, sy in (("l", -1.0), ("r", 1.0)):
        parts = []
        x, y, z = 292.0, sy * 22.0, -6.0
        # the bolt itself, head proud under the wing
        parts.append(_lathe(
            [(0.0, 0.0), (0.0, 4.6), (2.6, 5.4), (3.4, 5.4), (3.4, 2.1),
             (22.0, 2.1), (22.0, 0.0)], x, y, z - 12.0, axis="z", seg=14))
        # the captive nut plate in the fuselage above it
        parts.append(shapes.rounded_box(x, y, z + 7.0, 15.0, 13.0, 2.4, r=0.8))
        for dx in (-5.0, 5.0):
            parts.append(_lathe(
                [(0.0, 0.0), (2.0, 0.0), (2.0, 1.3), (0.0, 1.3)],
                x + dx, y, z + 8.6, axis="z", seg=8))
        out[f"wing_bolt_{tag}"] = mesh.join(*parts)
    return out


def _servo_arms():
    """Output arms, between each servo and the pushrod it already drove."""
    out = {}
    servos = (("ail_l", 285.0, -19.0, 8.0), ("ail_r", 285.0, 19.0, 8.0),
              ("rudder", 288.0, 8.0, 23.0), ("stab", 288.0, -8.0, 23.0))
    for tag, x, y, z in servos:
        parts = []
        # the spline boss the arm clamps onto
        parts.append(_lathe(
            [(0.0, 0.0), (0.0, 3.4), (3.0, 3.4), (3.0, 2.2), (5.2, 2.2),
             (5.2, 0.0)], x, y, z, axis="z", seg=14))
        # the arm, with the hole pattern along it
        parts.append(shapes.rounded_box(x + 5.5, y, z + 2.0,
                                        16.0, 3.6, 2.0, r=0.8))
        for k in range(3):
            parts.append(_lathe(
                [(0.0, 0.8), (2.2, 0.8), (2.2, 1.5), (0.0, 1.5)],
                x + 8.0 + k * 3.2, y, z + 1.4, axis="z", seg=8))
        out[f"servo_arm_{tag}"] = mesh.join(*parts)
    return out


def _battery_straps():
    """Hook-and-loop straps over each pack.

    Two lithium packs were sitting on their trays held down by nothing.
    """
    out = {}
    packs = (("lipo", 240.0, 0.0, 2.0, 42.0, 26.0), ("rx", 102.0, 0.0, 4.0, 30.0, 18.0))
    for tag, x, y, z, ln, wd in packs:
        parts = []
        for dx in (-ln * 0.26, ln * 0.26):
            # round the pack in a rounded rectangle rather than four straight
            # runs, so it reads as webbing pulled tight rather than a wire
            path = []
            for k in range(29):
                a = 2 * math.pi * k / 28
                path.append((x + dx,
                             (wd / 2 + 1.6) * math.sin(a),
                             z + (8.6) * math.cos(a)))
            parts.append(mesh.pipe(path, 1.5, segments=8))
            # the buckle on top
            parts.append(shapes.rounded_box(x + dx, 0.0, z + 10.6,
                                            4.2, 7.0, 1.8, r=0.5))
            parts.append(_lathe(
                [(0.0, 2.0), (1.2, 2.0), (1.2, 3.0), (0.0, 3.0)],
                x + dx, 0.0, z + 11.6, axis="z", seg=10))
        out[f"battery_strap_{tag}"] = mesh.join(*parts)
    return out


def _nose_steering():
    """The link from the rudder servo down to the nose leg."""
    parts = []
    p0 = (292.0, 8.0, 23.0)
    p1 = (150.0, 4.0, -6.0)
    p2 = (104.0, 2.0, -20.0)
    parts.append(mesh.pipe([p0, p1, p2], 1.1, segments=8))
    # the bellcrank partway along, on its post
    parts.append(_lathe(
        [(0.0, 0.0), (0.0, 4.2), (2.0, 4.2), (2.0, 0.0)],
        150.0, 4.0, -6.0, axis="z", seg=12))
    parts.append(_lathe(
        [(0.0, 0.0), (7.0, 0.0), (7.0, 1.2), (0.0, 1.2)],
        150.0, 4.0, -12.0, axis="z", seg=8))
    # the steering arm on the leg itself
    parts.append(shapes.rounded_box(102.0, 2.0, -20.0, 9.0, 2.2, 4.0, r=0.6))
    return {"nose_steering_link": mesh.join(*parts)}
