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
    # A socket each side, not a sleeve through the middle.
    #
    # The sleeve ran from y -46 to +46 at z -6, which is the middle of the
    # intake duct: the duct is 41 mm across at this station and the wing sits
    # in it. Each panel's stub spar plugs into its own socket on the fuselage
    # side frame, outboard of the duct, and the load crosses the fuselage
    # through the frames.
    y0 = spec.INTAKE["duct_r_end"] + 3.5
    for sy in (-1.0, 1.0):
        # the socket the stub spar plugs into
        parts.append(_lathe(
            [(sy * (y0 - 2.0), 2.4), (sy * (y0 + 22.0), 2.4),
             (sy * (y0 + 22.0), 4.6), (sy * (y0 - 2.0), 4.6)],
            256.0, 0.0, -6.0, axis="y", seg=22))
        # the collar that lands it on the frame
        parts.append(_lathe(
            [(-3.0, 4.6), (3.0, 4.6), (3.0, 8.2), (-3.0, 8.2)],
            256.0, sy * (y0 + 1.5), -6.0, axis="y", seg=20))
    out["wing_joiner"] = mesh.join(*parts)

    for tag, sy in (("l", -1.0), ("r", 1.0)):
        parts = []
        # outboard of the duct, like everything else that picks the wing up
        x, y, z = 292.0, sy * (spec.INTAKE["duct_r_end"] + 7.0), -6.0
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
    # On the servos, from the same table the servos are placed from.
    #
    # This was a third set of hard-coded positions and not one of them was on
    # a servo: the aileron arms at y +/-19 for servos at +/-55, the tail arms
    # at x 288 z 23 for servos that are not there either. Four output arms
    # turning in mid air, two of them inside the intake duct.
    for tag in ("ail_l", "ail_r", "rudder", "stab_l", "stab_r"):
        sx, sy, sz, sl, _sw, sh = spec.equipment(f"servo_{tag}")
        # the output shaft stands on the top face, towards one end
        x, y, z = sx + sl * 0.30, sy, sz + sh / 2
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
    # From the equipment table, not from a second set of numbers.
    #
    # The lipo strap was at x 240 and the pack it straps at x 117; the rx
    # strap at x 102 and its pack at 208. Both were tightened round nothing,
    # and the lipo one was inside the intake duct.
    packs = []
    for tag, key in (("lipo", "lipo_3s_900"), ("rx", "rx_battery")):
        px, py, pz, pl, pw, ph = spec.equipment(key)[:6]
        packs.append((tag, px, py, pz, pl, pw, ph))
    for tag, x, y, z, ln, wd, ht in packs:
        parts = []
        for dx in (-ln * 0.26, ln * 0.26):
            # round the pack in a rounded rectangle rather than four straight
            # runs, so it reads as webbing pulled tight rather than a wire
            # Over the top and down to the tray, not all the way round.
            #
            # A closed loop passes UNDER the pack, which is through whatever
            # the pack is sitting on -- the former, the seat pan, the nose
            # gear door. A strap is anchored to the tray at each side.
            path = []
            for k in range(21):
                a = math.pi * (-0.06 + 1.12 * k / 20)
                path.append((x + dx,
                             y - (wd / 2 + 1.6) * math.cos(a),
                             z + (ht / 2 + 0.9) * math.sin(a)))
            parts.append(mesh.pipe(path, 1.5, segments=8))
            # the buckle on top
            parts.append(shapes.rounded_box(x + dx, y, z + ht / 2 + 2.0,
                                            4.2, 7.0, 1.8, r=0.5))
            parts.append(_lathe(
                [(0.0, 2.0), (1.2, 2.0), (1.2, 3.0), (0.0, 3.0)],
                x + dx, y, z + ht / 2 + 2.9, axis="z", seg=10))
        out[f"battery_strap_{tag}"] = mesh.join(*parts)
    return out


def _nose_steering():
    """The link from the rudder servo down to the nose leg."""
    # Over the duct, to a leg that is now ahead of it.
    #
    # The run used to go from the rudder servo straight down the middle of the
    # fuselage at z -6, which for 95 % of its length was inside the intake.
    sx, sy, sz = spec.equipment("servo_rudder")[:3]
    gx = spec.GEAR["nose_x"]
    parts = []
    # Outboard of the duct the whole way, over its shoulder, then in and down
    # to the leg ahead of the throat.
    p0 = (sx - 8.0, sy * 0.62, sz + 10.0)
    p1 = (210.0, 25.0, 18.0)
    p2 = (140.0, 24.0, 12.0)
    p3 = (100.0, 19.0, 8.0)
    p4 = (spec.INTAKE["x_throat"] - 6.0, 6.0, 2.0)
    p5 = (gx, 2.0, -18.0)
    parts.append(mesh.pipe([p0, p1, p2, p3, p4, p5], 1.1, segments=8, subdiv=2))
    # the bellcrank partway along, on its post, beside the duct
    parts.append(_lathe(
        [(0.0, 0.0), (0.0, 4.2), (2.0, 4.2), (2.0, 0.0)],
        140.0, 24.0, 12.0, axis="z", seg=12))
    parts.append(_lathe(
        [(0.0, 0.0), (7.0, 0.0), (7.0, 1.2), (0.0, 1.2)],
        140.0, 24.0, 6.0, axis="z", seg=8))
    # the steering arm on the leg itself
    parts.append(shapes.rounded_box(gx, 2.0, -18.0, 9.0, 2.2, 4.0, r=0.6))
    return {"nose_steering_link": mesh.join(*parts)}
