"""Retractable tricycle gear shown locked down, with open lined wheel bays."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import shapes

# these full-size gear and bay dimensions belong in spec.py.
G = dict(spec.GEAR, nose_x=40.0, main_wheel_r=750.0 / (2 * spec.SCALE_TO_FULL),
         nose_wheel_r=550.0 / (2 * spec.SCALE_TO_FULL))
B = dict(wall=0.6, door=0.65, clearance=1.0, ground=-65.0,
         nose_top=-3.0, main_top=-6.0, main_roof=-2.0,
         nose_roof=2.0, nose_bay_length=20.0, nose_bay_width=12.0,
         nose_bay_x=36.0, main_bay_length=30.0, main_bay_width=36.0,
         main_bay_y=49.0, main_bay_floor=-10.0, nose_bay_floor=-5.0)
SEG = spec.RES["small_revolve"]


def build():
    out = {}
    out.update(_nose())
    out.update(_mains())
    return out


def _wheel(x, y, z, r, w):
    """A wheel with a rounded tread, lathed about the +y axis so it lies in the
    x-z plane and rolls the right way."""
    profile = [(-w * 0.40, r * 0.49), (-w * 0.52, r * 0.67),
               (-w * 0.48, r * 0.84), (-w * 0.34, r * 0.97)]
    for i in range(25):
        t = i / 24
        radius = r * (0.984 if i % 6 in (2, 3) else 1.0)
        profile.append((-w * 0.30 + w * 0.60 * t, radius))
    profile.extend([(w * 0.34, r * 0.97), (w * 0.48, r * 0.84),
                    (w * 0.52, r * 0.67), (w * 0.40, r * 0.49)])
    v, f = mesh.revolve_closed(profile, SEG * 2)
    return [(x + pz, y + px, z + py) for px, py, pz in v], f


def _strut(x, y, z_top, length, r, rake=0.0, slider=False):
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
        return [(x + px * sa + pz * ca, y + py, z_top - px * ca + pz * sa)
                for (px, py, pz) in verts]

    parts = []
    split = length * 0.52
    if slider:
        v, f = mesh.revolve_closed(
            [(split - r, 0.0), (length, 0.0), (length, r * 0.74),
             (split - r, r * 0.74)], 30)
        return place(v), f
    # outer cylinder with its wiper gland, then the sliding member
    parts.append((place(mesh.revolve_closed(
        [(0.0, 0.0), (split, 0.0), (split, r * 0.72), (split - 1.2, r * 1.06),
         (split - 3.0, r * 1.10), (4.0, r * 1.10), (1.5, r * 0.96),
         (0.0, r * 0.80)], 34)[0]), mesh.revolve_closed(
        [(0.0, 0.0), (split, 0.0), (split, r * 0.72), (split - 1.2, r * 1.06),
         (split - 3.0, r * 1.10), (4.0, r * 1.10), (1.5, r * 0.96),
         (0.0, r * 0.80)], 34)[1]))
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
    """The nose unit sits ahead of the chin mouth, not through its airflow.

    A forward wheel pocket and its open door pair surround the trunnion;
    the long exposed slider reaches the same ground plane as the main tyres.
    """
    x, r = G["nose_x"], G["strut_r"]
    z_top = B["nose_top"]
    z_ax = B["ground"] + G["nose_wheel_r"]
    length = z_top - z_ax
    out = {"gear_nose_strut": _strut(
        x, 0.0, z_top, length - G["nose_wheel_r"] - r, r)}
    out["gear_nose_chrome_slider"] = _strut(
        x, 0.0, z_top, length - G["nose_wheel_r"] - r, r, slider=True)
    out["wheel_nose"] = _wheel(x, 0.0, z_ax, G["nose_wheel_r"], G["nose_wheel_w"])
    fork = []
    for sgn in (-1.0, 1.0):
        y = sgn * (G["nose_wheel_w"] / 2 + B["clearance"])
        fork.append(mesh.pipe([(x, 0.0, z_ax + G["nose_wheel_r"] + r),
                               (x, y, z_ax + G["nose_wheel_r"]),
                               (x, y, z_ax)], r * 0.5, 14))
    out["gear_nose_fork"] = mesh.join(*fork)
    out["wheel_hub_nose"] = _hub(x, 0.0, z_ax, G["nose_wheel_r"], G["nose_wheel_w"])
    out["gear_nose_steering_actuator"] = _actuator(
        (x - r * 2, -r * 2, z_top - length * 0.3),
        (x + r * 2, r * 2, z_top - length * 0.3), r * 0.35)
    out["gear_nose_shimmy_damper"] = _actuator(
        (x - r, -r * 2, z_top - length * 0.4),
        (x + r * 2, r * 1.5, z_top - length * 0.4), r * 0.25)
    out["gear_nose_drag_stay"] = mesh.pipe(
        [(B["nose_bay_x"] - B["nose_bay_length"] / 2 + r, 0.0, z_top),
         (x - r * 3, 0.0, z_top - length * 0.2),
         (x, 0.0, z_top - length * 0.4)], r * 0.4, 12)
    out.update(_bay("nose", B["nose_bay_x"], 0.0, B["nose_bay_length"],
                    B["nose_bay_width"], B["nose_bay_floor"], B["nose_roof"]))
    return out


def _mains():
    """One leg per side, named per side.

    They were one object called `gear_main_struts` and one called
    `wheel_main`, each holding both sides. That hides the thing the structure
    audit exists to check -- that a left part really is the mirror of its
    right -- and it means the viewer cannot retract one leg without the other.
    """
    out = {}
    x, r, z_top = G["main_x"], G["strut_r"], B["main_top"]
    z_ax = B["ground"] + G["main_wheel_r"]
    for side, sgn in (("l", -1.0), ("r", 1.0)):
        y = sgn * G["main_y"]
        x_ax = x + r * 3
        wheel_y = y + sgn * (G["main_wheel_w"] / 2 + r)
        out[f"gear_main_{side}"] = _strut(x, y, z_top, z_top - z_ax - r * 2, r)
        out[f"gear_main_chrome_slider_{side}"] = _strut(
            x, y, z_top, z_top - z_ax - r * 2, r, slider=True)
        out[f"gear_main_trailing_link_{side}"] = mesh.pipe(
            [(x, y, z_ax + r * 2), (x_ax, y, z_ax),
             (x_ax, wheel_y, z_ax)], r * 0.65, 18)
        out[f"wheel_main_{side}"] = _wheel(
            x_ax, wheel_y, z_ax, G["main_wheel_r"], G["main_wheel_w"])
        out[f"wheel_hub_main_{side}"] = _hub(
            x_ax, wheel_y, z_ax, G["main_wheel_r"], G["main_wheel_w"])
        out[f"brake_{side}"] = _brake(x_ax, wheel_y, z_ax, sgn)
        knee = (x + r, y + sgn * r * 3, z_top - G["main_leg"] * 0.3)
        lower = (x, y, z_top - G["main_leg"] * 0.65)
        anchor = (x, sgn * (B["main_bay_y"] + B["main_bay_width"] / 2 - r), z_top)
        out[f"gear_main_side_stay_{side}"] = mesh.pipe([anchor, knee, lower], r * 0.45, 14)
        out[f"gear_main_downlock_{side}"] = _actuator(anchor, knee, r * 0.3)
        out[f"gear_main_drag_stay_{side}"] = mesh.pipe(
            [(x - B["main_bay_length"] / 2 + r, y, z_top),
             (x - r * 3, y, z_top - G["main_leg"] * 0.25), lower], r * 0.45, 14)
        out[f"gear_main_retract_actuator_{side}"] = _actuator(
            (x + r * 3, y + sgn * r * 4, z_top), lower, r * 0.38)
        out.update(_bay(f"main_{side}", x, sgn * B["main_bay_y"],
                        B["main_bay_length"], B["main_bay_width"],
                        B["main_bay_floor"], B["main_roof"], sgn))
    return out


def _actuator(p0, p1, radius):
    """Expose the smaller piston rather than disguising a hydraulic ram as a rod."""
    mid = tuple(p0[k] + (p1[k] - p0[k]) * 0.6 for k in range(3))
    return mesh.join(mesh.pipe([p0, mid], radius, 14),
                     mesh.pipe([mid, p1], radius * 0.55, 14))


def _hub(x, y, z, radius, width):
    v, f = mesh.revolve_closed(
        [(-width * 0.45, radius * 0.12), (width * 0.45, radius * 0.12),
         (width * 0.45, radius * 0.50), (width * 0.30, radius * 0.53),
         (-width * 0.30, radius * 0.53), (-width * 0.45, radius * 0.50)], SEG)
    return [(x + pz, y + px, z + py) for px, py, pz in v], f


def _bay(name, x, y, length, width, floor, roof, sgn=None):
    """Five lined walls leave the underside open to the deployed wheel.

    Doors hang from the long edges, with a separate strut door on each main
    unit. The interior ribs stop at the mouth rather than closing the bay
    with the solid block that previously stood in for a retract mechanism.
    """
    t = B["wall"]
    walls = [shapes.rounded_box(x, y, roof, length, width, t, r=t / 4, seg=3)]
    for sign in (-1.0, 1.0):
        walls.append(shapes.rounded_box(
            x + sign * (length - t) / 2, y, (floor + roof) / 2,
            t, width, roof - floor, r=t / 4, seg=3))
        walls.append(shapes.rounded_box(
            x, y + sign * (width - t) / 2, (floor + roof) / 2,
            length, t, roof - floor, r=t / 4, seg=3))
    out = {f"gear_bay_{name}": mesh.join(*walls)}
    ribs = []
    for i in range(1, 5):
        xx = x - length / 2 + length * i / 5
        ribs.append(mesh.pipe(
            [(xx, y - width / 2 + t, floor),
             (xx, y - width / 2 + t, roof - t),
             (xx, y + width / 2 - t, roof - t),
             (xx, y + width / 2 - t, floor)], t / 2, 8))
    out[f"gear_bay_lining_{name}"] = mesh.join(*ribs)
    lines = []
    for offset in (-t, t):
        lines.append(mesh.pipe(
            [(x - length / 2 + t * 2, y + offset, floor + t),
             (x - length / 2 + t * 2, y + offset, roof - t * 2),
             (x + length * 0.3, y + offset, roof - t * 2),
             (x + length * 0.3, y + width * 0.25 + offset, roof - t * 2),
             (x, y + width * 0.25 + offset, floor + t)], t * 0.18, 10))
    out[f"gear_bay_hydraulic_lines_{name}"] = mesh.join(*lines)
    for sign in ((-1.0, 1.0) if sgn is None else (sgn,)):
        yy = y + sign * width / 2
        suffix = "l" if sign < 0 else "r"
        depth = width / 2
        out[f"gear_door_{name}_{suffix}"] = shapes.rounded_box(
            x, yy, floor - depth / 2, length, B["door"], depth, r=t / 3, seg=3)
        out[f"gear_door_actuator_{name}_{suffix}"] = _actuator(
            (x, yy - sign * depth / 2, roof - t),
            (x, yy, floor - depth / 2), t * 0.65)
        out[f"gear_door_hinge_{name}_{suffix}"] = mesh.pipe(
            [(x - length / 2, yy, floor), (x + length / 2, yy, floor)], t, 12)
        latches = []
        for end in (-1.0, 1.0):
            xx = x + end * length * 0.35
            latches.append(shapes.rounded_box(
                xx, yy - sign * t, floor - depth + t,
                t * 2, t, t * 2, r=t / 4, seg=3))
            latches.append(mesh.pipe(
                [(xx - t / 2, yy - sign * t, floor - depth + t),
                 (xx - t / 2, yy - sign * t * 2, floor - depth + t),
                 (xx + t / 2, yy - sign * t * 2, floor - depth + t),
                 (xx + t / 2, yy - sign * t, floor - depth + t)], t / 4, 10))
        out[f"gear_door_latches_{name}_{suffix}"] = mesh.join(*latches)
    if sgn is not None:
        out[f"gear_door_strut_{name}"] = shapes.rounded_box(
            G["main_x"], sgn * (G["main_y"] - G["strut_r"]),
            floor - G["main_leg"] / 4, G["strut_r"] * 3, B["door"],
            G["main_leg"] / 2, r=t / 3, seg=3)
    return out


def _brake(x, y, z, sgn):
    """The brake stack shares the wheel's transverse axle, not a vertical axis.

    Alternating disc edges and an inboard caliper distinguish the brake from
    the hub without pretending that cylinders added to a disc are drillings.
    """
    r = G["main_wheel_r"] * 0.48
    t = B["wall"]
    parts = []
    for i in range(4):
        yy = y - sgn * (G["main_wheel_w"] / 2 + i * t)
        v, f = mesh.revolve_closed(
            [(0.0, r * 0.30), (0.0, r), (t * 0.7, r), (t * 0.7, r * 0.30)], SEG)
        parts.append(([(x + pz, yy + px, z + py) for px, py, pz in v], f))
    parts.append(shapes.rounded_box(
        x - r * 0.8, y - sgn * (G["main_wheel_w"] / 2 + t), z,
        r * 0.6, t * 5, r * 1.1, r=t / 2, seg=4))
    parts.append(mesh.pipe(
        [(x - r * 0.8, y - sgn * G["main_wheel_w"] / 2, z + r / 2),
         (x - r, y - sgn * G["main_wheel_w"], z + r),
         (G["main_x"], sgn * G["main_y"], B["main_top"])], t / 3, 10))
    return mesh.join(*parts)
