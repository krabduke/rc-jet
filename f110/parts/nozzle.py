"""Variable convergent-divergent nozzle at an intermediate area setting."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
N = spec.NOZZLE
THROAT_R = N["r_throat"] * 0.80
EXIT_R = N["r_exit"] - 16.0
HINGE_X = N["x_throat"] + 55.0
HINGE_R = THROAT_R + (EXIT_R - THROAT_R) * (
    (HINGE_X - N["x_throat"]) / (N["x_exit"] - N["x_throat"])) + 32.0
RING_R = N["actuator_ring_r"] + 64.0
EXT_START_X = N["actuator_ring_x"] + 54.0
EXT_GAP = math.radians(8.0)
EXT_SPAN = 2 * math.pi / N["n_ext_flaps"] - EXT_GAP


def build():
    out = {}
    out.update(_flaps())
    out.update(_seals())
    out.update(_external())
    out.update(_actuation())
    return out


def sector_plate(x0, r0, x1, r1, dphi, thickness, n_seg=8, n_len=8,
                 taper=1.0, crown=0.0):
    verts, faces = [], []
    prof = []
    for i in range(n_len + 1):
        f = i / n_len
        prof.append((f, r0 + (r1 - r0) * f))
    loop = [(f, r - thickness) for (f, r) in prof] + list(reversed(prof))
    n = len(loop)
    for k in range(n_seg + 1):
        u = 2 * k / n_seg - 1
        for (f, r) in loop:
            a = u * dphi * (1 + (taper - 1) * f) / 2
            rr = r + crown * math.sin(math.pi * f) * (1 - u * u)
            verts.append((x0 + (x1 - x0) * f,
                          rr * math.cos(a), rr * math.sin(a)))
    for k in range(n_seg):
        b0, b1 = k * n, (k + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((b0 + i, b0 + i2, b1 + i2, b1 + i))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = n_seg * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def _flaps():
    dphi = 2 * math.pi / N["n_flaps"] - math.radians(N["gap_deg"])
    conv = sector_plate(N["x_conv_start"], N["r_conv_start"],
                        N["x_throat"], THROAT_R,
                        dphi, N["flap_thickness"], crown=5.0)
    div = sector_plate(N["x_throat"], THROAT_R,
                       N["x_exit"], EXIT_R,
                       dphi, N["flap_thickness"], taper=0.96, crown=7.0)
    # hinge knuckles at each end of the convergent flap
    hinge = mesh.cylinder(-28.0, 28.0, 17.0, 14)
    hv = mesh.rot_z(hinge[0], math.pi / 2)
    hv = mesh.rot_x(hv, math.pi / 2)
    hv = mesh.translate(hv, N["x_throat"], THROAT_R - 6.0, 0.0)

    cv, cf = mesh.join(conv, (hv, hinge[1]))
    return {
        "nozzle_flaps_convergent": mesh.replicate(cv, cf, N["n_flaps"]),
        "nozzle_flaps_divergent": mesh.replicate(div[0], div[1], N["n_flaps"]),
    }


def _seals():
    """Interleaved seals sit behind the gap between adjacent flaps and slide as
    the nozzle opens -- without them the gaps would simply leak thrust."""
    dphi = math.radians(N["gap_deg"]) * 3.4
    phase = math.pi / N["n_flaps"]
    conv = sector_plate(N["x_conv_start"], N["r_conv_start"] + 3.0,
                        N["x_throat"], THROAT_R + 3.0,
                        dphi, N["seal_thickness"], n_seg=4)
    div = sector_plate(N["x_throat"], THROAT_R + 3.0,
                       N["x_exit"], EXIT_R + 3.0,
                       dphi, N["seal_thickness"], n_seg=4)
    v, f = mesh.join(conv, div)
    internal = mesh.replicate(v, f, N["n_seals"], phase)
    forward = sector_plate(EXT_START_X, N["ext_flap_r_start"] + 16.0,
                           HINGE_X - 16.0, HINGE_R + 16.0,
                           EXT_GAP + math.radians(4.0), 5.0,
                           taper=1.35, crown=20.0)
    # on the aft external flaps, which run from HINGE_R - 14 to EXIT_R + 12
    # on their outer face: a seal rides the flaps either side of the gap it
    # closes. At HINGE_R + 2 to EXIT_R + 28 it stood 11 mm clear of both,
    # all the way along, and closed nothing.
    aft = sector_plate(HINGE_X + 14.0, HINGE_R - 14.0 + 5.0,
                       N["x_exit"] - 10.0, EXIT_R + 12.0 + 5.0,
                       math.radians(16.2), 5.0, taper=1.06, crown=14.0)
    ev, ef = mesh.join(forward, aft)
    external = mesh.replicate(ev, ef, N["n_ext_flaps"])
    return {"nozzle_seals": mesh.join(internal, external)}


def _external():
    """External fairing flaps -- the visible outer surface of the nozzle, which
    also sets its boat-tail drag."""
    forward = sector_plate(EXT_START_X, N["ext_flap_r_start"],
                           HINGE_X - 16.0, HINGE_R,
                           EXT_SPAN, 9.0, n_len=12, taper=0.80, crown=18.0)
    aft = sector_plate(HINGE_X + 14.0, HINGE_R - 14.0,
                       N["x_exit"] - 16.0, EXIT_R + 12.0,
                       EXT_SPAN * 0.80, 9.0, n_len=12, taper=0.94, crown=12.0)
    knuckles = []
    for z0, z1, radius in [(-36.0, -15.0, 14.0), (-13.0, 13.0, 18.0),
                            (15.0, 36.0, 14.0), (-43.0, 43.0, 7.0)]:
        hv, hf = mesh.cylinder(z0, z1, radius, 16)
        hv = mesh.rot_x(mesh.rot_z(hv, math.pi / 2), math.pi / 2)
        knuckles.append((mesh.translate(hv, HINGE_X, HINGE_R + 6.0, 0.0), hf))
    lugs = [mesh.box(HINGE_X - 22.0, HINGE_R + 3.0, z,
                     40.0, 24.0, 12.0) for z in (-26.0, 26.0)]
    lugs.append(mesh.box(HINGE_X + 22.0, HINGE_R - 2.0, 0.0,
                         42.0, 28.0, 18.0))
    v, f = mesh.join(forward, aft, *knuckles, *lugs)
    return {"nozzle_ext_flaps": mesh.replicate(v, f, N["n_ext_flaps"],
                                               math.pi / N["n_ext_flaps"])}


def _actuation():
    """Unison ring driven by six actuators; links carry the ring's axial motion
    into flap rotation, which is what varies throat area."""
    out = {}
    ring = mesh.ring_torus(N["actuator_ring_x"], RING_R, 20.0, SEG, 14)
    out["nozzle_actuator_ring"] = ring

    acts = []
    for k in range(N["n_actuators"]):
        a = 2 * math.pi * k / N["n_actuators"]
        x0 = N["actuator_ring_x"] - N["actuator_len"]
        body_v, body_f = mesh.cylinder(x0, x0 + N["actuator_len"] * 0.66,
                                       N["actuator_r"], 18)
        rod_v, rod_f = mesh.cylinder(x0 + N["actuator_len"] * 0.60,
                                     N["actuator_ring_x"],
                                     N["actuator_r"] * 0.42, 14)
        v = mesh.translate(body_v, 0, RING_R + 26.0, 0)
        v2 = mesh.translate(rod_v, 0, RING_R + 26.0, 0)
        mnt_v, mnt_f = mesh.box(x0 - 16.0, (RING_R + 26.0 + 442.0) / 2, 0.0,
                                34.0, RING_R + 26.0 - 442.0, 40.0)
        jv, jf = mesh.join((v, body_f), (v2, rod_f), (mnt_v, mnt_f))
        acts.append((mesh.rot_x(jv, a), jf))
    out["nozzle_actuators"] = mesh.join(*acts)

    links = []
    for k in range(N["n_ext_flaps"]):
        a = 2 * math.pi * k / N["n_ext_flaps"] + math.pi / N["n_ext_flaps"]
        x_end = N["x_conv_start"] + 210.0
        f = (x_end - EXT_START_X) / (HINGE_X - 16.0 - EXT_START_X)
        skin_r = N["ext_flap_r_start"] + (HINGE_R - N["ext_flap_r_start"]) * f
        skin_r += 18.0 * math.sin(math.pi * f)
        path = [(N["actuator_ring_x"] + 4.0, RING_R, 0.0),
                (x_end, skin_r + 54.0, 0.0)]
        link = mesh.pipe(path, N["link_r"], 14)
        lug = mesh.box(x_end, skin_r + 25.0, 0.0, 38.0, 64.0, 24.0)
        pv, pf = mesh.cylinder(-27.0, 27.0, 10.0, 14)
        pv = mesh.rot_x(mesh.rot_z(pv, math.pi / 2), math.pi / 2)
        pv = mesh.translate(pv, x_end, skin_r + 42.0, 0.0)
        lv, lf = mesh.join(link, lug, (pv, pf))
        links.append((mesh.rot_x(lv, a), lf))
    out["nozzle_links"] = mesh.join(*links)
    return out
