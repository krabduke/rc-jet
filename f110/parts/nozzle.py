"""Variable convergent-divergent nozzle, modelled in the maximum augmented
position: 12 convergent flaps, 12 divergent flaps, interleaved seals, external
fairing flaps, the unison actuator ring and its six actuators and links."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
N = spec.NOZZLE


def build():
    out = {}
    out.update(_flaps())
    out.update(_seals())
    out.update(_external())
    out.update(_actuation())
    return out


def sector_plate(x0, r0, x1, r1, dphi, thickness, n_seg=8, n_len=4):
    """A tapered plate occupying one angular sector -- the shape every nozzle
    flap and seal in this assembly is built from."""
    verts, faces = [], []
    prof = []
    for i in range(n_len + 1):
        f = i / n_len
        prof.append((x0 + (x1 - x0) * f, r0 + (r1 - r0) * f))
    loop = prof + [(x, r - thickness) for (x, r) in reversed(prof)]
    n = len(loop)
    for k in range(n_seg + 1):
        a = -dphi / 2 + dphi * k / n_seg
        ca, sa = math.cos(a), math.sin(a)
        for (x, r) in loop:
            verts.append((x, r * ca, r * sa))
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
                        N["x_throat"], N["r_throat"],
                        dphi, N["flap_thickness"])
    div = sector_plate(N["x_throat"], N["r_throat"],
                       N["x_exit"], N["r_exit"],
                       dphi, N["flap_thickness"])
    # hinge knuckles at each end of the convergent flap
    hinge = mesh.cylinder(-28.0, 28.0, 17.0, 14)
    hv = mesh.rot_z(hinge[0], math.pi / 2)
    hv = mesh.rot_x(hv, math.pi / 2)
    hv = mesh.translate(hv, N["x_throat"], N["r_throat"] - 6.0, 0.0)

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
                        N["x_throat"], N["r_throat"] + 3.0,
                        dphi, N["seal_thickness"], n_seg=4)
    div = sector_plate(N["x_throat"], N["r_throat"] + 3.0,
                       N["x_exit"], N["r_exit"] + 3.0,
                       dphi, N["seal_thickness"], n_seg=4)
    v, f = mesh.join(conv, div)
    return {"nozzle_seals": mesh.replicate(v, f, N["n_seals"], phase)}


def _external():
    """External fairing flaps -- the visible outer surface of the nozzle, which
    also sets its boat-tail drag."""
    dphi = 2 * math.pi / N["n_ext_flaps"] - math.radians(2.4)
    v, f = sector_plate(N["actuator_ring_x"] + 40.0, N["ext_flap_r_start"],
                        N["x_exit"] - 10.0, N["ext_flap_r_end"],
                        dphi, 9.0, n_seg=8, n_len=5)
    return {"nozzle_ext_flaps": mesh.replicate(v, f, N["n_ext_flaps"],
                                               math.pi / N["n_ext_flaps"])}


def _actuation():
    """Unison ring driven by six actuators; links carry the ring's axial motion
    into flap rotation, which is what varies throat area."""
    out = {}
    ring = mesh.ring_torus(N["actuator_ring_x"], N["actuator_ring_r"], 20.0, SEG, 14)
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
        v = mesh.translate(body_v, 0, N["actuator_ring_r"] + 26.0, 0)
        v2 = mesh.translate(rod_v, 0, N["actuator_ring_r"] + 26.0, 0)
        mnt_v, mnt_f = mesh.box(x0 - 16.0, N["actuator_ring_r"] + 12.0, 0.0,
                                34.0, 56.0, 40.0)
        jv, jf = mesh.join((v, body_f), (v2, rod_f), (mnt_v, mnt_f))
        acts.append((mesh.rot_x(jv, a), jf))
    out["nozzle_actuators"] = mesh.join(*acts)

    links = []
    for k in range(N["n_flaps"]):
        a = 2 * math.pi * k / N["n_flaps"]
        path = [(N["actuator_ring_x"] + 4.0, N["actuator_ring_r"] - 14.0, 0.0),
                (N["x_conv_start"] + 96.0, N["r_conv_start"] - 26.0, 0.0)]
        lv, lf = mesh.pipe(path, N["link_r"], 10)
        links.append((mesh.rot_x(lv, a), lf))
    out["nozzle_links"] = mesh.join(*links)
    return out
