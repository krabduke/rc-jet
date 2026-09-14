"""Augmentor (afterburner): lobed forced mixer, perforated screech liner,
16 spraybars, and the V-gutter flameholder."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh

SEG = spec.RES["revolve_segments"]
A = spec.AUGMENTOR


def build():
    out = {}
    out.update(_mixer())
    out.update(_liner())
    out.update(_spraybars())
    out.update(_flameholder())
    return out


def _mixer():
    """Twelve-lobe forced mixer. The lobes interdigitate core and bypass flow so
    the two streams are mixed before the flameholder sees them -- the lobe depth
    grows from zero at the inlet to its full value at the trailing edge."""
    nx, nt = 18, SEG
    t = 4.0
    r_mean = (A["mixer_r_outer"] + A["mixer_r_inner"]) / 2
    x0, x1 = A["mixer_x0"], A["mixer_x1"]
    lobes = A["mixer_lobes"]

    def surf(offset):
        verts = []
        for i in range(nx):
            f = i / (nx - 1)
            x = x0 + (x1 - x0) * f
            amp = A["mixer_lobe_depth"] * (f ** 1.6)
            for k in range(nt):
                th = 2 * math.pi * k / nt
                r = r_mean + amp * math.cos(lobes * th) + offset
                verts.append((x, r * math.cos(th), r * math.sin(th)))
        return verts

    v_out, v_in = surf(t / 2), surf(-t / 2)
    verts = v_out + v_in
    n_in_off = len(v_out)
    faces = []
    for i in range(nx - 1):
        for k in range(nt):
            k2 = (k + 1) % nt
            a0 = i * nt + k
            a1 = i * nt + k2
            b0 = (i + 1) * nt + k
            b1 = (i + 1) * nt + k2
            faces.append((a0, a1, b1, b0))
            faces.append((a0 + n_in_off, b0 + n_in_off,
                          b1 + n_in_off, a1 + n_in_off))
    # close the leading and trailing edges between the two skins
    for k in range(nt):
        k2 = (k + 1) % nt
        faces.append((k, k + n_in_off, k2 + n_in_off, k2))
        b = (nx - 1) * nt
        faces.append((b + k, b + k2, b + k2 + n_in_off, b + k + n_in_off))
    return {"mixer": (verts, faces)}


def _liner():
    """Augmentor screech liner: a perforated shell standing off the duct wall.
    The holes damp the high-frequency combustion instability that gives the
    part its name; the annulus behind them carries cooling air aft."""
    x0, x1 = A["spraybar_x"] - 60.0, spec.STATION["augmentor_exit"]
    r = A["liner_r"]
    t = A["liner_thickness"]
    # A screech liner is not a plain tube. It is a corrugated sleeve hung off
    # the casing in overlapping segments, each stepping out over the one aft
    # of it so cooling air is fed in behind every joint. The perforations are
    # cut in below; this is the shell they are cut from, and it was two
    # x-stations of constant radius.
    L = x1 - x0
    n_seg = 6
    prof = [(x0, r - t), (x1, r - t)]
    outer = []
    for k in range(n_seg):
        f0 = k / n_seg
        f1 = (k + 1) / n_seg
        step = t * 0.55 * (n_seg - k) / n_seg
        outer += [(f0, step), (f0 + 0.012, step + t * 0.5),
                  (f1 - 0.012, step + t * 0.5), (f1, step)]
    for (f, extra) in reversed(outer):
        prof.append((x0 + L * f, r + extra))
    liner = mesh.revolve_closed(prof, SEG)

    n_ax, n_rad = 18, 48
    cutters = []
    for i in range(n_ax):
        xi = x0 + (x1 - x0) * (i + 0.5) / n_ax
        phase = (math.pi / n_rad) if i % 2 else 0.0
        cv, cf = mesh.cylinder(-14.0, 14.0, A["screech_hole_r"], 8)
        cv = mesh.rot_z(cv, math.pi / 2)
        cv = mesh.translate(cv, xi, r - t / 2, 0.0)
        cutters.append(mesh.replicate(cv, cf, n_rad, phase))

    return {"augmentor_liner": liner,
            "cut:augmentor_liner": mesh.join(*cutters)}


def _spraybars():
    """Sixteen radial fuel spraybars in the augmentor, each fed from a boss on
    the casing and drilled with a row of spray orifices facing downstream."""
    bars = []
    x = A["spraybar_x"]
    for k in range(A["n_spraybars"]):
        a = 2 * math.pi * k / A["n_spraybars"]
        path = [(x, A["spraybar_r_root"] + 60.0, 0.0),
                (x, A["spraybar_r_tip"], 0.0)]
        bar = mesh.pipe(path, A["spraybar_r"], 14)
        boss_v, boss_f = mesh.cylinder(-20.0, 20.0, 24.0, 16)
        boss_v = mesh.rot_z(boss_v, math.pi / 2)
        boss_v = mesh.translate(boss_v, x, A["spraybar_r_root"] + 52.0, 0.0)
        # spray orifice bosses down the trailing side
        tips = []
        for j in range(9):
            rr = A["spraybar_r_tip"] + (A["spraybar_r_root"] - A["spraybar_r_tip"]) * j / 8
            tv, tf = mesh.cylinder(0.0, 7.0, 2.6, 8)
            tv = mesh.translate(tv, x + A["spraybar_r"] * 0.6, rr, 0.0)
            tips.append((tv, tf))
        v, f = mesh.join(bar, (boss_v, boss_f), *tips)
        bars.append((mesh.rot_x(v, a), f))
    return {"spraybars": mesh.join(*bars)}


def _flameholder():
    """Three concentric V-gutter rings tied together by eight radial gutters.
    The V section sheds a recirculating wake that holds the flame against a
    through-flow far faster than any flame speed."""
    w = A["flameholder_v_width"]
    x = A["flameholder_x"]
    rings = []
    for r in A["flameholder_rings"]:
        prof = [(x - w * 0.5, r), (x + w * 0.6, r + w * 0.5),
                (x + w * 0.6, r + w * 0.5 - 5.0), (x - w * 0.5 + 6.0, r),
                (x + w * 0.6, r - w * 0.5 + 5.0), (x + w * 0.6, r - w * 0.5)]
        rings.append(mesh.revolve_closed(prof, SEG))

    gutters = []
    r_lo = A["flameholder_rings"][0] - w * 0.5
    r_hi = A["flameholder_rings"][-1] + w * 0.5
    for k in range(A["n_radial_gutters"]):
        a = 2 * math.pi * k / A["n_radial_gutters"]
        gv, gf = mesh.box(x + w * 0.05, (r_lo + r_hi) / 2, 0.0,
                          w * 0.9, r_hi - r_lo, A["gutter_width"])
        gutters.append((mesh.rot_x(gv, a), gf))

    # struts carrying the flameholder off the liner
    struts = []
    for k in range(6):
        a = 2 * math.pi * k / 6 + math.pi / 6
        sv, sf = mesh.box(x, (r_hi + A["liner_r"]) / 2, 0.0,
                          w * 0.7, A["liner_r"] - r_hi, 16.0)
        struts.append((mesh.rot_x(sv, a), sf))

    return {"flameholder": mesh.join(*rings, *gutters, *struts)}
