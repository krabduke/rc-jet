"""Static structure: casings, flanges, vane rows, bypass duct, frames,
and the variable-geometry actuation that drives the variable vane rows."""

import math
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import spec
import mesh
import airfoil
from parts import common

SEG = spec.RES["revolve_segments"]


def build():
    out = {}
    out.update(_vane_rows())
    out.update(_casings())
    out.update(_flanges())
    out.update(_bypass_duct())
    out.update(_frames())
    out.update(_variable_geometry())
    return out


# --------------------------------------------------------------------------

def _vane_rows():
    out = {}
    for row in spec.FAN_ROWS + spec.HPC_ROWS:
        if not row.rotor:
            out[f"vanes_{row.name}"] = common.build_row(row)
    return out


def _casings():
    out = {}
    for (name, x0, x1, r0, r1, wall) in spec.CASINGS:
        out[name] = mesh.revolve_closed(
            common.shell_profile(x0, x1, r0, r1, wall), segments=SEG)
    return out


def _flanges():
    out = {}
    for (name, x, r_in, r_out, thick, bolts) in spec.FLANGES:
        out[name] = mesh.flange(x, r_in, r_out, thick, bolts,
                                bolt_r=(r_out - r_in) * 0.26)
    return out


def radial_strut(x, chord, thickness, r0, r1, twist=0.0, n_span=5, n_chord=20):
    """One radial strut or frame arm: a symmetric section swept from r0 to r1."""
    sect = airfoil.section_points(n_chord, thickness / chord, 0.0)
    n = len(sect)
    verts = []
    for j in range(n_span):
        f = j / (n_span - 1)
        r = r0 + (r1 - r0) * f
        g = math.radians(twist * f)
        cg, sg = math.cos(g), math.sin(g)
        for (u, v) in sect:
            du = (u - 0.4) * chord
            dv = v * chord
            dx = du * cg - dv * sg
            dt = du * sg + dv * cg
            phi = dt / r
            verts.append((x + dx, r * math.cos(phi), r * math.sin(phi)))
    faces = []
    for j in range(n_span - 1):
        a, b = j * n, (j + 1) * n
        for i in range(n):
            i2 = (i + 1) % n
            faces.append((a + i, a + i2, b + i2, b + i))
    faces.append(tuple(range(n - 1, -1, -1)))
    base = (n_span - 1) * n
    faces.append(tuple(range(base, base + n)))
    return verts, faces


def _bypass_duct():
    """Annular bypass duct: inner wall (the core's outer skin), outer wall,
    and the load-carrying struts that cross the duct at the fan frame."""
    b = spec.BYPASS
    x0, x1 = spec.STATION["splitter"], spec.STATION["mixer_front"]
    out = {}

    # ribs off: the outside of this wall is the bypass duct, so a hoop
    # stiffener here would sit in the airflow rather than behind it
    out["bypass_inner_wall"] = mesh.revolve_closed(
        common.shell_profile(x0, x1, b["inner_radius_fwd"] - 7.0,
                             b["inner_radius_aft"] - 7.0, 7.0, ribs=False),
        segments=SEG)

    # splitter leading edge -- the knife that divides core from bypass flow
    nose = []
    for i in range(13):
        a = math.pi * (0.5 + i / 12.0)
        nose.append((x0 - 34.0 + 34.0 * math.cos(a) * -1.0,
                     b["splitter_radius"] + 7.0 * math.sin(a)))
    out["splitter"] = mesh.revolve_closed(
        nose + [(x0 + 60.0, b["splitter_radius"] - 7.0),
                (x0 + 60.0, b["splitter_radius"] + 7.0)], SEG)

    sv, sf = radial_strut(spec.STATION["fan_frame"], b["strut_chord"],
                          b["strut_thickness"],
                          b["inner_radius_fwd"] - 2.0, b["outer_radius_fwd"] + 2.0,
                          twist=6.0)
    out["bypass_struts"] = mesh.replicate(sv, sf, b["n_struts"])
    return out


def _frames():
    """Major structural frames -- the load paths from the bearings out to the
    mounts. Inlet case, fan frame hub, and the turbine rear frame."""
    out = {}

    out["inlet_case"] = mesh.revolve_closed(
        # 574 not 586: with a 10 mm wall and the bolting lands on top this
        # shell reached 604.5, which is inside casing_inlet's 596 bore. The
        # two concentric shells were sharing metal over their whole length.
        common.shell_profile(spec.STATION["inlet_lip"], spec.STATION["fan_face"],
                             574.0, 574.0, 10.0, ribs=False), segments=SEG)

    # inlet lip -- rolled-over leading edge
    lip = []
    for i in range(15):
        a = math.pi * (0.5 + i / 14.0)
        lip.append((spec.STATION["inlet_lip"] + 26.0 - 26.0 * math.sin(a),
                    591.0 + 22.0 * math.cos(a)))
    out["inlet_lip"] = mesh.revolve_closed(
        lip + [(spec.STATION["inlet_lip"] + 30.0, 613.0),
               (spec.STATION["inlet_lip"] + 30.0, 569.0)], SEG)

    # fan frame hub: carries No.1 and No.2 bearings
    xf = spec.STATION["fan_frame"]
    # A frame hub is a bearing housing, not a ring of pipe: it has a bore step
    # for each race, a bolted retainer flange at the front and a web out to the
    # strut roots. Both hubs in this engine were single mesh.tube calls.
    out["fan_frame_hub"] = mesh.revolve_closed(
        [(xf - 90.0, 104.0), (xf - 30.0, 104.0), (xf - 30.0, 112.0),
         (xf + 40.0, 112.0), (xf + 40.0, 104.0), (xf + 90.0, 104.0),
         (xf + 90.0, 152.0), (xf + 74.0, 152.0), (xf + 74.0, 138.0),
         (xf + 30.0, 138.0), (xf + 30.0, 170.0), (xf - 40.0, 170.0),
         (xf - 40.0, 140.0), (xf - 74.0, 140.0), (xf - 74.0, 166.0),
         (xf - 90.0, 166.0)], segments=72)
    fv, ff = radial_strut(xf, 150.0, 24.0, 168.0, spec.BYPASS["inner_radius_fwd"] - 6.0)
    out["fan_frame_struts"] = mesh.replicate(fv, ff, 8)

    # turbine rear frame: carries No.5 bearing, takes the aft mount load
    xt = spec.STATION["turbine_frame"]
    out["turbine_frame_hub"] = mesh.revolve_closed(
        [(xt - 70.0, 104.0), (xt - 20.0, 104.0), (xt - 20.0, 114.0),
         (xt + 34.0, 114.0), (xt + 34.0, 104.0), (xt + 70.0, 104.0),
         (xt + 70.0, 168.0), (xt + 54.0, 168.0), (xt + 54.0, 150.0),
         (xt + 24.0, 150.0), (xt + 24.0, 190.0), (xt - 30.0, 190.0),
         (xt - 30.0, 152.0), (xt - 56.0, 152.0), (xt - 56.0, 180.0),
         (xt - 70.0, 180.0)], segments=72)
    tv, tf = radial_strut(xt, 170.0, 32.0, 188.0, 466.0)
    out["turbine_frame_struts"] = mesh.replicate(tv, tf, 8)

    # engine mounts
    a = spec.ACCESSORIES
    mounts = []
    for x in (a["mount_fwd_x"], a["mount_aft_x"]):
        r = a["mount_pad_r"] if x < 1000 else 480.0
        for ang in (55.0, 125.0):
            tr_v, tr_f = mesh.cylinder(-a["mount_trunnion_r"] * 1.6,
                                       a["mount_trunnion_r"] * 1.6,
                                       a["mount_trunnion_r"], 20)
            tr_v = mesh.rot_z(tr_v, math.pi / 2)
            tr_v = mesh.rot_x(tr_v, math.radians(ang))
            tr_v = mesh.translate(tr_v, x, 0, 0)
            tr_v = [(px, py + r * math.cos(math.radians(ang)),
                     pz + r * math.sin(math.radians(ang))) for (px, py, pz) in tr_v]
            mounts.append((tr_v, tr_f))
    out["mount_trunnions"] = mesh.join(*mounts)
    return out


def _variable_geometry():
    """Unison rings and levers for the variable vane rows. The IGV plus the
    first four variable stator rows are all driven, which is what lets this
    engine hold surge margin across the F-16's flight envelope."""
    out = {}
    variable_rows = [r for r in spec.FAN_ROWS + spec.HPC_ROWS if r.variable]
    parts = []
    for row in variable_rows:
        r_tip = max(row.r_tip_le, row.r_tip_te)
        ring_r = r_tip + 34.0
        parts.append(mesh.ring_torus(row.x + row.chord * 0.4, ring_r, 11.0, SEG, 12))

        # one spindle + lever per vane
        lever = []
        sp_v, sp_f = mesh.cylinder(0.0, 40.0, 7.0, 12)
        sp_v = mesh.rot_z(sp_v, -math.pi / 2)
        sp_v = mesh.translate(sp_v, row.x + row.chord * 0.4, r_tip + 2.0, 0.0)
        lever.append((sp_v, sp_f))
        lv, lf = mesh.box(row.x + row.chord * 0.4, r_tip + 30.0, 14.0,
                          14.0, 8.0, 34.0)
        lever.append((lv, lf))
        one_v, one_f = mesh.join(*lever)
        parts.append(mesh.replicate(one_v, one_f, row.count))

    out["variable_vane_actuation"] = mesh.join(*parts)
    return out
