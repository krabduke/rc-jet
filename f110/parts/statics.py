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
    out.update(_borescope_bosses())
    out.update(_mount_pads())
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
            common.shell_profile(x0, x1, r0, r1, wall,
                                 bore=spec.casing_bore(name, x0, x1, r0, r1),
                                 cap=spec.CASING_CAP.get(name)),
            segments=SEG)
    return out


def _casing_outer(x):
    for (name, x0, x1, r0, r1, wall) in spec.CASINGS:
        if x0 <= x <= x1:
            return spec.casing_inner(name, x) + wall
    return 300.0


def _flanges():
    out = {}
    for (name, x, r_in, r_out, thick, bolts) in spec.FLANGES:
        ring = mesh.tube(x - thick / 2, x + thick / 2, r_in, r_out, 96)
        bolt_r = (r_out - r_in) * 0.26
        pitch = (r_in + r_out) / 2 + (r_out - r_in) * 0.18
        fwd = mesh.bolt_ring(x - thick / 2, pitch, bolts, bolt_r, thick * 0.55)
        aft = mesh.bolt_ring(x + thick / 2, pitch, bolts, bolt_r, thick * 0.55)
        out[name] = mesh.join(ring, fwd, aft)
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

    # Splitter leading edge -- the knife that divides core from bypass flow.
    #
    # The nose loop used to run from 34 mm ahead of x0, out to x0 and back,
    # so its "tip" pointed aft and the loop crossed itself: a bow-tie, not an
    # edge. It is a knife now, sharp 30 mm ahead of the core cowl and
    # thickening to the cowl's 7 mm wall, which it laps by 10 mm.
    rs, t, ln = b["splitter_radius"], 7.0, 30.0
    n = 10
    upper = [(x0 - ln + ln * i / n, rs - t / 2 + (t / 2) * math.sin(0.5 * math.pi * i / n))
             for i in range(n + 1)]
    lower = [(x0 - ln + ln * i / n, rs - t / 2 - (t / 2) * math.sin(0.5 * math.pi * i / n))
             for i in range(n, 0, -1)]
    out["splitter"] = mesh.revolve_closed(
        upper + [(x0 + 10.0, rs), (x0 + 10.0, rs - t)] + lower, SEG)

    sv, sf = radial_strut(spec.STATION["fan_frame"], b["strut_chord"],
                          b["strut_thickness"],
                          b["inner_radius_fwd"] - 2.0, b["outer_radius_fwd"] + 2.0,
                          twist=6.0)
    out["bypass_struts"] = mesh.replicate(sv, sf, b["n_struts"])
    return out


def _centre_body():
    """The front frame's stationary nose: an ogive to the guide vanes' hub
    radius, a cylinder they stand on, and an aft bulkhead that the No.1
    bearing sump is bolted to. A 3 mm shell, closed at the back."""
    c = spec.CENTRE_BODY
    R, t, x_aft = c["hub_radius"], c["wall"], c["x_aft"]
    n = 30
    nose = []
    for i in range(n + 1):
        f = i / n
        nose.append((c["x_nose"] + (c["x_shoulder"] - c["x_nose"]) * f,
                     c["tip_radius"] + (R - c["tip_radius"]) * f ** c["ogive_power"]))
    # the bulkhead's inner edge overlaps the No.1 sump's aft face by 4 mm,
    # which is where the two are bolted together
    r_bulk = spec.BEARINGS[0][3]
    outer = nose + [(x_aft, R)]
    back = [(x_aft, r_bulk), (x_aft - t, r_bulk), (x_aft - t, R - t)]
    inner = [(x, max(r - t, 0.8)) for (x, r) in reversed(nose)]
    return mesh.revolve_ring(outer + back + inner, SEG)


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

    out["centre_body"] = _centre_body()

    # fan frame hub: carries No.1 and No.2 bearings
    xf = spec.STATION["fan_frame"]
    # A frame hub is a bearing housing, not a ring of pipe: it has a bore step
    # for each race, a bolted retainer flange at the front and a web out to the
    # strut roots. Both hubs in this engine were single mesh.tube calls.
    #
    # It was a solid ring from r 104 -- 8 mm off the LP shaft -- to r 170,
    # from x 610 to 790, which is where the No.2 and No.3 bearings and their
    # sumps are, and where the HP shaft starts. So the hub's metal filled the
    # sumps, the races, and 15 mm of a shaft turning at 14,000 rpm. A frame
    # hub carries its struts' load down onto the bearing housings and stops:
    # this is a ring under the strut roots, bolted round the No.2 sump and
    # tied to the No.3 sump's forward end, finished short of the HP shaft.
    r_s2 = spec.BEARINGS[1][3] + 16.0          # No.2 sump's outer face
    # it ends 4 mm ahead of the HP shaft and of the No.3 bearing's outer race
    x_aft = min(spec.SHAFTS["hp_x0"], spec.BEARINGS[2][1] - 23.0) - 4.0
    out["fan_frame_hub"] = mesh.revolve_closed(
        [(xf - 84.0, r_s2 - 4.0), (x_aft, r_s2 - 4.0),
         (x_aft, r_s2 + 14.0), (x_aft - 14.0, r_s2 + 14.0),
         (x_aft - 14.0, r_s2 + 10.0), (xf - 70.0, r_s2 + 10.0),
         (xf - 70.0, r_s2 + 16.0), (xf - 84.0, r_s2 + 16.0)], segments=72)
    # A 90 mm chord at x 664, not 150 at 700.
    #
    # `radial_strut` lays its chord from -0.4 to +0.6 of x, so at 150 the
    # struts ran 640 to 790 -- and the HP compressor drum's forward face is
    # at 728. Every one of the eight passed through it: at that station the
    # drum is a shell between r 307 and 333 and the struts sweep r 168 to
    # 480, so a static frame member ran through the wall of the rotor and
    # out the other side. The room is tighter than the stations suggest:
    # the third fan rotor's blades reach x 619 and the drum's face is at
    # 728, so the whole window is 109 mm. At -0.4/+0.6 of chord that allows
    # 97 mm of chord, and a frame strut is thick because it is structural
    # and carries the service lines through the bypass.
    # ...out to the bypass duct's inner wall, not 6 mm short of it. A frame
    # strut ends ON the casing it carries the load into; stopping short left
    # a 1.8 mm gap, and the core gas path had no continuous structure from
    # the splitter to the compressor case.
    fv, ff = radial_strut(664.0, 90.0, 20.0, 168.0,
                          spec.BYPASS["inner_radius_fwd"] + 2.0)
    out["fan_frame_struts"] = mesh.replicate(fv, ff, 8)

    # turbine rear frame: carries No.5 bearing, takes the aft mount load
    xt = spec.STATION["turbine_frame"]
    #
    # Like the fan frame's, this was a solid ring from r 104 that filled the
    # No.5 bearing and its sump. It is a band from the sump's outer face to
    # the strut roots, as long as the struts' chord, with a flange each end.
    r_s5 = spec.BEARINGS[4][3] + 16.0          # No.5 sump's outer face
    x0, x1 = xt - 80.0, xt + 110.0
    out["turbine_frame_hub"] = mesh.revolve_closed(
        [(x0, r_s5 - 2.0), (x1, r_s5 - 2.0), (x1, 196.0), (x1 - 12.0, 196.0),
         (x1 - 12.0, 190.0), (x0 + 12.0, 190.0), (x0 + 12.0, 196.0),
         (x0, 196.0)], segments=72)
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
        case_r = max((spec.casing_inner(name, row.x + row.chord * 0.4) + wall
                      for name, x0, x1, r0, r1, wall in spec.CASINGS
                      if x0 <= row.x + row.chord * 0.4 <= x1), default=r_tip)
        ring_r = max(r_tip, case_r) + 34.0
        parts.append(mesh.ring_torus(row.x + row.chord * 0.4, ring_r, 11.0, SEG, 12))

        # one spindle + lever per vane
        lever = []
        sp_v, sp_f = mesh.cylinder(0.0, 40.0, 7.0, 12)
        sp_v = mesh.rot_z(sp_v, math.pi / 2)
        sp_v = mesh.translate(sp_v, row.x + row.chord * 0.4, r_tip + 2.0, 0.0)
        lever.append((sp_v, sp_f))
        lv, lf = mesh.box(row.x + row.chord * 0.4, r_tip + 30.0, 14.0,
                          14.0, 8.0, 34.0)
        lever.append((lv, lf))
        one_v, one_f = mesh.join(*lever)
        parts.append(mesh.replicate(one_v, one_f, row.count))

    out["variable_vane_actuation"] = mesh.join(*parts)
    return out


def _borescope_bosses():
    pieces = []
    for (bx, ang) in spec.BORESCOPE:
        cr = _casing_outer(bx)
        bv, bf = mesh.revolve_closed(
            [(-5.0, 6.0), (8.0, 6.0), (8.0, 18.0), (14.0, 18.0),
             (14.0, 24.0), (-5.0, 24.0)], segments=16)
        bv = mesh.rot_x(mesh.rot_z(bv, math.pi / 2), math.radians(ang))
        a = math.radians(ang)
        pt = (bx, cr * math.cos(a), cr * math.sin(a))
        pieces.append((mesh.translate(bv, pt[0], pt[1], pt[2]), bf))
    return {"borescope_bosses": mesh.join(*pieces)}


def _mount_pads():
    pads = []
    for x in (spec.ACCESSORIES["mount_fwd_x"], spec.ACCESSORIES["mount_aft_x"]):
        pad_r = spec.ACCESSORIES["mount_pad_r"] if x < 1000 else 480.0
        for ang in (55.0, 125.0):
            a = math.radians(ang)
            cr = _casing_outer(x)
            pv, pf = mesh.revolve_closed(
                # 96 mm proud of the casing, not 22. The trunnion it carries
                # is a yoke whose inner jaw does not begin until r 674, and
                # the pad stopped at 604 -- so the engine's forward mount was
                # a pad and a trunnion that never touched each other.
                [(-2.0, 6.0), (14.0, 6.0), (14.0, 30.0), (96.0, 30.0),
                 (96.0, 38.0), (-2.0, 38.0)], segments=20)
            pv = mesh.rot_x(mesh.rot_z(pv, math.pi / 2), a)
            pt = (x, cr * math.cos(a), cr * math.sin(a))
            pads.append((mesh.translate(pv, pt[0], pt[1], pt[2]), pf))
    return {"mount_pads": mesh.join(*pads)}
