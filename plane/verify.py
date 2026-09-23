"""Check the built aircraft against spec.py by measurement and by design rule.

Dimensions come from build/parts.csv, so the envelope and fit checks measure
what actually got built. The balance and stability checks are computed from the
mass and geometry tables -- for an aircraft those are the numbers that decide
whether it flies, not just whether it fits.

Exits non-zero on any failure.
"""

import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import spec  # noqa: E402


class Check:
    def __init__(self):
        self.fails, self.n = [], 0

    def ok(self, label, detail=""):
        self.n += 1
        print(f"  ok  {label:44s} {detail}")

    def band(self, label, got, lo, hi, unit="", note=""):
        self.n += 1
        if lo <= got <= hi:
            print(f"  ok  {label:44s} {got:8.3f}{unit}  [{lo:g}..{hi:g}] {note}")
            return True
        self.fails.append(f"{label}: {got:.3f}{unit} outside [{lo:g}..{hi:g}]")
        return False

    def true(self, label, cond, detail=""):
        self.n += 1
        if cond:
            print(f"  ok  {label:44s} {detail}")
            return True
        self.fails.append(f"{label}: {detail}")
        return False


def _mac(root, tip, semi_span, sweep):
    lam = tip / root
    mac = (2.0 / 3.0) * root * (1 + lam + lam * lam) / (1 + lam)
    y = (semi_span / 3.0) * (1 + 2 * lam) / (1 + lam)
    return mac, y


def main():
    path = os.path.join(ROOT, "build", "parts.csv")
    if not os.path.exists(path):
        print("build/parts.csv missing -- run `make build` first")
        return 1
    rows = list(csv.DictReader(open(path)))
    by = {r["name"]: r for r in rows}
    f = lambda r, k: float(r[k])

    c = Check()

    print("\nENVELOPE")
    x_min = min(f(r, "x_min_mm") for r in rows)
    x_max = max(f(r, "x_max_mm") for r in rows)
    y_min = min(f(r, "y_min_mm") for r in rows)
    y_max = max(f(r, "y_max_mm") for r in rows)
    length, span = x_max - x_min, y_max - y_min
    c.band("overall length", length, 0, spec.ENVELOPE_LENGTH, " mm")
    c.band("overall span", span, 0, spec.ENVELOPE_SPAN, " mm")
    c.ok("plan fit", f"{length:.0f} x {span:.0f} mm inside "
                     f"{spec.ENVELOPE_LENGTH:.0f} x {spec.ENVELOPE_SPAN:.0f}")

    print("\nENGINE REUSE")
    eng = [r for r in rows if r["name"].startswith("engine_")]
    # exact, not a floor: this catches parts silently dropping out of the
    # vendored build as well as arriving. Bump it when the turbofan gains
    # parts upstream and you have re-vendored and re-checked the fit.
    c.true("engine objects present", len(eng) == 113, f"{len(eng)} objects")
    ex0 = min(f(r, "x_min_mm") for r in eng)
    ex1 = max(f(r, "x_max_mm") for r in eng)
    # The INSTALLED length, which is not the whole turbofan. `engine_mount`
    # leaves eleven parts out -- the entire variable nozzle, which the
    # airframe builds itself, plus the oil tank, the heat exchanger, the
    # engine control and the ignition exciters, which are airframe-mounted.
    # So the band runs from the inlet lip to the augmentor exit, not to the
    # tail, and it comes off the engine's own station table rather than a
    # number typed here: at 4630 it was asking for 140 mm of a turbofan that
    # installs 123.
    # loaded by path, under its own module name: `f110/spec.py` is called
    # `spec` and so is the aircraft's, and it imports nothing but dataclasses
    import importlib.util
    _s = importlib.util.spec_from_file_location(
        "f110_spec", os.path.join(ROOT, "f110", "spec.py"))
    espec = importlib.util.module_from_spec(_s)
    _s.loader.exec_module(espec)
    inst = (espec.STATION["augmentor_exit"]
            - espec.STATION["inlet_lip"]) * spec.ENGINE_SCALE
    c.band("engine installed length", ex1 - ex0, inst - 2.0, inst + 2.0, " mm",
           f"1:{1/spec.ENGINE_SCALE:.0f} scale, nozzle not installed")
    c.band("engine inlet on firewall", ex0, spec.ENGINE_X - 1, spec.ENGINE_X + 1, " mm")

    # The engine has to physically fit the fuselage at every station it
    # occupies -- and that is a per-vertex question, the same one
    # `audit_fit` answers for everything else in the aeroplane.
    #
    # It used to compare each part's whole BOUNDING BOX against the section
    # at each station, which for a long part is the width of its widest
    # point wherever that is. The electrical loom runs from the fan face to
    # the augmentor and is widest at the accessory gearbox, so every station
    # it crossed was told the loom was 16.9 mm out. The answer never moved
    # when the loom moved, because it was never measuring the loom here.
    #
    # `fus.outside_by` is the section the skin is actually lofted from,
    # including the intake chin under the forebody.
    sys.path.insert(0, os.path.join(HERE, "parts"))
    from parts import fuselage as fus
    from parts import engine_mount as em
    worst, worst_x = 1e9, 0
    x0f, x1f = spec.FUSELAGE[0][0], spec.FUSELAGE[-1][0]
    for name, (verts, _faces) in em.build().items():
        if name.startswith("cut:") or not name.startswith("engine_"):
            continue
        for (x, y, z) in verts:
            if not (x0f <= x <= x1f):
                continue
            clear = -fus.outside_by(x, y, z, spec.FUSELAGE_SKIN)
            if clear < worst:
                worst, worst_x = clear, x
    c.true("engine clears fuselage internals", worst > 0.5,
           f"{worst:.1f} mm min clearance at x={worst_x:.0f}")

    print("\nBALANCE")
    mac = spec.mean_aero_chord()
    le = spec.mac_leading_edge_x()
    cg = spec.cg_x()
    c.ok("mean aerodynamic chord", f"{mac:.1f} mm, LE at x={le:.1f}")
    c.band("CG position", spec.cg_frac_mac(),
           spec.TARGET_CG_FRAC - spec.CG_TOLERANCE,
           spec.TARGET_CG_FRAC + spec.CG_TOLERANCE, " MAC",
           f"x={cg:.1f} mm")
    # The 250-420 g band was set when this aircraft's own equipment list was
    # a third uncounted -- no ECU battery, no data link, no telemetry, no
    # fuel pump, no filter, no retract -- and its turbine was declared at
    # 62 g. Counting all of it, and giving the engine a mass a turbine could
    # actually have, puts it at 558 g. The band follows the aircraft, not
    # the other way round.
    c.band("all-up weight", spec.total_mass_g(), 380, 620, " g")
    c.band("wing loading", spec.wing_loading_g_dm2(), 55, 130, " g/dm2",
           "fast EDF-jet territory")

    print("\nGROUND HANDLING")
    g = spec.GEAR
    main_aft = (g["main_x"] - cg) / mac
    c.band("main gear aft of CG", main_aft, 0.06, 0.22, " MAC")
    c.true("nose gear ahead of CG", g["nose_x"] < cg,
           f"nose x={g['nose_x']:.0f} < CG {cg:.0f}")
    nose_load = (g["main_x"] - cg) / (g["main_x"] - g["nose_x"])
    c.band("nose gear static load", nose_load * 100, 6.0, 18.0, " %AUW")

    print("\nSTABILITY")
    sw = spec.wing_area_mm2()
    hm, hy = _mac(spec.HTAIL["root_chord"], spec.HTAIL["tip_chord"],
                  spec.HTAIL["semi_span"], spec.HTAIL["sweep_le"])
    sh = (spec.HTAIL["root_chord"] + spec.HTAIL["tip_chord"]) * spec.HTAIL["semi_span"]
    xh = spec.HTAIL["x_root_le"] + hy * math.tan(math.radians(spec.HTAIL["sweep_le"])) \
         + 0.25 * hm
    vh = (sh * (xh - cg)) / (sw * mac)
    c.band("horizontal tail volume", vh, 0.12, 0.45, "",
           f"arm {xh-cg:.0f} mm")

    vm, vz = _mac(spec.VTAIL["root_chord"], spec.VTAIL["tip_chord"],
                  spec.VTAIL["height"], spec.VTAIL["sweep_le"])
    sv = (spec.VTAIL["root_chord"] + spec.VTAIL["tip_chord"]) / 2 * spec.VTAIL["height"]
    xv = spec.VTAIL["x_root_le"] + vz * math.tan(math.radians(spec.VTAIL["sweep_le"])) \
         + 0.25 * vm
    vv = (sv * (xv - cg)) / (sw * spec.SPAN)
    c.band("vertical tail volume", vv, 0.030, 0.095, "",
           f"arm {xv-cg:.0f} mm")

    c.band("wing aspect ratio", spec.SPAN ** 2 / sw, 1.6, 3.4, "",
           "delta")
    c.band("taper ratio", spec.taper_ratio(), 0.20, 0.45, "")

    print("\nSTRUCTURE")
    # A straight spanwise spar leaves a swept delta's planform entirely; this
    # check is here because the first version of the wing did exactly that.
    from parts import common as pcommon
    W = spec.WING
    worst, worst_y = 1e9, 0.0
    for i in range(41):
        y = spec.SPAR["span"] / 2 * i / 40
        fr = y / W["semi_span"]
        chord = pcommon.local_chord(W["root_chord"], W["tip_chord"], fr)
        x_le = pcommon.le_x_at(W["x_root_le"], W["semi_span"], W["sweep_le"], fr)
        x = x_le + chord * spec.SPAR["x_frac"]
        m = min(x - x_le, x_le + chord - x)
        if m < worst:
            worst, worst_y = m, y
    c.true("spar stays inside the wing planform", worst > 4.0,
           f"{worst:.1f} mm margin at y={worst_y:.0f} mm")
    c.band("spar span vs wing span", spec.SPAR["span"], 0, spec.SPAN, " mm")

    # The rear spar is the flaperon hinge backing, so it has to sit between the
    # hinge line and the wing's trailing edge, not behind it.
    ST = spec.STRUCTURE
    hinge = 1.0 - spec.FLAPERON["chord_frac"]
    c.true("rear spar ahead of the flaperon hinge",
           ST["rear_spar_frac"] < hinge,
           f"{ST['rear_spar_frac']:.2f} c vs hinge at {hinge:.2f} c")

    ribs = [k for k in by if k.startswith("rib_")]
    c.true("wing ribs per side", len(ribs) == 2 * ST["n_wing_ribs"],
           f"{len(ribs)} ribs, {ST['n_wing_ribs']} per side")
    # Every rib must be inside the skin it supports, or it pokes through.
    worst_rib, who = 1e9, ""
    for r in rows:
        if not r["name"].startswith(("rib_", "former_", "longeron_",
                                     "stringers", "fin_rib_")):
            continue
        for key, lo, hi in (("y", -spec.SPAN / 2, spec.SPAN / 2),):
            m = min(float(r[f"{key}_max_mm"]) - lo, hi - float(r[f"{key}_min_mm"]))
            if m < worst_rib:
                worst_rib, who = m, r["name"]
    c.true("internal structure inside the span", worst_rib > 0.0,
           f"{worst_rib:.1f} mm margin, worst {who}")

    formers = [k for k in by if k.startswith("former_")]
    c.true("formers between the bulkheads",
           len(formers) == len(ST["former_x"]), f"{len(formers)} formers")
    bhd_x = [b[1] for b in spec.BULKHEADS]
    clash = [x for x in ST["former_x"]
             if any(abs(x - bx) < 6.0 for bx in bhd_x)]
    c.true("no former clashes a bulkhead", not clash, f"{len(clash)} clashes")

    print("\nSKIN DETAIL")
    SD = spec.SKIN_DETAIL
    seams = [k for k in by if k.startswith("seam_ring_")]
    c.true("one seam per production joint",
           len(seams) == len(SD["seam_x"]), f"{len(seams)} seam rings")
    # Seams are relief on the skin, so they must stand off it -- but by less
    # than the skin is thick, or they read as ledges rather than laps.
    c.band("seam standoff", SD["seam_h"], 0.15, spec.FUSELAGE_SKIN, " mm")
    c.true("seams inside the body",
           all(spec.FUSELAGE[0][0] < x < spec.FUSELAGE[-1][0]
               for x in SD["seam_x"]), f"{len(SD['seam_x'])} stations")
    # A bonded composite joint has no fasteners in it. This airframe carried
    # 720 rivet heads flanking its seams, which is a sheet-metal convention
    # that says the wrong thing about how the aeroplane is made.
    c.true("no rivets on bonded joints", "rivets" not in by,
           "seams are bonded; hatches are screwed")

    # Nothing inside the aircraft may poke out through its skin. Bounding
    # boxes cannot answer this -- a stringer's box is as wide as the widest
    # station it passes -- so this is a per-vertex test against the section at
    # each vertex's own station. It found 26 parts placed by eye.
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    try:
        import audit_fit
        through = audit_fit.check()
    except Exception as exc:
        through = [(0, f"audit failed: {exc}", None)]
    c.true("nothing pokes through the skin", not through,
           "all internals inside" if not through
           else f"{len(through)} parts, worst {through[0][1]}")

    print("\nCOMPLETENESS")
    # One name per subsystem, and the names the aeroplane actually has.
    #
    # This list was still the radio-controlled model's: a 3-cell lipo, a
    # receiver and its battery, a turbine ECU, control horns and clevises,
    # wing fences, static dischargers. None of those are on a 14.5 m
    # aircraft and none of them have been built for a long time, so twelve
    # of the nineteen failures in this file were a list describing a
    # different aeroplane -- which is also why the two real ones underneath,
    # the engine's clearance to the fuselage and seven parts through the
    # skin, went unread.
    want = ["fuselage_skin", "wing_l", "wing_r", "flaperon_l", "flaperon_r",
            "stabilator_l", "stabilator_r", "vtail_fin", "rudder",
            "intake_lip", "duct_inlet", "canopy_glass", "canopy_frame",
            "wheel_nose", "wheel_main_l", "wheel_main_r", "wheel_hub_nose",
            "gear_main_l", "gear_main_r", "brake_l", "brake_r",
            "gear_door_nose_l", "gear_door_main_l", "gear_main_side_stay_l",
            "gear_nose_steering_actuator",
            "spar_carbon", "spar_rear", "stringer_01", "longeron_1",
            "former_01", "rib_r_01", "fin_rib_1", "bhd_firewall",
            "hinge_flaperon_l", "hinge_rudder", "rudder_servo",
            "stab_servo_l", "panel_screws", "seam_lengthwise",
            "fcs_fcc_envelope", "fcs_loom_trunk_upper_l", "fcs_loom_tail_l",
            "pitot", "navlight_port", "navlight_tail", "antennas",
            "panel_battery", "panel_avionics", "panel_hydraulics",
            "vg_l1", "wing_strake_r", "nose_strakes",
            "fuel_tank", "fuel_pump", "naca_inlet_l", "mount_ring",
            "bypass_slots", "tailpipe_shroud",
            "bay_fire_bottle", "bay_cooling_inlet", "bay_doors",
            "engine_fan_disc_assembly", "engine_combustor_liner_outer",
            "engine_gearbox", "engine_hpc_drum", "nozzle_actuator_00"]
    missing = [w for w in want if w not in by]
    c.true("key parts present", not missing, f"{len(want)} checked")
    for m in missing:
        c.fails.append(f"missing part: {m}")
    c.true("every object has a material",
           all(r["material"] for r in rows), f"{len(rows)} objects")
    # A name in MATERIAL_MAP that is not in PALETTE silently falls back to
    # the default, so a part comes out the wrong material and nothing says
    # so. Caught exactly that on the turbofan: six parts were assigned a
    # "steel_polished" that does not exist.
    unknown = sorted({v for v in spec.MATERIAL_MAP.values()
                      if v not in spec.PALETTE})
    c.true("every material name is real", not unknown,
           f"{len(spec.PALETTE)} in palette"
           + (f", unknown: {', '.join(unknown)}" if unknown else ""))
    c.true("no empty meshes", all(int(r["verts"]) > 0 for r in rows), "all non-empty")

    print("\n" + "=" * 66)
    if c.fails:
        print(f"FAIL  {len(c.fails)} of {c.n} checks")
        for x in c.fails:
            print("   x " + x)
        return 1
    print(f"PASS  all {c.n} checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
