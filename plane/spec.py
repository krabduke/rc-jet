"""
RC jet — master specification.

ALL dimensions are millimetres, angles degrees, masses grams.
Nose at x = 0, aft is +x. +z is up, +y is right. Same convention as the engine.

Every other module consumes this file. No geometry module contains a literal
dimension; if a number describes the aircraft, it lives here.

BRIEF
-----
Fit inside 500 x 300 mm in plan. Single engine, reusing the F110-GE-129 model
from the sibling project, scaled down. Everything else was chosen here.

CONFIGURATION (chosen, not derived)
-----------------------------------
A cropped-delta single-engine fighter — the layout that goes with the engine we
already have, and the one that survives a 300 mm span: a delta keeps enough
area and root chord to be stable at this size where a straight wing would not.
Scale is set by the span limit: 1:33 of a notional 15.05 m / 9.90 m fighter
gives 456 mm length on a 300 mm span, inside the envelope on both axes.

Chin inlet, bubble canopy, all-moving stabilators, single swept fin, tricycle
gear. Flaperons for roll and pitch trim, stabilators for pitch, rudder for yaw.
"""

import math

# --------------------------------------------------------------------------
# Envelope and scale
# --------------------------------------------------------------------------

ENVELOPE_LENGTH = 500.0        # hard limit, plan view
ENVELOPE_SPAN   = 300.0        # hard limit, plan view

SCALE = 1.0 / 33.0             # of the notional full-size fighter
LENGTH = 440.3      # body ends at 438; the nozzle protrudes to 440.3
SPAN   = 300.0

# The engine is the sibling project's F110-GE-129, scaled by the same factor.
ENGINE_SCALE = SCALE           # 4630 mm -> 140.3 mm long, 1180 mm fan -> 35.8 mm
ENGINE_X = 300.0               # station of the engine's inlet flange
ENGINE_Z = -1.0                # thrust line, below the fuselage datum

# --------------------------------------------------------------------------
# Fuselage loft (chosen)
# Sections are superellipses: |y/w|^n + |z/h|^n = 1, centred at zc.
# n rises to ~3 through the mid body, which is what gives a fighter its flat-
# sided, slab-cheeked look rather than a round tube.
# (x, half_width, half_height, z_centre, exponent)
# --------------------------------------------------------------------------

FUSELAGE = [
    (  0.0,  1.8,  1.8,  0.0, 2.2),
    ( 18.0,  8.5,  7.5,  0.5, 2.2),
    ( 42.0, 16.0, 14.0,  1.2, 2.4),
    ( 68.0, 23.0, 19.5,  2.0, 2.6),
    ( 98.0, 28.5, 23.5,  2.0, 2.8),
    (138.0, 31.0, 26.5,  1.2, 3.0),
    (180.0, 32.0, 29.0,  0.0, 3.0),
    (230.0, 32.0, 30.0, -1.0, 3.0),
    (280.0, 30.0, 29.0, -1.0, 2.8),
    (330.0, 27.0, 27.0, -1.0, 2.6),
    (380.0, 24.0, 25.0, -1.0, 2.4),
    (410.0, 19.5, 20.5, -1.0, 2.2),
    (438.0, 15.5, 16.0, -1.0, 2.0),
]

FUSELAGE_SKIN = 1.2            # foam/skin thickness for the shelled body

# --------------------------------------------------------------------------
# Wing — cropped delta (chosen)
# --------------------------------------------------------------------------

# The wing is defined by a planform table, not by a root chord and a sweep
# angle. A cropped delta does not have a straight leading edge: it starts as a
# highly swept root extension, unsweeps through the mid span and finishes
# nearly straight at the tip, and that curve is what makes it look like a wing
# rather than a triangle with the corner cut off.
#
# (span fraction, leading edge x, chord)
WING_PLANFORM = [
    (0.00, 132.0, 236.0),
    (0.14, 146.0, 224.0),
    (0.30, 170.0, 202.0),
    (0.48, 200.0, 172.0),
    (0.66, 232.0, 140.0),
    (0.82, 260.0, 112.0),
    (0.93, 280.0,  92.0),
    (1.00, 292.0,  78.0),
]

WING = {
    "x_root_le":   132.0,
    "z_root":       -6.0,       # mid-low mounted
    "root_chord":  236.0,
    "tip_chord":    78.0,
    "semi_span":   150.0,       # 300 mm total, the envelope limit
    "sweep_le":     40.0,       # nominal, for the checks; the table rules
    "dihedral":      0.0,
    "incidence":     0.0,
    "thickness":     0.098,     # t/c at the root, where the spar has to be deep
    "thickness_tip": 0.052,     # and at the tip, where thickness is only drag
    "camber":        0.0,       # symmetric
    "washout":       1.5,       # deg, tip trailing edge up: tip stalls last
    "le_root_ext":  28.0,       # leading-edge root extension (strake)
    "tip_rail":     True,
}

FLAPERON = {
    "span_in":       0.38,      # fraction of semi-span
    "span_out":      0.94,
    "chord_frac":    0.26,      # of local chord
    "gap":           1.2,
    "deflect":     -12.0,       # modelled deflection, deg
}

# --------------------------------------------------------------------------
# Tail (chosen)
# --------------------------------------------------------------------------

HTAIL = {
    "x_root_le":   392.0,   # clear of the wing TE at 360 mm
    "z_root":      -22.0,   # below the wing wake sheet, not in it
    "root_chord":   72.0,
    "tip_chord":    31.0,
    "semi_span":    62.0,
    "sweep_le":     34.0,
    "thickness":     0.072,
    "thickness_tip": 0.045,
    "root_y":       21.0,   # the fuselage side at this station
    "anhedral":     -6.0,
    "deflect":      -4.0,       # all-moving stabilator, modelled position
}

VTAIL = {
    "x_root_le":   330.0,
    "z_root":       24.0,       # where the fin leaves the fuselage spine
    "root_chord":  106.0,
    "tip_chord":    46.0,
    "height":       78.0,
    "sweep_le":     42.0,
    "thickness":     0.078,
    "thickness_tip": 0.048,
    "rudder_chord":  0.28,
    "rudder_span":   0.86,
    "deflect":       6.0,
}

VENTRAL = {
    "x_le":        372.0,
    "chord":        56.0,
    "depth":        20.0,
    "sweep":        38.0,
    "cant":         22.0,        # deg outboard
    "thickness":     4.0,
}

# --------------------------------------------------------------------------
# Inlet and duct (chosen) — chin inlet, as on the engine's real airframe
# --------------------------------------------------------------------------

INTAKE = {
    "x_lip":        52.0,
    "x_throat":     86.0,
    "x_duct_end":  300.0,       # meets the engine face
    "lip_width":    40.0,
    "lip_height":   19.0,
    "z_lip":       -17.0,
    "lip_radius":    2.6,
    "duct_r_end":   19.5,       # wraps the 17.9 mm fan radius with clearance
    "splitter_gap":  3.0,       # boundary-layer diverter standoff
    "wall":          1.1,
}

# --------------------------------------------------------------------------
# Canopy (chosen)
# --------------------------------------------------------------------------

CANOPY = {
    "x_front":      78.0,
    "x_rear":      178.0,
    "z_base":       22.0,
    "height":       19.0,
    "half_width":   16.0,
    "frame":         1.6,
    "windscreen_x": 96.0,
}

# --------------------------------------------------------------------------
# Cockpit (chosen) — sized by what is under the floor, not by the canopy
# --------------------------------------------------------------------------
#
# The flight pack fills the bay from the datum up to z = 20, so the tub floor
# has to sit on top of it. That leaves 19 mm from floor to canopy crown, which
# is why the pilot is a bust: a full figure would need a footwell the battery
# is already in. Every z here is checked against tools/audit_fit's canopy_top.

COCKPIT = {
    "x_front":        95.0,     # footwell bulkhead, under the windscreen
    "x_rear":        152.0,     # ends on bhd_cockpit; the receiver is behind it
    "z_floor":        20.6,     # on top of the flight pack
    "depth":           3.2,     # tub side height above the floor
    "half_width":     12.4,
    "half_width_aft": 11.0,
    "wall":            1.0,
    "x_panel":        99.0,
    "z_panel_top":    30.2,
    "panel_cant":     19.0,     # deg, top laid back towards the pilot
    "x_hud":         105.0,
    "x_pedals":      106.0,
    "x_seat":        126.0,     # seat pan front edge
    "seat_half_width": 7.0,
    "seat_recline":   16.0,     # deg
    # The canopy is 19 mm deep from the tub floor and the flight pack is
    # under the floor, so a 1:34 pilot does not fit and never could. This
    # is a scale-RC pilot: a bust sized to the hatch rather than to a
    # person, which is what those figures actually are.
    "helmet_r":        3.6,
}

def canopy_profile(t):
    """Canopy half-width and height at t along its length, 0 at the
    windscreen base and 1 at the spine.

    The skin needs this as much as the canopy does: the aperture the skin has
    to be cut for is the canopy's own plan outline, and the two have to agree
    exactly or the glass lands on air. So it lives here, with the numbers it
    is made of, rather than in either module.
    """
    w = CANOPY["half_width"] * math.sin(math.pi * min(t * 1.12, 1.0)) ** 0.62
    h = CANOPY["height"] * math.sin(math.pi * min(t * 1.06, 1.0)) ** 0.52
    return max(w, 0.4), max(h, 0.3)


# --------------------------------------------------------------------------
# Landing gear (chosen) — tricycle, fixed, modelled down
# --------------------------------------------------------------------------

GEAR = {
    "nose_x":       98.0,
    "nose_leg":     34.0,
    "nose_wheel_r":  9.0,
    "nose_wheel_w":  5.0,
    # The wing moved forward, so the CG did, so the main gear has to follow:
    # it has to sit just behind the CG or the aircraft tips onto its tail,
    # and not so far behind that the nose leg carries too much to steer.
    "main_x":      252.0,
    "main_y":       44.0,
    "main_leg":     36.0,
    "main_wheel_r": 11.0,
    "main_wheel_w":  6.0,
    "strut_r":       2.2,
}

# --------------------------------------------------------------------------
# Structure (chosen)
# --------------------------------------------------------------------------

SPAR = {
    "x_frac":        0.30,      # of local chord
    "outer_r":       2.0,       # 4 mm carbon tube
    "inner_r":       1.3,
    "span":        286.0,
}

BULKHEADS = [
    # (name, x, thickness)
    ("bhd_nose",     66.0, 2.0),
    ("bhd_cockpit", 150.0, 2.0),
    ("bhd_spar",    230.0, 2.5),
    ("bhd_firewall",298.0, 3.0),
    ("bhd_tail",    430.0, 2.0),
]

# --------------------------------------------------------------------------
# RC hardware (chosen) — each a box with a real mass, used for the CG solve
# (name, x_centre, y_centre, z_centre, length, width, height, mass_g)
# --------------------------------------------------------------------------

# The equipment bay, as a layout rather than a pile.
#
# Every box below was previously positioned on its own, by eye, against the
# fuselage section -- and the section is not what decides where there is room.
# The intake duct is: it fills the bottom of the body from the chin inlet all
# the way to the engine and climbs as it goes, so the usable volume is a
# wedge above it that is 30 mm deep at the cockpit and 7 mm deep at the
# firewall. Placed independently, twenty-five items produced sixty-seven
# box-on-box overlaps: both batteries in the same place, the fuel filter
# inside the duct, the fuel lines inside the engine's LP shaft.
#
# These positions were solved against the duct and the skin together, and
# `verify.py` fails if any two of them overlap or if one leaves the body.
#
# (x, y, z centre, length, width, height, mass g)
EQUIPMENT = [
    ("lipo_3s_900",   117.0,   0.0,  10.0, 58.0, 32.0, 20.0,  75.0),
    ("retract_nose",   74.0,   0.0,   1.0, 28.0, 15.0, 13.0,  22.0),
    ("fuel_filter",   166.0, -20.5,  11.5, 28.0, 13.0, 13.0,  12.0),
    ("turbine_ecu",   169.0,   0.0,  10.0, 34.0, 22.0, 11.0,  26.0),
    ("fuel_pump",     164.0,  20.5,  11.5, 24.0, 13.0, 13.0,  34.0),
    ("receiver",      166.0,   0.0,  19.0, 22.0, 16.0,  6.0,   7.0),
    ("kill_switch",   187.0,   4.0,  23.0, 16.0, 10.0,  8.0,   6.0),
    ("telemetry_gps", 202.0, -12.0,  16.0, 24.0, 14.0,  8.0,  24.0),
    ("rx_battery",    208.0,   5.0,  19.0, 36.0, 18.0, 12.0,  46.0),
    ("ecu_battery",   254.0,   0.0,  22.7, 32.0, 18.0,  7.0,  38.0),
]


def equipment(name):
    """(x, y, z, length, width, height) for one bay item."""
    for e in EQUIPMENT:
        if e[0] == name:
            return e[1:7]
    raise KeyError(name)


# A turbine aircraft has no electronic speed controller -- that is the part
# that drives a brushless motor, and there is no motor. It had one, a 45 mm
# box in the middle of the bay, left over from when this was an EDF. What a
# turbine needs instead is the ECU and its own battery, which are both here.
HARDWARE = [
    # In the wing, lying flat in its 19 mm of thickness, which is where an
    # aileron servo goes. At y +/-19 in the fuselage they were inside the
    # saddle tanks and inside the main gear retracts, and there is no
    # station at that width that is not one or the other.
    ("servo_ail_l",   261.0,-55.0,  -6.0, 23.0, 22.0, 12.0,   5.5),
    ("servo_ail_r",   261.0, 55.0,  -6.0, 23.0, 22.0, 12.0,   5.5),
    ("servo_stab",    288.0, -8.0,   7.0, 23.0, 12.0, 22.0,   5.5),
    ("servo_rudder",  288.0,  8.0,   7.0, 23.0, 12.0, 22.0,   5.5),
]

# Distributed masses that are not discrete boxes: (name, x_centre, mass_g)
DISTRIBUTED = [
    ("airframe_skin",   232.0, 62.0),
    ("wing_structure",  218.0, 18.0),
    # 62 g was the figure that let this aircraft balance while a third of
    # its hardware went uncounted. A turbine with its shaft, discs, casings,
    # mount ring, thrust tube and tailpipe cannot weigh what a phone weighs;
    # the smallest turbine anyone actually flies is six hundred grams, and
    # 140 g is the lightest this one could credibly be built.
    ("engine",          370.0, 140.0),
    ("wiring_misc",     250.0, 12.0),
    ("gear_assembly",   219.0, 14.0),
]

TARGET_CG_FRAC = 0.25          # of mean aerodynamic chord
CG_TOLERANCE   = 0.03          # +/- 3 % MAC is the acceptance band

# --------------------------------------------------------------------------
# Materials: object-name prefix -> material key
# --------------------------------------------------------------------------

MATERIAL_MAP = {
    # airframe hardware
    "thrust_tube":          "hot_metal",
    "tailpipe_cone":        "hot_metal",
    "wing_joiner":          "carbon",
    "wing_bolt_l":          "steel",
    "wing_bolt_r":          "steel",
    "servo_arm_ail_l":      "plastic",
    "servo_arm_ail_r":      "plastic",
    "servo_arm_rudder":     "plastic",
    "servo_arm_stab":       "plastic",
    "battery_strap_lipo":   "fabric",
    "battery_strap_rx":     "fabric",
    "nose_steering_link":   "steel",

    "canopy_latch": "alu",
    "rx_mount": "board",
    "rx_battery": "lipo",
    "access_tray": "ply",
    "stringer_": "spruce",
    "clevis_": "steel",
    "horn_": "steel",
    "wheel_hub_": "alu",
    "wing_fence_": "airframe",
    "vg_": "airframe",
    "telemetry_gps": "board",
    "antenna_": "carbon",
    "bypass_slots": "duct",
    "mount_rails": "carbon",
    "mount_ring": "alu",
    "cockpit_tub": "plastic",
    "coaming": "fabric",
    "console_": "plastic",
    "instrument_panel": "board",
    "panel_instruments": "plastic",
    "hud_glass": "visor",
    "hud_frame": "plastic",
    "seat_pan": "fabric",
    "seat_back": "fabric",
    "seat_headbox": "fabric",
    "seat_harness": "fabric",
    "seat_rails": "steel",
    "ejection_handle": "warning",
    "rudder_pedals": "alu",
    "control_stick": "plastic",
    "throttle_lever": "plastic",
    "pilot_torso": "flightsuit",
    "pilot_arms": "flightsuit",
    "pilot_helmet": "helmet",
    "pilot_visor": "visor",
    "pilot_mask": "plastic",
    "cooling_exit": "duct",
    "naca_inlet": "duct",
    "avionics_tray": "ply",
    "kill_switch": "board",
    "ecu_battery": "lipo",
    "turbine_ecu": "board",
    "gear_door_actuator": "steel",
    "retract_": "steel",
    "fuel_lines": "carbon",
    "fuel_filter": "alu",
    "fuel_pump": "servo",
    "fuel_hopper": "lipo",
    "fuel_tank": "lipo",
    "bellcranks": "servo",
    "pushrod_linkages": "carbon",
    "nose_strakes": "airframe",
    "intake_lip_ring": "duct",
    "bl_diverter": "airframe",
    "fuselage":   "airframe",
    "wing":       "airframe",
    "flaperon":   "control",
    "htail":      "control",
    "stabilator": "control",
    "vtail":      "airframe",
    "rudder":     "control",
    "ventral":    "airframe",
    "intake":     "duct",
    "duct":       "duct",
    "canopy":     "glass",
    "frame":      "airframe",
    "bhd":        "ply",
    "spar":       "carbon",
    "pushrod":    "carbon",
    "lipo":       "lipo",
    "esc":        "board",
    "receiver":   "board",
    "servo":      "servo",
    "wheel":      "rubber",
    "strut":      "steel",
    "gear":       "steel",
    "engine":     "engine",
    "control_horns":      "steel",
    "clevises":           "steel",
    "pitot":              "steel",
    "antennas":           "plastic",
    "wing_fences":        "airframe",
    "vortex_generators":  "airframe",
    "navlight_port":      "lens_red",
    "navlight_stbd":      "lens_green",
    "navlight_tail":      "lens_white",
    "gear_door":          "airframe",
    "brake_":             "steel",
    "wheel_hubs":         "alu",
    "panel_":             "airframe",
    "tailpipe_shroud":    "hot_metal",
    "pylon_":             "airframe",
    "missile_":           "ordnance",
    "static_dischargers": "steel",
    "former":             "ply",
    "longeron":           "spruce",
    "stringers":          "spruce",
    "rib":                "ply",
    "fin_rib":            "ply",
    "spar_rear":          "carbon",
    "hinge_flaperon":     "steel",
    "hinge_rudder":       "steel",
    "seam":               "airframe",
    "panel_screws":       "steel",
    "wing_seams":         "airframe",
    "doublers":           "airframe",
}
DEFAULT_MATERIAL = "airframe"

PALETTE = {
    # name:         base colour (linear RGB),   metallic, roughness
    "airframe":    ((0.255, 0.278, 0.302), 0.05, 0.62),
    "control":     ((0.300, 0.322, 0.348), 0.05, 0.58),
    "duct":        ((0.118, 0.124, 0.133), 0.10, 0.70),
    "glass":       ((0.180, 0.240, 0.280), 0.20, 0.12),
    "ply":         ((0.470, 0.372, 0.226), 0.00, 0.76),
    "carbon":      ((0.055, 0.058, 0.064), 0.35, 0.34),
    "lipo":        ((0.140, 0.170, 0.310), 0.10, 0.52),
    "board":       ((0.060, 0.180, 0.110), 0.10, 0.60),
    "servo":       ((0.080, 0.082, 0.086), 0.05, 0.55),
    "rubber":      ((0.040, 0.041, 0.044), 0.00, 0.88),
    "steel":       ((0.480, 0.492, 0.510), 1.00, 0.28),
    "engine":      ((0.412, 0.432, 0.462), 1.00, 0.34),
    "alu":         ((0.560, 0.570, 0.585), 1.00, 0.22),
    "plastic":     ((0.150, 0.152, 0.158), 0.00, 0.44),
    "fabric":      ((0.095, 0.098, 0.108), 0.00, 0.90),
    "hot_metal":   ((0.300, 0.282, 0.268), 1.00, 0.46),
    "lens_red":    ((0.620, 0.055, 0.048), 0.00, 0.18),
    "lens_green":  ((0.055, 0.520, 0.140), 0.00, 0.18),
    "lens_white":  ((0.760, 0.770, 0.790), 0.00, 0.18),
    "spruce":      ((0.545, 0.452, 0.288), 0.00, 0.70),
    "ordnance":    ((0.216, 0.230, 0.218), 0.10, 0.52),
    # the cockpit
    "helmet":      ((0.720, 0.730, 0.742), 0.00, 0.26),
    "visor":       ((0.240, 0.180, 0.060), 0.60, 0.10),
    "flightsuit":  ((0.118, 0.140, 0.108), 0.00, 0.86),
    "warning":     ((0.620, 0.440, 0.030), 0.00, 0.42),
}

# Resolution. Forty chord points and fourteen spanwise stations made the main
# wing a 532-vertex object closed off with a flat rib at the tip, which is
# what made it read as blocky. These are the numbers the sibling F110 project
# uses for its blade rows.
TESS = 2.0   # global tessellation multiplier, applied in mesh.py

RES = {"fuse_sections": 80, "fuse_stations": 72, "airfoil_pts": 72,
       "wing_stations": 30, "revolve": 72, "small_revolve": 28}


# --------------------------------------------------------------------------
# Derived geometry
# --------------------------------------------------------------------------

def taper_ratio():
    return WING["tip_chord"] / WING["root_chord"]


def _planform_strips(n=81):
    """(chord, leading-edge x, strip width) along the semi span.

    Everything derived from the wing -- area, MAC, where the MAC is -- is
    integrated over the planform table rather than assumed trapezoidal. The
    closed-form taper formulae describe a straight-edged wing and this one is
    not straight-edged, so they would quietly describe a different aeroplane
    from the one that gets built.
    """
    dy = WING["semi_span"] / n
    out = []
    for i in range(n):
        f = (i + 0.5) / n
        x_le, c = _planform_at(f)
        out.append((c, x_le, dy))
    return out


def _planform_at(f):
    tbl = WING_PLANFORM
    if f <= tbl[0][0]:
        return tbl[0][1], tbl[0][2]
    if f >= tbl[-1][0]:
        return tbl[-1][1], tbl[-1][2]
    for i in range(len(tbl) - 1):
        f0, x0, c0 = tbl[i]
        f1, x1, c1 = tbl[i + 1]
        if f0 <= f <= f1:
            t = (f - f0) / (f1 - f0)
            return x0 + (x1 - x0) * t, c0 + (c1 - c0) * t
    return tbl[-1][1], tbl[-1][2]


def mean_aero_chord():
    """MAC by integration: (2/S) * int c^2 dy over the semi span."""
    strips = _planform_strips()
    num = sum(c * c * dy for (c, _, dy) in strips)
    return num / sum(c * dy for (c, _, dy) in strips)


def mac_spanwise():
    """Spanwise station of the MAC, from the root, by integration."""
    strips = _planform_strips()
    n = len(strips)
    num = 0.0
    for i, (c, _, dy) in enumerate(strips):
        y = (i + 0.5) / n * WING["semi_span"]
        num += c * y * dy
    return num / sum(c * dy for (c, _, dy) in strips)


def mac_leading_edge_x():
    """Leading-edge station at the MAC's spanwise position."""
    return _planform_at(mac_spanwise() / WING["semi_span"])[0]


def target_cg_x():
    return mac_leading_edge_x() + TARGET_CG_FRAC * mean_aero_chord()


def wing_area_mm2():
    """Reference wing area, both sides, integrated over the planform."""
    return 2.0 * sum(c * dy for (c, _, dy) in _planform_strips())


def all_masses():
    """(name, x, mass) for every mass in the aircraft."""
    out = [(n, x, m) for (n, x, m) in DISTRIBUTED]
    out += [(h[0], h[1], h[7]) for h in HARDWARE]
    out += [(e[0], e[1], e[7]) for e in EQUIPMENT]
    return out


def total_mass_g():
    return sum(m for (_, _, m) in all_masses())


def cg_x():
    ms = all_masses()
    return sum(x * m for (_, x, m) in ms) / sum(m for (_, _, m) in ms)


def cg_frac_mac():
    return (cg_x() - mac_leading_edge_x()) / mean_aero_chord()


def wing_loading_g_dm2():
    return total_mass_g() / (wing_area_mm2() / 10000.0)


# --------------------------------------------------------------------------
# Built-up structure
# --------------------------------------------------------------------------
# The airframe is not a solid block: it is formers and longerons carrying a
# stressed skin, wing ribs on a spar, and hinge lines where surfaces move.
# Modelling that is what makes a cutaway worth looking at.

# Wing surface detail, sized as fractions of the local section rather than in
# absolute millimetres -- a 4 mm vortex generator is right at the root and
# absurd at a 58 mm tip chord.
WING_DETAIL = {
    "fence_stations": [0.46, 0.70],
    "fence_h":        0.030,   # of local chord
    "fence_chord":    0.44,
    "fence_t":         1.4,
    "n_vg":             12,
    "vg_from":        0.34,
    "vg_to":          0.90,
    "vg_x":           0.62,    # chord fraction
    "vg_h":           0.016,   # of local chord
    "vg_chord":       0.050,
    "vg_t":            0.9,
    "vg_yaw":         14.0,    # deg, alternating: counter-rotating pairs
    "horn_h":          7.0,
}

STRUCTURE = {
    # lightweight formers, between the load-bearing bulkheads above
    "former_x":     [30.0, 48.0, 88.0, 110.0, 130.0, 170.0, 190.0,
                     210.0, 250.0, 268.0, 316.0, 344.0, 372.0, 400.0],
    "former_t":       1.5,
    "former_hole":    0.74,   # inner contour, as a fraction of the section
    "n_longerons":      4,
    "longeron_r":     2.6,
    "n_stringers":     12,
    "stringer_r":     1.1,
    "n_wing_ribs":      9,
    "rib_t":          1.6,
    "rib_hole":       0.58,   # lightening cut-out, fraction of the section
    "rear_spar_frac": 0.70,   # of local chord
    "rear_spar_r":    1.4,
    "rear_spar_span": 0.945,  # of semi-span, so the tube ends inside the tip
    "n_fin_ribs":       4,
    "rib_inset":      0.9,    # keep ribs under the skin, not through it
    "hinge_r":        0.75,   # the hinge shows in the flaperon gap, so it
    "hinge_knuckles":  11,    # has to be scaled like real hardware
}

# Skin detail: panel seams, fastener rows and the fine surface relief that
# separates a model of an aeroplane from a smooth blob the same shape.
SKIN_DETAIL = {
    "seam_x":       [42.0, 68.0, 98.0, 150.0, 200.0, 252.0, 298.0,
                     340.0, 384.0, 416.0],
    "seam_h":         0.45,   # how far a seam stands off the skin
    "seam_w":         1.5,
    "n_lengthwise":     4,
    "screws_per_panel": 12,
    "screw_r":        0.75,
    # (x0, x1, angle_from, angle_to, standoff) -- panels follow the section
    "panels": [
        ("battery",   188.0, 268.0,  58.0, 122.0, 0.9),
        ("receiver",  166.0, 202.0,  16.0,  54.0, 0.8),
        ("avionics",  272.0, 312.0,  58.0, 122.0, 0.9),
        ("gearbay",   258.0, 300.0, 236.0, 304.0, 0.8),
        ("fuel",      312.0, 352.0, 236.0, 304.0, 0.8),
    ],
}

# Probes and aerials
TAILPIPE = {
    "x_front":     424.0,
    "x_rear":      438.0,
    "r_front":      16.4,
    "r_rear":       14.6,
    "wall":          0.9,
    "petals":         12,
}

PROBE = {
    "pitot_tip":    -28.0,   # ahead of the nose datum; keeps inside 500 mm
    "pitot_r":        1.1,
    "pitot_z":        2.0,
    "fairing_r":      2.4,
}
