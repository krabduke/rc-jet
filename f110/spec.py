"""
F110-GE-129 augmented low-bypass turbofan -- master dimensional specification.

ALL dimensions are millimetres. ALL angles are degrees. The engine axis is +X,
station 0.0 at the fan face, looking downstream. Radius is measured from the axis.

Every other module in this project consumes this file. No geometry module may
contain a literal dimension; if a number describes the engine, it belongs here.

CONFIDENCE
----------
`envelope` and `architecture` are published figures (see SOURCES).
Axial stations, annulus radii, blade counts, chords and twist distributions are
NOT published by GE. They are derived here to be self-consistent with the
published envelope, bypass ratio and stage count, using standard turbomachinery
practice (hub/tip ratios, stage loading, aspect ratios, solidity ~1.0-1.5).
They are engineering-plausible, not manufacturer data. Flagged `DERIVED`.
"""

from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# Published figures
# --------------------------------------------------------------------------

SOURCES = {
    "envelope": "GE Aviation F110-GE-129 product sheet; USAF F-16C/D fact sheet",
    "architecture": "GE F110 family technical descriptions (3-fan / 9-HPC / 1-HPT / 2-LPT)",
    "derived": "Standard axial turbomachinery sizing practice -- see module docstring",
}

ENGINE_NAME = "F110-GE-129"

# --- Published ---
LENGTH = 4630.0            # 182.3 in
MAX_DIAMETER = 1180.0      # 46.5 in
FAN_TIP_DIAMETER = 1180.0  # 46.5 in
DRY_WEIGHT_KG = 1805.0     # 3980 lb
THRUST_DRY_N = 76300.0     # 17155 lbf
THRUST_AB_N = 131600.0     # ~29000 lbf
BYPASS_RATIO = 0.76
OVERALL_PRESSURE_RATIO = 30.7
N_FAN_STAGES = 3
N_HPC_STAGES = 9
N_HPT_STAGES = 1
N_LPT_STAGES = 2

# --------------------------------------------------------------------------
# Axial stations (DERIVED) -- x in mm from fan face
# --------------------------------------------------------------------------

STATION = {
    "inlet_lip":          -420.0,
    "igv":                 -90.0,
    "fan_face":              0.0,
    "fan_exit":            560.0,
    "splitter":            580.0,
    "fan_frame":           700.0,
    "hpc_inlet":           760.0,
    "hpc_exit":           1540.0,
    "diffuser_exit":      1640.0,
    "combustor_front":    1620.0,
    "combustor_exit":     1980.0,
    "hpt_inlet":          1980.0,
    "hpt_exit":           2110.0,
    "lpt_inlet":          2130.0,
    "lpt_exit":           2400.0,
    "turbine_frame":      2520.0,
    "mixer_front":        2520.0,
    "mixer_exit":         2760.0,
    "flameholder":        2860.0,
    "augmentor_front":    2760.0,
    "augmentor_exit":     3640.0,
    "nozzle_throat":      3940.0,
    "nozzle_exit":        4210.0,
    "tail":               4210.0,
}

# Bypass duct (DERIVED). Splitter radius is solved from BYPASS_RATIO against the
# fan-exit annulus -- see docs/derivation.md.
BYPASS = {
    "splitter_radius":   486.0,
    "outer_radius_fwd":  570.0,
    "outer_radius_aft":  520.0,
    "inner_radius_fwd":  486.0,
    "inner_radius_aft":  450.0,
    "n_struts":          10,
    "strut_chord":       180.0,
    "strut_thickness":    28.0,
}


@dataclass
class BladeRow:
    """One rotor or stator row."""
    name: str
    x: float               # axial station of the row's leading edge, mm
    count: int             # number of airfoils
    r_hub_le: float        # hub radius at leading edge
    r_tip_le: float        # tip radius at leading edge
    r_hub_te: float        # hub radius at trailing edge
    r_tip_te: float        # tip radius at trailing edge
    chord: float           # axial chord, mm
    twist_hub: float       # stagger angle at hub, deg
    twist_tip: float       # stagger angle at tip, deg
    thickness: float = 0.08     # t/c ratio
    camber: float = 0.06        # max camber as fraction of chord
    rotor: bool = True
    variable: bool = False      # variable-geometry vane row
    shrouded: bool = False      # part-span or tip shroud
    cooled: bool = False        # film-cooled (hot section)
    lean: float = 0.0           # tangential lean, deg


# --------------------------------------------------------------------------
# FAN -- 3 stages, LP spool (DERIVED)
# --------------------------------------------------------------------------

FAN_ROWS = [
    BladeRow("igv",      -90.0, 28, 232.0, 590.0, 234.0, 590.0,  70.0, 12.0,  4.0,
             thickness=0.05, camber=0.03, rotor=False, variable=True),
    BladeRow("fan_r1",    20.0, 32, 236.0, 590.0, 268.0, 588.0, 165.0, 42.0, 62.0,
             thickness=0.07, camber=0.05, shrouded=False),
    BladeRow("fan_s1",   215.0, 60, 272.0, 587.0, 292.0, 584.0,  75.0, 26.0, 14.0,
             thickness=0.06, camber=0.07, rotor=False, variable=True),
    BladeRow("fan_r2",   310.0, 44, 296.0, 583.0, 318.0, 580.0, 115.0, 38.0, 58.0,
             thickness=0.07, camber=0.055),
    BladeRow("fan_s2",   445.0, 72, 320.0, 579.0, 328.0, 576.0,  60.0, 24.0, 12.0,
             thickness=0.055, camber=0.07, rotor=False),
    BladeRow("fan_r3",   520.0, 52, 330.0, 575.0, 340.0, 572.0,  90.0, 35.0, 54.0,
             thickness=0.065, camber=0.05),
    BladeRow("fan_ogv",  620.0, 80, 344.0, 570.0, 350.0, 568.0,  55.0, 20.0, 10.0,
             thickness=0.05, camber=0.065, rotor=False),
]

SPINNER = {
    "length":       400.0,
    "base_radius":  232.0,
    "tip_radius":     9.0,
    "x_nose":      -400.0,
    "ogive_power":    1.08,   # r = R * (x/L)^p -- sharp conical-ogive military nose
}

# --------------------------------------------------------------------------
# HP COMPRESSOR -- 9 stages (DERIVED)
# Annulus contracts from (340, 460) to (330, 355); drum-type rotor.
# --------------------------------------------------------------------------

# Nine stages between station 760 and station 1540, which is what the table
# above says the compressor is.
#
# They used to run to 1602 -- 62 mm past their own declared exit -- and
# everything downstream is placed from that exit. The diffuser began upstream
# of the two blade rows feeding it and enclosed the ninth-stage rotor; the
# combustor dome's head, which bulges 64 mm forward of the combustor front,
# reached back over the ninth stator and into the rotating drum; the fuel
# nozzles' stems reached forward into the compressor's aft flange. Scaled
# about the inlet to fit, which leaves 4.3 to 7.5 mm between one row's
# trailing edge and the next row's leading edge -- 20 to 25 % of chord, which
# is what an axial gap in a compressor this tight actually is.
HPC_ROWS = [
    BladeRow("hpc_r1",  787.6, 38, 340.0, 458.0, 344.0, 452.0, 66.0, 40.0, 58.0),
    BladeRow("hpc_s1",  859.5, 52, 345.0, 451.0, 348.0, 446.0, 46.0, 26.0, 16.0,
             rotor=False, variable=True),
    BladeRow("hpc_r2",  913.0, 44, 349.0, 445.0, 352.0, 439.0, 58.0, 40.0, 56.0),
    BladeRow("hpc_s2",  977.5, 58, 353.0, 438.0, 355.0, 433.0, 41.0, 26.0, 16.0,
             rotor=False, variable=True),
    BladeRow("hpc_r3", 1025.4, 50, 356.0, 432.0, 358.0, 427.0, 51.0, 39.0, 55.0),
    BladeRow("hpc_s3", 1082.6, 64, 359.0, 426.0, 360.0, 421.0, 37.0, 25.0, 15.0,
             rotor=False, variable=True),
    BladeRow("hpc_r4", 1126.8, 56, 361.0, 420.0, 362.0, 415.0, 45.0, 38.0, 54.0),
    BladeRow("hpc_s4", 1177.5, 70, 362.0, 414.0, 363.0, 410.0, 33.0, 25.0, 15.0,
             rotor=False),
    BladeRow("hpc_r5", 1217.1, 62, 363.0, 409.0, 364.0, 404.0, 40.0, 38.0, 53.0),
    BladeRow("hpc_s5", 1263.2, 76, 364.0, 403.0, 364.0, 399.0, 29.0, 24.0, 14.0,
             rotor=False),
    BladeRow("hpc_r6", 1298.2, 68, 364.0, 398.0, 364.0, 393.0, 35.0, 37.0, 52.0),
    BladeRow("hpc_s6", 1338.8, 82, 364.0, 392.0, 364.0, 388.0, 26.0, 24.0, 14.0,
             rotor=False),
    BladeRow("hpc_r7", 1370.1, 74, 364.0, 387.0, 363.0, 382.0, 31.0, 37.0, 51.0),
    BladeRow("hpc_s7", 1406.1, 88, 363.0, 381.0, 362.0, 377.0, 23.0, 23.0, 13.0,
             rotor=False),
    BladeRow("hpc_r8", 1434.6, 80, 362.0, 376.0, 361.0, 371.0, 27.0, 36.0, 50.0),
    BladeRow("hpc_s8", 1466.0, 94, 361.0, 370.0, 360.0, 367.0, 20.0, 23.0, 13.0,
             rotor=False),
    BladeRow("hpc_r9", 1490.8, 86, 360.0, 366.0, 359.0, 362.0, 24.0, 36.0, 49.0),
    BladeRow("hpc_s9", 1519.4, 100, 359.0, 361.0, 358.0, 358.0, 18.0, 23.0, 13.0,
             rotor=False),
]

# The compressor exits where its last blade row ends.
#
# STATION["hpc_exit"] was typed as 1540 and the last two rows of the HPC sit
# at 1553 and 1584: the diffuser began 62 mm upstream of the blades feeding
# it, so it enclosed the ninth-stage rotor, and the fuel nozzles' stems
# reached forward into the compressor's aft flange. Derived from the rows
# there is nothing to keep in step.
STATION["hpc_exit"] = HPC_ROWS[-1].x + HPC_ROWS[-1].chord + 4.0


# --------------------------------------------------------------------------
# COMBUSTOR -- annular (DERIVED)
# --------------------------------------------------------------------------

COMBUSTOR = {
    "x_front":            1620.0,
    "x_exit":             1980.0,
    "liner_outer_r":       432.0,
    "liner_inner_r":       296.0,
    "casing_outer_r":      470.0,
    "casing_inner_r":      262.0,
    "dome_r_outer":        420.0,
    "dome_r_inner":        308.0,
    "n_fuel_nozzles":       20,
    "nozzle_r":            364.0,   # pitch radius of the fuel nozzle ring
    "nozzle_stem_len":     110.0,
    "nozzle_tip_r":         16.0,
    "swirler_r":            30.0,
    "n_igniters":            2,
    "igniter_angles":  [40.0, 220.0],
    "n_cooling_rings":       6,
    "n_dilution_holes":     40,
    "dilution_hole_r":       9.0,
    "diffuser_len":        100.0,
}

# --------------------------------------------------------------------------
# TURBINES (DERIVED) -- annulus expands through the hot section
# --------------------------------------------------------------------------

HPT_ROWS = [
    BladeRow("hpt_ngv", 1985.0, 46, 318.0, 392.0, 316.0, 396.0, 58.0, 22.0, 10.0,
             thickness=0.16, camber=0.14, rotor=False, cooled=True),
    BladeRow("hpt_r1",  2055.0, 72, 315.0, 397.0, 312.0, 402.0, 48.0, -34.0, -52.0,
             thickness=0.14, camber=0.16, cooled=True, shrouded=True),
]

LPT_ROWS = [
    BladeRow("lpt_s1", 2140.0, 62, 310.0, 406.0, 308.0, 414.0, 46.0, 20.0,  9.0,
             thickness=0.13, camber=0.13, rotor=False, cooled=True),
    BladeRow("lpt_r1", 2198.0, 88, 307.0, 416.0, 304.0, 424.0, 42.0, -32.0, -50.0,
             thickness=0.12, camber=0.15, shrouded=True),
    BladeRow("lpt_s2", 2252.0, 70, 303.0, 426.0, 301.0, 436.0, 40.0, 19.0,  8.0,
             thickness=0.12, camber=0.13, rotor=False),
    BladeRow("lpt_r2", 2304.0, 96, 300.0, 438.0, 298.0, 448.0, 38.0, -30.0, -48.0,
             thickness=0.11, camber=0.14, shrouded=True),
]

# --------------------------------------------------------------------------
# SHAFTS AND BEARINGS (DERIVED)
# LP shaft runs the full length inside the hollow HP shaft (concentric spools).
# --------------------------------------------------------------------------

SHAFTS = {
    "lp_outer_r":       96.0,
    "lp_inner_r":       74.0,
    "lp_x0":          -140.0,
    "lp_x1":          2420.0,
    "hp_outer_r":      150.0,
    "hp_inner_r":      118.0,
    "hp_x0":           730.0,
    "hp_x1":          2120.0,
}

BEARINGS = [
    # name, x, shaft outer radius at that station, housing radius, type
    ("brg_1_lp_thrust",   -60.0,  96.0, 150.0, "ball"),
    ("brg_2_lp_roller",   690.0,  96.0, 148.0, "roller"),
    ("brg_3_hp_thrust",   745.0, 150.0, 214.0, "ball"),
    ("brg_4_hp_roller",  2128.0, 150.0, 212.0, "roller"),
    ("brg_5_lp_roller",  2410.0,  96.0, 152.0, "roller"),
]

# --------------------------------------------------------------------------
# AUGMENTOR / AFTERBURNER (DERIVED)
# --------------------------------------------------------------------------

AUGMENTOR = {
    "duct_r":             428.0,
    "liner_r":            404.0,
    "liner_thickness":      6.0,
    "n_screech_holes_ax":  26,
    "n_screech_holes_rad": 72,
    "screech_hole_r":       4.0,
    "mixer_lobes":         12,
    "mixer_lobe_depth":    72.0,
    "mixer_x0":          2520.0,
    "mixer_x1":          2760.0,
    "mixer_r_outer":      428.0,
    "mixer_r_inner":      300.0,
    "n_spraybars":         16,
    "spraybar_r":           9.0,
    "spraybar_x":        2800.0,
    "spraybar_r_root":    424.0,
    "spraybar_r_tip":     140.0,
    "flameholder_rings":  [150.0, 250.0, 350.0],
    "flameholder_v_width": 34.0,
    "flameholder_x":     2860.0,
    "n_radial_gutters":     8,
    "gutter_width":        30.0,
}

# --------------------------------------------------------------------------
# VARIABLE CONVERGENT-DIVERGENT NOZZLE (DERIVED)
# Modelled in the MAX AUGMENTED position (throat and exit open).
# --------------------------------------------------------------------------

NOZZLE = {
    "n_flaps":               12,
    "x_conv_start":      3640.0,
    "x_throat":          3940.0,
    "x_exit":            4210.0,
    "r_conv_start":       428.0,
    "r_throat":           330.0,
    "r_exit":             396.0,
    "flap_thickness":      14.0,
    "gap_deg":              1.6,    # tangential gap between adjacent flaps
    "n_seals":              12,     # interleaved external seals
    "seal_thickness":       10.0,
    "actuator_ring_x":   3590.0,
    "actuator_ring_r":    452.0,
    "n_actuators":           6,
    "actuator_len":       240.0,
    "actuator_r":          34.0,
    "link_r":              14.0,
    "n_ext_flaps":          12,
    "ext_flap_r_start":   452.0,
    "ext_flap_r_end":     404.0,
}

# --------------------------------------------------------------------------
# CASINGS (DERIVED)
# Each entry: (name, x0, x1, r0, r1, wall thickness)
# --------------------------------------------------------------------------

CASINGS = [
    ("casing_inlet",     -420.0,    0.0, 596.0, 596.0, 10.0),
    ("casing_fan",          0.0,  580.0, 596.0, 580.0, 12.0),
    ("casing_bypass",     580.0, 2520.0, 580.0, 476.0,  9.0),
    ("casing_hpc",        760.0, 1600.0, 466.0, 372.0, 14.0),
    ("casing_combustor", 1600.0, 1990.0, 478.0, 420.0, 16.0),
    ("casing_turbine",   1990.0, 2520.0, 420.0, 462.0, 15.0),
    ("casing_augmentor", 2520.0, 3640.0, 470.0, 434.0, 11.0),
]

# Bolted flange rings: (name, x, radius, outer radius, thickness, bolt count)
FLANGES = [
    ("flange_inlet",       0.0, 596.0, 626.0, 20.0, 36),
    ("flange_fan_rear",  580.0, 580.0, 612.0, 22.0, 36),
    ("flange_hpc_fwd",   760.0, 466.0, 498.0, 20.0, 30),
    # At the compressor exit, where the compressor casing bolts to the
    # combustor casing. It was at 1600, which is 60 mm downstream of the
    # exit -- out in the combustor, where the fuel nozzles come in through
    # the casing, and two of them ran through it.
    ("flange_hpc_aft",  1534.0, 470.0, 504.0, 24.0, 30),
    ("flange_comb_aft", 1990.0, 476.0, 510.0, 26.0, 32),
    ("flange_turb_aft", 2520.0, 462.0, 496.0, 24.0, 32),
    ("flange_aug_aft",  3640.0, 434.0, 466.0, 20.0, 28),
]

# --------------------------------------------------------------------------
# ACCESSORIES (DERIVED)
# --------------------------------------------------------------------------

ACCESSORIES = {
    "gearbox_x":          860.0,
    "gearbox_len":        460.0,
    "gearbox_width":      280.0,
    "gearbox_height":     190.0,
    "gearbox_r":          560.0,     # radial offset of gearbox centre from axis
    "gearbox_angle":     -90.0,      # mounted at bottom dead centre
    "towershaft_r":        26.0,
    "towershaft_angle":   -90.0,
    "towershaft_x":       790.0,
    "towershaft_r0":      170.0,
    "towershaft_r1":      540.0,
    "n_fuel_lines":          4,
    "fuel_line_r":          11.0,
    "n_oil_lines":           3,
    "oil_line_r":            8.0,
    "harness_r":             7.0,
    "n_harnesses":           3,
    "mount_fwd_x":        700.0,
    "mount_aft_x":       2520.0,
    "mount_trunnion_r":    46.0,
    "mount_pad_r":        600.0,
}

# --------------------------------------------------------------------------
# Material assignment: object-name prefix -> material key (see materials.py)
# --------------------------------------------------------------------------

MATERIAL_MAP = {
    # externally mounted line-replaceable units
    "oil_tank":            "steel",
    "heat_exchanger":      "steel",
    "engine_control":      "casing_alloy",
    "ignition_exciters":   "steel",
    "vbv_doors":           "casing_alloy",
    "borescope_ports":     "steel",
    "t5_harness":          "steel",
    "antiice_duct":        "steel",
    "mount_links_rear":    "titanium",
    "fan_containment":     "titanium",
    "inlet_probes":        "steel",
    "turbine_cooling_manifold": "steel",
    "bearing_sumps":       "steel",

    "spinner":     "titanium",
    "fan_r":       "titanium",
    "fan_s":       "titanium",
    "fan_ogv":     "composite",
    "igv":         "titanium",
    "hpc_r":       "titanium",
    "hpc_s":       "steel",
    "combustor":   "hot_nickel",
    "fuel_nozzle": "steel",
    "igniter":     "steel",
    "hpt":         "hot_nickel",
    "lpt":         "hot_nickel",
    "shaft":       "steel",
    "brg":         "steel",
    "casing":      "casing_alloy",
    "flange":      "casing_alloy",
    "bypass":      "casing_alloy",
    "mixer":       "hot_nickel",
    "flameholder": "hot_nickel",
    "spraybar":    "steel",
    "liner":       "thermal_barrier",
    "nozzle_flap": "thermal_barrier",
    "nozzle_seal": "thermal_barrier",
    "ext_flap":    "casing_alloy",
    "actuator":    "steel",
    "link":        "steel",
    "gearbox":     "casing_alloy",
    "towershaft":  "steel",
    "fuel_line":   "steel",
    "oil_line":    "steel",
    "harness":     "rubber",
    "mount":       "steel",
    "strut":       "casing_alloy",
    "hpc_drum":    "titanium",
    "hpc_front":   "steel",
    "hpc_rear":    "steel",
    "fan_disc":    "titanium",
    "fan_frame":   "casing_alloy",
    "interstage":  "steel",
    "splitter":    "titanium",
    "diffuser":    "hot_nickel",
    "turbine":     "hot_nickel",
    "fuel_manifold": "steel",
    "variable_vane": "steel",
    "inlet":       "casing_alloy",
    "towershaft":  "steel",
    "swirler":     "hot_nickel",
    "dome":        "hot_nickel",
}

DEFAULT_MATERIAL = "casing_alloy"

# --------------------------------------------------------------------------
# Alloys: base colour (linear RGB), metallic, roughness.
# glTF cannot carry the node-driven roughness used for rendering, so this table
# is also shipped to the web viewer and reapplied there by material name.
# --------------------------------------------------------------------------

PALETTE = {
    "titanium":        ((0.360, 0.372, 0.398), 1.00, 0.20),
    "steel":           ((0.520, 0.532, 0.552), 1.00, 0.14),
    "casing_alloy":    ((0.216, 0.226, 0.244), 1.00, 0.46),
    "hot_nickel":      ((0.300, 0.210, 0.150), 1.00, 0.50),
    "thermal_barrier": ((0.470, 0.440, 0.384), 0.15, 0.72),
    "composite":       ((0.170, 0.176, 0.190), 0.00, 0.48),
    "rubber":          ((0.055, 0.055, 0.059), 0.00, 0.88),
}

# --------------------------------------------------------------------------
# Tessellation -- global mesh resolution knobs
# --------------------------------------------------------------------------

RES = {
    # Raised from 28/9/96/24/16. At 96 segments a 1.18 m fan case is drawn
    # with 39 mm facets, which the eye reads as a faceted tube rather than a
    # turned surface; 144 puts that at 26 mm. The aerofoils dominate the
    # count -- 2,044 of them -- so chord and span points are what actually
    # move the total.
    # span 13 -> 15. Rewriting the blade platforms as swept sectors took 70 k
    # vertices of interior wall out of the model -- a wall between every
    # angular step of every platform, inside the solid where nothing could
    # see it. Spent on the blades instead, which is what the eye is on.
    "airfoil_chord_pts": 40,
    "airfoil_span_pts": 15,
    "revolve_segments": 144,
    "small_revolve": 36,
    "pipe_segments": 22,
}


def all_blade_rows():
    """Every airfoil row in the engine, front to back."""
    return FAN_ROWS + HPC_ROWS + HPT_ROWS + LPT_ROWS


def total_airfoil_count():
    return sum(r.count for r in all_blade_rows())
