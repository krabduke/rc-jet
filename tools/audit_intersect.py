"""No part of the aircraft may occupy another part's space.

    python3 tools/audit_intersect.py            (runs itself under Blender)
    python3 tools/audit_intersect.py --shrink   (after a fix: drop what is fixed)

Every pair of parts whose material overlaps by TOL or more -- measured
exactly, both ways, buried parts included; see tools/_interfere.py -- must be
one of two things:

*   Declared in EXPECTED: meant to be that way, a pin in its bore, a rib inside
    a closed skin. A rule that excuses nothing, or names a part that does not
    exist, fails the audit. A permission that no longer matches anything is a
    hole a regression can fall into unseen, and the list had grown to more
    dead rules than live ones before this was enforced.
*   On the KNOWN list: a real defect, written down with how deep it is and
    where it is. The list only gets shorter. A pair not on it fails, a pair
    that gets deeper fails, and a pair that has been fixed fails until
    --shrink takes it off. --shrink never adds anything.
"""
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _intersect

# Pairs that share material on purpose.
#
# A built-up airframe is mostly parts inside other parts: that is what
# "built-up" means. Ribs live inside a closed wing skin, formers and
# bulkheads inside the fuselage, doublers are bonded to the skin they
# reinforce, and audit_fit.py already requires exactly that arrangement -- it
# fails if an internal part gets OUT. So the two tests are opposites and both
# have to hold.
#
# Three of these are conventions rather than perfect assemblies, and they are
# written down as such rather than quietly filtered:
EXPECTED = [
    # the piano hinge's pin runs through the flaperon's own knuckle
    ("flaperon_", "hinge_flaperon_"),
    # a screw head is let into the panel it holds down
    ("panel_screws", "panel_"),
    # ----------------------------------------------------------------
    # A feature let into the skin lands on the frame member behind it. On a
    # built-up airframe the member is cut there and a doubler goes in, which
    # is the same statement as ("panel_", "doublers") above.
    # ----------------------------------------------------------------
    ("naca_inlet_", "stringer"),
    ("cooling_exit", "longeron"), ("cooling_exit", "stringer"),
    ("gear_door_", "longeron"), ("gear_door_", "stringer"),
    ("stringer", "canopy_frame"),

    # A seam is a line ON a surface, so it meets whatever crosses that
    # surface -- vents, hatches, the fin root, the wing skin.
    ("seam_", "bypass_slots"), ("seam_", "vtail_fin"),
    ("seam_", "gear_door_"),
    ("duct_seam", "gear_door_"),

    # A piano hinge pin runs the whole span of the surface it carries, so
    # every rib it passes is notched for it.
    ("rib_", "hinge_flaperon_"), ("fin_rib", "hinge_rudder"),

    # The intake duct owns the middle of the fuselage, so the tail servos,
    # their arms and the main gear retracts live in the wing root -- which is
    # 19.7 mm thick, empty, and where the leg they drive already is.
    ("gear_door_actuator_", "wing_"),
    ("gear_door_actuator_", "rib_"),
    # an actuator is pinned to the door it opens, and a steering link runs
    # down into the well past the door hinged beside it
    ("gear_door_actuator_", "gear_door_"),
    # ----------------------------------------------------------------
    # The landing gear, joint by joint. Nothing on a gear is welded to
    # anything: a door hangs on a hinge line, its latches are bolted to it,
    # and every stay, link and ram ends in a clevis with a pin through it.
    # Each of these overlaps is that pin. They only showed up once the rod
    # ends were built -- before, each member ended in a flat disc that
    # stopped short of the thing it is attached to.
    # ----------------------------------------------------------------
    ("gear_door_hinge_", "gear_door_"), ("gear_door_hinge_", "gear_bay_"),
    ("gear_door_latches_", "gear_door_"),
    ("gear_main_side_stay_", "gear_main_"),
    ("gear_main_drag_stay_", "gear_main_"),
    ("gear_main_chrome_slider_", "gear_main_"),
    ("gear_main_drag_stay_", "wing_"), ("gear_main_drag_stay_", "rib_"),
    ("gear_main_downlock_", "wing_"),
    ("gear_nose_chrome_slider", "gear_nose_drag_stay"),
    # Six actuators on the unison ring and twelve links off it, so every
    # actuator shares its clock angle with an even-numbered link and they
    # meet at the ring. That is the joint: the ram drives the ring, the ring
    # drives the links, the links drive the flaps.
    ("nozzle_link_", "nozzle_actuator_"),
    # The hinge knuckles bolt to the canopy frame, the nose fork is on the
    # bottom of the slider, the pedals slide on their rail, and the bay
    # cooling scoop is let into the skin and lands on the stringer behind
    # it -- the same statement as the naca_inlet block at the top of this
    # list. Each of these appeared the moment the part stopped being a
    # cylinder or a cube and grew the fitting it attaches by.
    ("canopy_frame", "frame_canopy_rear_hinge"),
    ("gear_nose_chrome_slider", "gear_nose_fork"),
    ("rudder_pedals", "rudder_pedals_rail"),
    ("bay_cooling_inlet", "stringer"),
    # ----------------------------------------------------------------
    # The rest of the aeroplane, joint by joint.
    #
    # A nose leg is one assembly: the slider, the fork, the drag stay, the
    # shimmy damper and the steering actuator all pin to the strut. So do
    # the main legs' trailing links and their strut doors. The leading-edge
    # flap hinges to the wing's ribs at its own hinge line. The flight
    # control loom passes through the frames and the duct inlets it runs
    # past, as a loom does, and lands on the generator it is powered from.
    # An access hatch is a hole in the skin with a lid on it. A servo drives
    # the surface it is bolted to. The canopy's strut lifts the canopy. The
    # bay's fire bottle and cooling inlet are mounted on the engine mount
    # ring and the longeron beside it. The HUD's projector feeds its
    # combiner.
    # ----------------------------------------------------------------
    ("gear_nose_chrome_slider", "gear_nose_strut"),
    ("gear_nose_fork", "gear_nose_strut"),
    ("gear_nose_drag_stay", "gear_nose_strut"),
    ("gear_nose_shimmy_damper", "gear_nose_strut"),
    ("gear_nose_steering_actuator", "gear_nose_strut"),
    ("gear_nose_drag_stay", "gear_nose_shimmy_damper"),
    ("gear_main_trailing_link_", "gear_main_"),
    ("gear_door_strut_main_", "gear_main_"),
    ("rib_", "leading_edge_flap_"),
    ("former", "fcs_loom_trunk_"), ("longeron", "fcs_loom_trunk_"),
    ("stringer", "fcs_loom_trunk_"),
    ("engine_generator", "fcs_loom_tail_"),
    ("panel_", "fuselage_skin"),
    ("stab_servo_", "stabilator_"), ("rudder_servo", "fin_rib"),
    ("rudder_servo", "fuselage_skin"),
    ("strut_canopy_actuator", "canopy_frame"),
    ("strut_canopy_actuator", "canopy_glass"),
    ("strut_canopy_actuator", "frame_canopy_seal"),
    ("mount_ring", "bay_cooling_inlet"),
    ("longeron", "bay_cooling_inlet"),
    ("hud_frame_projector", "hud_frame_combiner"),
    # The canopy, the windscreen and their frame are let into the skin, and
    # the two panes meet at the bow. The engine bay's doors are a hole in
    # the skin at a production joint. The nose gear's bay is cut into the
    # structure, so its lining and the leg inside it meet the stringers and
    # the seam the bay interrupts. The fin-tip fairing caps the fin and
    # clears the rudder hinge. The forward mount links tie the engine's own
    # mount pads to the airframe's ring, which is what a mount link is, and
    # the bay's cooling and fire equipment hangs off the same ring.
    ("canopy_windscreen_glass", "fuselage_skin"),
    ("frame_windscreen_bow", "fuselage_skin"),
    ("frame_canopy_seal", "fuselage_skin"),
    ("frame_canopy_breaker_cord", "fuselage_skin"),
    ("former", "frame_canopy_seal"),
    ("bay_doors", "fuselage_skin"), ("bay_doors", "seam_ring_"),
    ("gear_bay_lining_nose", "gear_bay_nose"),
    ("gear_bay_lining_nose", "stringer"),
    ("stringer", "gear_nose_strut"), ("seam_lengthwise", "gear_nose_strut"),
    ("fin_tip_ecm_fairing", "vtail_fin"),
    ("fin_tip_ecm_fairing", "hinge_rudder"),
    ("bay_mount_links_fwd", "mount_ring"),
    ("bay_mount_links_fwd", "engine_mount_pads"),
    ("bay_mount_links_fwd", "engine_harnesses"),
    ("bay_mount_links_fwd", "engine_fan_cowl_door"),
    ("bay_bleed_offtake", "mount_ring"),
    ("bay_bleed_offtake", "engine_bleed_pipes"),
    ("control_stick_boots", "cockpit_tub"),
    # a loom goes through a bulkhead in a grommet, which is a hole with a
    # seal in it and not a part of its own
    ("bhd_", "fcs_loom_"),
    # ----------------------------------------------------------------
    # The last of it, and all of it is fitting.
    #
    # The loom's four channels terminate in the cockpit's consoles and run
    # aft along the canopy rail past the tub, clipped alongside the fuel
    # lines that share the only route over the tank. The canopy's frame
    # holds its glass, the seal seals it and the breaker cord is bonded to
    # it. The bay's fire bottle and its mount links are carried across the
    # bay frames, which have clearance holes for them, and those links bolt
    # to the engine's own casings -- that is what a mount link is. The fire
    # detection loop runs round the engine and over its accessories, which
    # is where a fire starts. The firewall's oil and fuel connections land
    # on the engine's unions past its anti-ice duct. The bay's cooling air
    # is bled from the duct and leaves through a hole in the skin. The
    # boundary-layer diverter is let into the skin over the stringers, and
    # the windscreen bow into the structure at its own station. A grip is
    # on the stick it grips.
    #
    # The bottle and the engine's variable-vane actuation share the bay
    # annulus, which is seven millimetres between the casing and the skin.
    # That is what an engine bay measures on an aircraft this size.
    # ----------------------------------------------------------------
    ("former", "bay_fire_bottle"), ("former", "bay_mount_links_fwd"),
    ("console_", "fcs_loom_trunk_"), ("canopy_glass", "fcs_loom_trunk_"),
    ("cockpit_tub", "fcs_loom_trunk_"), ("fuel_lines", "fcs_loom_trunk_"),
    ("frame_canopy_seal", "canopy_windscreen_glass"),
    ("canopy_frame", "canopy_windscreen_glass"),
    ("frame_windscreen_bow", "canopy_windscreen_glass"),
    ("frame_windscreen_bow", "canopy_glass"),
    ("frame_canopy_breaker_cord", "canopy_glass"),
    ("stringer", "frame_windscreen_bow"),
    ("bay_fire_loop", "engine_"), ("bay_oil_connections", "engine_"),
    ("bay_fuel_connections", "engine_"),
    ("gear_bay_lining_main_", "gear_bay_main_"),
    ("bl_diverter", "stringer"),
    ("bay_mount_links_fwd", "engine_casing_"),
    ("bay_mount_links_fwd", "engine_fan_containment"),
    ("bay_mount_links_fwd", "engine_flange_"),
    ("bay_cooling_exit", "fuselage_skin"),
    ("bay_cooling_inlet", "duct_coupling"),
    ("control_stick", "control_stick_hotas"),
    ("bay_fire_bottle", "engine_variable_vane_actuation"),

    # The all-moving tail's pivot. The shaft runs through the fuselage skin
    # into the stabilator's root rib, the arm is clamped on its inboard end,
    # and the pushrod passes through a hole in the doubler on its way forward.
    ("stab_pivot_", "fuselage_skin"), ("stab_pivot_", "stabilator_"),

    # internal structure, inside the skin, which is the point of it
    ("former_", "fuselage_skin"),
    ("bhd_", "fuselage_skin"),
    ("seam_ring", "fuselage_skin"),
    ("seam_lengthwise", "fuselage_skin"),
    ("doublers", "fuselage_skin"),
    ("panel_", "doublers"),
    # the canopy's sill is bonded to the frame it lands on, and crosses the
    # skin's production seam at that station
    ("canopy_glass", "stringer"),
    ("canopy_glass", "seam_"), ("canopy_glass", "canopy_frame"),
    ("rib_", "wing_"),
    ("spar_", "wing_"),
    ("spar_", "fuselage_skin"),
    ("fin_rib", "vtail_fin"),
    ("hinge_", "wing_"),
    ("hinge_", "rudder"),

    # The wing panels start at the fuselage side now, so they no longer run
    # through the body, the tanks in it, or each other. Only the skin joint
    # remains, which is the joint.
    ("wing_", "fuselage_skin"),

    # A longeron runs the whole length of the fuselage, so everything that
    # crosses the fuselage crosses it: the wing carry-through and the engine
    # mount rails both do. On a real airframe the longeron is notched for
    # them, or they are bonded to its face -- either way the joint is where
    # the two share material.
    ("longeron", "former_"),
    ("longeron", "bhd_"),
    ("stringer", "former_"),
    ("stringer", "wing_"),
    # and the wing root ribs reach into the fuselage for the same reason the
    # panels do -- they are lofted to the centreline
    ("rib_", "fuselage_skin"),

    # the intake duct passes through the frames it is carried by
    ("duct_inlet", "bhd_"),
    # the duct's own hardware: the hoops it is carried on, the bond line down
    # its flanks and the flange at the engine end are all part of the duct,
    # and they pass through the same frame it does
    ("duct_frames", "duct_inlet"), ("duct_seam", "duct_inlet"),
    ("duct_coupling", "duct_inlet"),
    ("duct_frames", "former_"), ("duct_frames", "bhd_"),
    ("duct_frames", "stringer"), ("duct_frames", "longeron"),
    ("duct_frames", "seam_"), ("duct_frames", "fuselage_skin"),
    ("duct_seam", "former_"), ("duct_seam", "bhd_"),
    ("duct_seam", "stringer"), ("duct_seam", "longeron"),
    ("duct_seam", "fuselage_skin"),
    ("duct_coupling", "bhd_"),
    ("duct_coupling", "longeron"),
    ("duct_seam", "engine_"),
    ("duct_coupling", "duct_seam"),
    # the retracts straddle the duct and the diverter stands on the lip
    ("duct_frames", "bl_diverter"),
    ("duct_seam", "fuel_"),
    ("duct_coupling", "fuel_"), ("duct_coupling", "cooling_exit"),
    ("intake_lip", "duct_inlet"),
    ("bypass_slots", "fuselage_skin"),

    # the canopy sits in its cutout; the transparency is in the frame
    ("canopy_glass", "fuselage_skin"),
    ("canopy_glass", "canopy_frame"),
    ("canopy_frame", "fuselage_skin"),

    # engine installation: the tube is clamped to the rails that carry it
    # the tailpipe passes through the tail bulkhead, which is an aperture
    ("engine_", "mount_"),
    # and the turbofan's own interlocks, which are declared upstream too
    ("engine_", "engine_"),

    # systems in their trays and bays
    ("fuel_", "former_"),
    ("fuel_", "fuel_"),
    ("gear_", "wheel_"),
    ("gear_", "fuselage_skin"),
    ("gear_door", "fuselage_skin"),
    ("brake_", "gear_main"),
    ("brake_", "wheel_"),

    # control surfaces meet the surfaces they hinge from
    ("flaperon_", "wing_"),
    ("rudder", "vtail_fin"),
    ("stabilator_", "fuselage_skin"),
    ("ventral_", "fuselage_skin"),
    ("wing_strake_", "fuselage_skin"),
    ("nose_strakes", "fuselage_skin"),
    ("vg_", "wing_"),

    # detail fitted into the skin it sits in
    ("navlight_", "wing_"),
    ("navlight_", "vtail_fin"),
    ("pitot", "fuselage_skin"),
    ("antenna", "fuselage_skin"),
    ("antennas", "fuselage_skin"),

    # ------------------------------------------------------------------
    # The cockpit, which is one assembly pretending to be twenty-two parts.
    #
    # Two things make almost all of this unavoidable. The first is that this
    # test runs on the geometry layer, which is pure Python and knows nothing
    # about booleans: the cockpit aperture is cut from the skin, the formers
    # and the two upper stringers by a modifier in assemble.py, so what is
    # tested here is the frame before the hole is in it. Every part under the
    # canopy is therefore inside the skin, and the ones abreast of a former
    # are inside that too. The second is that a cockpit is nested by nature: a
    # pilot sits in a seat, his hands are on the controls, the helmet is on
    # his head and the mask is clipped to the helmet. Each of those overlaps
    # IS the assembly.
    # ------------------------------------------------------------------
    ("cockpit_tub", "fuselage_skin"), ("cockpit_tub", "former_"),
    ("cockpit_tub", "bhd_"), ("cockpit_tub", "stringer"),
    # the windscreen frame lands on the tub edge, which is the sill
    ("cockpit_tub", "canopy_"),
    ("console_", "cockpit_tub"), ("console_", "fuselage_skin"),
    ("console_", "former_"), ("console_", "stringer"),
    ("instrument_panel", "fuselage_skin"),
    ("instrument_panel", "coaming"),
    ("instrument_panel", "panel_instruments"),
    ("instrument_panel", "seam_"),
    ("panel_instruments", "coaming"),
    ("panel_instruments", "seam_"),
    ("coaming", "canopy_glass"),
    ("hud_glass", "hud_frame"), ("hud_", "coaming"),
    # the seat: pan, back, headbox, rails and harness are one seat
    ("seat_", "seat_"),
    ("seat_", "cockpit_tub"), ("seat_", "seam_"),
    ("seat_", "console_"),
    ("ejection_handle", "seam_"),
    ("rudder_pedals", "cockpit_tub"),
    ("control_stick", "console_"),
    ("throttle_lever", "console_"),
    # the pilot, who is in the seat with his hands on the controls
    ("pilot_", "pilot_"),
    ("pilot_", "seat_"), ("pilot_", "fuselage_skin"),
    ("pilot_", "console_"),
    ("pilot_", "control_stick"), ("pilot_", "throttle_lever"),
    ("pilot_", "seam_"),
    ("bl_diverter", "fuselage_skin"),
    ("tailpipe_shroud", "fuselage_skin"),
    ("naca_inlet_", "fuselage_skin"),
    ("cooling_exit_", "fuselage_skin"),
    ("canopy_latch_", "canopy_frame"),
    ("wheel_hub_", "wheel_"),

    # ------------------------------------------------------------------
    # Joints the check could not reach until it stopped spending its
    # budget on the joints it had already been told about. An airframe is
    # a lattice -- stringers notch into formers, formers sit on longerons,
    # every box in the bay is screwed to one of them -- so most of what
    # follows is that lattice being written down.
    # ------------------------------------------------------------------

    # frame and stringer interlocks
    ("stringer", "longeron"), ("stringer", "bhd_"),
    ("stringer", "duct_inlet"),
    ("longeron", "duct_inlet"),
    ("former_", "wing_"), ("former_", "canopy_frame"),
    ("former_", "cooling_exit_"), ("former_", "ventral_"),
    ("former_", "antenna"),
    ("former_", "gear_"),
    ("former_", "naca_inlet_"),
    ("bhd_", "wing_"),
    ("bhd_", "cooling_exit_"),
    ("mount_ring", "mount_rails"), ("mount_rails", "ventral_"),
    ("ventral_", "mount_rails"),

    # skin relief: the seams cross each other, the panels sit between them
    # and the doublers are under both
    ("seam_ring", "seam_lengthwise"), ("seam_ring", "doublers"),
    ("seam_ring", "panel_"), ("seam_ring", "wing_"),
    ("seam_lengthwise", "wing_"),
    ("seam_lengthwise", "duct_inlet"),
    ("panel_", "seam_lengthwise"),
    ("intake_lip", "fuselage_skin"),
    ("vtail_fin", "fuselage_skin"),
    ("intake_lip", "bl_diverter"),

    # the wing passes through the fuselage, and the gear folds into it
    ("bhd_spar", "wing_"), ("gear_door", "wing_"),
    ("spar_", "rib_"),

    # equipment is bolted to the frame it sits on
    ("fuel_", "bhd_"),
    ("cooling_exit_", "mount_ring"),

    # control runs land on what they move
    ("hinge_rudder", "vtail_fin"),
    ("navlight_tail", "hinge_"),
    ("engine_", "ventral_"),

    # canopy, tail and skin joints that the wider net reached
    ("canopy_glass", "former_"), ("canopy_glass", "bhd_"),
    ("canopy_glass", "seam_lengthwise"), ("canopy_latch_", "former_"),
    ("former_", "bypass_slots"), ("tailpipe_", "bypass_slots"),
    ("wing_seams", "fuel_tank"),
    ("gear_door", "rib_"), ("gear_door", "stringer"),
    # a door hinges to the structure behind it, which is what a hinge is
    ("gear_door", "bhd_"), ("gear_door", "former_"),
    ("naca_inlet_", "wing_"),
    ("former_", "mount_rails"),

    # Equipment against the frame it is screwed to, and through the
    # lightening holes it passes. In a 56 mm fuselage every box touches a
    # longeron, a stringer or a former -- that contact IS the mounting, and
    # a former is mostly hole by area.
    ("fuel_pump", "longeron"), ("fuel_filter", "longeron"),
    ("fuel_lines", "longeron"), ("fuel_lines", "former_"),
    ("fuel_lines", "fuel_tank"), ("fuel_lines", "stringer"),

    # an aileron servo lives in the wing, bolted to a rib and the joiner
    ("former_", "vtail_fin"),
    ("seam_lengthwise", "doublers"),
    ("fuel_lines", "mount_ring"),
    ("naca_inlet_", "fuel_tank"),
    ("cooling_exit_", "seam_ring"),
    ("seat_back", "seat_pan"),
    ("canopy_glass", "seat_back"),
    # the loom is clipped along the frame and plugs into every box in the
    # bay, so it shares material with all of them by construction
    ("cooling_exit_", "stringer"),
    ("longeron", "gear_door_actuator_"),
    # Joints made while closing the assembly, each of which IS the joint:
    # the retract jack's hoses run up into the bay's hydraulic lines, and the
    # rudder ram's rod-end lands on the rudder's balance horn.
    ("gear_main_retract_actuator_", "gear_bay_hydraulic_lines_"),
    ("rudder_servo", "rudder"),
    # The variable nozzle is one mechanism and its parts are assembled
    # through each other: the unison ring rides on the augmentor case, the
    # six actuators bolt to the case and drive the ring, the links run from
    # the ring to the flaps, the convergent flaps hinge on the case's aft
    # flange, and each seal bridges the gap between two flaps -- which is
    # the only thing a nozzle seal is for.
    #
    # None of this showed until the flaps stopped being 28-vertex facets.
    # A 0.3-unit hinge overlap has to have a vertex in it to be found.
    ("nozzle_unison_ring", "engine_casing_augmentor"),
    ("nozzle_actuator_", "engine_casing_augmentor"),
    ("nozzle_actuator_", "nozzle_unison_ring"),
    ("nozzle_link_", "engine_casing_augmentor"),
    ("nozzle_link_", "nozzle_unison_ring"),
    ("nozzle_external_flap_", "engine_casing_augmentor"),
    ("nozzle_flap_", "nozzle_seal_"),
    ("nozzle_link_", "nozzle_flap_"),
    ("nozzle_link_", "engine_flange_aug_aft"),
    # The tail frame is a ring round the engine in the tailcone, so the
    # cooling slots in that tailcone and the mount rails bolted to the frames
    # pass through it -- which is what a frame is for.
    ("bhd_tail", "bypass_slots"),
    ("bhd_tail", "bay_doors"),
    ("bhd_tail", "ventral_"),   # the ventral fins bolt to this frame
    # the stabilator servo drives the pivot it is built on
    ("stab_servo_", "stab_pivot_"),
]

PKG = "plane/parts"
UNIT = 33.0           # mm of full-size aeroplane per drawing unit (spec.SCALE_TO_FULL)
TOL = 0.3            # mm, full size: deeper than this is sharing material

# Real defects, in mm of full-size overlap, deepest first. Each one is a part
# through a part that nobody meant. Fix them and --shrink; never add to it.
# --- KNOWN: rewritten by --shrink, never by hand to add ---
KNOWN = {
    ("gear_main_l", "wing_l"): 275.9,   # at (248.7, -41.4, -6.3)
    ("gear_main_r", "wing_r"): 275.9,   # at (248.7, 41.4, -6.3)
    ("brake_l", "wing_l"): 262.1,   # at (251.8, -43.9, -6.0)
    ("brake_r", "wing_r"): 262.1,   # at (251.8, 43.9, -6.0)
    ("gear_main_retract_actuator_l", "wing_l"): 249.5,   # at (257.9, -51.6, -6.3)
    ("gear_main_retract_actuator_r", "wing_r"): 249.5,   # at (257.9, 51.6, -6.3)
    ("bl_diverter", "gear_nose_strut"): 239.0,   # at (45.1, -1.3, -14.2)
    ("gear_main_side_stay_l", "wing_l"): 228.5,   # at (250.2, -64.1, -6.1)
    ("gear_main_side_stay_r", "wing_r"): 228.5,   # at (250.2, 64.1, -6.1)
    ("gear_bay_main_l", "rib_l_02"): 223.0,   # at (237.1, -49.0, -2.0)
    ("gear_bay_main_r", "rib_r_02"): 223.0,   # at (237.1, 49.0, -2.0)
    ("gear_bay_lining_main_l", "rib_l_02"): 221.7,   # at (242.8, -49.0, -2.8)
    ("gear_bay_lining_main_r", "rib_r_02"): 221.7,   # at (242.8, 49.0, -2.8)
    ("gear_bay_lining_main_l", "rib_l_03"): 212.0,   # at (243.2, -49.0, -2.8)
    ("gear_bay_lining_main_r", "rib_r_03"): 212.0,   # at (243.2, 49.0, -2.8)
    ("gear_bay_main_l", "rib_l_03"): 190.8,   # at (237.5, -49.0, -2.0)
    ("gear_bay_main_r", "rib_r_03"): 190.8,   # at (237.5, 49.0, -2.0)
    ("gear_bay_hydraulic_lines_main_l", "wing_l"): 175.6,   # at (238.2, -48.4, -3.3)
    ("gear_bay_hydraulic_lines_main_r", "wing_r"): 175.6,   # at (238.2, 48.4, -3.3)
    ("gear_main_side_stay_l", "rib_l_03"): 156.5,   # at (252.3, -57.3, -11.0)
    ("gear_main_side_stay_r", "rib_r_03"): 156.5,   # at (252.3, 57.3, -11.0)
    ("gear_door_actuator_main_l", "gear_main_side_stay_l"): 149.5,   # at (252.1, -60.4, -7.7)
    ("gear_door_actuator_main_r", "gear_main_side_stay_r"): 149.5,   # at (252.1, 60.4, -7.7)
    ("rudder_servo", "seam_lengthwise"): 149.2,   # at (393.9, 0.0, 20.6)
    ("gear_bay_main_l", "rib_l_01"): 148.2,   # at (252.0, -49.0, -2.3)
    ("gear_bay_main_r", "rib_r_01"): 148.2,   # at (252.0, 49.0, -2.3)
    ("gear_main_downlock_l", "rib_l_03"): 139.7,   # at (253.0, -60.9, -9.6)
    ("gear_main_downlock_r", "rib_r_03"): 139.7,   # at (253.0, 60.9, -9.6)
    ("gear_door_actuator_main_l", "gear_main_downlock_l"): 136.8,   # at (252.2, -60.4, -7.7)
    ("gear_door_actuator_main_r", "gear_main_downlock_r"): 136.8,   # at (252.2, 60.4, -7.7)
    ("gear_bay_main_l", "spar_carbon"): 133.1,   # at (252.0, -67.0, -5.9)
    ("gear_bay_main_r", "spar_carbon"): 133.1,   # at (252.0, 67.0, -5.9)
    ("gear_bay_lining_main_l", "wing_l"): 120.3,   # at (243.0, -66.3, -2.9)
    ("gear_bay_lining_main_r", "wing_r"): 120.3,   # at (243.0, 66.3, -2.9)
    ("bay_cooling_inlet", "bhd_firewall"): 118.8,   # at (302.8, -17.4, 14.4)
    ("gear_bay_nose", "longeron_3"): 111.0,   # at (36.0, -6.0, -1.4)
    ("gear_bay_nose", "longeron_4"): 111.0,   # at (36.0, 6.0, -1.4)
    ("gear_bay_main_l", "wing_l"): 102.1,   # at (266.8, -66.4, -9.8)
    ("gear_bay_main_r", "wing_r"): 102.1,   # at (266.8, 66.4, -9.8)
    ("bl_diverter", "former_03"): 91.1,   # at (89.4, 17.9, -13.0)
    ("bhd_nose", "bl_diverter"): 89.6,   # at (65.0, 9.9, -13.8)
    ("bhd_cockpit", "canopy_frame"): 88.4,   # at (149.0, 11.5, 21.0)
    ("rib_l_01", "seam_lengthwise"): 75.3,   # at (179.0, -31.9, -5.6)
    ("rib_r_01", "seam_lengthwise"): 75.3,   # at (179.0, 31.9, -5.6)
    ("gear_bay_lining_nose", "longeron_3"): 73.6,   # at (38.2, -5.5, -1.7)
    ("gear_bay_lining_nose", "longeron_4"): 73.6,   # at (38.2, 5.5, -1.7)
    ("gear_nose_drag_stay", "seam_lengthwise"): 73.0,   # at (31.6, -0.1, -8.0)
    ("bay_doors", "former_11"): 72.6,   # at (316.0, -2.6, -27.2)
    ("fcs_loom_tail_l", "mount_ring"): 70.1,   # at (310.0, -5.6, -23.2)
    ("fcs_loom_tail_r", "mount_ring"): 70.1,   # at (310.0, 5.6, -23.2)
    ("bay_doors", "stab_servo_l"): 69.3,   # at (400.8, -11.4, -20.4)
    ("bay_doors", "stab_servo_r"): 69.3,   # at (400.8, 11.4, -20.4)
    ("bhd_tail", "rudder_servo"): 68.6,   # at (382.0, 5.0, 21.3)
    ("gear_main_l", "rib_l_02"): 67.9,   # at (253.9, -45.5, -17.7)
    ("gear_main_r", "rib_r_02"): 67.9,   # at (253.9, 45.5, -17.7)
    ("gear_bay_lining_main_l", "spar_carbon"): 66.8,   # at (248.8, -66.3, -6.4)
    ("gear_bay_lining_main_r", "spar_carbon"): 66.8,   # at (248.8, 66.3, -6.4)
    ("bay_bleed_offtake", "bay_mount_links_fwd"): 63.9,   # at (334.0, 15.2, 13.5)
    ("bay_fire_bottle", "bhd_tail"): 62.2,   # at (377.0, 21.3, 1.1)
    ("antennas", "panel_battery"): 59.4,   # at (262.0, 2.4, 27.8)
    ("gear_nose_chrome_slider", "gear_nose_shimmy_damper"): 58.8,   # at (40.1, -1.6, -34.7)
    ("gear_bay_nose", "longeron_1"): 57.4,   # at (36.0, 6.0, 2.1)
    ("gear_bay_nose", "longeron_2"): 57.4,   # at (36.0, -6.0, 2.1)
    ("bay_fire_loop", "ventral_l"): 56.4,   # at (385.5, -15.2, -11.0)
    ("bay_fire_loop", "ventral_r"): 56.4,   # at (385.5, 15.2, -11.0)
    ("former_12", "rudder_servo"): 55.9,   # at (344.8, -1.6, 23.0)
    ("former_13", "rudder_servo"): 55.3,   # at (371.2, 1.4, 22.1)
    ("former_11", "rudder_servo"): 55.1,   # at (316.8, -1.9, 24.0)
    ("doublers", "vtail_fin"): 53.3,   # at (342.0, 1.3, 25.6)
    ("bl_diverter", "gear_nose_drag_stay"): 53.1,   # at (43.5, 0.1, -11.6)
    ("fin_tip_ecm_fairing", "navlight_tail"): 51.9,   # at (427.7, 0.0, 98.4)
    ("bhd_tail", "seam_lengthwise"): 51.1,   # at (382.0, 21.6, -0.2)
    ("intake_lip", "stringer_09"): 50.2,   # at (60.3, -7.8, -13.7)
    ("intake_lip", "stringer_10"): 50.2,   # at (60.3, 7.8, -13.7)
    ("bay_doors", "stabilator_l"): 45.2,   # at (400.8, -15.4, -17.2)
    ("bay_doors", "stabilator_r"): 45.1,   # at (400.8, 15.4, -17.2)
    ("engine_inlet_case", "wing_seams"): 44.3,   # at (306.4, -16.4, 4.7)
    ("engine_gearbox", "fcs_loom_tail_l"): 42.3,   # at (333.4, -5.9, -22.3)
    ("canopy_glass", "frame_canopy_rear_hinge"): 42.1,   # at (163.2, 4.1, 22.4)
    ("fcs_fcc_envelope", "fcs_loom_trunk_lower_l"): 41.3,   # at (130.8, 0.0, -3.8)
    ("fcs_fcc_envelope", "fcs_loom_trunk_lower_r"): 41.3,   # at (130.8, 0.0, -3.8)
    ("gear_bay_main_l", "seam_ring_06"): 40.6,   # at (252.0, -49.0, -2.3)
    ("gear_bay_main_r", "seam_ring_06"): 40.6,   # at (252.0, 49.0, -2.3)
    ("engine_casing_inlet", "wing_seams"): 39.8,   # at (306.4, -17.0, 5.0)
    ("bay_mount_links_fwd", "engine_variable_vane_actuation"): 39.6,   # at (320.0, 17.6, 7.5)
    ("bay_mount_links_fwd", "engine_mount_trunnions"): 38.5,   # at (334.0, -10.4, 13.5)
    ("control_stick_boots", "rudder_pedals"): 36.1,   # at (110.7, 3.6, 8.0)
    ("gear_bay_nose", "stringer_08"): 35.0,   # at (36.0, -6.0, -4.9)
    ("gear_bay_nose", "stringer_11"): 35.0,   # at (36.0, 6.0, -4.9)
    ("duct_frames", "duct_seam"): 34.1,   # at (275.6, 20.5, -5.3)
    ("fcs_fcc_envelope", "fcs_loom_trunk_upper_l"): 33.8,   # at (129.7, -7.1, -1.1)
    ("fcs_fcc_envelope", "fcs_loom_trunk_upper_r"): 33.7,   # at (130.8, 0.0, -3.8)
    ("antennas", "seam_lengthwise"): 32.9,   # at (249.4, 0.0, 28.7)
    ("gear_main_l", "gear_main_retract_actuator_l"): 32.4,   # at (253.5, -45.9, -17.7)
    ("gear_main_r", "gear_main_retract_actuator_r"): 32.4,   # at (253.4, 46.0, -17.7)
    ("fuel_lines", "wing_seams"): 31.8,   # at (212.6, -21.4, 5.2)
    ("bay_doors", "ventral_l"): 31.2,   # at (379.0, -17.8, -19.5)
    ("bay_doors", "ventral_r"): 31.0,   # at (379.0, 18.0, -19.7)
    ("fuselage_skin", "stab_servo_l"): 30.2,   # at (400.0, -11.2, -20.6)
    ("fuselage_skin", "stab_servo_r"): 30.2,   # at (400.0, 11.2, -20.6)
    ("control_stick_boots", "rudder_pedals_rail"): 29.7,   # at (110.7, 3.6, 8.0)
    ("bay_mount_links_fwd", "engine_fan_door_hardware"): 29.1,   # at (328.8, -13.6, 11.2)
    ("bay_cooling_inlet", "fcs_loom_trunk_lower_l"): 28.9,   # at (303.2, -17.1, 19.5)
    ("bhd_nose", "retract_nose"): 28.8,   # at (65.0, -6.2, -3.6)
    ("bl_diverter", "seam_lengthwise"): 28.8,   # at (43.5, 0.1, -11.6)
    ("canopy_frame", "frame_canopy_seal"): 28.4,   # at (141.1, -13.9, 22.2)
    ("frame_canopy_rear_hinge", "frame_canopy_seal"): 28.4,   # at (164.4, -4.2, 22.0)
    ("bay_fire_bottle", "engine_casing_bypass"): 27.8,   # at (344.1, 16.4, 3.9)
    ("throttle_lever", "throttle_lever_hotas"): 27.5,   # at (112.0, -8.6, 28.5)
    ("antennas", "seam_ring_06"): 26.1,   # at (262.0, -2.1, 28.7)
    ("seam_ring_09", "ventral_r"): 26.0,   # at (383.0, 18.8, -18.2)
    ("seam_ring_09", "ventral_l"): 25.9,   # at (383.0, -18.8, -18.2)
    ("former_03", "frame_windscreen_bow"): 24.3,   # at (88.0, -8.0, 21.7)
    ("canopy_frame", "frame_windscreen_bow"): 24.0,   # at (80.3, 2.4, 22.3)
    ("cockpit_tub", "frame_canopy_seal"): 24.0,   # at (152.0, 0.0, 9.6)
    ("bay_fire_bottle", "bypass_slots"): 23.6,   # at (343.5, 24.5, 3.7)
    ("bay_bleed_offtake", "engine_flange_inlet"): 23.1,   # at (320.0, 13.5, 14.2)
    ("gear_nose_strut", "seam_ring_01"): 22.7,   # at (42.0, 1.3, -23.3)
    ("fuel_pump", "stringer_01"): 21.9,   # at (154.1, 26.5, 12.2)
    ("bl_diverter", "duct_inlet"): 20.5,   # at (54.0, 0.0, -13.5)
    ("bay_bleed_offtake", "engine_variable_vane_actuation"): 20.0,   # at (320.0, 14.1, 11.8)
    ("gear_bay_hydraulic_lines_nose", "longeron_4"): 19.8,   # at (39.0, 3.7, -1.8)
    ("canopy_windscreen_glass", "seam_lengthwise"): 19.6,   # at (78.4, 0.0, 23.5)
    ("canopy_frame", "frame_canopy_breaker_cord"): 18.6,   # at (133.2, -14.8, 22.9)
    ("canopy_windscreen_glass", "former_03"): 18.6,   # at (88.1, 7.8, 22.3)
    ("naca_inlet_l", "seam_ring_06"): 18.0,   # at (251.4, -29.8, 15.5)
    ("naca_inlet_r", "seam_ring_06"): 18.0,   # at (251.4, 29.8, 15.5)
    ("bay_bleed_offtake", "bay_fire_loop"): 17.7,   # at (332.0, 16.8, 14.4)
    ("fcs_loom_trunk_upper_l", "seat_bucket"): 16.2,   # at (133.6, -7.0, 6.5)
    ("fcs_loom_trunk_upper_r", "seat_bucket"): 16.2,   # at (133.6, 7.0, 6.5)
    ("bay_bleed_offtake", "engine_fan_containment"): 15.4,   # at (320.0, 13.5, 14.2)
    ("cockpit_tub", "frame_windscreen_bow"): 15.3,   # at (95.9, 11.9, 21.8)
    ("bhd_cockpit", "frame_canopy_seal"): 14.1,   # at (151.0, 10.6, 21.2)
    ("bay_bleed_offtake", "engine_casing_fan"): 13.8,   # at (320.0, 13.5, 14.2)
    ("bay_bleed_offtake", "engine_casing_inlet"): 13.8,   # at (320.0, 13.5, 14.2)
    ("nose_strakes", "seam_ring_03"): 13.0,   # at (98.9, -28.5, 10.1)
    ("engine_flange_aug_aft", "nozzle_external_flap_00"): 12.7,   # at (423.3, 3.1, 12.3)
    ("engine_flange_aug_aft", "nozzle_external_flap_03"): 12.7,   # at (423.3, 13.0, -5.0)
    ("engine_flange_aug_aft", "nozzle_external_flap_05"): 12.7,   # at (423.3, 3.1, -14.3)
    ("engine_flange_aug_aft", "nozzle_external_flap_06"): 12.7,   # at (423.3, -3.1, -14.3)
    ("engine_flange_aug_aft", "nozzle_external_flap_08"): 12.7,   # at (423.3, -13.0, -5.0)
    ("engine_flange_aug_aft", "nozzle_external_flap_11"): 12.7,   # at (423.3, -3.1, 12.3)
    ("gear_bay_hydraulic_lines_nose", "gear_nose_strut"): 12.7,   # at (39.0, 2.3, -1.8)
    ("engine_flange_aug_aft", "nozzle_external_flap_01"): 12.5,   # at (423.3, 11.6, 6.2)
    ("engine_flange_aug_aft", "nozzle_external_flap_02"): 12.5,   # at (423.3, 12.0, 5.4)
    ("engine_flange_aug_aft", "nozzle_external_flap_04"): 12.5,   # at (423.3, 7.2, -12.6)
    ("engine_flange_aug_aft", "nozzle_external_flap_07"): 12.5,   # at (423.3, -11.6, -8.2)
    ("engine_flange_aug_aft", "nozzle_external_flap_09"): 12.5,   # at (423.3, -12.0, 5.4)
    ("engine_flange_aug_aft", "nozzle_external_flap_10"): 12.5,   # at (423.3, -7.2, 10.6)
    ("canopy_glass", "frame_canopy_seal"): 12.0,   # at (166.5, 1.9, 22.4)
    ("fcs_loom_trunk_lower_l", "fcs_loom_trunk_upper_l"): 10.6,   # at (131.7, -8.1, -1.9)
    ("fcs_loom_trunk_lower_r", "fcs_loom_trunk_upper_r"): 10.6,   # at (131.7, 8.1, -1.9)
    ("frame_canopy_seal", "frame_windscreen_bow"): 9.1,   # at (95.8, 11.2, 22.3)
    ("canopy_windscreen_glass", "stringer_03"): 8.9,   # at (94.9, 10.9, 22.3)
    ("canopy_windscreen_glass", "stringer_04"): 8.9,   # at (94.9, -10.9, 22.3)
    ("gear_bay_nose", "gear_door_actuator_nose_l"): 8.3,   # at (36.0, 0.0, 1.7)
    ("gear_bay_nose", "gear_door_actuator_nose_r"): 8.3,   # at (36.0, 0.0, 1.7)
    ("bay_bleed_offtake", "engine_vbv_doors"): 7.1,   # at (331.9, 14.4, 11.5)
    ("bay_mount_links_fwd", "engine_vanes_fan_ogv"): 6.8,   # at (331.4, 12.0, 11.6)
    ("bay_mount_links_fwd", "engine_vanes_fan_s2"): 6.8,   # at (327.8, -13.5, 10.3)
    ("frame_canopy_seal", "stringer_03"): 6.8,   # at (96.1, 10.8, 22.8)
    ("frame_canopy_seal", "stringer_04"): 6.8,   # at (96.1, -10.8, 22.8)
    ("bhd_tail", "seam_ring_09"): 6.7,   # at (383.0, 0.0, 23.7)
    ("nozzle_external_flap_00", "nozzle_link_00"): 5.9,   # at (426.0, 0.5, 11.4)
    ("nozzle_external_flap_00", "nozzle_link_01"): 5.9,   # at (426.0, 5.8, 10.0)
    ("nozzle_external_flap_01", "nozzle_link_01"): 5.9,   # at (426.0, 6.6, 9.5)
    ("nozzle_external_flap_01", "nozzle_link_02"): 5.9,   # at (426.0, 10.5, 5.6)
    ("nozzle_external_flap_02", "nozzle_link_02"): 5.9,   # at (426.0, 11.0, 4.8)
    ("nozzle_external_flap_02", "nozzle_link_03"): 5.9,   # at (426.0, 12.4, -0.5)
    ("nozzle_external_flap_03", "nozzle_link_03"): 5.9,   # at (426.0, 12.4, -1.5)
    ("nozzle_external_flap_03", "nozzle_link_04"): 5.9,   # at (426.0, 11.0, -6.8)
    ("nozzle_external_flap_04", "nozzle_link_04"): 5.9,   # at (426.0, 10.5, -7.6)
    ("nozzle_external_flap_04", "nozzle_link_05"): 5.9,   # at (426.0, 6.6, -11.5)
    ("nozzle_external_flap_05", "nozzle_link_05"): 5.9,   # at (426.0, 5.8, -12.0)
    ("nozzle_external_flap_05", "nozzle_link_06"): 5.9,   # at (426.0, 0.5, -13.4)
    ("nozzle_external_flap_06", "nozzle_link_06"): 5.9,   # at (426.0, -0.5, -13.4)
    ("nozzle_external_flap_06", "nozzle_link_07"): 5.9,   # at (426.0, -5.8, -12.0)
    ("nozzle_external_flap_07", "nozzle_link_07"): 5.9,   # at (426.0, -6.6, -11.5)
    ("nozzle_external_flap_07", "nozzle_link_08"): 5.9,   # at (426.0, -10.5, -7.6)
    ("nozzle_external_flap_08", "nozzle_link_08"): 5.9,   # at (426.0, -11.0, -6.8)
    ("nozzle_external_flap_08", "nozzle_link_09"): 5.9,   # at (426.0, -12.4, -1.5)
    ("nozzle_external_flap_09", "nozzle_link_09"): 5.9,   # at (426.0, -12.4, -0.5)
    ("nozzle_external_flap_09", "nozzle_link_10"): 5.9,   # at (426.0, -11.0, 4.8)
    ("nozzle_external_flap_10", "nozzle_link_10"): 5.9,   # at (426.0, -10.5, 5.6)
    ("nozzle_external_flap_10", "nozzle_link_11"): 5.9,   # at (426.0, -6.6, 9.5)
    ("nozzle_external_flap_11", "nozzle_link_00"): 5.9,   # at (426.0, -0.5, 11.4)
    ("nozzle_external_flap_11", "nozzle_link_11"): 5.9,   # at (426.0, -5.8, 10.0)
    ("stringer_08", "stringer_09"): 5.6,   # at (112.3, -21.1, -16.0)
    ("stringer_10", "stringer_11"): 5.6,   # at (112.1, 19.9, -15.9)
    ("bay_mount_links_fwd", "engine_blades_fan_r3"): 5.2,   # at (331.8, -11.6, 11.8)
    ("fcs_loom_tail_l", "former_13"): 5.2,   # at (372.9, -6.0, -21.1)
    ("hud_glass_combiner_1", "hud_glass_projector_lens"): 4.9,   # at (106.0, 0.0, 30.4)
    ("bay_fire_loop", "fcs_loom_tail_l"): 4.5,   # at (331.8, -5.7, -22.3)
    ("bay_fire_loop", "fcs_loom_tail_r"): 4.5,   # at (331.8, 5.7, -22.3)
    ("bay_fire_bottle", "fuselage_skin"): 4.4,   # at (352.5, 24.5, 4.0)
    ("gear_bay_main_l", "gear_door_actuator_main_l"): 4.3,   # at (252.0, -49.0, -2.3)
    ("gear_bay_main_r", "gear_door_actuator_main_r"): 4.3,   # at (252.0, 49.0, -2.3)
    ("canopy_frame", "seam_lengthwise"): 2.4,   # at (80.9, -2.6, 23.0)
    ("leading_edge_flap_l", "wing_l"): 1.7,   # at (291.5, -134.6, -5.8)
    ("leading_edge_flap_r", "wing_r"): 1.7,   # at (291.5, 134.6, -5.8)
}
# --- end KNOWN ---

if __name__ == "__main__":
    import _interfere
    sys.exit(_interfere.intersect_main(__file__, ROOT, PKG, EXPECTED, KNOWN,
                                       TOL, UNIT))
