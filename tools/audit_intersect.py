"""No part of the aircraft may occupy another part's space.

    python3 tools/audit_intersect.py
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
    # ----------------------------------------------------------------
    # A feature let into the skin lands on the frame member behind it. On a
    # built-up airframe the member is cut there and a doubler goes in, which
    # is the same statement as ("panel_", "doublers") above.
    # ----------------------------------------------------------------
    ("naca_inlet_", "stringer"), ("naca_inlet_", "longeron"),
    ("cooling_exit", "longeron"), ("cooling_exit", "stringer"),
    ("gear_door_", "longeron"), ("gear_door_", "stringer"),
    ("stringer", "canopy_frame"), ("longeron", "canopy_frame"),

    # A seam is a line ON a surface, so it meets whatever crosses that
    # surface -- vents, hatches, the fin root, the wing skin.
    ("seam_", "bypass_slots"), ("seam_", "vtail_fin"),
    ("wing_seams", "engine_antiice_duct"), ("seam_", "pushrod_linkages"),
    ("seam_", "retract_main_"), ("seam_", "nose_steering_link"),
    ("seam_", "battery_strap_"), ("seam_", "gear_door_"),
    ("duct_seam", "gear_door_"),

    # A strap is anchored to its tray and to the frame the tray is bonded to,
    # and shares the bay with whatever else is in it.
    ("battery_strap_", "fuselage_skin"), ("battery_strap_", "longeron"),
    ("battery_strap_", "former_"), ("battery_strap_", "stringer"),
    ("battery_strap_", "telemetry_gps"), ("battery_strap_", "seat_"),
    ("battery_strap_", "gear_door_"),

    # A piano hinge pin runs the whole span of the surface it carries, so
    # every rib it passes is notched for it.
    ("rib_", "hinge_flaperon_"), ("fin_rib", "hinge_rudder"),

    # The thrust tube is held off the frame on standoff mounts, and the
    # nozzle passes through the tail bulkhead.
    ("former_", "thrust_tube"), ("longeron", "thrust_tube"),
    ("engine_nozzle_", "bhd_tail"),

    # The wing root is where the wing passes through the fuselage side, so
    # everything mounted in it meets the skin and the frame there.
    ("retract_main_", "fuselage_skin"), ("retract_main_", "stringer"),
    ("retract_main_", "longeron"), ("servo_stab", "fuselage_skin"),
    ("servo_rudder", "fuselage_skin"), ("servo_stab", "stringer"),
    ("servo_rudder", "stringer"),

    # The intake duct owns the middle of the fuselage, so the tail servos,
    # their arms and the main gear retracts live in the wing root -- which is
    # 19.7 mm thick, empty, and where the leg they drive already is.
    ("servo_stab", "wing_"), ("servo_rudder", "wing_"),
    ("servo_arm_stab", "wing_"), ("servo_arm_rudder", "wing_"),
    ("retract_main_", "wing_"), ("wiring", "wing_"),
    ("servo_ail_", "wing_"), ("servo_arm_ail_", "wing_"),
    ("retract_main_", "rib_"), ("servo_stab", "rib_"), ("servo_rudder", "rib_"),
    ("servo_arm_", "rib_"), ("gear_door_actuator_", "wing_"),
    ("gear_door_actuator_", "rib_"), ("gear_door_actuator_", "retract_main_"),
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
    ("nose_steering_link", "gear_door_"),
    # A control run goes fore and aft through the frame, which is what the
    # lightening holes in a bulkhead are partly for -- see fuselage._bulkheads.
    ("nose_steering_link", "former_"), ("nose_steering_link", "longeron"),
    ("nose_steering_link", "bhd_"), ("pushrod_linkages", "bhd_"),
    ("pushrod_linkages", "former_"),
    # the loom lands on the engine's connector block and runs beside its case
    ("wiring", "engine_"), ("wiring", "mount_ring"), ("wiring", "mount_rails"),
    ("wiring", "fuel_lines"), ("fuel_lines", "avionics_tray"),
    # the loom runs along the inside of the skin, past the vents cut in it
    ("wiring", "cooling_exit"), ("wiring", "naca_inlet_"),
    # a control run goes through a hole in the rib it passes, and under the
    # access panel over it
    ("pushrod_linkages", "fin_rib"), ("pushrod_linkages", "panel_"),
    # the leads leave the fuselage through a grommet at the wing root, which
    # is the only way to reach servos that are in the wing
    ("wiring", "fuselage_skin"),
    ("retract_main_", "spar_"), ("servo_stab", "spar_"),
    ("servo_rudder", "spar_"),

    # A hinge is bolted to a rib at one end and to the surface it carries at
    # the other; a hatch is screwed down onto the skin it covers.
    ("flaperon_hinge_", "flaperon_"), ("flaperon_hinge_", "rib_"),
    ("flaperon_hinge_", "wing_"), ("servo_hatch_", "wing_"),

    # The all-moving tail's pivot. The shaft runs through the fuselage skin
    # into the stabilator's root rib, the arm is clamped on its inboard end,
    # and the pushrod passes through a hole in the doubler on its way forward.
    ("stab_pivot_", "fuselage_skin"), ("stab_pivot_", "stabilator_"),
    ("horn_s", "stab_pivot_"), ("horn_s", "fuselage_skin"),
    ("doublers", "pushrod_linkages"),

    # internal structure, inside the skin, which is the point of it
    ("former_", "fuselage_skin"),
    ("bhd_", "fuselage_skin"),
    ("longeron", "fuselage_skin"),
    ("stringer", "fuselage_skin"),
    ("seam_ring", "fuselage_skin"),
    ("seam_lengthwise", "fuselage_skin"),
    ("doublers", "fuselage_skin"),
    ("panel_", "doublers"),
    ("panel_screws", "fuselage_skin"), ("panel_screws", "panel_"),
    # the canopy's sill is bonded to the frame it lands on, and crosses the
    # skin's production seam at that station
    ("canopy_glass", "stringer"), ("canopy_glass", "longeron"),
    ("canopy_glass", "seam_"), ("canopy_glass", "canopy_frame"),
    ("rib_", "wing_"),
    ("spar_", "wing_"),
    ("spar_", "fuselage_skin"),
    ("fin_rib", "vtail_fin"),
    ("hinge_", "wing_"),
    ("hinge_", "rudder"),
    ("hinge_", "flaperon_"),
    ("wing_joiner", "spar_carbon"),
    ("wing_joiner", "fuselage_skin"),
    ("wing_bolt_", "wing_"),
    ("wing_bolt_", "fuselage_skin"),

    # The wing panels start at the fuselage side now, so they no longer run
    # through the body, the tanks in it, or each other. Only the skin joint
    # remains, which is the joint.
    ("wing_", "fuselage_skin"),

    # A longeron runs the whole length of the fuselage, so everything that
    # crosses the fuselage crosses it: the wing carry-through and the engine
    # mount rails both do. On a real airframe the longeron is notched for
    # them, or they are bonded to its face -- either way the joint is where
    # the two share material.
    ("longeron", "wing_"),
    ("longeron", "mount_rails"),
    ("longeron", "former_"),
    ("longeron", "bhd_"),
    ("stringer", "former_"),
    ("stringer", "wing_"),
    # and the wing root ribs reach into the fuselage for the same reason the
    # panels do -- they are lofted to the centreline
    ("rib_", "fuselage_skin"),
    ("rib_", "former_"),

    # the intake duct passes through the frames it is carried by
    ("duct_inlet", "fuselage_skin"),
    ("duct_inlet", "former_"),
    ("duct_inlet", "bhd_"),
    ("duct_inlet", "access_tray"),
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
    ("duct_seam", "seam_"), ("duct_seam", "fuselage_skin"),
    ("duct_coupling", "former_"), ("duct_coupling", "bhd_"),
    ("duct_coupling", "stringer"), ("duct_coupling", "longeron"),
    ("duct_coupling", "engine_"), ("duct_coupling", "mount_"),
    ("duct_seam", "engine_"),
    ("duct_coupling", "avionics_tray"), ("duct_coupling", "fuselage_skin"),
    ("duct_coupling", "duct_seam"), ("duct_coupling", "duct_frames"),
    # the retracts straddle the duct and the diverter stands on the lip
    ("duct_frames", "retract_"), ("duct_frames", "bl_diverter"),
    ("duct_frames", "wiring"), ("duct_frames", "battery_strap_"),
    ("duct_coupling", "servo_"),
    ("duct_seam", "wing_joiner"), ("duct_seam", "spar_"),
    ("duct_seam", "fuel_"), ("duct_frames", "fuel_"),
    ("duct_coupling", "wing_bolt_"), ("duct_coupling", "bellcranks"),
    ("duct_coupling", "fuel_"), ("duct_coupling", "cooling_exit"),
    ("intake_guard", "intake_lip"), ("intake_guard", "duct_inlet"),
    ("intake_guard", "fuselage_skin"),
    ("intake_lip", "duct_inlet"),
    ("intake_lip_ring", "intake_lip"),
    ("bypass_slots", "fuselage_skin"),

    # the canopy sits in its cutout; the transparency is in the frame
    ("canopy_glass", "fuselage_skin"),
    ("canopy_glass", "canopy_frame"),
    ("canopy_frame", "fuselage_skin"),

    # engine installation: the tube is clamped to the rails that carry it
    # the tailpipe passes through the tail bulkhead, which is an aperture
    ("tailpipe_shroud", "bhd_tail"), ("tailpipe_cone", "bhd_tail"),
    ("thrust_tube", "mount_rails"),
    ("thrust_tube", "mount_ring"),
    ("thrust_tube", "engine_"),
    ("tailpipe_cone", "tailpipe_shroud"),
    ("engine_", "mount_"),
    ("engine_", "former_"),
    ("engine_", "fuselage_skin"),
    # and the turbofan's own interlocks, which are declared upstream too
    ("engine_", "engine_"),

    # systems in their trays and bays
    ("lipo_3s", "battery_strap_lipo"),
    ("rx_battery", "battery_strap_rx"),
    ("battery_strap_", "avionics_tray"),
    ("battery_strap_", "access_tray"),
    ("servo_", "former_"),
    ("servo_arm_", "servo_"),
    ("servo_arm_", "pushrod_linkages"),
    ("pushrod_linkages", "former_"),
    ("pushrod_linkages", "bellcranks"),
    ("wiring", "former_"),
    ("fuel_", "former_"),
    ("fuel_", "fuel_"),
    ("access_tray", "duct_inlet"),
    ("access_tray", "former_"),
    ("avionics_tray", "former_"),
    ("retract_", "former_"),
    ("retract_", "gear_"),
    ("gear_", "wheel_"),
    ("gear_", "fuselage_skin"),
    ("gear_door", "fuselage_skin"),
    ("nose_steering_link", "gear_nose_strut"),
    ("nose_steering_link", "servo_"),
    ("brake_", "gear_main"),
    ("brake_", "wheel_"), ("brake_", "gear_door"),
    ("pneumatic", "former_"),
    ("air_trap", "former_"),

    # control surfaces meet the surfaces they hinge from
    ("flaperon_", "wing_"),
    ("rudder", "vtail_fin"),
    ("stabilator_", "fuselage_skin"),
    ("ventral_", "fuselage_skin"),
    ("wing_strake_", "wing_"),
    ("wing_strake_", "fuselage_skin"),
    ("nose_strakes", "fuselage_skin"),
    ("vg_", "wing_"),
    ("wing_fence_", "wing_"),
    ("horn_", "flaperon_"),
    ("horn_", "rudder"),
    ("horn_", "stabilator_"),
    ("clevis_", "horn_"),
    ("clevis_", "pushrod_linkages"),

    # detail fitted into the skin it sits in
    ("navlight_", "fuselage_skin"),
    ("navlight_", "wing_"),
    ("navlight_", "vtail_fin"),
    ("pitot", "fuselage_skin"),
    ("antenna", "fuselage_skin"),
    ("antennas", "fuselage_skin"),
    ("static_dischargers", "wing_"),
    ("static_dischargers", "vtail_fin"),

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
    ("cockpit_tub", "longeron"), ("cockpit_tub", "lipo_3s"),
    ("cockpit_tub", "seam_"), ("cockpit_tub", "battery_strap_"),
    # the windscreen frame lands on the tub edge, which is the sill
    ("cockpit_tub", "canopy_"),
    ("console_", "cockpit_tub"), ("console_", "fuselage_skin"),
    ("console_", "former_"), ("console_", "stringer"),
    ("console_", "seam_"),
    ("instrument_panel", "fuselage_skin"),
    ("instrument_panel", "cockpit_tub"), ("instrument_panel", "coaming"),
    ("instrument_panel", "panel_instruments"),
    ("instrument_panel", "former_"), ("instrument_panel", "seam_"),
    ("panel_instruments", "fuselage_skin"), ("panel_instruments", "coaming"),
    ("panel_instruments", "seam_"),
    ("coaming", "fuselage_skin"), ("coaming", "canopy_glass"),
    ("coaming", "cockpit_tub"), ("coaming", "seam_"),
    ("hud_glass", "hud_frame"), ("hud_", "coaming"),
    ("hud_", "fuselage_skin"), ("hud_", "canopy_glass"),
    ("hud_", "seam_"),
    # the seat: pan, back, headbox, rails and harness are one seat
    ("seat_", "seat_"),
    ("seat_", "fuselage_skin"), ("seat_", "former_"),
    ("seat_", "cockpit_tub"), ("seat_", "bhd_"), ("seat_", "seam_"),
    ("seat_", "console_"), ("seat_", "stringer"), ("seat_", "longeron"),
    ("ejection_handle", "seat_"), ("ejection_handle", "fuselage_skin"),
    ("ejection_handle", "former_"),
    ("ejection_handle", "seam_"), ("ejection_handle", "cockpit_tub"),
    ("rudder_pedals", "fuselage_skin"), ("rudder_pedals", "cockpit_tub"),
    ("rudder_pedals", "seam_"),
    ("control_stick", "console_"), ("control_stick", "fuselage_skin"),
    ("control_stick", "seam_"),
    ("throttle_lever", "console_"), ("throttle_lever", "fuselage_skin"),
    ("throttle_lever", "seam_"),
    # the pilot, who is in the seat with his hands on the controls
    ("pilot_", "pilot_"),
    ("pilot_", "seat_"), ("pilot_", "fuselage_skin"),
    ("pilot_", "cockpit_tub"), ("pilot_", "console_"),
    ("pilot_", "control_stick"), ("pilot_", "throttle_lever"),
    ("pilot_", "canopy_glass"), ("pilot_", "seam_"),
    ("pilot_", "former_"), ("pilot_", "stringer"),
    ("bl_diverter", "fuselage_skin"),
    ("tailpipe_shroud", "fuselage_skin"),
    ("naca_inlet_", "fuselage_skin"),
    ("cooling_exit_", "fuselage_skin"),
    ("canopy_latch_", "canopy_frame"),
    ("canopy_latch_", "fuselage_skin"),
    ("wheel_hub_", "wheel_"),
    ("kill_switch", "fuselage_skin"),
    ("telemetry_gps", "avionics_tray"),
    ("receiver", "former_"),
    ("ecu_battery", "avionics_tray"),
    ("turbine_ecu", "avionics_tray"),

    # ------------------------------------------------------------------
    # Joints the check could not reach until it stopped spending its
    # budget on the joints it had already been told about. An airframe is
    # a lattice -- stringers notch into formers, formers sit on longerons,
    # every box in the bay is screwed to one of them -- so most of what
    # follows is that lattice being written down.
    # ------------------------------------------------------------------

    # frame and stringer interlocks
    ("stringer", "longeron"), ("stringer", "bhd_"),
    ("stringer", "mount_rails"), ("stringer", "mount_ring"),
    ("stringer", "duct_inlet"), ("stringer", "ventral_"),
    ("stringer", "bypass_slots"), ("stringer", "tailpipe_"),
    ("stringer", "access_tray"), ("stringer", "ecu_battery"),
    ("longeron", "mount_ring"), ("longeron", "duct_inlet"),
    ("longeron", "ventral_"), ("longeron", "bl_diverter"),
    ("longeron", "access_tray"), ("longeron", "engine_"),
    ("former_", "wing_"), ("former_", "canopy_frame"),
    ("former_", "cooling_exit_"), ("former_", "ventral_"),
    ("former_", "stabilator_"), ("former_", "antenna"),
    ("former_", "gear_"), ("former_", "wing_joiner"),
    ("former_", "naca_inlet_"), ("former_", "instrument_panel"),
    ("bhd_", "wing_"), ("bhd_", "wiring"), ("bhd_", "seat_pan"),
    ("bhd_", "cooling_exit_"), ("bhd_", "wing_bolt_"),
    ("mount_ring", "mount_rails"), ("mount_rails", "ventral_"),
    ("ventral_", "mount_rails"), ("ventral_", "longeron"),

    # skin relief: the seams cross each other, the panels sit between them
    # and the doublers are under both
    ("seam_ring", "seam_lengthwise"), ("seam_ring", "doublers"),
    ("seam_ring", "panel_"), ("seam_ring", "wing_"),
    ("seam_ring", "stabilator_"), ("seam_lengthwise", "wing_"),
    ("seam_lengthwise", "duct_inlet"), ("seam_lengthwise", "naca_inlet_"),
    ("wing_seams", "duct_inlet"), ("panel_", "seam_lengthwise"),
    ("naca_inlet_", "wing_joiner"), ("access_tray", "fuselage_skin"),
    ("nose_steering_link", "fuselage_skin"), ("intake_lip", "fuselage_skin"),
    ("vtail_fin", "fuselage_skin"), ("servo_arm_", "fuselage_skin"),
    ("intake_lip", "bl_diverter"), ("bl_diverter", "longeron"),

    # the wing passes through the fuselage, and the gear folds into it
    ("wing_joiner", "wing_"), ("bhd_spar", "wing_"), ("gear_door", "wing_"),
    ("spar_", "rib_"),

    # equipment is bolted to the frame it sits on
    ("servo_", "longeron"), ("servo_", "bhd_"), ("servo_", "duct_inlet"),
    ("bellcranks", "longeron"), ("bellcranks", "mount_ring"),
    ("bellcranks", "duct_inlet"), ("bellcranks", "wing_bolt_"),
    ("bellcranks", "cooling_exit_"), ("kill_switch", "bhd_"),
    ("kill_switch", "duct_inlet"), ("avionics_tray", "bhd_"),
    ("avionics_tray", "duct_inlet"), ("retract_", "duct_inlet"),
    ("fuel_pump", "mount_rails"), ("fuel_pump", "mount_ring"),
    ("fuel_pump", "duct_inlet"), ("fuel_", "bhd_"),
    ("battery_strap_", "duct_inlet"), ("wiring", "battery_strap_"),
    ("esc_40a", "duct_inlet"), ("gear_door_actuator_", "duct_inlet"),
    ("cooling_exit_", "mount_ring"), ("access_tray", "stringer"),
    ("access_tray", "longeron"), ("turbine_ecu", "former_"),

    # control runs land on what they move
    ("horn_", "pushrod_linkages"), ("horn_", "ventral_"),
    ("clevis_", "ventral_"), ("clevis_", "vtail_fin"),
    ("clevis_", "mount_rails"), ("fin_rib", "clevis_"),
    ("hinge_", "static_dischargers"), ("hinge_rudder", "vtail_fin"),
    ("navlight_tail", "hinge_"), ("rib_", "pushrod_linkages"),
    ("flaperon_", "pushrod_linkages"), ("stringer", "pushrod_linkages"),
    ("engine_", "pushrod_linkages"), ("fuel_", "pushrod_linkages"),
    ("engine_mount_", "pushrod_linkages"), ("engine_", "ventral_"),

    # canopy, tail and skin joints that the wider net reached
    ("canopy_glass", "former_"), ("canopy_glass", "bhd_"),
    ("canopy_glass", "seam_lengthwise"), ("canopy_latch_", "former_"),
    ("spar_", "pushrod_linkages"), ("spar_", "mount_ring"),
    ("clevis_", "longeron"), ("clevis_", "stringer"),
    ("former_", "bypass_slots"), ("tailpipe_", "bypass_slots"),
    ("tailpipe_", "longeron"), ("tailpipe_", "stringer"),
    ("seam_lengthwise", "stabilator_"), ("seam_ring", "duct_inlet"),
    ("ventral_", "stabilator_"), ("ventral_", "thrust_tube"),
    ("stringer", "thrust_tube"), ("wing_seams", "fuel_tank"),
    ("intake_lip_ring", "stringer"), ("servo_arm_", "stringer"),
    ("gear_door", "rib_"), ("gear_door", "stringer"),
    # a door hinges to the structure behind it, which is what a hinge is
    ("gear_door", "bhd_"), ("gear_door", "former_"),
    ("mount_ring", "pushrod_linkages"), ("pushrod_linkages", "fuselage_skin"),
    ("tailpipe_cone", "fuselage_skin"), ("gear_nose_strut", "duct_inlet"),
    ("gear_door_actuator_", "duct_inlet"), ("bhd_", "battery_strap_"),
    ("seat_pan", "wiring"), ("naca_inlet_", "wing_"),
    ("retract_nose", "bl_diverter"), ("former_", "mount_rails"),

    # Equipment against the frame it is screwed to, and through the
    # lightening holes it passes. In a 56 mm fuselage every box touches a
    # longeron, a stringer or a former -- that contact IS the mounting, and
    # a former is mostly hole by area.
    ("turbine_ecu", "former_"), ("turbine_ecu", "longeron"),
    ("ecu_battery", "former_"), ("ecu_battery", "longeron"),
    ("rx_battery", "former_"), ("rx_battery", "longeron"),
    ("lipo_3s_900", "former_"), ("lipo_3s_900", "longeron"),
    ("lipo_3s_900", "gear_door"), ("lipo_3s_900", "stringer"),
    ("telemetry_gps", "former_"), ("telemetry_gps", "longeron"),
    ("kill_switch", "former_"), ("kill_switch", "longeron"),
    ("receiver", "former_"), ("receiver", "longeron"),
    ("fuel_pump", "longeron"), ("fuel_filter", "longeron"),
    ("fuel_lines", "longeron"), ("fuel_lines", "former_"),
    ("fuel_lines", "fuel_tank"), ("fuel_lines", "stringer"),
    ("canopy_latch_", "longeron"), ("canopy_latch_", "stringer"),
    ("wing_bolt_", "fuel_tank"), ("retract_nose", "stringer"),

    # an aileron servo lives in the wing, bolted to a rib and the joiner
    ("servo_ail_", "wing_"), ("servo_ail_", "wing_joiner"),
    ("servo_ail_", "rib_"), ("servo_ail_", "spar_"),
    ("former_", "vtail_fin"), ("seat_pan", "stringer"),
    ("seat_back", "stringer"), ("wiring", "rx_battery"),
    ("wiring", "canopy_frame"), ("wiring", "lipo_3s_900"),
    ("seam_lengthwise", "doublers"), ("rx_mount", "receiver"),
    ("fuel_lines", "mount_ring"), ("bellcranks", "fuel_tank"),
    ("naca_inlet_", "fuel_tank"), ("wing_", "pushrod_linkages"),
    ("longeron", "pushrod_linkages"), ("cooling_exit_", "seam_ring"),
    ("seat_back", "bhd_"), ("seat_back", "seat_pan"),
    ("canopy_glass", "seat_back"), ("canopy_glass", "instrument_panel"),
    # the loom is clipped along the frame and plugs into every box in the
    # bay, so it shares material with all of them by construction
    ("wiring", "turbine_ecu"), ("wiring", "ecu_battery"),
    ("wiring", "receiver"), ("wiring", "kill_switch"),
    ("wiring", "telemetry_gps"), ("wiring", "servo_"),
    ("wiring", "longeron"), ("wiring", "stringer"), ("wiring", "bhd_"),
    ("wiring", "duct_inlet"), ("wiring", "canopy_glass"),
    ("wiring", "avionics_tray"), ("wiring", "access_tray"),
    ("duct_inlet", "pushrod_linkages"), ("avionics_tray", "servo_"),
    ("ecu_battery", "fuel_lines"), ("rx_mount", "receiver"),
    ("telemetry_gps", "stringer"), ("rib_", "seam_ring"),
    ("vtail_fin", "pushrod_linkages"), ("ventral_", "pushrod_linkages"),
    ("thrust_tube", "fuselage_skin"), ("clevis_", "fuselage_skin"),
    ("avionics_tray", "wing_bolt_"), ("wing_bolt_", "duct_inlet"),
    ("cooling_exit_", "stringer"), ("intake_lip", "longeron"),
    ("longeron", "gear_door_actuator_"), ("fuel_pump", "rx_mount"),
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
    ("nozzle_flap_", "engine_casing_augmentor"),
    ("nozzle_seal_", "engine_casing_augmentor"),
    ("nozzle_flap_", "nozzle_seal_"),
    ("nozzle_flap_", "nozzle_external_flap_"),
    ("nozzle_seal_", "nozzle_external_flap_"),
    ("nozzle_link_", "nozzle_flap_"),
    ("nozzle_link_", "engine_flange_aug_aft"),
    # The tail frame is a ring round the engine in the tailcone, so the
    # cooling slots in that tailcone and the mount rails bolted to the frames
    # pass through it -- which is what a frame is for.
    ("bhd_tail", "bypass_slots"), ("bhd_tail", "mount_rails"),
    ("bhd_tail", "bay_doors"), ("bhd_tail", "engine_mount_links_rear"),
    ("bhd_tail", "ventral_"),   # the ventral fins bolt to this frame
    # the stabilator servo drives the pivot it is built on
    ("stab_servo_", "stab_pivot_"),
]

if __name__ == "__main__":
    sys.exit(0 if _intersect.run(ROOT, "plane/parts", EXPECTED) else 1)
