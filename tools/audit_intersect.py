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
    # internal structure, inside the skin, which is the point of it
    ("former_", "fuselage_skin"),
    ("bhd_", "fuselage_skin"),
    ("longeron", "fuselage_skin"),
    ("stringer", "fuselage_skin"),
    ("seam_ring", "fuselage_skin"),
    ("seam_lengthwise", "fuselage_skin"),
    ("doublers", "fuselage_skin"),
    ("panel_", "doublers"),
    ("panel_screws", "fuselage_skin"),
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
    ("brake_line_", "gear_main_struts"),
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
    ("instrument_panel", "fuselage_skin"),
    ("seat_", "fuselage_skin"),
    ("seat_", "former_"),
    ("bl_diverter", "fuselage_skin"),
    ("tailpipe_shroud", "fuselage_skin"),
    ("naca_inlet_", "fuselage_skin"),
    ("cooling_exit_", "fuselage_skin"),
    ("canopy_latch_", "canopy_frame"),
    ("canopy_latch_", "fuselage_skin"),
    ("wheel_hub_", "wheel_"),
    ("gps_puck", "avionics_tray"),
    ("telemetry_sensor", "avionics_tray"),
    ("kill_switch", "fuselage_skin"),
    ("ecu_battery", "avionics_tray"),
    ("turbine_ecu", "avionics_tray"),
    ("rx_mount", "former_"),
    ("receiver", "rx_mount"),
    ("esc_40a", "avionics_tray"),

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
    ("wing_joiner", "wing_"), ("bhd_spar", "wing_"), ("gear_doors", "wing_"),
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
    ("gear_doors", "rib_"), ("gear_doors", "stringer"),
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
    ("lipo_3s_900", "gear_doors"), ("lipo_3s_900", "stringer"),
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
]

if __name__ == "__main__":
    sys.exit(0 if _intersect.run(ROOT, "plane/parts", EXPECTED) else 1)
