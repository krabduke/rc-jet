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
    ("data_link", "avionics_tray"),
    ("kill_switch", "fuselage_skin"),
    ("ecu_battery", "avionics_tray"),
    ("turbine_ecu", "avionics_tray"),
    ("rx_mount", "former_"),
    ("receiver", "rx_mount"),
    ("esc_40a", "avionics_tray"),
]

if __name__ == "__main__":
    sys.exit(0 if _intersect.run(ROOT, "plane/parts", EXPECTED) else 1)
