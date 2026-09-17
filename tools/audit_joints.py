"""Does the aeroplane hold together, and does every system run through?

    python3 tools/audit_joints.py

Every other audit here is one-sided. `audit_intersect` lists parts sharing
material and `audit_fit` lists parts that got too close -- both are looking for
things that touch when they should not. A control run that stops 40 mm short of the actuator it drives passes both, because not
touching is exactly what they want to see.

That is the defect this catches, and it is the commonest one in a model built
a part at a time: something moves, the thing that lands on it does not, and
the only witness is a render from the one angle where the joint is not hidden
behind something else.

`audit_intersect.EXPECTED` is nearly this list already -- naming two parts
there says they are meant to be one assembly -- but it is a permission, not a
requirement. Nothing there fails when one of them drifts away; the entry just
stops applying. The circuits below are the same knowledge stated as an
obligation.

Three checks:

  ASSEMBLY   every part is attached to the machine, however indirectly
  CIRCUITS   each declared run of material is continuous, link by link
  MODULES    no two modules build a part under the same name, and no cutter
             is aimed at a part its own module does not build

CONTACT is 1 units. Parts that are bolted, welded or bonded together in
this model interpenetrate, so anything further apart than that is not a joint.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import _joints  # noqa: E402

CONTACT_MM = 1.0
ROOT_PART = 'fuselage_skin'
PKG = 'plane/parts'
UNIT = 'units'

CIRCUITS = [
    ("air: lip to duct to the engine's own inlet",
     ['intake_lip', 'duct_inlet', 'duct_coupling', 'engine_casing_inlet']),
    ('the duct is framed and sealed',
     ['duct_inlet', 'duct_frames', 'duct_seam']),
    ('the engine case is one pressure vessel, front to back',
     ['engine_casing_inlet', 'engine_casing_fan', 'engine_casing_bypass',
      'engine_casing_turbine', 'engine_casing_augmentor']),
    ('and the core case hangs inside the bypass duct',
     ['engine_splitter', 'engine_bypass_inner_wall', 'engine_casing_combustor',
      'engine_casing_turbine']),
    ('core gas path: fan to compressor to combustor to turbine',
     ['engine_blades_fan_r3', 'engine_splitter', 'engine_fan_frame_struts',
      'engine_casing_hpc', 'engine_diffuser', 'engine_combustor_dome']),
    ('gas path: turbine to mixer to augmentor',
     ['engine_lpt_disc_assembly', 'engine_turbine_frame_hub',
      'engine_turbine_frame_struts', 'engine_mixer',
      'engine_augmentor_liner']),
    ('the nozzle hangs off the augmentor case and is actuated',
     ['engine_casing_augmentor', 'nozzle_actuator_00', 'nozzle_unison_ring',
      'nozzle_link_00', 'nozzle_flap_00', 'nozzle_external_flap_00']),
    ('the low spool is one shaft',
     ['engine_fan_disc_assembly', 'engine_shaft_lp', 'engine_lpt_disc_assembly']),
    ('the high spool is one shaft',
     ['engine_hpc_drum', 'engine_hpc_rear_cone', 'engine_shaft_hp',
      'engine_hpt_disc']),
    ('engine fuel: lines to manifold to nozzles',
     ['engine_fuel_lines', 'engine_fuel_manifold', 'engine_fuel_nozzles']),
    ('the engine hangs off the airframe',
     ['engine_mount_trunnions', 'engine_mount_links_fwd', 'mount_ring',
      'fuselage_skin']),
    ('and is located aft',
     ['engine_mount_links_rear', 'mount_rails']),
    ('engine bay cooling reaches its exit louvres',
     ['engine_bay_cooling_inlet', 'cooling_exit_l']),
    ('the accessory gearbox is driven off the high spool',
     ['engine_shaft_hp', 'engine_towershaft', 'engine_gearbox']),
    ('left wing onto the fuselage through the spar',
     ['wing_l', 'spar_carbon', 'fuselage_skin']),
    ('the flaperon hangs on the wing',
     ['wing_l', 'hinge_flaperon_l', 'flaperon_l']),
    ('the leading-edge flap hangs on the wing',
     ['wing_l', 'leading_edge_flap_l']),
    ('the strake blends into the fuselage',
     ['wing_strake_l', 'fuselage_skin']),
    ('the stabilator turns on its pivot, driven by its servo',
     ['fuselage_skin', 'stab_pivot_l', 'stabilator_l']),
    ('the stabilator servo reaches the pivot',
     ['stab_servo_l', 'stab_pivot_l']),
    ('the rudder hangs on the fin',
     ['vtail_fin', 'hinge_rudder', 'rudder']),
    ('the rudder servo reaches the rudder',
     ['rudder_servo', 'rudder']),
    ('the fin is on the fuselage',
     ['vtail_fin', 'fuselage_skin']),
    ('flight control: computer to loom to the tail',
     ['fcs_fcc_envelope', 'fcs_loom_trunk_upper_l', 'fuselage_skin',
      'fcs_loom_tail_l', 'stab_servo_l']),
    ('flight control: computer to loom to the wing',
     ['fcs_fcc_envelope', 'fcs_loom_trunk_lower_l', 'fuselage_skin']),
    ('the stick and pedals are in the cockpit',
     ['cockpit_tub', 'control_stick', 'fuselage_skin']),
    ('the seat is in the tub',
     ['cockpit_tub', 'seat_bucket', 'seat_back']),
    ('the pilot is in the seat',
     ['seat_bucket', 'seat_back', 'pilot_torso', 'pilot_helmet']),
    ('the canopy closes onto the frame and the windscreen bow',
     ['canopy_glass', 'canopy_frame', 'frame_windscreen_bow']),
    ('and is opened by its strut',
     ['strut_canopy_actuator', 'canopy_frame']),
    ('skin onto structure: formers and longerons',
     ['fuselage_skin', 'former_05', 'longeron_1']),
    ('stringers tie the formers',
     ['former_05', 'stringer_05', 'former_06']),
    ('the bulkheads close the fuselage',
     ['fuselage_skin', 'bhd_firewall']),
    ('main gear: wheel to hub to trailing link to leg',
     ['wheel_main_l', 'wheel_hub_main_l', 'gear_main_trailing_link_l',
      'gear_main_l']),
    ('main gear is braced and retracted',
     ['gear_main_l', 'gear_main_side_stay_l', 'gear_bay_main_l']),
    ('the main retract actuator reaches the leg',
     ['gear_main_retract_actuator_l', 'gear_main_l']),
    ('main gear brake on the wheel',
     ['wheel_hub_main_l', 'brake_l']),
    ('nose gear: wheel to fork to strut, and the strut is braced to the bay',
     ['wheel_nose', 'wheel_hub_nose', 'gear_nose_fork', 'gear_nose_strut',
      'gear_nose_drag_stay', 'gear_bay_nose']),
    ('the nose gear door is hung and driven',
     ['gear_door_nose_l', 'gear_door_hinge_nose_l', 'gear_bay_nose']),
    ('the main gear bay is lined',
     ['gear_bay_main_l', 'gear_bay_lining_main_l']),
    ("hydraulics reach the main leg's retract jack",
     ['gear_bay_hydraulic_lines_main_l', 'gear_main_retract_actuator_l']),
]


def main():
    parts, collisions, cut_owner, built_by, failures = _joints.load(ROOT, PKG)
    bad = []

    print(f"\n{len(parts)} parts from {len(set(built_by.values()))} modules")

    print("\nMODULES")
    for name, why in failures:
        print(f"  x   {name:26s} did not build: {why}")
        bad.append(f"{name} did not build")
    for key, first, second in collisions:
        print(f"  x   {key:26s} built by both {first} and {second}")
        bad.append(f"{key} is built twice")
    for target, owners in sorted(cut_owner.items()):
        for owner in owners:
            if target not in built_by:
                print(f"  x   {target:26s} cut declared by {owner}, and"
                      f" nothing builds it -- the cutter has no target")
                bad.append(f"{target} cutter from {owner} has no target")
    if not bad:
        print("  ok  every part name is built once, by one module, and every"
              " cutter reaches its target")

    print(f"\nASSEMBLY  (contact within {CONTACT_MM:g} {UNIT})")
    graph = _joints.contact_graph(parts, CONTACT_MM)
    groups = _joints.components(graph)
    main_group = next((g for g in groups if ROOT_PART in g), set())
    loose = [g for g in groups if g is not main_group]
    if not loose:
        print(f"  ok  all {len(main_group)} parts hang together off {ROOT_PART}")
    for g in loose:
        names = ", ".join(sorted(g))
        print(f"  x   detached: {names}")
        bad.append(f"detached: {names}")

    print("\nCIRCUITS")
    for label, chain in CIRCUITS:
        breaks = _joints.broken_links(parts, graph, chain)
        if not breaks:
            print(f"  ok  {label}")
            continue
        for (_i, a, b, why) in breaks:
            extra = ""
            if why == "no contact":
                A, B = _joints.match(parts, a), _joints.match(parts, b)
                d = min(_joints.gap(parts[x], parts[y]) for x in A for y in B)
                extra = f" ({d:.0f} {UNIT} apart)"
            print(f"  x   {label}: {a} -> {b}, {why}{extra}")
            bad.append(f"{label}: {a} -> {b} {why}")

    print()
    if not bad:
        print("PASS  it is one assembly and every circuit is joined")
        return 0
    print(f"FAIL  {len(bad)} joints are not made")
    return 1


if __name__ == "__main__":
    sys.exit(main())
