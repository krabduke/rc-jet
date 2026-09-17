"""The F110 installation has discrete frame mounts, not a model turbine ring.

The engine stays in aircraft drawing units through ENGINE_SCALE; its load
paths and removable belly must carry and release a full-size powerplant.
"""

import sys, os

HERE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(HERE, "f110"))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import importlib

import spec                                    # the aircraft spec
import math
import mesh
from parts import fuselage

try:
    from parts import intake
except (ImportError, SyntaxError, AttributeError, NameError):
    intake = None


def _duct_clearance(x):
    # Guard intake during its concurrent rewrite; fall back to INTAKE's outer envelope.
    if intake is not None:
        return intake.duct_top(x)
    i = spec.INTAKE
    return max(i["z_lip"], spec.ENGINE_Z) + max(
        i["lip_height"] / 2, i["duct_r_end"]) + i["wall"] + 3.2



def _load_engine():
    """Import the engine package with f110/ ahead on the path, so its modules
    resolve their own `spec` and not the aircraft's."""
    saved = {k: v for k, v in sys.modules.items()
             if k in ("spec", "mesh", "airfoil", "materials")
             or k.startswith("parts")}
    for k in list(saved):
        del sys.modules[k]

    f110_dir = os.path.join(HERE, "f110")
    sys.path.insert(0, f110_dir)
    try:
        espec = importlib.import_module("spec")
        # Coarsen the engine before anything is built.
        # Coarsened, but not by as much as it was: at 32 segments a casing
        # came out as a 128-vertex prism, which is a worse surface than
        # anything else on the airframe and the engine is the thing people
        # open the cutaway to look at. Two thirds of full resolution.
        # Raised from 20/6/64/16/12. The turbofan is 82 per cent of this
        # aircraft's geometry and the whole point of the cutaway, so it was
        # the one thing being coarsened hardest. These are close to the
        # engine's own settings now; it is still the engine that dominates
        # the file, which is the right thing for it to dominate.
        espec.RES.update({
            "airfoil_chord_pts": 38,
            "airfoil_span_pts": 12,
            "revolve_segments": 140,
            "small_revolve": 34,
            "pipe_segments": 20,
        })
        mods = [importlib.import_module(f"parts.{m}") for m in
                ("rotating", "statics", "combustor", "turbine",
                 "augmentor", "nozzle", "accessories")]
        built, arrays = {}, {}
        for m in mods:
            built.update(m.build())
            arrays.update(getattr(m, "ARRAYS", {}))
        return built, arrays, espec
    finally:
        sys.path.remove(f110_dir)
        for k in list(sys.modules):
            if k in ("spec", "mesh", "airfoil", "materials") or k.startswith("parts"):
                del sys.modules[k]
        sys.modules.update(saved)


ARRAYS = {}

# Parts of the full-size engine that this aircraft does not install.
#
# These four are airframe-mounted line-replaceable units: on a real
# installation the oil tank, the fuel/oil heat exchanger, the engine control
# and the ignition exciters live in the nacelle, outboard of the engine's own
# envelope, not on the engine as it leaves the stand. Scaled 1:33 into a
# 440 mm fuselage they reach 32-38 mm off the axis where the fuselage inner
# half-width is under 25 mm, so they come straight out through the skin.
#
# The aircraft has its own versions of all four at its own scale -- fuel_tank,
# fuel_pump, turbine_ecu, ecu_battery -- and fitting a scaled F110 oil tank
# beside a model fuel hopper would be the wrong object twice. So the airframe
# takes the engine and leaves the nacelle kit behind.
#
# The variable nozzle goes the same way, and for a sharper reason: hardware.py
# builds one. The engine's own nozzle is a static ring of flaps at one area;
# the airframe's is twelve separate flaps, twelve links, twelve seals and six
# actuators posed on a unison ring, so the nozzle can be shown at dry power
# and at reheat. Both were being built, in the same 17 mm of tailpipe --
# `nozzle_unison_ring` and `engine_nozzle_actuator_ring` have identical
# bounding boxes -- so every flap in the aeroplane was two flaps, and the
# audits allowed it because ("nozzle", "nozzle") reads as one assembly.
NOT_INSTALLED = ("oil_tank", "heat_exchanger", "engine_control",
                 "ignition_exciters",
                 "nozzle_actuator_ring", "nozzle_actuators",
                 "nozzle_ext_flaps", "nozzle_flaps_convergent",
                 "nozzle_flaps_divergent", "nozzle_links", "nozzle_seals")


def build():
    built, arrays, espec = _load_engine()

    s = spec.ENGINE_SCALE
    # place the engine's own inlet-lip station on the firewall
    dx = spec.ENGINE_X - espec.STATION["inlet_lip"] * s
    dz = spec.ENGINE_Z

    out = {}
    ARRAYS.clear()
    for name, (verts, faces) in built.items():
        # str.lstrip strips characters, not a prefix, so do it by hand:
        # "turbine_cooling_manifold".lstrip("cut:") is "rbine_..."
        bare = name[4:] if name.startswith("cut:") else name
        if bare in NOT_INSTALLED:
            continue
        key = name if name.startswith("cut:") else f"engine_{name}"
        if name.startswith("cut:"):
            key = f"cut:engine_{name[4:]}"
        out[key] = ([(x * s + dx, y * s, z * s + dz) for (x, y, z) in verts],
                    faces)
    for name, count in arrays.items():
        ARRAYS[f"engine_{name}"] = count

    out.update(_bay_installation(espec))
    return out


# Which of the engine's parts turn with which spool. The fan and the low
# turbine share the LP shaft; the compressor and the high turbine share the
# HP shaft, which turns faster and the other way. Everything else is static.
LP_PARTS = ("blades_fan_", "fan_disc_assembly", "spinner", "shaft_lp",
            "lpt_disc_assembly", "blades_lpt_r")
HP_PARTS = ("blades_hpc_", "hpc_drum", "hpc_front_cone", "hpc_rear_cone",
            "shaft_hp", "hpt_disc", "blades_hpt_r")



def _bay_installation(espec):
    """Everything in the engine bay that is not the engine.

    Four agents were asked for this and none delivered it: two refused after
    misreading the airframe, one was killed mid-edit, and one handed the Edit
    tool a corrupted path. So it is written here.

    An F110-GE-129 is 4.6 m long and 1,996 kg and the bay around it is thin --
    at station 350 the fuselage is 25.9 by 26.3 half-width against a case a
    little over 15, so there are about ten drawing units of annulus to work
    in. Everything below is sized for that, which is also why a real
    installation looks the way it does: nothing in an engine bay is where it
    is because someone liked it there.
    """
    out = {}
    x0 = spec.ENGINE_X                      # the fan face, on the firewall
    x1 = x0 + 4630.0 * spec.ENGINE_SCALE    # the nozzle exit
    z = spec.ENGINE_Z

    def half(x):
        w, h, zc, _ = fuselage.station_at(x)
        return w, h, zc

    # ---- bay ventilation -------------------------------------------------
    # An engine bay is ventilated or it cooks. Air is taken from the duct's
    # bypass ahead of the face, runs aft between case and skin, and leaves
    # through the gap round the jet pipe. The inlet is on the upper left
    # shoulder because the right shoulder carries the bleed duct.
    w, h, zc = half(x0 - 6.0)
    out["engine_bay_cooling_inlet"] = mesh.join(
        mesh.pipe([(x0 - 14.0, -w * 0.52, zc + h * 0.62),
                   (x0 - 4.0, -w * 0.58, zc + h * 0.66),
                   (x0 + 10.0, -w * 0.60, zc + h * 0.60)], 2.6, 14),
        mesh.box(x0 - 13.0, -w * 0.52, zc + h * 0.66, 7.0, 6.4, 2.2))

    # ---- the tailpipe shroud, and the exit it makes -----------------------
    # The visible feature on every real installation: the annular gap between
    # the jet pipe and the skin that the bay air leaves through. Without it
    # the tail simply closes over the engine, which is what it did.
    w, h, zc = half(x1 - 4.0)
    r_pipe = 11.0
    r_skin = min(w, h) - 1.6
    # The jet pipe and its shroud belong to hardware.py, which builds them
    # with the nozzle. Building a second one here put two shrouds in the same
    # 14 units of tailcone -- audit_intersect found them sharing material and
    # the render showed it as a doubled edge. What is left is only the annular
    # exit between that shroud and the skin, which is the part of this that
    # belongs to the bay.
    # Outside hardware.py's shroud, which the built model puts at 16.4, not
    # outside the bare jet pipe -- starting at r_pipe + 2.6 = 13.6 put this
    # annulus inside the shroud rather than around it.
    out["engine_bay_cooling_exit"] = mesh.tube(x1 - 8.0, x1 - 2.0,
                                               17.0, max(r_skin, 17.8), 48)

    # ---- fire detection and suppression -----------------------------------
    # A continuous sensing loop round the case at two stations, on standoffs,
    # and one bottle with its discharge line. Two loops rather than one so a
    # single break does not blind the bay.
    loops = []
    for xs in (x0 + 32.0, x0 + 86.0):
        w, h, zc = half(xs)
        r = min(w, h) - 4.2
        loops.append(mesh.ring_torus(xs, r, 0.55, 40, 8))
        for k in range(6):
            a = 2.0 * math.pi * k / 6.0
            loops.append(mesh.pipe(
                [(xs, (r - 1.6) * math.cos(a), zc + (r - 1.6) * math.sin(a)),
                 (xs, (r + 0.9) * math.cos(a), zc + (r + 0.9) * math.sin(a))],
                0.45, 6))
    out["engine_fire_loop"] = mesh.join(*loops)

    w, h, zc = half(x0 + 16.0)
    bx, by, bz = x0 + 16.0, w * 0.55, zc + h * 0.34
    out["engine_fire_bottle"] = mesh.join(
        mesh.pipe([(bx - 9.0, by, bz), (bx + 9.0, by, bz)], 4.0, 20),
        mesh.pipe([(bx + 9.0, by, bz), (bx + 26.0, by * 0.8, bz - 4.0),
                   (bx + 44.0, by * 0.5, bz - 6.0)], 0.9, 10),
        mesh.box(bx, by + 4.4, bz, 14.0, 1.6, 7.0))

    # ---- bleed air --------------------------------------------------------
    # Customer bleed off the high compressor, through a precooler, and forward
    # to the environmental pack. It runs on the right shoulder, opposite the
    # cooling inlet, because two ducts on one shoulder do not fit.
    w, h, zc = half(x0 + 52.0)
    out["engine_bleed_offtake"] = mesh.join(
        mesh.pipe([(x0 + 52.0, w * 0.36, zc + h * 0.30),
                   (x0 + 44.0, w * 0.58, zc + h * 0.52),
                   (x0 + 20.0, w * 0.62, zc + h * 0.58),
                   # stops ON the firewall. Forward of x0 is the intake duct,
                   # which fills the section; the ECS duct picks the air up on
                   # the other side and that is the airframe's plumbing, not
                   # the engine's.
                   # x0 + 8, not x0 + 2: with a 2.2 radius the pipe's own
                   # wall grazed the duct's at the firewall and audit_duct
                   # caught one vertex of it.
                   (x0 + 8.0, w * 0.60, zc + h * 0.60)], 2.2, 14),
        # The precooler sits ENTIRELY aft of the firewall. Centred at x0 + 8
        # with an 18-unit length its forward face landed at x 299, one unit
        # inside the duct -- a box in the air the engine breathes, found by
        # audit_duct as a single vertex out of a hundred and twenty.
        mesh.box(x0 + 20.0, w * 0.62, zc + h * 0.58, 18.0, 5.0, 6.0))

    # ---- what the airframe plugs into ------------------------------------
    # Fuel one side, oil the other, on bosses at the pylon face, with the
    # lines running forward. Separated left and right so a leak in one is not
    # a leak in both.
    for tag, sgn, key in (("fuel", -1.0, "engine_fuel_connections"),
                          ("oil", 1.0, "engine_oil_connections")):
        w, h, zc = half(x0 + 20.0)
        y = sgn * w * 0.60
        parts = []
        for dz in (-3.0, 2.5):
            parts.append(mesh.pipe([(x0 + 20.0, y * 0.86, zc + dz),
                                    (x0 + 20.0, y, zc + dz)], 1.5, 12))
            # forward only as far as the firewall. Run on to x0 - 22 and the
            # line is at y 13 where the duct bore is 19 wide -- a fuel line
            # through the air the engine breathes, which audit_duct caught at
            # 36 % of the part's vertices.
            parts.append(mesh.pipe([(x0 + 20.0, y, zc + dz),
                                    (x0 + 10.0, y * 0.97, zc + dz - 0.6),
                                    (x0 + 2.0, y * 0.92, zc + dz - 1.2)],
                                   1.1, 10))
        parts.append(mesh.box(x0 + 20.0, y, zc, 5.0, 2.0, 11.0))
        out[key] = mesh.join(*parts)

    # ---- the belly doors --------------------------------------------------
    # A fighter drops its engine out of the bottom, so the underside aft of
    # the firewall is two removable panels on hinges with latches down the
    # centreline. They are the reason the bay is reachable at all.
    doors = []
    for sgn in (-1.0, 1.0):
        a0, a1 = (188.0, 262.0) if sgn < 0 else (278.0, 352.0)
        # 0.25 units is 8 mm full size -- the step at a panel edge. At 0.9 the
        # doors stood 30 mm off the belly, which is a blister, not a door.
        doors.append(fuselage.surface_patch(x0 + 8.0, x1 - 34.0,
                                            a0, a1, 0.25, nx=10, na=6))
        for k in range(5):
            xs = x0 + 16.0 + k * 20.0
            w, h, zc = half(xs)
            doors.append(mesh.pipe(
                [(xs - 3.0, sgn * 2.0, zc - h + 1.0),
                 (xs + 3.0, sgn * 2.0, zc - h + 1.0)], 0.7, 8))
    out["engine_bay_doors"] = mesh.join(*doors)

    # The forward mount links.
    #
    # The engine's own forward trunnion is at x 332.5-335.3, y +/-12.8,
    # z 11.3-16.5. The airframe's thrust ring is at x 306-315. Between them
    # was 18 units of nothing: the engine was carried by its rear links and
    # by the fact that nobody asked. A forward mount is two links, not a
    # collar -- a collar cannot take thermal growth along the case.
    links = []
    for sgn in (-1.0, 1.0):
        links.append(mesh.pipe(
            [(313.0, sgn * 20.5, 4.0), (322.0, sgn * 17.0, 9.0),
             (334.0, sgn * 12.0, 13.5)], 1.6, 12, subdiv=3))
        for at in ((313.0, sgn * 20.5, 4.0), (334.0, sgn * 12.0, 13.5)):
            ev, ef = mesh.revolve_ring(
                [(0.0, 1.6), (0.0, 3.2), (2.4, 3.2), (2.4, 1.6)], 14)
            links.append(([(pz + at[0], py + at[1], px + at[2])
                           for (px, py, pz) in ev], ef))
    out["engine_mount_links_fwd"] = mesh.join(*links)
    return out


def pivots():
    """The rotating assemblies, about the engine's own centreline.

    The engine axis is x, at y = 0 and the thrust line in z. Giving each rotor
    that pivot means the viewer can spool the engine up without the parts
    orbiting the middle of the aeroplane.
    """
    built, _, espec = _load_engine()
    s = spec.ENGINE_SCALE
    dx = spec.ENGINE_X - espec.STATION["inlet_lip"] * s
    out = {}
    for name in built:
        if name.startswith("cut:"):
            continue
        lp = any(name.startswith(p) for p in LP_PARTS)
        hp = any(name.startswith(p) for p in HP_PARTS)
        if not (lp or hp):
            continue
        # pivot on the axis at the part's own mid-station, so the origin sits
        # inside the part rather than out at the inlet
        xs = [v[0] for v in built[name][0]]
        xm = (min(xs) + max(xs)) / 2 * s + dx
        out[f"engine_{name}"] = ((xm, 0.0, spec.ENGINE_Z), (1.0, 0.0, 0.0),
                                 1.0 if lp else -1.6, "spool_lp" if lp else "spool_hp")
    return out
