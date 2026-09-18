# The aeroplane is full size

This was an RC model. It is not one any more. Nothing about the *shape* was
wrong — at 1:33 the drawing is 14.53 m long with a 9.90 m span and a 1.29 m
duct around the fan, which is an F-16-class airframe almost exactly. What was
wrong was the *content*: servo arms, bellcranks, pushrods and clevises hanging
in the airstream, LiPo packs, a receiver, a kill switch, a wing joiner tube,
and hatch covers standing proud of the spine like luggage.

## Scale

Drawing units do not change. **One drawing unit = 33 mm full size.** Every
part file keeps the numbers it has; what changes is what those numbers mean
and what belongs at that size.

    spec.SCALE_TO_FULL = 33.0

Aero and every user-visible dimension are full scale: `MM = 0.001` becomes
`MM = 0.033` in `tools/tunnel_config.py` and `viewer/index.html`, and the
viewer reads out metres, not millimetres.

| | drawing | full size | for comparison |
|---|---|---|---|
| length | 440.3 | **14.53 m** | F-16C 15.03, Gripen 14.1 |
| span | 300.0 | **9.90 m** | F-16C 9.96 |
| wing area | — | **54.0 m²** | F-16C 27.9 |
| aspect ratio | — | **1.81** | F-16C 3.20 |
| MAC | — | **5.90 m** | |
| fuselage | — | **2.11 × 1.98 m** | |
| inlet capture | — | **0.65 m²** | F110 needs ~0.55 |
| fan case | — | **1.29 m** | F110-GE-129 fan 1.18 m |

## What the aeroplane is

**VX-J1** — a single-seat, single-engine agility demonstrator. Not a fighter:
no radar, no weapons, no pylons, no countermeasures. Its whole reason to
exist is to carry as much wing as an F110 can drag around, so it corners
harder than anything with a cockpit in it.

Design point, clean. Every figure here is computed from the `MASS_FULL` table
in `plane/spec.py`, which is the one place mass is defined — nothing below is
quoted independently of it:

| | full internal fuel | half internal fuel |
|---|---|---|
| mass | **11 006 kg** | **9 456 kg** |
| wing loading | **204 kg/m²** | **175 kg/m²** |
| thrust/weight | **1.20** | **1.39** |
| 4 g corner speed | **104 m/s** | **97 m/s** |

Empty 7 736 kg; pilot and unusable oil 170 kg; internal fuel 3 100 kg in three
cells (forward 1 000, wing 1 300, aft 800). Thrust is one F110-GE-129 at 129 kN
with afterburner. Even at full fuel the wing loading is less than half an
F-16's 431 kg/m².

The corner speeds assume CLmax 1.2 at sea level. That is an assumption about
the wing, not a measurement of it: no tunnel or CFD result behind this model
has resolved CLmax, and the panel method in `tools/` cannot — it is inviscid
and says nothing about stall. At CLmax 1.1 the full-fuel corner speed is
109 m/s and at 1.3 it is 100 m/s, so read it as 104 ±5 m/s and do not treat
the wing's maximum lift as known.

Low aspect ratio is deliberate. A cropped delta this slender carries a stable
leading-edge vortex to high α, so it trades induced-drag efficiency in a
sustained turn for enormous instantaneous lift — the Mirage/F-106/Rafale
bargain, taken further because there is no mission to compromise for.

## What must not appear anywhere on the outside

Full-size aeroplanes actuate their control surfaces from inside the wing.
Everything in this list is deleted or moved inboard of the skin:

- servo arms, bellcranks, pushrods, clevises, control horns, snake outers
- wing joiner tube, wing bolts, dowels
- LiPo packs, receiver, telemetry/GPS board, kill switch, turbine ECU,
  fuel hopper, RC fuel plumbing, arming plug
- hatch covers standing proud of the skin — full-size access panels are
  **flush**, and read as a panel line plus a fastener row, nothing more
- external ribs, fences or stringers showing through a surface they should
  be under

Replace them with what a real aeroplane has there: sealed integral hydraulic
actuators inside the wing and the fin, a flight-control computer in an
avionics bay, wet-wing and fuselage fuel cells, an APU, an environmental
control pack, gear bays with doors, and an accessory gearbox on the engine.

## The intake, which is the headline defect

Today the inlet mouth spans z −26.5 to −7.5 at station 52, and the fuselage's
own underside at station 52 is at z −14.6. So seven units of the mouth are
buried in solid fuselage and twelve hang in open air with no fairing round
them. The duct is a loose tube threaded through a closed body. This is
exactly what "the air intake doesn't actually lead to the engine, it just
hits the rest of the body" describes, and it is measurable, not a matter of
taste.

The fix is an F-16 chin inlet, built in three pieces:

1. **Drop the mouth clear of the forebody.** The roof of a chin inlet *is*
   the forebody underside. Set `z_lip = -24.0` so the mouth's top edge sits
   just under the keel at station 52 instead of inside it.

2. **Make the body wrap the duct.** The fuselage's lower surface gains a
   chin: at each station from 30 to 195 the keel is pushed down far enough to
   enclose the duct's outer wall with 2.0 units of clearance, rolled on and
   off with a cosine so there is no crease. Everything that reads the section
   — seams, formers, bulkheads, the panel solve — then follows the chin for
   free, because they all go through `fuselage.station_at`.

3. **Cut the mouth open.** The skin is lofted closed nose to tail. The canopy
   already solves this: `fuselage.canopy_aperture()` returns a cutter solid
   and `common.add_cut` hands it to a boolean. The inlet does the same with a
   solid swept along the duct bore from station 40 to station 96.

After that a ray entering the mouth reaches the fan face without crossing
solid material, which is the only test that matters.
