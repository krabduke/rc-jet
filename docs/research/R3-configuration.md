# R3 — Configuration for maximum agility

## Decision boundary

This is a configuration decision memo, not a correction to `plane/spec.py`. No geometry or stability constant is changed. The supplied CG and neutral-point estimates imply a static margin of **−4% MAC** (35% − 39%), not −3%; neither estimate is a validated stability derivative. `TARGET_CG_FRAC = 0.25` is a separate design target, not proof of the loaded CG.

**Low aspect ratio does not by itself establish a higher usable maximum lift coefficient.** Polhamus, NASA TN D-3767 (1966), separates potential and vortex lift and compares the suction analogy with sharp-edged delta-wing experiments; it does not establish an aircraft-level optimum at AR 1.81. NASA TN D-4739 (1968), covering AR 0.25–2.0, also shows why vortex-flow drag cannot simply be identified with attached-flow induced drag. These results support investigating vortex lift, not claiming that progressively lower AR necessarily improves instantaneous turn rate. The VX-J1 has a cropped, finite-thickness wing, body and controls: usable lift requires a trimmed, controllable whole-aircraft measurement. Sources: NASA NTRS records **19670003842** and **19680022518**, including their indexed report summaries. The comparison below separates project inputs from explicit aerodynamic assumptions; an assumed lift curve must not be presented as a discovered optimum.

The apparent conflict between the loaded CG and `TARGET_CG_FRAC` is not evidence that either constant should be changed: one describes a mass distribution, the other a target. Running a geometry verifier cannot establish the aerodynamic neutral point or choose an agility optimum. This study therefore changes only this memo, not the specification.

## Immediate configuration finding

Retain the existing stabilator attachment geometry during this study. Its attachment is not the question being decided, and changing `HTAIL` would violate the memo-only scope. The first configuration decision is whether the 9.90 m span envelope is negotiable: at constant 54 m², AR 2.5 requires 11.62 m span and AR 3.5 requires 13.75 m. These follow directly from b = sqrt(AR × S). Higher aspect ratio cannot be evaluated honestly while silently retaining the present span.

The comparison below is a transparent screening calculation, not a validated flight envelope. In particular, an instantaneous-turn optimum cannot be inferred from aspect ratio without a measured or explicitly assumed usable lift limit.

## Decision criteria and scope

Compare coordinated, level, subsonic turns with separate lift, load-factor and thrust limits. Instantaneous rate permits energy loss; sustained rate requires thrust to equal drag. Neither measures roll reversal, pitch pointing at nearly zero airspeed, or departure recovery. All aerodynamic coefficients, installed-thrust assumptions and structural limits introduced below are study assumptions, not validated VX-J1 performance.

The baseline mass is 11,006 kg from the supplied `MASS_FULL` entries, including 3,100 kg of fuel. This is not the half-fuel condition described in `docs/FULL_SCALE.md`: its 2,900 kg internal-fuel description also disagrees with the mass table. Hold the requested 11,006 kg fixed for the principal comparison rather than silently changing the design point. With standard gravity 9.80665 m/s², weight is 107.93 kN; 129 kN gives static thrust/weight 1.195. Installed thrust in a turn remains an unmeasured input.

A fair aspect-ratio sweep holds 54 m² and mass fixed, not span: span follows b = sqrt(AR × S). A fair area sweep must state whether span or aspect ratio is fixed. Both cases matter because the existing 9.90 m span is called an envelope limit in `WING`; relaxing it is a configuration decision, not a free aerodynamic improvement.

## Options

(to be populated: four configurations with computed turn rates)

## Recommendation

(pending)

