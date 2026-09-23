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

The baseline mass is 11,006 kg from the supplied `MASS_FULL` entries, including 3,100 kg of fuel. This is the full-fuel column of `docs/FULL_SCALE.md`, which gives the same 3,100 kg of internal fuel; its half-fuel column is 9,456 kg. Hold 11,006 kg fixed for the principal comparison rather than silently changing the design point. With standard gravity 9.80665 m/s², weight is 107.93 kN; 129 kN gives static thrust/weight 1.195. Installed thrust in a turn remains an unmeasured input.

A fair aspect-ratio sweep holds 54 m² and mass fixed, not span: span follows b = sqrt(AR × S). A fair area sweep must state whether span or aspect ratio is fixed. Both cases matter because the existing 9.90 m span is called an envelope limit in `WING`; relaxing it is a configuration decision, not a free aerodynamic improvement.

## Options

Four planforms at the same 54.0 m² and 11,006 kg, sea level, full afterburner, computed with `aero/agility.py`'s `Jet` model and its planform overrides. Span follows from aspect ratio. The lift slope is Helmbold's lifting-surface value (`kp_helmbold`) for every row, because the slender-wing value the published numbers use is increasingly too generous as AR rises; that is also why the baseline row's corner speed is 264 kt here and 252 kt in `tools/check_agility.py`. Vortex lift is held at Kv = π and the alpha ceiling at 30° for every row — both assumptions, and the first flatters the higher-AR rows, whose leading-edge vortex is weaker than a slender delta's. Oswald efficiency is held at 0.70. Sustained rates are shown across the CD0 band 0.018 / 0.022 / 0.024.

| | AR | span | CLmax (assumed curve) | corner speed | best instantaneous turn | best sustained turn |
|---|---|---|---|---|---|---|
| A — as drawn | 1.81 | 9.90 m | 1.59 | 264 kt | 37.0 °/s at 136 m/s | 20.2 / 20.1 / 20.0 °/s |
| B | 2.5 | 11.62 m | 1.81 | 247 kt | 39.5 °/s at 127 m/s | 24.0 / 23.8 / 23.8 °/s |
| C | 3.0 | 12.73 m | 1.94 | 239 kt | 40.9 °/s at 123 m/s | 26.4 / 26.3 / 26.2 °/s |
| D | 3.5 | 13.75 m | 2.05 | 233 kt | 42.0 °/s at 120 m/s | 28.6 / 28.5 / 28.4 °/s |

Two results with different standing:

- **Sustained turn rises steeply with aspect ratio, +18 % at B and +42 % at D.** It comes from induced drag falling as 1/AR at a fixed Oswald factor, which is the most robust relation in the model, and it hardly moves across the CD0 band.
- **Instantaneous turn rises only 7–14 %**, and that gain rests on the assumed lift curve: with Kv held at π, a higher-AR wing is credited with slender-delta vortex lift it would not fully make. Treat the instantaneous column as an upper bound for B–D.

None of this measures roll, pitch pointing, departure or supersonic behaviour, and B–D all break the 9.90 m span envelope in `WING`.

## Recommendation

Keep configuration A, as specified. The study does not justify changing the span envelope on the strength of a screening model, and the memo's scope is to inform that decision, not make it.

If the envelope becomes negotiable, B (AR 2.5, 11.6 m) is the candidate to examine first: it buys most of the robust sustained-turn gain (+18 %) for the smallest change in span, while its instantaneous gain is the least dependent on the assumed vortex lift. Before any change, measure CLmax and the vortex breakdown angle for A and B in a trimmed whole-aircraft condition; those two numbers decide the instantaneous column, which this model can only assume.

