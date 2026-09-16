# Numbers that came from outside this repo

Everything here has a source. Nothing in this file is a guess, and nothing in
the code should silently disagree with it.

## Tyre load sensitivity — the number that decides the whole claim

A tyre's coefficient of friction falls as vertical load rises. This is the
single most important physical fact for a car that makes several times its own
weight in downforce, because it means downforce does **not** convert to grip
one for one.

Milliken & Milliken, *Race Car Vehicle Dynamics*, reproduced on Wikipedia:

| vertical load (lbf) | Fy/Fz max | slip angle |
|---|---|---|
| 900 | 1.10 | 5.6° |
| 1350 | 1.08 | 6.0° |
| 1800 | 0.97 | 6.7° |

Fitting `mu = mu0 * (Fz/Fz0)^-k` across the full range, 900 → 1800 lbf:

    1.10 -> 0.97 over a doubling  =>  k = -log2(0.97/1.10) = 0.181

The same source states the general rule that maximum horizontal force is
proportional to `Fz^0.7` to `Fz^0.9`, which is `k = 0.10` to `0.30`.

**So the honest band is k = 0.15 to 0.25, centred on 0.18.** An earlier brief
in this project said 0.10 to 0.15. That was too generous and would have
overstated this car's advantage.

Source: <https://en.wikipedia.org/wiki/Tire_load_sensitivity>

## What a real fan car achieves

**Gordon Murray Automotive T.50** — 400 mm fan at up to 7,000 rpm on a 980 kg
car. Increases downforce by up to 50 % and cuts drag by 12.5 %. In braking
mode it doubles downforce and pulls the car up 33 ft shorter from 150 mph.
Quoted downforce: 220 kg at 155 mph, 460 kg at 226 mph.

Note carefully: 220 → 460 kg over 69.3 → 101 m/s is a ratio of 2.09 against a
V² ratio of 2.12. That is V²-scaling, so those are **total aero** figures and
the T.50's fan works mainly by controlling the diffuser's boundary layer — it
multiplies a speed-dependent effect. It is *not* a sealed-plenum pump.

**Brabham BT46B** — fan driven off the Alfa engine through four clutches,
about 30 bhp to the fan, flexible skirts along the engine bay and past the
rear axle. **Chaparral 2J** — two fans on a separate engine, plastic skirts.
Both were sealed-plenum pumps, which is the mechanism that works at zero road
speed. No measured downforce figure for either could be sourced; do not quote
one.

Sources: <https://en.wikipedia.org/wiki/Gordon_Murray_Automotive_T.50>,
<https://www.motorsport.com/f1/news/banned-tech-brabham-bt46b-fan/4808234/>

## The driver

F1 drivers see up to about 6 g braking and in the fastest corners, with
sustained cornering peaking around 6 g. A 6.5 kg helmeted head weighs 26 kg at
4 g. Lateral tolerance is far higher than vertical, and tolerance trades
against duration.

`spec.DRIVER_G_LIMIT` is 7.0. That is above what an F1 driver sustains today,
and it is very likely the real ceiling on this car rather than the tyre. If
the simulation says the car could pull more than 7 g and the driver cannot,
**that is the result**, not a bug: past that point extra downforce buys
nothing and the design should stop paying for it.

Source: <https://www.formula1-dictionary.net/g_force.html>
