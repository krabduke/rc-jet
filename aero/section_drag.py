"""Section drag for this aeroplane's aerofoils, from NeuralFoil.

WHY THIS FILE EXISTS

viscous.js worked out profile drag from flat-plate friction, a form factor and
a laminar-separation-bubble term I calibrated by hand:

    BUBBLE = 0.020        "so that an ordinary section at Re 262,000 comes out
                           at about 0.016, which is what the low-Reynolds
                           measurements give"

That target was wrong. NeuralFoil -- a neural network trained on XFOIL, so it
carries a real integral boundary layer and finds the bubble rather than
approximating it -- says 0.0082 for a 10 % section at Re 260,000 and CL 0.15.
The correlation was out by a factor of two at the Reynolds number this
aeroplane actually flies at, and it was the largest single piece of its drag.

    t/c    Re        NeuralFoil     my build-up    ratio
    0.10   260,000    0.00816        0.01627       1.99
    0.10   430,000    0.00649        0.00798       1.23
    0.10  1,200,000   0.00502        0.00790       1.57

So the correlation goes and a table takes its place. The table is generated
here, offline, and committed -- the viewer stays dependency-free and what
ships is what was computed.

    python3 -m venv .venv && .venv/bin/pip install aerosandbox
    .venv/bin/python aero/section_drag.py

Sections are NACA 4-digit symmetric, which is what tools/panelgeom.py builds
them as. Reynolds spans a wing tip at stall to a fuselage at full throttle;
CL spans nothing to the most a section here carries.
"""

import json
import os
import sys

import numpy as np
import aerosandbox as asb
import neuralfoil as nf

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TC = [0.04, 0.05, 0.06, 0.07, 0.08, 0.10, 0.12]
RE = [5.0e4, 1.0e5, 1.7e5, 2.6e5, 4.3e5, 7.0e5, 1.2e6, 2.0e6]
CL = [0.0, 0.1, 0.2, 0.3, 0.45, 0.6, 0.8]


def sweep():
    out = []
    for tc in TC:
        name = f"naca00{round(tc*100):02d}"
        af = asb.Airfoil(name)
        row = []
        for re in RE:
            # a fine alpha sweep, then read cd off at each CL we want
            al = np.linspace(-2.0, 12.0, 141)
            r = nf.get_aero_from_airfoil(af, alpha=al, Re=re, model_size="large")
            cl, cd = np.asarray(r["CL"]), np.asarray(r["CD"])
            # CL is monotonic over this range for a symmetric section below
            # stall; where the net says otherwise, take the attached branch
            k = int(np.argmax(cl))
            cl, cd = cl[:k+1], cd[:k+1]
            vals = []
            for want in CL:
                if want <= cl[0]:
                    vals.append(float(cd[0]))
                elif want >= cl[-1]:
                    # past what this section makes at this Reynolds number:
                    # hold the last value rather than extrapolate a stall
                    vals.append(float(cd[-1]))
                else:
                    vals.append(float(np.interp(want, cl, cd)))
            row.append([round(v, 6) for v in vals])
            print(f"  {name}  Re {re:9.0f}   cd {row[-1][0]:.5f} .. {row[-1][-1]:.5f}")
        out.append(row)
    return out


def main():
    table = sweep()
    doc = {
        "source": "NeuralFoil 0.3.3 (trained on XFOIL), via AeroSandbox",
        "sections": "NACA 4-digit symmetric",
        "tc": TC, "re": RE, "cl": CL,
        "cd": table,
    }
    p = os.path.join(HERE, "viewer", "section_drag.json")
    with open(p, "w") as fh:
        json.dump(doc, fh)
    print(f"\n  -> {p}  ({len(TC)}x{len(RE)}x{len(CL)} = "
          f"{len(TC)*len(RE)*len(CL)} points)")


if __name__ == "__main__":
    main()
