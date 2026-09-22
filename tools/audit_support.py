"""Every piece of the aircraft must be fastened to something.

    python3 tools/audit_support.py            (runs itself under Blender)
    python3 tools/audit_support.py --shrink   (after a fix: drop what is fixed)

A part is often several closed pieces -- a bolt ring is one piece per bolt, a
blade row one per blade. audit_joints.py asks whether the PARTS form one
assembly, and a part passes that as long as one of its pieces touches
something. This asks it of every piece: each must cross, touch within TOL, or
sit inside some other piece, of any part, its own included. A bolt standing
off its flange, a vane stopping short of its casing, a control surface hung
in the air beside its hinge all fail here and pass there.

DETACHED is how many free pieces each part is known to have. The list only
gets shorter: more than it says fails, and fewer fails until --shrink lowers
it. --shrink never adds anything.
"""
import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PKG = "plane/parts"
UNIT = 33.0           # mm of full-size aeroplane per drawing unit (spec.SCALE_TO_FULL)
TOL = 0.3            # mm, full size: further off than this is not touching

# --- DETACHED: rewritten by --shrink, never by hand to add ---
DETACHED = {
    "duct_coupling": 5,
    "engine_blades_hpt_r1": 1,
    "engine_vanes_fan_ogv": 72,
    "fcs_fcc_envelope": 11,
    "flaperon_r": 1,
    "panel_screws": 80,
    "tailpipe_shroud": 4,
    "vg_l11": 1,
    "vg_l12": 1,
    "vg_l8": 1,
    "vg_r11": 1,
    "vg_r12": 1,
    "vg_r8": 1,
}
# --- end DETACHED ---

if __name__ == "__main__":
    import _interfere
    sys.exit(_interfere.support_main(__file__, ROOT, PKG, DETACHED, TOL, UNIT))
