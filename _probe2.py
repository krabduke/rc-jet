import sys, traceback
sys.path.insert(0, 'plane')
from parts import systems
for fn in ("_fuel_system", "_avionics", "_cockpit", "_aerials"):
    try:
        d = getattr(systems, fn)()
        print(f"{fn}: OK -> {sorted(d)}")
    except Exception as e:
        print(f"{fn}: FAIL {type(e).__name__}: {e}")
        traceback.print_exc()
