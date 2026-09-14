"""Fail the build if a vendored copy has drifted.

    python3 tools/check_vendor.py
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

fails = 0
for mod, label in (("vendor_viewer", "viewer modules"),
                   ("vendor_engine", "power unit"),
                   ("vendor_f110", "turbofan")):
    try:
        m = __import__(mod)
    except ImportError:
        continue
    ok, why = m.check(ROOT) if mod == "vendor_viewer" else m.check()
    print(f"  {'ok  ' if ok else 'x   '}{label:16s} {why}")
    if not ok:
        fails += 1

print("\n" + ("PASS  every vendored copy matches its source"
              if not fails else f"FAIL  {fails} vendored copies have drifted"))
sys.exit(0 if not fails else 1)
