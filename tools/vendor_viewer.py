"""Copy the shared viewer modules into a project's viewer/ directory.

The hypercar and the RC jet share the flow field, the CFD view and the tunnel
panel. They were hand-copied, which is how the engines ended up three modules
apart, so the copy is scripted and digested: `check()` compares each copy with
both its recorded digest and the file in _shared/viewer, so editing a copy or
letting the source move on both fail the build.

    python3 tools/vendor_viewer.py
"""

import hashlib
import json
import os
import shutil
import sys

FILES = ["flowfield.js", "panelkernel.js", "panelflow.js", "paneltunnel.js",
         "cfd-view.js", "tunnel-ui.js", "anim.js"]

# The solver's own validation travels with the solver. A check that lives
# only in _shared is a check this project's `make verify` does not run, and
# the point of vendoring is that what ships is what was tested.
TOOLS = ["check_panelkernel.mjs", "check_panelflow.mjs"]


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()[:12]


def _src(root):
    return os.path.join(os.path.dirname(root), "_shared", "viewer")


def vendor(root):
    src = _src(root)
    if not os.path.isdir(src):
        raise SystemExit(f"no shared viewer sources at {src}")
    dst = os.path.join(root, "viewer")
    man = {"source": "_shared/viewer", "files": {}}
    for f in FILES:
        s = os.path.join(src, f)
        shutil.copy2(s, os.path.join(dst, f))
        man["files"][f] = digest(s)
    tsrc = os.path.join(os.path.dirname(src), "tools")
    for f in TOOLS:
        s = os.path.join(tsrc, f)
        # the shared tools import '../viewer/x.js', which resolves the same
        # way from this project's tools/ as it does from _shared/tools/
        shutil.copy2(s, os.path.join(root, "tools", f))
        man["files"]["../tools/" + f] = digest(s)
    with open(os.path.join(dst, "VIEWER_VENDOR.json"), "w") as fh:
        json.dump(man, fh, indent=1)
    return man


def check(root):
    p = os.path.join(root, "viewer", "VIEWER_VENDOR.json")
    if not os.path.exists(p):
        return False, "viewer/VIEWER_VENDOR.json missing -- run `make vendor-viewer`"
    man = json.load(open(p))
    src = _src(root)
    reachable = os.path.isdir(src)
    bad = []
    for rel, want in man["files"].items():
        here = os.path.normpath(os.path.join(root, "viewer", rel))
        if not os.path.exists(here):
            bad.append(rel + " (missing)")
        elif digest(here) != want:
            bad.append(rel + " (edited in place)")
        if reachable:
            up = os.path.normpath(os.path.join(src, rel))
            if os.path.exists(up) and digest(up) != want:
                bad.append(rel + " (shared copy moved on)")
    if bad:
        return False, "run `make vendor-viewer`: " + ", ".join(bad[:4])
    if not reachable:
        # a checkout of this project alone still verifies its own copies
        return True, (f"{len(man['files'])} viewer modules match their "
                      "manifest (shared source not reachable)")
    return True, f"{len(man['files'])} shared viewer modules, matching source"


if __name__ == "__main__":
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    m = vendor(root)
    print(f"vendored {len(m['files'])} viewer modules from {m['source']}")
