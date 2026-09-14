"""Copy the F110 engine sources into f110/.

The aeroplane does not re-model its engine: it imports the generators from the
sibling turbofan project and positions the result. That only stays true if the
copy in f110/ is actually the engine, and this copy was unscripted -- made once
by hand, with nothing to say which version it was or whether it had drifted.
The car had the same arrangement and was found carrying an engine three
modules and 111 parts behind.

So the copy is scripted, and it records which commit it came from. `make
vendor` refreshes it; verify.py checks the manifest still matches what is on
disk, so a stale copy fails the build instead of quietly shipping.

    python3 tools/vendor_f110.py [path-to-turbofan-repo]
"""

import hashlib
import json
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_SUBDIR = "engine"
DEFAULT_SRC = os.path.join(os.path.dirname(ROOT), "3d jet engine")

# What the car needs in order to build the engine. Anything the engine's own
# assemble.py pulls in has to be here, or the import fails at build time.
FILES = ["spec.py", "mesh.py", "airfoil.py", "materials.py"]
PART_MODULES = ["__init__.py", "common.py", "rotating.py", "statics.py",
                "combustor.py", "turbine.py", "augmentor.py", "nozzle.py",
                "accessories.py"]


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        h.update(fh.read())
    return h.hexdigest()[:12]


def vendor(src=DEFAULT_SRC):
    eng = os.path.join(src, "engine")
    if not os.path.isdir(eng):
        raise SystemExit(f"no engine sources at {eng}")
    dst = os.path.join(ROOT, "f110")
    os.makedirs(os.path.join(dst, "parts"), exist_ok=True)

    manifest = {"source": os.path.basename(src), "files": {}}
    try:
        manifest["commit"] = subprocess.check_output(
            ["git", "-C", src, "rev-parse", "--short", "HEAD"],
            text=True).strip()
    except Exception:
        manifest["commit"] = "unknown"

    for f in FILES:
        s = os.path.join(eng, f)
        if not os.path.exists(s):
            continue
        shutil.copy2(s, os.path.join(dst, f))
        manifest["files"][f] = digest(s)
    for f in PART_MODULES:
        s = os.path.join(eng, "parts", f)
        if not os.path.exists(s):
            continue
        shutil.copy2(s, os.path.join(dst, "parts", f))
        manifest["files"]["parts/" + f] = digest(s)

    # drop any stale __pycache__, which will happily shadow a changed module
    for d, _, _ in os.walk(dst):
        if d.endswith("__pycache__"):
            shutil.rmtree(d, ignore_errors=True)

    with open(os.path.join(dst, "VENDOR.json"), "w") as fh:
        json.dump(manifest, fh, indent=1)
    return manifest


def check(src=DEFAULT_SRC):
    """True if the vendored copy matches both its manifest and its source.

    The first version compared each vendored file against a digest of itself,
    which catches someone editing the copy but is blind to the thing that
    actually went wrong: the source moving on while the copy stays put. That
    is self-referential -- it can only ever say "this copy is the copy it was
    when it was made". So it now also diffs against the upstream sources when
    they are reachable, and says so when they are not.
    """
    p = os.path.join(ROOT, "f110", "VENDOR.json")
    if not os.path.exists(p):
        return False, "f110/VENDOR.json missing -- run `make vendor`"
    man = json.load(open(p))
    bad = []
    for rel, want in man["files"].items():
        f = os.path.join(ROOT, "f110", rel)
        if not os.path.exists(f):
            bad.append(rel + " (missing)")
        elif digest(f) != want:
            bad.append(rel + " (changed)")
    if bad:
        return False, ", ".join(bad[:4])

    eng = os.path.join(src, SRC_SUBDIR)
    if not os.path.isdir(eng):
        return True, (f"{len(man['files'])} files from "
                      f"{man.get('commit', '?')} (source not reachable, "
                      "upstream drift unchecked)")
    drift = []
    for rel in man["files"]:
        up = os.path.join(eng, rel)
        if not os.path.exists(up):
            drift.append(rel + " (gone upstream)")
        elif digest(up) != man["files"][rel]:
            drift.append(rel + " (upstream changed)")
    if drift:
        return False, "stale, run `make vendor`: " + ", ".join(drift[:4])
    return True, (f"{len(man['files'])} files from "
                  f"{man.get('commit', '?')}, matching source")


if __name__ == "__main__":
    m = vendor(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SRC)
    print(f"vendored {len(m['files'])} files from {m['source']} @ {m['commit']}")
