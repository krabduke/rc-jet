"""The viewer's manifest must match the build it claims to describe.

viewer/parts.json is generated from build/parts.csv, but it is a committed
file that the page fetches directly, and nothing forced the two to agree:
`make manifest` ran *after* `make verify` in the `all` chain, so every run of
`make build && make verify` left the manifest a build behind.

It drifted, and stayed drifted across four projects at once. The V8's viewer
still listed `exhaust_manifolds`, a part deleted when the duplicate primaries
went; its cutaway hid `plenum`, which had become `plenum_l`/`plenum_r`; the RC
jet still carried `gps_puck`, `esc_40a` and `lipo_3s_1300` after the equipment
table was rewritten. The page looked right because a missing part is simply
not drawn -- exactly the failure that shows nothing.

So: regenerate the manifest to a scratch file and compare it byte for byte.
This touches nothing; a stale or hand-edited viewer/parts.json fails here.

    python3 tools/audit_manifest.py
"""

import os
import subprocess
import sys
import tempfile


def run(root):
    gen = os.path.join(root, "tools", "make_manifest.py")
    live = os.path.join(root, "viewer", "parts.json")
    if not os.path.exists(live):
        print(f"x   viewer/parts.json missing -- run `make manifest`")
        return False
    with tempfile.TemporaryDirectory() as tmp:
        fresh = os.path.join(tmp, "parts.json")
        r = subprocess.run([sys.executable, gen, fresh],
                           capture_output=True, text=True, cwd=root)
        if r.returncode:
            print("x   the manifest generator failed:\n" + (r.stderr or r.stdout))
            return False
        a, b = open(live, "rb").read(), open(fresh, "rb").read()
    if a == b:
        print(f"  ok   viewer/parts.json matches build/parts.csv "
              f"({len(a):,} bytes)")
        return True
    print("x   viewer/parts.json is stale -- run `make manifest` and commit it")
    _explain(live, fresh_bytes=b)
    return False


def _explain(live, fresh_bytes):
    """Name what changed, rather than reporting a byte count."""
    import json
    try:
        old = json.load(open(live))["parts"]
        new = json.loads(fresh_bytes.decode())["parts"]
    except Exception:
        return
    gone, added = sorted(set(old) - set(new)), sorted(set(new) - set(old))
    if gone:
        print(f"      {len(gone)} parts the viewer still lists: "
              + ", ".join(gone[:8]) + (" ..." if len(gone) > 8 else ""))
    if added:
        print(f"      {len(added)} parts the viewer has never heard of: "
              + ", ".join(added[:8]) + (" ..." if len(added) > 8 else ""))
    changed = [k for k in set(old) & set(new) if old[k] != new[k]]
    if changed:
        print(f"      {len(changed)} parts whose geometry or group moved: "
              + ", ".join(sorted(changed)[:8])
              + (" ..." if len(changed) > 8 else ""))
    if not (gone or added or changed):
        print("      same parts, different header -- a spec value changed")


if __name__ == "__main__":
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ok = run(root)
    print("\n" + ("PASS  the viewer describes the build it ships with"
                  if ok else "FAIL  the viewer and the build disagree"))
    sys.exit(0 if ok else 1)
