"""Find parts whose solid bodies occupy the same space.

Bounding boxes cannot answer this. A turbocharger sits in the vee under the
plenum and its box overlaps by design; two pipes driven through each other
have boxes that barely touch. What matters is whether the material of one
part is inside another.

Two passes, cheap then exact.

1.  Every triangle of every part is rasterised into a shared voxel grid, so
    each voxel knows which parts have surface in it. Parts sharing voxels
    either touch or interpenetrate -- a filter, not an answer.

2.  For each candidate pair, the vertices of the smaller part that fall in
    shared voxels are tested against the larger one by counting how many
    times a ray leaving each vertex crosses its surface. Odd means inside.

The ray direction is fixed at +x and the target's triangles are bucketed by
the (y, z) they span, which is exactly the set a ray at that (y, z) can
cross. The triangle list must never be filtered by anything else: dropping
triangles opens the mesh, and a point outside an open mesh reads as inside.

Parts that are MEANT to be inside other parts -- a rib inside a wing skin, a
bolt in its hole, a flange bolted onto the casing it overlaps -- are declared
per project in EXPECTED, so what the report lists is only what nobody
intended.
"""

import glob
import importlib
import math
import os
import sys
from collections import Counter


def _tris(verts, faces):
    for f in faces:
        for k in range(1, len(f) - 1):
            yield verts[f[0]], verts[f[k]], verts[f[k + 1]]


def load_parts(root, pkg):
    """Import every part module under root/pkg and build it. No bpy."""
    sys.path.insert(0, os.path.join(root, os.path.dirname(pkg)))
    sys.path.insert(0, root)
    out = {}
    for f in sorted(glob.glob(os.path.join(root, pkg, "*.py"))):
        name = os.path.basename(f)[:-3]
        if name == "__init__":
            continue
        try:
            m = importlib.import_module(f"parts.{name}")
        except Exception:
            continue
        if not hasattr(m, "build"):
            continue
        try:
            built = m.build()
        except Exception:
            continue
        for k, v in built.items():
            if not k.startswith("cut:"):
                out[k] = v
    return out


def _voxels(verts, faces, h):
    out = set()
    for a, b, c in _tris(verts, faces):
        n = max(abs(a[i] - b[i]) for i in range(3))
        n = max(n, max(abs(a[i] - c[i]) for i in range(3)))
        n = max(n, max(abs(b[i] - c[i]) for i in range(3)))
        steps = min(24, max(1, int(n / h) + 1))
        for i in range(steps + 1):
            for j in range(steps + 1 - i):
                u = i / steps
                v = j / steps
                w = 1.0 - u - v
                out.add((int((a[0] * w + b[0] * u + c[0] * v) // h),
                         int((a[1] * w + b[1] * u + c[1] * v) // h),
                         int((a[2] * w + b[2] * u + c[2] * v) // h)))
    return out


def _inside(p, tri_list, eps=1e-9):
    px, py, pz = p
    hits = 0
    for (a, b, c) in tri_list:
        if (a[1] > py) == (b[1] > py) == (c[1] > py):
            continue
        if (a[2] > pz) == (b[2] > pz) == (c[2] > pz):
            continue
        e1 = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
        e2 = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
        hx, hy, hz = 0.0, -e2[2], e2[1]          # (1,0,0) x e2
        det = e1[0] * hx + e1[1] * hy + e1[2] * hz
        if -eps < det < eps:
            continue
        inv = 1.0 / det
        s = (px - a[0], py - a[1], pz - a[2])
        u = (s[0] * hx + s[1] * hy + s[2] * hz) * inv
        if u < 0.0 or u > 1.0:
            continue
        qx = s[1] * e1[2] - s[2] * e1[1]
        qy = s[2] * e1[0] - s[0] * e1[2]
        qz = s[0] * e1[1] - s[1] * e1[0]
        v = qx * inv
        if v < 0.0 or u + v > 1.0:
            continue
        if (e2[0] * qx + e2[1] * qy + e2[2] * qz) * inv > eps:
            hits += 1
    return hits % 2 == 1


def _expected(a, b, rules):
    for pa, pb in rules:
        if (a.startswith(pa) and b.startswith(pb)) or \
           (a.startswith(pb) and b.startswith(pa)):
            return True
    return False


def run(root, pkg, expected=(), limit=110, report=90, threshold=0.20):
    parts = load_parts(root, pkg)
    allv = [p for (v, _f) in parts.values() for p in v]
    lo = [min(p[i] for p in allv) for i in range(3)]
    hi = [max(p[i] for p in allv) for i in range(3)]
    h = math.dist(lo, hi) / 320.0

    vox = {}
    for name, (v, f) in parts.items():
        for key in _voxels(v, f, h):
            vox.setdefault(key, []).append(name)

    pair = Counter()
    for names in vox.values():
        if len(names) < 2:
            continue
        u = sorted(set(names))
        for i in range(len(u)):
            for j in range(i + 1, len(u)):
                pair[(u[i], u[j])] += 1

    found = []
    for (a, b), _shared in pair.most_common(limit):
        if _expected(a, b, expected):
            continue
        va, fa = parts[a]
        vb, fb = parts[b]
        A, B = a, b
        if len(va) > len(vb):
            A, B, va, fa, vb, fb = b, a, vb, fb, va, fa
        keys = {k for k, n in vox.items() if A in n and B in n}
        cand = [p for p in va
                if (int(p[0] // h), int(p[1] // h), int(p[2] // h)) in keys]
        if not cand:
            continue
        cell = h * 2.0
        buckets = {}
        for tri in _tris(vb, fb):
            y0 = min(t[1] for t in tri); y1 = max(t[1] for t in tri)
            z0 = min(t[2] for t in tri); z1 = max(t[2] for t in tri)
            for iy in range(int(y0 // cell), int(y1 // cell) + 1):
                for iz in range(int(z0 // cell), int(z1 // cell) + 1):
                    buckets.setdefault((iy, iz), []).append(tri)
        samp = cand[::max(1, len(cand) // 200)]
        n_in = 0
        for pt in samp:
            tb = buckets.get((int(pt[1] // cell), int(pt[2] // cell)))
            if tb and _inside(pt, tb):
                n_in += 1
        frac = n_in / len(samp)
        if frac >= threshold:
            found.append((frac, n_in, len(samp), A, B))

    found.sort(reverse=True)
    print(f"{len(parts)} parts, {len(pair)} pairs in contact, "
          f"voxel {h:.1f} mm")
    if found:
        print(f"\n{len(found)} pairs share material that nothing declared:")
        for frac, ni, ns, a, b in found[:report]:
            print(f"  {frac*100:3.0f}%  {ni:4d}/{ns:<4d}  {a}  inside  {b}")
    ok = not found
    print("\n" + ("PASS  no part occupies another's space"
                  if ok else "FAIL  parts are sharing material"))
    return ok
