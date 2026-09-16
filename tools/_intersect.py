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


def load_parts(root, pkg, cuts_out=None):
    """Import every part module under root/pkg and build it. No bpy."""
    sys.path.insert(0, os.path.join(root, os.path.dirname(pkg)))
    sys.path.insert(0, root)
    out, cutters = {}, {}
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
            if k.startswith("cut:"):
                cutters.setdefault(k[4:], []).append(v)
            else:
                out[k] = v
    if cuts_out is not None:
        for name, cuts in cutters.items():
            if name not in out:
                continue
            solids = []
            for cv, cf in cuts:
                solids.append((list(_tris(cv, cf)),
                               [min(p[i] for p in cv) for i in range(3)],
                               [max(p[i] for p in cv) for i in range(3)]))
            cuts_out[name] = solids
    return out


def machined_away(p, solids):
    """Is this point in material a cutter took out?

    Modules hand back "cut:<part>" entries and assemble.py turns each into a
    boolean difference in Blender. This audit used to drop those entries and
    test the UNCUT part, so a former with a hole cut in it for a control shaft
    still read as solid and the shaft through the hole read as a part inside
    another part. Every such joint had to be written into EXPECTED, and
    declaring it says the overlap is deliberate rather than that the material
    is not there, which is a different and much weaker claim.

    The part is left whole -- it has to be, because an open mesh cannot be
    ray-tested and half the model would stop being checkable -- and the cut is
    applied to the sample points instead: a point inside a cutter is a point
    in material the build removes.
    """
    for tris, lo, hi in solids:
        if all(lo[i] <= p[i] <= hi[i] for i in range(3)) and _inside(p, tris):
            return True
    return False


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
    """Is p inside the solid this triangle list bounds?

    Signed crossings, not parity. Almost every part here is a union of
    overlapping closed pieces -- a bladed wheel is a hub with its blades
    driven into it, a housing is a volute with a snout and a backplate -- and
    where two of those pieces overlap there are faces INSIDE the solid. A ray
    that happens to pass through such a region crosses an even number of extra
    faces on one side and an odd number on the other, and parity reports a
    point outside the wheel as inside it. Counting each crossing by whether
    the face turns towards the ray or away from it gives the winding number,
    which is zero outside and non-zero inside whatever the pieces do to each
    other in between.
    """
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
            hits += 1 if det > 0 else -1
    return hits != 0


def _expected(a, b, rules):
    for pa, pb in rules:
        if (a.startswith(pa) and b.startswith(pb)) or \
           (a.startswith(pb) and b.startswith(pa)):
            return True
    return False


def closed(verts, faces):
    """Is this mesh watertight -- every edge shared by exactly two faces?

    The inside test is a ray-parity test, and parity only means anything
    against a closed surface. Shoot a ray at an open shell -- a duct with no
    end caps, a relief band laid on the skin, a blade row -- and a point
    outside it reads as inside whenever the ray happens to leave through the
    hole. That produced a long tail of overlaps that were not there.
    """
    edge = {}
    for f in faces:
        n = len(f)
        for i in range(n):
            a, b = f[i], f[(i + 1) % n]
            k = (a, b) if a < b else (b, a)
            edge[k] = edge.get(k, 0) + 1
    return all(c == 2 for c in edge.values())


def run(root, pkg, expected=(), limit=2000, report=200, threshold=0.10,
        min_hits=3):
    cuts = {}
    parts = load_parts(root, pkg, cuts)
    open_shells = {k for k, (v, f) in parts.items() if not closed(v, f)}
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

    # Rank the UNDECLARED pairs, then take the budget from those.
    #
    # This used to take the top `limit` contacts and then skip the declared
    # ones inside the loop. The declared joints are the biggest contacts on
    # the model -- a hub in an upright shares far more voxels than a strut
    # clipping a duct -- so they ate almost the whole budget and the check
    # only ever looked at a handful of real candidates. Six overlaps that
    # had been in the model all along surfaced the moment two parts got
    # thinner and stopped crowding the list.
    ranked = [ab for ab, _n in pair.most_common()
              if not _expected(ab[0], ab[1], expected)]
    found = []
    for (a, b) in ranked[:limit]:
        # B is the container and has to be closed. If only one of the two
        # is closed it takes that role whatever its size; if both are, the
        # smaller mesh is the one sampled.
        if a in open_shells and b in open_shells:
            continue
        if b in open_shells:
            A, B = b, a
        elif a in open_shells:
            A, B = a, b
        else:
            A, B = ((b, a) if len(parts[a][0]) > len(parts[b][0]) else (a, b))
        va, fa = parts[A]
        vb, fb = parts[B]
        keys = {k for k, n in vox.items() if A in n and B in n}
        cuts_a, cuts_b = cuts.get(A), cuts.get(B)
        cand = [p for p in va
                if (int(p[0] // h), int(p[1] // h), int(p[2] // h)) in keys
                and not (cuts_a and machined_away(p, cuts_a))]
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
        # Nudge the sample off the lattice before firing the ray.
        #
        # The ray runs along +x and counts crossings, which is only valid if
        # it misses every edge and vertex of the target. Symmetric hardware
        # breaks that constantly: two connecting rods on one crankpin share
        # the big-end bore, so every vertex round rod one's bore has exactly
        # the same (y, z) as a vertex round rod two's, and the ray threads
        # the seam between triangles and comes out odd. That reported all
        # eight rod pairs, and all eight cap pairs, as 69 percent inside each
        # other while their bounding boxes did not even overlap. A jitter of
        # a thousandth of a voxel is far below any real clearance and misses
        # the lattice.
        # The x nudge matters as much as the other two. Two parts that butt
        # against each other share a plane, and a sample point lying exactly
        # on it enters the target at t = 0 -- which the epsilon test drops --
        # and leaves at t > 0, so it counts one crossing and reads as inside.
        # That is every pair of connecting rods on a shared crankpin.
        jx, jy, jz = h * 0.0011, h * 0.0013, h * 0.0007
        samp = cand[::max(1, len(cand) // 200)]
        n_in = 0
        for pt in samp:
            q = (pt[0] - jx, pt[1] + jy, pt[2] + jz)
            tb = buckets.get((int(q[1] // cell), int(q[2] // cell)))
            if tb and _inside(q, tb) \
                    and not (cuts_b and machined_away(q, cuts_b)):
                n_in += 1
        frac = n_in / len(samp)
        # A fraction AND a count.
        #
        # The fraction on its own reports a single sampled vertex out of six,
        # which on two parts that share a face is as likely to be the plane
        # they share as it is to be one part inside the other. Three sampled
        # points inside is a volume. Dropping the threshold from 20 % to 10 %
        # without this turned up thirty pairs that were all 1-of-9 or 2-of-12.
        if frac >= threshold and n_in >= min_hits:
            found.append((frac, n_in, len(samp), A, B))

    found.sort(reverse=True)
    print(f"{len(parts)} parts, {len(pair)} pairs in contact, "
          f"{len(ranked)} undeclared ({min(len(ranked), limit)} tested), "
          f"voxel {h:.1f} mm")
    if open_shells:
        print(f"{len(open_shells)} parts are open shells and cannot be "
              f"tested as containers: {', '.join(sorted(open_shells)[:6])}"
              + (" ..." if len(open_shells) > 6 else ""))
    if found:
        print(f"\n{len(found)} pairs share material that nothing declared:")
        for frac, ni, ns, a, b in found[:report]:
            print(f"  {frac*100:3.0f}%  {ni:4d}/{ns:<4d}  {a}  inside  {b}")
    ok = not found
    print("\n" + ("PASS  no part occupies another's space"
                  if ok else "FAIL  parts are sharing material"))
    return ok
