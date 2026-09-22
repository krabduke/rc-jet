"""Load a model's parts for the audits, without Blender.

The interference check that used to live here -- a voxel filter, then a
signed ray count on up to 200 sampled vertices of the smaller part of each
pair -- has been replaced by tools/_interfere.py, which is exact. What stays is
what the other audits share: building every part module's geometry, the
EXPECTED rule match, and the test for material a cutter removes.
"""

import glob
import importlib
import os
import sys


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
        # A module that will not import or build is an error, not an empty
        # module. Skipping it used to take every part it makes out of every
        # audit at once, and each of them would then pass on nothing.
        m = importlib.import_module(f"parts.{name}")
        if not hasattr(m, "build"):
            continue
        built = m.build()
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
