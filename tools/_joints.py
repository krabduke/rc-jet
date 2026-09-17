"""Does the assembly hold together?

Every audit in these repos so far asks whether parts stay out of each other's
way: `audit_intersect` lists parts sharing material, `audit_clearance` and
`audit_fit` list parts that got too close. Those are all one-sided. A pipe
that stops 40 mm short of the flange it is supposed to bolt to passes every
one of them, because not touching is exactly what they are looking for.

That is the defect this module exists to find, and it is the commonest one in
a model built a part at a time: the flange moves, the pipe does not, and
nothing complains. It is also invisible in a render from any angle where
something else is in front of the joint.

Three things are checked here, all off one contact graph.

`collisions`  Two modules building a part under the same name. Assembly merges
              the module dictionaries, so the second silently replaces the
              first and a whole sub-assembly vanishes -- it is still in the
              source, still reviewed, and not in the model.

`orphan_cuts` A module declaring `cut:<part>` for a part some OTHER module
              builds. Assembly applies a module's cutters to that same
              module's objects, so a cutter aimed across a module boundary is
              dropped and the hole is never made.

`chains`      An ordered list of part groups which must form one continuous
              run of material -- air in to air out, oil pump to oil gallery,
              port to tailpipe. Consecutive groups have to be in contact. This
              is the check that answers "does the intake actually reach the
              engine", and it answers it about the built geometry rather than
              about the intent.

Contact is decided by surface proximity: every part's triangles are sampled
onto a grid whose cell is the tolerance, and two parts sharing a cell are
touching to within about that tolerance. Sampling the triangles rather than
taking the vertices matters -- a bolt head landing in the middle of one large
face has no vertex anywhere near the plate's own vertices.
"""

import fnmatch
import math


def _tris(verts, faces):
    for f in faces:
        for k in range(1, len(f) - 1):
            yield verts[f[0]], verts[f[k]], verts[f[k + 1]]


def _cells(verts, faces, h):
    """Every grid cell this part's surface passes through.

    A triangle is walked at the grid pitch rather than tested exactly: the
    result is a superset of the cells it truly crosses only by rounding, and
    contact is a proximity question anyway.
    """
    out = set()
    inv = 1.0 / h
    for a, b, c in _tris(verts, faces):
        e = max(max(abs(a[i] - b[i]), abs(a[i] - c[i]), abs(b[i] - c[i]))
                for i in range(3))
        # 40, not 16. The cap is there to stop one enormous triangle
        # costing everything, but at 16 a 300 mm triangle is sampled every
        # 19 mm on a 2 mm grid -- so the rasterisation has holes in it and
        # two parts that genuinely touch can be reported as separate. That
        # is a false PASS on the assembly check and a false failure on a
        # circuit, which is the worst way for this tool to be wrong.
        n = min(40, int(e * inv) + 1)
        for i in range(n + 1):
            for j in range(n + 1 - i):
                u, v = i / n, j / n
                w = 1.0 - u - v
                out.add((int(math.floor((a[0] * w + b[0] * u + c[0] * v) * inv)),
                         int(math.floor((a[1] * w + b[1] * u + c[1] * v) * inv)),
                         int(math.floor((a[2] * w + b[2] * u + c[2] * v) * inv))))
    return out


def contact_graph(parts, tol):
    """name -> set of names whose surface comes within about `tol`.

    Cells are dilated by one step along each axis before they are compared, so
    two surfaces that fall either side of a cell boundary still meet. Without
    that the answer depends on where the origin happens to be -- and the
    dilation is done once per occupied cell rather than once per surface
    sample, which is the difference between eight lookups and eight inserts
    for every triangle in the model.
    """
    occ = {}
    for name, (v, f) in parts.items():
        for cell in _cells(v, f, tol):
            occ.setdefault(cell, set()).add(name)
    graph = {k: set() for k in parts}
    for (cx, cy, cz) in occ:
        near = None
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    got = occ.get((cx + dx, cy + dy, cz + dz))
                    if got:
                        near = set(got) if near is None else near | got
        if near is None or len(near) < 2:
            continue
        for a in near:
            graph[a] |= near
    for k in graph:
        graph[k].discard(k)
    return graph


def components(graph):
    """Connected groups of parts, largest first."""
    seen, out = set(), []
    for start in graph:
        if start in seen:
            continue
        stack, group = [start], set()
        seen.add(start)
        while stack:
            n = stack.pop()
            group.add(n)
            for m in graph[n]:
                if m not in seen:
                    seen.add(m)
                    stack.append(m)
        out.append(group)
    out.sort(key=len, reverse=True)
    return out


def match(parts, pattern):
    """Part names matching a glob. A bare name must exist."""
    if any(c in pattern for c in "*?["):
        return sorted(n for n in parts if fnmatch.fnmatchcase(n, pattern))
    return [pattern] if pattern in parts else []


def broken_links(parts, graph, chain):
    """Consecutive groups in `chain` that are not in contact.

    Returns (index, left pattern, right pattern, why) per break, so the report
    can name the link rather than just the circuit.
    """
    out = []
    groups = [(p, match(parts, p)) for p in chain]
    for i in range(len(groups) - 1):
        (pa, A), (pb, B) = groups[i], groups[i + 1]
        if not A:
            out.append((i, pa, pb, f"nothing is named {pa}"))
            continue
        if not B:
            out.append((i, pa, pb, f"nothing is named {pb}"))
            continue
        if any(b in graph[a] for a in A for b in B):
            continue
        out.append((i, pa, pb, "no contact"))
    return out


# --------------------------------------------------------------------------
# the gap, measured, for a link that failed
# --------------------------------------------------------------------------

def _pt_tri(p, a, b, c):
    ab = [b[i] - a[i] for i in range(3)]
    ac = [c[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    d1 = sum(ab[i] * ap[i] for i in range(3))
    d2 = sum(ac[i] * ap[i] for i in range(3))
    if d1 <= 0 and d2 <= 0:
        return math.dist(p, a)
    bp = [p[i] - b[i] for i in range(3)]
    d3 = sum(ab[i] * bp[i] for i in range(3))
    d4 = sum(ac[i] * bp[i] for i in range(3))
    if d3 >= 0 and d4 <= d3:
        return math.dist(p, b)
    vc = d1 * d4 - d3 * d2
    if vc <= 0 <= d1 and d3 <= 0:
        t = d1 / (d1 - d3) if d1 != d3 else 0.0
        return math.dist(p, [a[i] + t * ab[i] for i in range(3)])
    cp = [p[i] - c[i] for i in range(3)]
    d5 = sum(ab[i] * cp[i] for i in range(3))
    d6 = sum(ac[i] * cp[i] for i in range(3))
    if d6 >= 0 and d5 <= d6:
        return math.dist(p, c)
    vb = d5 * d2 - d1 * d6
    if vb <= 0 <= d2 and d6 <= 0:
        t = d2 / (d2 - d6) if d2 != d6 else 0.0
        return math.dist(p, [a[i] + t * ac[i] for i in range(3)])
    va = d3 * d6 - d5 * d4
    if va <= 0 and (d4 - d3) >= 0 and (d5 - d6) >= 0:
        den = (d4 - d3) + (d5 - d6)
        t = (d4 - d3) / den if den else 0.0
        return math.dist(p, [b[i] + t * (c[i] - b[i]) for i in range(3)])
    den = va + vb + vc
    u, w = vb / den, vc / den
    return math.dist(p, [a[i] + u * ab[i] + w * ac[i] for i in range(3)])


def gap(A, B, budget=60000):
    """Roughly the smallest distance between two surfaces.

    Vertices of each against triangles of the other, both decimated to stay
    inside `budget` point-triangle tests. This runs only on links that have
    already failed, so it is a diagnostic number for the report and not
    something the verdict depends on.
    """
    best = float("inf")
    for (pv, _pf), (qv, qf) in ((A, B), (B, A)):
        qt = list(_tris(qv, qf))
        if not qt or not pv:
            continue
        step_p = max(1, len(pv) * len(qt) // max(budget, 1))
        pts = pv[::step_p] or pv[:1]
        step_q = max(1, len(pts) * len(qt) // max(budget, 1))
        tris = qt[::step_q] or qt[:1]
        for p in pts:
            for t in tris:
                d = _pt_tri(p, *t)
                if d < best:
                    best = d
    return best


# --------------------------------------------------------------------------
# loading, with the bookkeeping the other loaders throw away
# --------------------------------------------------------------------------

def load(root, pkg):
    """Build every part module and record who built what.

    `_intersect.load_parts` merges the modules into one dictionary and
    swallows the exceptions, which is right for what it does and loses the
    two facts this audit needs: which module each name came from, and which
    module declared each cutter. Both only exist before the merge.

    Returns (parts, collisions, cut_owner, failures).
    """
    import glob
    import importlib
    import os
    import sys

    sys.path.insert(0, os.path.join(root, os.path.dirname(pkg)))
    sys.path.insert(0, root)
    pkgname = os.path.basename(pkg.rstrip("/"))

    parts, built_by, collisions, cut_owner, failures = {}, {}, [], {}, []
    for path in sorted(glob.glob(os.path.join(root, pkg, "*.py"))):
        name = os.path.basename(path)[:-3]
        if name == "__init__":
            continue
        try:
            module = importlib.import_module(f"{pkgname}.{name}")
        except Exception as exc:
            failures.append((name, f"{type(exc).__name__}: {exc}"))
            continue
        if not hasattr(module, "build"):
            continue
        try:
            built = module.build()
        except Exception as exc:
            failures.append((name, f"{type(exc).__name__}: {exc}"))
            continue
        for key, geom in built.items():
            if key.startswith("cut:"):
                cut_owner.setdefault(key[4:], []).append(name)
                continue
            if key in built_by:
                collisions.append((key, built_by[key], name))
            built_by[key] = name
            parts[key] = geom
    return parts, collisions, cut_owner, built_by, failures
