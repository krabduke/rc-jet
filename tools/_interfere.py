"""Exact interference and support checks, run inside Blender.

    python3 tools/audit_intersect.py      (re-runs itself under Blender)
    python3 tools/audit_support.py

This replaced a pure-Python check that sampled up to 200 vertices of the
smaller part of each pair, counted how many fell inside the other by a signed
ray crossing, and reported the pair only if a tenth of them, and at least
three, were inside. It passed on all four models while every one of them had
parts through parts. It was blind in four ways:

*   Signed crossings. A point inside a closed piece whose faces were wound
    inside-out scores -1, so it reads as outside, and a point in two
    overlapping pieces, one of them inside-out, scores 0. Two in five closed
    pieces across the four models were built inside-out.
*   It only looked at a pair whose SURFACES shared a voxel. A part buried
    entirely inside another -- an oil line inside a gearbox, a bearing in a
    hub with no bore -- has no surface near the other's, and was never tested.
*   The threshold. A tenth of 200 samples is twenty points: a bolt poking
    through the far side of a flange, or a strut clipping a duct, is a few.
*   One direction. Only the smaller part's vertices were tested, so a thin
    part punched straight through by a big one read as clear.

What is done instead, with mathutils' BVH, which is exact:

1.  Every pair whose boxes meet is tested -- not only pairs whose surfaces
    touch -- for triangles that cross (BVHTree.overlap) and for one part
    lying wholly inside the other.
2.  Depth, both ways. A vertex of either part inside the other's material is
    found by ray PARITY against each closed piece on its own, so winding
    cannot fool it, and its depth is its distance to the other's surface.
    Where triangles cross, the depth is also taken as how far each crosses
    the other's plane -- which is what catches a pipe driven through a thin
    wall, where no vertex of either is inside the other.
3.  Material a cutter removes ("cut:<part>" from a module) is not material.

A pair fails when its depth is at least the tolerance. Flush faces, a nut on
its washer and a pin in its bore meet at zero depth and pass.
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _intersect


# --------------------------------------------------------------------------
# Running under Blender
# --------------------------------------------------------------------------

def in_blender():
    try:
        import mathutils  # noqa: F401
        return True
    except ImportError:
        return False


def blender_exe():
    for cand in (os.environ.get("BLENDER"), shutil.which("blender"),
                 "/Applications/Blender.app/Contents/MacOS/Blender"):
        if cand and os.path.exists(cand):
            return cand
    sys.exit("FAIL  Blender is needed for this audit: set BLENDER or put it on PATH")


def rerun_in_blender(script, args):
    """Run `script` again inside Blender and exit with its status."""
    cmd = [blender_exe(), "-b", "--factory-startup", "--python-exit-code", "1",
           "-P", os.path.abspath(script), "--"] + list(args)
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True)
    for line in p.stdout:
        if line.startswith(("Blender ", "Read prefs", "Read blend")) or \
                line.strip() == "Blender quit":
            continue
        sys.stdout.write(line)
    sys.exit(p.wait())


def script_args():
    return sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else sys.argv[1:]


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------

def _islands(verts, faces):
    """Split a part into its closed pieces: [(verts, faces)] per piece."""
    par = list(range(len(verts)))

    def find(i):
        while par[i] != i:
            par[i] = par[par[i]]
            i = par[i]
        return i
    for f in faces:
        r0 = find(f[0])
        for j in f[1:]:
            rj = find(j)
            if rj != r0:
                par[rj] = r0
    groups = {}
    for f in faces:
        groups.setdefault(find(f[0]), []).append(f)
    out = []
    for fs in groups.values():
        ids = sorted({i for f in fs for i in f})
        m = {i: k for k, i in enumerate(ids)}
        out.append(([tuple(verts[i]) for i in ids],
                    [tuple(m[i] for i in f) for f in fs]))
    return out


def _box(vs):
    return ([min(p[i] for p in vs) for i in range(3)],
            [max(p[i] for p in vs) for i in range(3)])


class Model:
    def __init__(self, root, pkg):
        from mathutils.bvhtree import BVHTree
        from mathutils import Vector
        self.V = Vector
        cuts = {}
        self.parts = _intersect.load_parts(root, pkg, cuts)
        self.names = sorted(self.parts)
        self.tree, self.box, self.isl = {}, {}, {}
        for n in self.names:
            v, f = self.parts[n]
            self.tree[n] = BVHTree.FromPolygons([tuple(p) for p in v],
                                                [tuple(x) for x in f],
                                                epsilon=0.0)
            self.box[n] = _box(v)
            self.isl[n] = [(BVHTree.FromPolygons(iv, if_, epsilon=0.0),) + _box(iv) + (iv,)
                           for iv, if_ in _islands(v, f)]
        # Cutters are tested a closed piece at a time, like the parts: a
        # cutter is often several solids joined -- the bores of a bank and
        # the crank's swept space -- and where two of them overlap, parity
        # against the whole mesh counts two surfaces and calls the point
        # uncut.
        self.cut = {}
        for n, solids in cuts.items():
            L = []
            for tris, lo, hi in solids:
                vs = [tuple(p) for t in tris for p in t]
                fs = [(3 * i, 3 * i + 1, 3 * i + 2) for i in range(len(tris))]
                welded, idx = {}, []
                for p in vs:
                    k = (round(p[0], 6), round(p[1], 6), round(p[2], 6))
                    idx.append(welded.setdefault(k, len(welded)))
                wv = [None] * len(welded)
                for k, i in welded.items():
                    wv[i] = k
                wf = [tuple(idx[j] for j in f) for f in fs]
                for iv, if_ in _islands(wv, wf):
                    L.append((BVHTree.FromPolygons(iv, if_, epsilon=0.0),) + _box(iv))
            self.cut[n] = L
        lo = [min(self.box[n][0][i] for n in self.names) for i in range(3)]
        hi = [max(self.box[n][1][i] for n in self.names) for i in range(3)]
        self.size = max(hi[i] - lo[i] for i in range(3))
        # A ray restarted from a hit must clear it. The trees are single
        # precision, so a fixed nudge of 1e-6 at a coordinate of 4000 lands
        # back on the same triangle and the parity comes out as noise.
        self.step = max(2e-3, self.size * 2e-6)
        self.rays = (Vector((0.57735, 0.57751, 0.57719)).normalized(),
                     Vector((-0.6123, 0.4518, -0.6488)).normalized())

    # -- inside tests ------------------------------------------------------
    def _parity(self, tree, p, d):
        c, o = 0, self.V(p)
        for _ in range(400):
            loc, _n, _i, _d = tree.ray_cast(o, d)
            if loc is None:
                break
            c += 1
            o = loc + d * self.step
        return c & 1

    def _in_solid(self, tree, lo, hi, p):
        if not all(lo[i] <= p[i] <= hi[i] for i in range(3)):
            return False
        return all(self._parity(tree, p, d) for d in self.rays)

    def cut_away(self, name, p):
        return any(self._in_solid(t, lo, hi, p) for t, lo, hi in self.cut.get(name, ()))

    def inside(self, name, p):
        """In the material of `name`: inside one of its closed pieces, and not
        in anything a cutter takes out of it."""
        return (any(self._in_solid(t, lo, hi, p) for t, lo, hi, _v in self.isl[name])
                and not self.cut_away(name, p))

    def meet(self, a, b, margin=0.0):
        (al, ah), (bl, bh) = self.box[a], self.box[b]
        return all(al[i] <= bh[i] + margin and bl[i] <= ah[i] + margin
                   for i in range(3))

    # -- depth -------------------------------------------------------------
    def _vertex_depth(self, a, b, stop=None, cap=20000):
        """Deepest vertex of a inside b's material, and where it is.

        At most `cap` vertices of a are tried, spread evenly, which is every
        vertex of all but the largest parts; the crossing depth covers the
        case sampling could miss, a thin wall with a tube through it. With
        `stop`, it returns as soon as a vertex that deep is found."""
        lo, hi = self.box[b]
        cand = [p for p in self.parts[a][0]
                if all(lo[i] <= p[i] <= hi[i] for i in range(3))]
        best, where = 0.0, None
        for p in cand[::max(1, len(cand) // cap)]:
            if not self.inside(b, p) or self.cut_away(a, p):
                continue
            d = self.tree[b].find_nearest(self.V(p))[3]
            if d > best:
                best, where = d, p
                if stop is not None and best >= stop:
                    break
        return best, where

    def _crossing_depth(self, a, b, hits):
        """How far crossing faces of a and b pass through each other."""
        va, fa = self.parts[a]
        vb, fb = self.parts[b]

        def plane(vs, f):
            # Newell normal: good for any planar or near-planar polygon
            nx = ny = nz = 0.0
            for i in range(len(f)):
                p, q = vs[f[i]], vs[f[(i + 1) % len(f)]]
                nx += (p[1] - q[1]) * (p[2] + q[2])
                ny += (p[2] - q[2]) * (p[0] + q[0])
                nz += (p[0] - q[0]) * (p[1] + q[1])
            ln = (nx * nx + ny * ny + nz * nz) ** 0.5
            if ln < 1e-12:
                return None
            n = (nx / ln, ny / ln, nz / ln)
            c = vs[f[0]]
            return n, n[0] * c[0] + n[1] * c[1] + n[2] * c[2]

        def straddle(vs, f, pl):
            n, d0 = pl
            ds = [n[0] * vs[i][0] + n[1] * vs[i][1] + n[2] * vs[i][2] - d0 for i in f]
            return min(max(ds), -min(ds))

        best, where = 0.0, None
        step = max(1, len(hits) // 20000)
        for ia, ib in hits[::step]:
            pa, pb = plane(va, fa[ia]), plane(vb, fb[ib])
            if pa is None or pb is None:
                continue
            s = min(straddle(va, fa[ia], pb), straddle(vb, fb[ib], pa))
            if s <= best:
                continue
            c = [sum(va[i][k] for i in fa[ia]) / len(fa[ia]) for k in range(3)]
            if self.cut_away(a, c) or self.cut_away(b, c):
                continue
            best, where = s, c
        return best, where

    def interference(self, tol, declared=lambda a, b: False):
        """Every pair sharing material at least `tol` deep:
        {(a, b): (depth, where)} with a < b.

        A declared pair is only asked whether it reaches `tol` -- that is all
        the rule check needs -- so it stops at the first point that deep."""
        out = {}
        for i, a in enumerate(self.names):
            for b in self.names[i + 1:]:
                if not self.meet(a, b, tol):
                    continue
                hits = self.tree[a].overlap(self.tree[b])
                if not hits:
                    # no crossing: either clear, or one wholly inside the other
                    va, vb = self.parts[a][0], self.parts[b][0]
                    if not (self.inside(b, va[0]) or self.inside(a, vb[0])):
                        continue
                stop = tol if declared(a, b) else None
                d, w = self._crossing_depth(a, b, hits) if hits else (0.0, None)
                for x, y in ((a, b), (b, a)):
                    if stop is not None and d >= stop:
                        break
                    dx, wx = self._vertex_depth(x, y, stop)
                    if dx > d:
                        d, w = dx, wx
                if d >= tol:
                    out[(a, b)] = (d, w)
        return out

    # -- support -----------------------------------------------------------
    def detached(self, tol):
        """Closed pieces that touch nothing: no crossing with, no vertex within
        `tol` of, and not inside, any other piece of any part (its own part's
        other pieces included). {part: [(gap_lower_bound, centre), ...]}"""
        pieces = [(n, t, (lo, hi), iv)
                  for n in self.names for t, lo, hi, iv in self.isl[n]]
        cell = max(self.size / 64.0, 1e-6)
        grid = {}

        def cells(lo, hi, m):
            for x in range(int((lo[0] - m) // cell), int((hi[0] + m) // cell) + 1):
                for y in range(int((lo[1] - m) // cell), int((hi[1] + m) // cell) + 1):
                    for z in range(int((lo[2] - m) // cell), int((hi[2] + m) // cell) + 1):
                        yield (x, y, z)
        for k, (_n, _t, (lo, hi), _v) in enumerate(pieces):
            for c in cells(lo, hi, 0.0):
                grid.setdefault(c, []).append(k)
        out = {}
        for k, (n, t, (lo, hi), vs) in enumerate(pieces):
            near = set()
            for c in cells(lo, hi, tol):
                near.update(grid.get(c, ()))
            near.discard(k)
            near = [j for j in near
                    if all(pieces[j][2][0][i] <= hi[i] + tol and
                           lo[i] <= pieces[j][2][1][i] + tol for i in range(3))]
            ok = any(t.overlap(pieces[j][1]) for j in near)
            if not ok:
                for j in near:
                    tj = pieces[j][1]
                    if any(tj.find_nearest(self.V(p), tol)[0] is not None
                           for p in vs):
                        ok = True
                        break
            if not ok:
                for j in near:
                    jl, jh = pieces[j][2]
                    if self._in_solid(pieces[j][1], jl, jh, vs[0]):
                        ok = True
                        break
            if not ok:
                centre = [round((lo[i] + hi[i]) / 2, 2) for i in range(3)]
                out.setdefault(n, []).append(centre)
        return out


# --------------------------------------------------------------------------
# Baselines
# --------------------------------------------------------------------------

def rewrite_block(path, start, end, body):
    """Replace the lines between two marker comments in a source file."""
    src = open(path).read()
    i = src.index(start) + len(start)
    j = src.index(end)
    open(path, "w").write(src[:i] + "\n" + body + src[j:])


def _fmt_known(entries, unit):
    """KNOWN block body: one pair a line, deepest first, with where it is."""
    lines = []
    for (a, b), (mm, w) in sorted(entries.items(), key=lambda kv: -kv[1][0]):
        at = "" if w is None else "   # at (%s)" % ", ".join("%.1f" % c for c in w)
        lines.append('    ("%s", "%s"): %.1f,%s' % (a, b, mm, at))
    return "KNOWN = {\n" + "\n".join(lines) + ("\n" if lines else "") + "}\n"


def _fmt_detached(entries):
    lines = ['    "%s": %d,' % (n, k) for n, k in sorted(entries.items())]
    return "DETACHED = {\n" + "\n".join(lines) + ("\n" if lines else "") + "}\n"


KNOWN_START = "# --- KNOWN: rewritten by --shrink, never by hand to add ---"
KNOWN_END = "# --- end KNOWN ---"
DET_START = "# --- DETACHED: rewritten by --shrink, never by hand to add ---"
DET_END = "# --- end DETACHED ---"


def intersect_main(path, root, pkg, expected, known, tol_mm, unit):
    """The gate for audit_intersect.py. See that file for what it enforces."""
    if not in_blender():
        rerun_in_blender(path, script_args())
    shrink = "--shrink" in script_args()
    m = Model(root, pkg)
    tol = tol_mm / unit
    rules = list(dict.fromkeys(tuple(r) for r in expected))

    def has(prefix):
        return any(n.startswith(prefix) for n in m.names)
    orphan = [r for r in rules if not (has(r[0]) and has(r[1]))]

    found = m.interference(
        tol, declared=lambda a, b: _intersect._expected(a, b, expected))
    live = set()
    undeclared = {}
    for (a, b), (d, w) in found.items():
        hit = [r for r in rules
               if (a.startswith(r[0]) and b.startswith(r[1])) or
                  (a.startswith(r[1]) and b.startswith(r[0]))]
        if hit:
            live.update(hit)
        else:
            undeclared[(a, b)] = (round(d * unit, 1), w)
    dead = [r for r in rules if r not in live and r not in orphan]

    new = {k: v for k, v in undeclared.items() if k not in known}
    worse = {k: v for k, v in undeclared.items()
             if k in known and v[0] > known[k] + max(tol_mm, 0.05 * known[k])}
    fixed = [k for k in known if k not in undeclared]

    print(f"{len(m.names)} parts, {len(found)} pairs share material at least "
          f"{tol_mm:g} mm deep: {len(found) - len(undeclared)} declared in "
          f"EXPECTED, {len(undeclared)} known defects")
    if shrink:
        kept = {k: (min(known[k], v[0]), v[1]) for k, v in undeclared.items()
                if k in known}
        rewrite_block(path, KNOWN_START, KNOWN_END, _fmt_known(kept, unit))
        print(f"KNOWN shrunk to {len(kept)} entries ({len(known) - len(kept)} "
              f"removed); nothing was added")
        fixed = []
        known = {k: v[0] for k, v in kept.items()}
    if new:
        print(f"\n{len(new)} pairs share material that nothing declared:")
        for (a, b), (mm, w) in sorted(new.items(), key=lambda kv: -kv[1][0]):
            print(f"  {mm:8.1f} mm  {a}  x  {b}   at {tuple(round(c, 1) for c in w)}")
    if worse:
        print(f"\n{len(worse)} known defects got deeper:")
        for k, (mm, _w) in worse.items():
            print(f"  {known[k]:8.1f} -> {mm:.1f} mm  {k[0]}  x  {k[1]}")
    if fixed:
        print(f"\n{len(fixed)} known defects are fixed -- run with --shrink "
              f"to take them off the list:")
        for a, b in fixed[:40]:
            print(f"  {a}  x  {b}")
    if dead:
        print(f"\n{len(dead)} EXPECTED rules excuse nothing -- delete them:")
        for r in dead[:60]:
            print(f"  {r}")
    if orphan:
        print(f"\n{len(orphan)} EXPECTED rules name a part that does not exist:")
        for r in orphan[:60]:
            print(f"  {r}")
    ok = not (new or worse or fixed or dead or orphan)
    print("\n" + ("PASS  no part occupies another's space beyond what is declared "
                  f"or on the known-defect list ({len(known)} entries)"
                  if ok else "FAIL  parts are sharing material"))
    return 0 if ok else 1


def support_main(path, root, pkg, detached_known, tol_mm, unit):
    """The gate for audit_support.py. See that file for what it enforces."""
    if not in_blender():
        rerun_in_blender(path, script_args())
    shrink = "--shrink" in script_args()
    m = Model(root, pkg)
    got = m.detached(tol_mm / unit)
    count = {n: len(c) for n, c in got.items()}
    new = {n: k for n, k in count.items() if k > detached_known.get(n, 0)}
    fewer = {n: k for n, k in detached_known.items() if count.get(n, 0) < k}
    pieces = sum(len(m.isl[n]) for n in m.names)
    print(f"{len(m.names)} parts, {pieces} closed pieces; "
          f"{sum(count.values())} touch nothing within {tol_mm:g} mm")
    if shrink:
        kept = {n: min(k, count.get(n, 0)) for n, k in detached_known.items()
                if count.get(n, 0)}
        rewrite_block(path, DET_START, DET_END, _fmt_detached(kept))
        print(f"DETACHED shrunk to {len(kept)} parts; nothing was added")
        fewer = {}
        detached_known = kept
    if new:
        print(f"\n{len(new)} parts have more free-floating pieces than allowed:")
        for n, k in sorted(new.items()):
            where = ", ".join(str(tuple(c)) for c in got[n][:3])
            print(f"  {n}: {k} (allowed {detached_known.get(n, 0)})  e.g. {where}")
    if fewer:
        print(f"\n{len(fewer)} parts have fewer than the list allows -- run with "
              f"--shrink:")
        for n, k in sorted(fewer.items()):
            print(f"  {n}: {count.get(n, 0)} (list says {k})")
    ok = not (new or fewer)
    print("\n" + ("PASS  every piece is fastened to something, bar the known "
                  f"list ({sum(detached_known.values())} pieces)"
                  if ok else "FAIL  pieces are floating free"))
    return 0 if ok else 1
