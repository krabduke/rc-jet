"""The aircraft as a closed quadrilateral surface, for the panel solve.

The old panelisation handed the solver a centroid, a normal and an area per
panel and nothing else. That is all a source-panel method needs, and it is
not enough for a real one: a doublet panel's influence is an integral round
its EDGES, the Kutta condition is a statement about the two panels either
side of a trailing EDGE, and the surface velocity is a gradient taken over a
panel's NEIGHBOURS. So this emits corners, structure and trailing edges.

Three things follow from the method rather than from taste:

  * The surface has to be CLOSED. The interior potential is pinned to the
    freestream continued, which is a statement about an inside, and a body
    with a hole in it does not have one. The old panelisation deliberately
    cut the intake mouth open so air could get in; here the mouth is closed
    and the engine's flow is a prescribed normal velocity on those panels
    instead -- which is what an intake actually is, and it also gets the
    exhaust right, which a hole never could.

  * Components may not INTERSECT. Two closed bodies passing through each
    other have panels buried inside one another, and a buried panel is an
    interior surface the solve will happily satisfy conditions on. So the
    wing, tail and fins start just outboard of the fuselage skin and are
    capped there. This is the ordinary "exposed wing and body" panelling and
    its cost is stated: the part of the wing inside the fuselage carries no
    panels, so its lift arrives as interference on the body rather than as
    wing area. See tools/check_panel_geom.py, which measures it.

  * Panels of different components may not come much closer than their own
    size, or the influence matrix is near-singular exactly where the two
    surfaces face each other -- the same failure the source solve had at a
    wingtip. The root gap is a panel width for that reason, not for looks.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, "plane"))

import spec                                    # noqa: E402
from parts import fuselage as fus              # noqa: E402

MM = 0.001


def _normal(q):
    d1 = [q[2][k] - q[0][k] for k in range(3)]
    d2 = [q[3][k] - q[1][k] for k in range(3)]
    n = [d1[1] * d2[2] - d1[2] * d2[1],
         d1[2] * d2[0] - d1[0] * d2[2],
         d1[0] * d2[1] - d1[1] * d2[0]]
    m = math.sqrt(sum(v * v for v in n))
    return [v / m for v in n] if m > 1e-12 else [0.0, 0.0, 0.0]


def orient(grid, ref):
    """Wind every quad so its normal points away from `ref(quad)`.

    Deriving the winding instead of reasoning about it. Mirroring a surface
    across the centreline turns it inside out, capping one end wants the
    opposite order from capping the other, and a vertical surface swaps two
    axes -- three separate chances to hand the solver a body whose normals
    point into it, which it will solve just as happily with the boundary
    condition inverted. An aerofoil section is convex and a swept one is
    star-shaped about its own centroid, so "away from the middle" is a
    complete answer for a lifting surface; the fuselage uses its own axis at
    each station.
    """
    out = []
    for row in grid:
        r = []
        for q in row:
            c = [sum(p[k] for p in q) / 4.0 for k in range(3)]
            o = ref(q)
            n = _normal(q)
            d = sum(n[k] * (c[k] - o[k]) for k in range(3))
            r.append(list(reversed(q)) if d < 0 else q)
        out.append(r)
    return out


def solid_angle(p, quads):
    """Signed solid angle a set of quads subtends at p. 4.pi inside a closed
    surface of outward normals, nothing outside."""
    tot = 0.0
    for q in quads:
        for (a, b, c) in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
            ax, ay, az = a[0]-p[0], a[1]-p[1], a[2]-p[2]
            bx, by, bz = b[0]-p[0], b[1]-p[1], b[2]-p[2]
            cx, cy, cz = c[0]-p[0], c[1]-p[1], c[2]-p[2]
            la = math.sqrt(ax*ax + ay*ay + az*az)
            lb = math.sqrt(bx*bx + by*by + bz*bz)
            lc = math.sqrt(cx*cx + cy*cy + cz*cz)
            if la < 1e-12 or lb < 1e-12 or lc < 1e-12:
                continue
            num = (ax*(by*cz - bz*cy) + ay*(bz*cx - bx*cz) + az*(bx*cy - by*cx))
            den = (la*lb*lc + (ax*bx + ay*by + az*bz)*lc
                   + (ax*cx + ay*cy + az*cz)*lb + (bx*cx + by*cy + bz*cz)*la)
            tot += 2.0 * math.atan2(num, den)
    return tot


class Geom:
    """Quads, who each one's neighbours are, and where the trailing edges are.

    Neighbours are carried explicitly rather than as structured (i, j) blocks
    because components get TRIMMED where they cross, and a trimmed block is
    not a block any more. The surface velocity is a gradient over a panel's
    neighbours, so it is the neighbour list that matters, not the grid it
    happened to be generated on.
    """

    def __init__(self):
        self.quads = []          # list of 4 (x, y, z) in mm
        self.nb = []             # 4 neighbour indices per panel, -1 for none
        self.te = []
        self.inflow = []
        self.controls = []       # {id, panels, hinge axis}
        self.parts = []          # (name, first, count)
        self.body = []           # which closed component each panel belongs to

    def patch(self, name, grid, ni, nj, wrap_i=False, wrap_j=False,
              body=None):
        start = len(self.quads)

        def at(i, j):
            if wrap_i:
                i %= ni
            if wrap_j:
                j %= nj
            if i < 0 or i >= ni or j < 0 or j >= nj:
                return -1
            return start + i * nj + j

        for i in range(ni):
            for j in range(nj):
                self.quads.append(grid[i][j])
                self.nb.append([at(i-1, j), at(i+1, j), at(i, j-1), at(i, j+1)])
                self.body.append(body if body is not None else name)
        self.parts.append((name, start, ni * nj))
        return start

    def prune(self, drop):
        """Remove the panels in `drop` and renumber everything that points at
        one. Neighbour links to a removed panel become -1, which is what an
        edge of a surface is."""
        keep = [i for i in range(len(self.quads)) if i not in drop]
        remap = {old: new for new, old in enumerate(keep)}
        r = lambda k: remap.get(k, -1)
        self.quads = [self.quads[i] for i in keep]
        self.nb = [[r(k) for k in self.nb[i]] for i in keep]
        self.body = [self.body[i] for i in keep]
        self.te = [[r(a), r(b)] for (a, b) in self.te
                   if a in remap and b in remap]
        self.inflow = [[r(i), v] for (i, v) in self.inflow if i in remap]
        for c in self.controls:
            keep = [k for k, i in enumerate(c["panels"]) if i in remap]
            c["axes"] = [c["axes"][k] for k in keep]
            c["panels"] = [remap[c["panels"][k]] for k in keep]
        self.controls = [c for c in self.controls if c["panels"]]
        parts = []
        for (name, start, count) in self.parts:
            kept = [remap[i] for i in range(start, start + count) if i in remap]
            if kept:
                parts.append((name, min(kept), len(kept)))
        self.parts = parts

    def link(self, a, b):
        """Make two panels neighbours.

        The structured blocks only know their own grid, so a tip cap and the
        surface it closes are strangers: the cap's panels had two neighbours
        each, both along the cap, and the surface gradient fitted over them
        has no information across the cap at all. Those were the panels
        reading Cp -16 on a wingtip.
        """
        for (x, y) in ((a, b), (b, a)):
            if y in self.nb[x]:
                continue
            for k in range(4):
                if self.nb[x][k] < 0:
                    self.nb[x][k] = y
                    break

    def component(self, name):
        return [self.quads[i] for i in range(len(self.quads))
                if self.body[i] == name]

    def emit(self):
        # to the micron, which is a hundredth of a panel on a 440 mm model
        # and a third of the manifest's size
        flat = []
        for q in self.quads:
            for p in q:
                flat.extend([round(p[0] * MM, 6), round(p[1] * MM, 6),
                             round(p[2] * MM, 6)])
        return {"n": len(self.quads), "quads": flat, "nb": self.nb,
                "te": self.te, "inflow": self.inflow,
                "controls": self.controls,
                "parts": [{"name": n, "start": s, "count": c}
                          for (n, s, c) in self.parts]}


# --------------------------------------------------------------------- body

def fuselage(g, n_x=20, n_theta=16):
    """The body: a lofted superellipse tube with both ends capped.

    Closed, including across the intake mouth and the nozzle. Those panels are
    recorded in `inflow` so the solve can push air through them instead of
    leaving a hole in the boundary condition.
    """
    x0, x1 = spec.FUSELAGE[0][0], spec.FUSELAGE[-1][0]
    xs = [x0 + (x1 - x0) * 0.5 * (1 - math.cos(math.pi * i / (n_x - 1)))
          for i in range(n_x)]
    rings = [fus.section_ring(x, segments=n_theta) for x in xs]

    grid = []
    for i in range(n_x - 1):
        row = []
        for j in range(n_theta):
            k = (j + 1) % n_theta
            # wound so the normal points out of the body
            row.append([rings[i][j], rings[i][k], rings[i + 1][k], rings[i + 1][j]])
        grid.append(row)
    g.patch("fuselage", grid, n_x - 1, n_theta, wrap_j=True, body="fuselage")

    # end caps, as triangles written as quads with a repeated corner
    for (ring, x, sgn) in ((rings[0], xs[0], -1), (rings[-1], xs[-1], 1)):
        w, h, zc, nn = fus.station_at(x)
        apex = (x, 0.0, zc)
        row = []
        for j in range(n_theta):
            k = (j + 1) % n_theta
            q = ([ring[j], ring[k], apex, apex] if sgn > 0
                 else [ring[k], ring[j], apex, apex])
            row.append(q)
        g.patch("nose_cap" if sgn < 0 else "tail_cap", [row], 1, n_theta,
                wrap_j=True, body="fuselage")


# ---------------------------------------------------------- lifting surfaces

def _naca(xc, tc, camber):
    """Half-thickness and camber line of a 4-digit section, closed at the
    trailing edge (the -0.1036 coefficient rather than -0.1015)."""
    t = 5 * tc * (0.2969 * math.sqrt(max(xc, 0.0)) - 0.1260 * xc
                  - 0.3516 * xc * xc + 0.2843 * xc ** 3 - 0.1036 * xc ** 4)
    if camber <= 0:
        return t, 0.0
    m, p = camber, 0.4
    yc = (m / (p * p) * (2 * p * xc - xc * xc) if xc < p
          else m / ((1 - p) ** 2) * ((1 - 2 * p) + 2 * p * xc - xc * xc))
    return t, yc


def _section(m, tc, camber):
    """Points round a section from the trailing edge, over the top, round the
    nose and back along the bottom. 2m-1 points, first and last coincident."""
    out = []
    for k in range(m - 1, -1, -1):
        xc = 0.5 * (1 - math.cos(math.pi * k / (m - 1)))
        t, yc = _naca(xc, tc, camber)
        out.append((xc, yc + t))
    for k in range(1, m):
        xc = 0.5 * (1 - math.cos(math.pi * k / (m - 1)))
        t, yc = _naca(xc, tc, camber)
        out.append((xc, yc - t))
    return out


def surface(g, name, stations, m=14, vertical=False, cap_root=True):
    """A closed lifting surface from a list of stations.

    Each station is (x_le, span, offset, chord, t/c, twist_deg), where `span`
    runs out along the surface -- outboard for a wing, upward for a fin -- and
    `offset` is the lateral position of its plane. The section wraps in i and
    the span runs in j; the two panels either side of the trailing edge go
    into the Kutta list. Both ends are capped, so the surface is closed --
    including the root, which is why it has to stop short of the body.

    Winding is derived rather than passed in -- see `orient`.
    """
    ns = len(stations) - 1
    sec = [_section(m, s[4], 0.0) for s in stations]
    npan = len(sec[0]) - 1

    def P(k, s):
        x_le, y, z, chord, tc, tw = stations[s]
        xc, zc = sec[s][k]
        a = math.radians(tw)
        # twist about the quarter chord
        dx, dz = (xc - 0.25) * chord, zc * chord
        rx = dx * math.cos(a) + dz * math.sin(a)
        rz = -dx * math.sin(a) + dz * math.cos(a)
        X = x_le + 0.25 * chord + rx
        if vertical:
            return (X, z + rz, y)
        return (X, y, z + rz)

    grid = []
    for i in range(npan):
        row = []
        for j in range(ns):
            grid[i:i] = []
            row.append([P(i, j), P(i, j + 1), P(i + 1, j + 1), P(i + 1, j)])
        grid.append(row)
    # the section's own mid-chord line at each span station, which every quad
    # on that station faces away from
    def mid(q):
        sp = sum(p[1] if not vertical else p[2] for p in q) / 4.0
        x = sum(p[0] for p in q) / 4.0
        j = 0
        for k in range(len(stations)):
            if abs(stations[k][1] - sp) < abs(stations[j][1] - sp):
                j = k
        x_le, span, off, chord, tc, tw = stations[j]
        cx = x_le + 0.5 * chord
        return ((cx, span, off) if not vertical else (cx, off, span))
    grid = orient(grid, mid)
    # The section is NOT wrapped in i.
    #
    # i runs from the trailing edge over the top, round the nose and back
    # along the bottom to the trailing edge again, so wrapping it joins the
    # two panels either side of the trailing edge -- across which the
    # potential is DISCONTINUOUS by the circulation, which is the whole point
    # of the Kutta condition. The surface velocity is a gradient of that
    # potential, and taking it across the jump put Cp at -222,000 on the
    # wing's trailing edge and -184 per radian on the lift slope.
    start = g.patch(name, grid, npan, ns, wrap_i=False, body=name)
    for j in range(ns):
        # i = 0 is the first panel on the upper surface, i = npan-1 the last
        # on the lower, and they meet at the trailing edge
        g.te.append([start + j, start + (npan - 1) * ns + j])

    # tip and root caps: pair each upper point with the lower point at the
    # same chordwise station
    for (s, flip, capname) in ((0, True, "root"), (ns, False, "tip")):
        if s == 0 and not cap_root:
            continue
        # The cap is coarser than the section it closes.
        #
        # Zipping every section point to its opposite number makes the first
        # few cap panels slivers: the section's points are cosine-bunched
        # towards the trailing edge and the thickness there is nearly zero,
        # so a cap panel spanning two adjacent points is a few microns of
        # nothing. Its neighbours' potentials differ by most of the
        # circulation, and the gradient fitted over them reported Cp -2,500
        # on the wingtip. Eight panels of real size, spanning several section
        # points each, is a cap.
        n_cap = min(8, m - 1)
        cut = [round(k * (m - 1) / n_cap) for k in range(n_cap + 1)]
        row = []
        for k in range(n_cap):
            a_, b_ = cut[k], cut[k + 1]
            row.append([P(a_, s), P(b_, s), P(npan - b_, s), P(npan - a_, s)])
        # a cap faces out along the span, so its reference is one station in
        step = 1 if s == 0 else -1
        x_le, span, off, chord, tc, tw = stations[s + step]
        inner = ((x_le + 0.5 * chord, span, off) if not vertical
                 else (x_le + 0.5 * chord, off, span))
        cap = g.patch(f"{name}_{capname}", orient([row], lambda q: inner),
                      1, len(row), body=name)
        # stitch the cap to the section it closes: cap panel k spans section
        # points k..k+1 on one side and npan-1-k..npan-k on the other
        # ...but not the one at the trailing edge.
        #
        # Cap panel 0 spans from the last upper panel to the last lower one,
        # which are the two sides of the Kutta jump. Joined to both, the
        # gradient stencil fits a plane through a step in the potential again
        # and reports Cp = -19,000,000. Every cap panel forward of it pairs
        # points that differ smoothly round the section, which is what a
        # gradient in the cap's own plane means.
        j = 0 if s == 0 else ns - 1
        for k in range(1, len(row)):
            g.link(cap + k, start + cut[k] * ns + j)
            g.link(cap + k, start + (npan - 1 - cut[k]) * ns + j)


def _fuse_half_width(x, z):
    """Half-width of the body at station x and height z, from the same
    superellipse the fuselage is lofted from."""
    w, h, zc, n = fus.station_at(x)
    t = abs((z - zc) / h)
    if t >= 1.0:
        return 0.0
    return w * (1.0 - t ** n) ** (1.0 / n)


def _wing_stations(ns):
    """Spanwise stations off the planform table, tip to tip.

    Tip to tip, not root to tip, and the two halves are one surface with no
    cap between them. A wing capped at the fuselage side is a wing with two
    tips: its bound circulation has to fall to zero at the root and it sheds a
    root vortex there, which a real wing-body does not have because the
    circulation carries through the body. Capped at the side of this fuselage
    the lift slope came out at 1.07 per radian against the 1.78 an aspect
    ratio 1.81 delta should have.

    The price is that the inner wing passes through the fuselage, so those
    panels are buried inside another closed body. That is the ordinary
    wing-body compromise and the alternative -- trimming both surfaces along
    their intersection curve -- is a different program.
    """
    W, PL = spec.WING, spec.WING_PLANFORM
    out = []
    for i in range(ns + 1):
        f = -1.0 + 2.0 * (i / ns)
        # planform table is (fraction, x_le, chord)
        af = abs(f)
        for k in range(len(PL) - 1):
            if PL[k][0] <= af <= PL[k + 1][0]:
                a, b = PL[k], PL[k + 1]
                t = (af - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
                x_le = a[1] + (b[1] - a[1]) * t
                chord = a[2] + (b[2] - a[2]) * t
                break
        else:
            x_le, chord = PL[-1][1], PL[-1][2]
        tc = W["thickness"] + (W["thickness_tip"] - W["thickness"]) * af
        tw = W["incidence"] - W["washout"] * af
        z = W["z_root"] + math.tan(math.radians(W["dihedral"])) * af * W["semi_span"]
        out.append((x_le, f * W["semi_span"], z, chord, tc, tw))
    return out


def _panel_stations(S, ns, span_key, sweep_key, dihedral_deg,
                    twist, z_key="z_root"):
    """Tip to tip, for the same reason the wing is."""
    out = []
    span = S[span_key]
    for i in range(ns + 1):
        f = -1.0 + 2.0 * (i / ns)
        y = f * span
        af = abs(f)
        x_le = S["x_root_le"] + abs(y) * math.tan(math.radians(S[sweep_key]))
        chord = S["root_chord"] + (S["tip_chord"] - S["root_chord"]) * af
        tc = S["thickness"] + (S["thickness_tip"] - S["thickness"]) * af
        z = S[z_key] + abs(y) * math.tan(math.radians(dihedral_deg))
        out.append((x_le, y, z, chord, tc, twist))
    return out


def _mirror(stations):
    return [(x, -y, z, c, t, w) for (x, y, z, c, t, w) in stations]


def _centroid(q):
    return [sum(p[k] for p in q) / 4.0 for k in range(3)]


def _size(q):
    d1 = [q[2][k] - q[0][k] for k in range(3)]
    d2 = [q[3][k] - q[1][k] for k in range(3)]
    n = [d1[1]*d2[2] - d1[2]*d2[1], d1[2]*d2[0] - d1[0]*d2[2],
         d1[0]*d2[1] - d1[1]*d2[0]]
    return math.sqrt(0.5 * math.sqrt(sum(v*v for v in n)))


def trim(g, clear=0.35):
    """Cut every component back where another one passes through it.

    Two closed bodies that intersect have panels buried inside one another,
    and a buried panel is an interior surface the solve will satisfy a
    boundary condition on as happily as a real one. Two bodies that merely
    GRAZE are worse: panels of different components facing each other across
    a fraction of their own size make the influence matrix near-singular
    exactly there, which on this aeroplane put the minimum Cp at -158 and the
    lift slope at three times what it should be.

    So the buried panels go, and so does anything left within `clear` of its
    own width of the other surface. What is left is the union of the
    components with a seam about a panel wide around each junction.

    `clear` is 0.35, and it matters more than it looks. At 0.75 the seam came
    out six per cent of the wetted area: the surface no longer closed -- a
    point inside the fuselage saw it subtend 2.4 pi rather than 4 -- and the
    interior condition, which is a statement about an inside, had a much
    poorer one to work with. Lift at 8 degrees was a quarter lower than it
    should be. At 0.35 the sum of the areas' normals is 0.001 of the wetted
    area and the solid angle is 4.1 pi, which is a closed body; the panels
    that nearly touch are further apart than the 0.12 widths that made the
    matrix near-singular, and the amount of surface reading an impossible
    pressure went DOWN.

    The seam is not a defect to apologise for. A wing capped at the side of
    the fuselage is a wing with two tips: its circulation has to fall to zero
    at the root and it sheds a root vortex the real aeroplane does not have,
    which cost 40 % of the lift slope. Left open at the seam, the doublet
    distribution carries straight on into the body, which is what the
    circulation actually does.
    """
    names = []
    for b in g.body:
        if b not in names:
            names.append(b)
    quads = {n: [] for n in names}
    for i, q in enumerate(g.quads):
        quads[g.body[i]].append(q)

    drop = set()
    for i, q in enumerate(g.quads):
        c = _centroid(q)
        sz = _size(q)
        mine = g.body[i]
        for n in names:
            if n == mine:
                continue
            if solid_angle(c, quads[n]) > 2.0 * math.pi:
                drop.add(i)
                break
            # not inside, but too close to be panelled against
            near = min(math.dist(c, _centroid(o)) for o in quads[n])
            if near < clear * sz:
                drop.add(i)
                break
    g.prune(drop)
    return len(drop)


def control(g, cid, comp, stations, hinge_frac, span_lo, span_hi,
            vertical=False, axis=None):
    """Tag the panels a control surface owns, and the line it turns about.

    A deflected control is NOT re-panelled. The influence matrix depends only
    on the geometry, so moving a flap would mean rebuilding and re-factoring
    it -- half a second, on every drag of a slider. Instead the panel keeps
    its place and its NORMAL turns, which changes the boundary condition by
    exactly what the deflection changes it by to first order. It is the same
    transpiration treatment a vortex lattice uses for a flap, and it is why
    the matrix can be built once.
    """
    def hinge_at(sp):
        """The hinge point at span position sp, and the direction the hinge
        line runs there."""
        k = min(range(len(stations)), key=lambda i: abs(stations[i][1] - sp))
        k2 = k + 1 if k + 1 < len(stations) else k - 1
        pts = []
        for i in (k, k2):
            x_le, span, off, chord, tc, tw = stations[i]
            x = x_le + hinge_frac * chord
            pts.append((x, off, span) if vertical else (x, span, off))
        d = [pts[1][j] - pts[0][j] for j in range(3)]
        m = math.sqrt(sum(v * v for v in d)) or 1.0
        return pts[0], [v / m for v in d]

    span_max = max(abs(s[1]) for s in stations) or 1.0
    idx, axes = [], []
    for i in range(len(g.quads)):
        if g.body[i] != comp:
            continue
        c = _centroid(g.quads[i])
        sp = c[2] if vertical else c[1]
        st = min(stations, key=lambda s: abs(s[1] - sp))
        x_le, span, off, chord, tc, tw = st
        if chord <= 0:
            continue
        f = (c[0] - x_le) / chord
        if f >= hinge_frac and span_lo <= abs(sp) / span_max <= span_hi:
            idx.append(i)
            # per panel, because a hinge on a swept surface is not one line:
            # taken as the root-to-tip direction it came out straight down the
            # span and the flaperon would have turned about the wrong axis
            axes.append(list(axis) if axis else hinge_at(sp)[1])
    if not idx:
        return
    g.controls.append({"id": cid, "panels": idx, "axes": axes})


def build(n_x=20, n_theta=16, gap=None):
    """The whole aeroplane, trimmed into one surface."""
    g = Geom()
    fuselage(g, n_x, n_theta)

    # ---- wing, tip to tip through the body
    # 28 points a side round the section, which is more than it looks.
    #
    # The neutral point is the least forgiving thing this solve produces: it
    # is a ratio of two integrals of surface pressure, and at 12 chordwise
    # points it read 45 % of the mean chord measured between 0 and 4 degrees
    # and 40 % measured between 6 and 12 -- a number that depends on where you
    # measure it is not a number. At 28 it is 39.6 % at every pair, which is
    # what a converged linear solution looks like. Spanwise resolution moves
    # it far less; this is a chordwise pressure distribution being resolved.
    surface(g, "wing", _wing_stations(10), m=28)

    # ---- stabilators, all-moving, at their deflection
    T = spec.HTAIL
    # built UNDEFLECTED: the deflection is a boundary condition, not geometry
    stab_st = _panel_stations(T, 8, "semi_span", "sweep_le", T["anhedral"], 0.0)
    surface(g, "stab", stab_st, m=12)

    # ---- fin, rooted below the spine so it trims into the body
    V = spec.VTAIL
    ns = 5
    st = []
    for i in range(ns + 1):
        f = -0.12 + 1.12 * (i / ns)
        z = V["z_root"] + f * V["height"]
        af = max(f, 0.0)
        st.append((V["x_root_le"] + af * V["height"] * math.tan(math.radians(V["sweep_le"])),
                   z, 0.0,
                   V["root_chord"] + (V["tip_chord"] - V["root_chord"]) * af,
                   V["thickness"] + (V["thickness_tip"] - V["thickness"]) * af,
                   0.0))
    surface(g, "fin", st, m=10, vertical=True)

    wing_st = _wing_stations(10)
    control(g, "flaperon", "wing", wing_st, 0.74, 0.30, 0.92)
    # all-moving, and its pivot is across the aircraft rather than along the
    # swept leading edge
    control(g, "stabilator", "stab", stab_st, 0.0, 0.0, 1.0, axis=(0, 1, 0))
    control(g, "rudder", "fin", st, 1.0 - V["rudder_chord"], 0.0,
            V["rudder_span"], vertical=True)

    g.trimmed = trim(g)
    return g


def emit(**kw):
    return build(**kw).emit()


if __name__ == "__main__":
    import json
    d = emit()
    print(f"{d['n']} panels, {len(d['te'])} trailing edges, "
          f"{sum(1 for nb in d['nb'] for k in nb if k < 0)} open edges")
    for p in d["parts"]:
        print(f"   {p['name']:16s} {p['count']:4d} panels")
