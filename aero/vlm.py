"""Vortex-lattice method.

A classical three-dimensional panel solver for lifting surfaces (Katz &
Plotkin, "Low-Speed Aerodynamics", ch. 12). Each panel carries a horseshoe
vortex: a bound segment along the panel quarter-chord and two trailing legs
running to far downstream. The circulations are found by enforcing flow
tangency at each panel's three-quarter-chord collocation point, then lift comes
from Kutta-Joukowski and induced drag from the downwash the system induces on
itself.

This is a real solve, not a coefficient lookup. What it cannot do is anything
viscous: no boundary layer, no separation, no stall, no profile drag, and no
compressibility. Within those limits it gives genuinely useful answers for lift
slope, spanwise loading, induced drag, and -- the reason it is here -- the
aerodynamic centre, which is what actually decides whether an aircraft is
stable or where a racing car's aerodynamic balance sits.

Ground effect is handled by the method of images: the whole vortex system is
mirrored in the ground plane, which makes the ground a streamline exactly.
"""

import math

import numpy as np

RHO = 1.225
FAR = 1.0e4          # how far downstream the trailing legs run, in metres


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------

class Surface:
    """One lifting surface, discretised into a chordwise x spanwise lattice.

    Corner points are supplied as functions of (spanwise fraction) so that
    taper, sweep, twist, dihedral and ground clearance are all expressible.
    """

    def __init__(self, name, le_root, chord_root, le_tip, chord_tip,
                 n_span=12, n_chord=4, twist_root=0.0, twist_tip=0.0,
                 mirror=True, symmetric_pair=True, planform=None):
        self.name = name
        self.le_root = np.asarray(le_root, dtype=float)
        self.le_tip = np.asarray(le_tip, dtype=float)
        self.chord_root = float(chord_root)
        self.chord_tip = float(chord_tip)
        self.n_span = int(n_span)
        self.n_chord = int(n_chord)
        self.twist_root = math.radians(twist_root)
        self.twist_tip = math.radians(twist_tip)
        self.mirror = mirror
        self.symmetric_pair = symmetric_pair
        # (span fraction, leading-edge x, chord) -- overrides the linear taper
        # so the lattice follows a curved leading edge exactly as the geometry
        # does. Without it the solver describes a different wing.
        self.planform = planform

    def _station(self, f):
        le = self.le_root + (self.le_tip - self.le_root) * f
        c = self.chord_root + (self.chord_tip - self.chord_root) * f
        if self.planform:
            t = self.planform
            if f <= t[0][0]:
                x_le, c = t[0][1], t[0][2]
            elif f >= t[-1][0]:
                x_le, c = t[-1][1], t[-1][2]
            else:
                x_le, c = t[-1][1], t[-1][2]
                for i in range(len(t) - 1):
                    if t[i][0] <= f <= t[i + 1][0]:
                        u = (f - t[i][0]) / (t[i + 1][0] - t[i][0])
                        x_le = t[i][1] + (t[i + 1][1] - t[i][1]) * u
                        c = t[i][2] + (t[i + 1][2] - t[i][2]) * u
                        break
            le = np.array([x_le, le[1], le[2]])
        tw = self.twist_root + (self.twist_tip - self.twist_root) * f
        return le, c, tw

    def _point(self, f, xc):
        """A point at spanwise fraction f and chord fraction xc, with twist
        applied about the quarter-chord."""
        le, c, tw = self._station(f)
        dx = (xc - 0.25) * c
        ct, st = math.cos(tw), math.sin(tw)
        return np.array([le[0] + 0.25 * c + dx * ct,
                         le[1],
                         le[2] - dx * st])

    def panels(self):
        """Yield (A, B, C, D, collocation, normal, area, y_mid) per panel.

        A-B is the leading edge of the panel strip, D-C the trailing edge,
        following the usual VLM convention.
        """
        out = []
        sides = [1.0, -1.0] if (self.mirror and self.symmetric_pair) else [1.0]
        for sgn in sides:
            for i in range(self.n_span):
                f0 = i / self.n_span
                f1 = (i + 1) / self.n_span
                for j in range(self.n_chord):
                    x0 = j / self.n_chord
                    x1 = (j + 1) / self.n_chord
                    p00 = self._point(f0, x0)
                    p10 = self._point(f1, x0)
                    p01 = self._point(f0, x1)
                    p11 = self._point(f1, x1)
                    for p in (p00, p10, p01, p11):
                        p[1] *= sgn
                    if sgn < 0:
                        # Negating y reverses the panel winding. Swapping the
                        # inboard/outboard corners restores it, so the normal
                        # comes out consistently and the bound-segment
                        # direction still agrees with it. Without this the
                        # mirrored half contributes force with the wrong sign
                        # and induced drag comes out negative.
                        p00, p10 = p10, p00
                        p01, p11 = p11, p01
                    # bound vortex at the panel quarter chord
                    a = p00 + 0.25 * (p01 - p00)
                    b = p10 + 0.25 * (p11 - p10)
                    # collocation at three-quarter chord, mid span
                    c_pt = 0.5 * ((p00 + 0.75 * (p01 - p00))
                                  + (p10 + 0.75 * (p11 - p10)))
                    n = np.cross(p11 - p00, p10 - p01)
                    area = 0.5 * np.linalg.norm(n)
                    nn = np.linalg.norm(n)
                    n = n / nn if nn > 1e-12 else np.array([0.0, 0.0, 1.0])
                    if n[2] < 0:      # guard; should not trigger now
                        n = -n
                    out.append(dict(a=a, b=b, col=c_pt, n=n, area=area,
                                    y=0.5 * (a[1] + b[1]),
                                    dy=abs(b[1] - a[1]),
                                    surface=self.name))
        return out


# --------------------------------------------------------------------------
# Biot-Savart
# --------------------------------------------------------------------------

def _seg_velocity(p, a, b, r_core=0.0):
    """Velocity at p induced by a straight vortex segment a->b of unit strength.

    With a Rankine core: inside r_core the filament rotates as a solid body,
    so the induced velocity falls linearly to zero at the centre instead of
    growing without bound. A real vortex has a viscous core; a lattice needs
    one because refining it moves collocation points ever closer to their
    neighbours' filaments.

    The cutoff used to be RELATIVE, `cr2 < core * |r0|^2 * max(r1,r2)^2`, which
    reads as scale-free and is not. A trailing leg is FAR = 1e4 chords long, so
    for those segments both |r0|^2 and max(r1,r2)^2 are about 1e8 and the
    effective cutoff radius came out around a tenth of a chord: every trailing
    vortex passing within 0.1 c of a collocation point was deleted outright.
    Refining the lattice puts more neighbours inside that radius, so the answer
    walked instead of converging -- and the span efficiency this file reports
    for AR 4 and 6, 0.22 and 0.23, is what that looks like when it is written
    down.
    """
    r1 = p - a
    r2 = p - b
    cr = np.cross(r1, r2)
    cr2 = float(np.dot(cr, cr))
    r1n = float(np.linalg.norm(r1))
    r2n = float(np.linalg.norm(r2))
    r0 = b - a
    L2 = float(np.dot(r0, r0))
    if cr2 < 1e-20 or r1n < 1e-12 or r2n < 1e-12 or L2 < 1e-24:
        return np.zeros(3)
    k = (float(np.dot(r0, r1)) / r1n - float(np.dot(r0, r2)) / r2n)
    k /= 4.0 * math.pi * cr2
    if r_core > 0.0:
        d2 = cr2 / L2                      # perpendicular distance, squared
        rc2 = r_core * r_core
        if d2 < rc2:
            k *= d2 / rc2
    return cr * k


# A fraction of the bound segment's own length, which is the local lattice
# scale. Small enough not to move the aerodynamics, big enough to stop a
# collocation point that lands on a trailing filament from returning infinity.
CORE_FRAC = 0.15


def _horseshoe_velocity(p, a, b, wake_dir):
    """Velocity at p from a unit horseshoe: trailing leg in, bound, trailing out.

    The core goes on the trailing legs only. They run to infinity and can pass
    as close as they like to another panel's collocation point, where a 1/r
    singularity is a numerical accident. The bound vortex is the opposite: the
    quarter-chord/three-quarter-chord arrangement puts each collocation point a
    deliberate distance from its own bound vortex, that distance is the
    diagonal of the influence matrix, and coring it at a fraction of the
    panel's SPAN destroys the solve as soon as the chordwise panels are shorter
    than the core -- which is what refinement does.
    """
    a_inf = a + wake_dir * FAR
    b_inf = b + wake_dir * FAR
    rc = CORE_FRAC * float(np.linalg.norm(b - a))
    return (_seg_velocity(p, a_inf, a, rc)
            + _seg_velocity(p, a, b)
            + _seg_velocity(p, b, b_inf, rc))


def _mirror(v):
    m = v.copy()
    m[2] = -m[2]
    return m


# --------------------------------------------------------------------------
# Solver
# --------------------------------------------------------------------------

class Solution:
    def __init__(self, **kw):
        self.__dict__.update(kw)


def trefftz_drag(panels, gamma, v_inf, s_ref, ground=False):
    """Induced drag from the Trefftz plane, in the full crossflow plane.

    Near-field Kutta-Joukowski drag is unreliable on swept wings: the bound
    vortex is no longer normal to the freestream and the velocity it induces
    at its own midpoint contaminates the streamwise force, which showed up
    here as a swept wing producing negative induced drag. The Trefftz plane
    avoids that entirely by working in the far wake, where the flow is
    two-dimensional and only the shed vorticity matters.

    That far wake is two-dimensional in (y, z), not in y alone. An earlier
    version summed every strip onto the y axis and ignored height, which is
    exact for one planar wing and nonsense for anything stacked: a racing
    car's front wing, rear wing and beam wing all shed at the same span
    stations at different heights, and collapsing them added their
    circulations together. It returned CDi above 20.

    Each strip sheds a trailing filament of +G at its inboard edge and -G at
    its outboard edge, at the strip's own height. The drag is the work done by
    the crossflow those filaments induce, taken normal to each wake element.
    """
    strips = {}
    for p, g in zip(panels, gamma):
        # key on the surface as well as the span station: two surfaces at the
        # same y are different sheets and must not be merged
        key = (p.get("surface", ""), round(float(p["a"][1]), 9),
               round(float(p["b"][1]), 9))
        st = strips.setdefault(key, {"G": 0.0,
                                     "yl": float(p["a"][1]),
                                     "yr": float(p["b"][1]),
                                     "zl": 0.0, "zr": 0.0, "n": 0})
        st["G"] += float(g)
        st["zl"] += float(p["a"][2])
        st["zr"] += float(p["b"][2])
        st["n"] += 1
    items = list(strips.values())
    for st in items:
        st["zl"] /= st["n"]
        st["zr"] /= st["n"]
        st["ym"] = 0.5 * (st["yl"] + st["yr"])
        st["zm"] = 0.5 * (st["zl"] + st["zr"])
        dy, dz = st["yr"] - st["yl"], st["zr"] - st["zl"]
        ds = math.hypot(dy, dz)
        st["ds"] = ds
        # unit normal to the wake element, in the crossflow plane
        st["ny"] = -dz / ds if ds > 1e-12 else 0.0
        st["nz"] = dy / ds if ds > 1e-12 else 1.0

    # trailing filaments: +G at the inboard edge, -G at the outboard edge
    fil = []
    for st in items:
        fil.append((st["yl"], st["zl"], st["G"]))
        fil.append((st["yr"], st["zr"], -st["G"]))
        if ground:
            fil.append((st["yl"], -st["zl"], -st["G"]))
            fil.append((st["yr"], -st["zr"], st["G"]))

    d = 0.0
    for st in items:
        vy = vz = 0.0
        for (fy, fz, fg) in fil:
            dy = st["ym"] - fy
            dz = st["zm"] - fz
            r2 = dy * dy + dz * dz
            if r2 < 1e-18:
                continue                     # principal value: skip the self
            k = fg / (2.0 * math.pi * r2)
            vy += -k * dz
            vz += k * dy
        d += 0.5 * RHO * st["G"] * (vy * st["ny"] + vz * st["nz"]) * st["ds"]
    return d / (0.5 * RHO * v_inf * v_inf * s_ref)


def solve(surfaces, alpha_deg, v_inf, s_ref, c_ref, b_ref,
          ground=False, x_ref=0.0):
    """Solve the lattice and return forces and moments.

    `ground` mirrors the entire vortex system in the plane z = 0, which makes
    the ground a streamline and is what produces ground effect.
    """
    panels = []
    for s in surfaces:
        panels.extend(s.panels())
    n = len(panels)

    # Non-dimensionalise on the reference chord before solving. The influence
    # matrix is built from cross products of position differences, so on a
    # 0.15 m model those quantities are ~1e-4 and the conditioning degrades
    # badly -- the same aircraft solved at metre scale and at model scale gave
    # different aerodynamic centres. Working in chords removes the dependence.
    L = c_ref if c_ref > 0 else 1.0
    for p in panels:
        p["a"] = p["a"] / L
        p["b"] = p["b"] / L
        p["col"] = p["col"] / L
        p["y"] = p["y"] / L
        p["dy"] = p["dy"] / L
    s_ref_n = s_ref / (L * L)
    b_ref_n = b_ref / L
    c_ref_n = 1.0
    x_ref_n = x_ref / L

    a = math.radians(alpha_deg)
    v_vec = np.array([math.cos(a), 0.0, math.sin(a)]) * v_inf
    wake = np.array([1.0, 0.0, 0.0])

    A = np.zeros((n, n))
    rhs = np.zeros(n)
    for i, pi in enumerate(panels):
        col, nrm = pi["col"], pi["n"]
        for j, pj in enumerate(panels):
            v = _horseshoe_velocity(col, pj["a"], pj["b"], wake)
            if ground:
                # the image system: mirrored geometry, traversed b->a so the
                # induced normal velocity cancels at the wall
                v = v + _horseshoe_velocity(col, _mirror(pj["b"]),
                                            _mirror(pj["a"]), wake)
            A[i, j] = float(np.dot(v, nrm))
        rhs[i] = -float(np.dot(v_vec, nrm))

    gamma = np.linalg.solve(A, rhs)

    # Kutta-Joukowski on each bound segment, using the local total velocity
    F = np.zeros(3)
    M_y = 0.0
    strips = {}
    for i, pi in enumerate(panels):
        mid = 0.5 * (pi["a"] + pi["b"])
        # Include the panel's own horseshoe. Its bound segment contributes
        # nothing at its own midpoint (the Biot-Savart kernel returns zero for
        # a collinear point), but its two trailing legs induce a large part of
        # the downwash -- and omitting them makes induced drag come out
        # negative, which is how this was caught.
        v_ind = np.zeros(3)
        for j, pj in enumerate(panels):
            v_ind += gamma[j] * _horseshoe_velocity(mid, pj["a"], pj["b"], wake)
            if ground:
                v_ind += gamma[j] * _horseshoe_velocity(
                    mid, _mirror(pj["b"]), _mirror(pj["a"]), wake)
        v_tot = v_vec + v_ind
        dl = pi["b"] - pi["a"]
        df = RHO * gamma[i] * np.cross(v_tot, dl)
        F += df
        M_y += -(mid[0] - x_ref_n) * df[2] + (mid[2]) * df[0]
        key = round(pi["y"], 6)
        strips.setdefault(key, 0.0)
        strips[key] += float(df[2])

    q = 0.5 * RHO * v_inf * v_inf
    lift = F[2] * math.cos(a) - F[0] * math.sin(a)
    cdi = trefftz_drag(panels, gamma, v_inf, s_ref_n, ground)
    drag = cdi * q * s_ref_n

    return Solution(
        panels=n,
        gamma=gamma,
        alpha=alpha_deg,
        CL=lift / (q * s_ref_n),
        CDi=cdi,
        Cm=M_y / (q * s_ref_n * c_ref_n),
        lift_N=lift,
        drag_N=drag,
        strips=dict(sorted(strips.items())),
        s_ref=s_ref_n, c_ref=c_ref_n, b_ref=b_ref_n,
    )


def lift_slope(surfaces, v_inf, s_ref, c_ref, b_ref, ground=False, x_ref=0.0,
               a1=0.0, a2=4.0):
    """dCL/dalpha per radian, from two solves."""
    s1 = solve(surfaces, a1, v_inf, s_ref, c_ref, b_ref, ground, x_ref)
    s2 = solve(surfaces, a2, v_inf, s_ref, c_ref, b_ref, ground, x_ref)
    return (s2.CL - s1.CL) / math.radians(a2 - a1), s1, s2


def neutral_point(surfaces, v_inf, s_ref, c_ref, b_ref, x_le_mac,
                  ground=False, a1=0.0, a2=4.0):
    """Longitudinal neutral point, as a fraction of the mean aerodynamic chord.

    Moments are referenced to the quarter-chord of the mean aerodynamic chord,
    not to the origin. Referencing to the origin makes Cm a small difference
    between large numbers and the answer becomes lattice-dependent noise.
    """
    x_ref = x_le_mac + 0.25 * c_ref
    s1 = solve(surfaces, a1, v_inf, s_ref, c_ref, b_ref, ground, x_ref=x_ref)
    s2 = solve(surfaces, a2, v_inf, s_ref, c_ref, b_ref, ground, x_ref=x_ref)
    dcl = s2.CL - s1.CL
    if abs(dcl) < 1e-9:
        return float("nan"), s1, s2
    dcm_dcl = (s2.Cm - s1.Cm) / dcl
    x_np = x_ref - dcm_dcl * c_ref
    return (x_np - x_le_mac) / c_ref, s1, s2


def efficiency(sol):
    """Span efficiency e, from CDi = CL^2 / (pi AR e)."""
    ar = sol.b_ref ** 2 / sol.s_ref
    if abs(sol.CDi) < 1e-12:
        return float("nan")
    return sol.CL ** 2 / (math.pi * ar * sol.CDi)
