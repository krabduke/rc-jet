/* A low-order panel method: the aircraft as one velocity potential.
 *
 * WHAT THIS REPLACES, AND WHY
 *
 * The wind tunnel used to be two solvers stuck together. A vortex lattice on
 * the lifting surfaces made the lift, and a separate source-panel solve made
 * the body's thickness. The wing was in BOTH -- as a zero-thickness sheet
 * carrying circulation and as a closed surface carrying sources -- which is
 * not a model of anything. Sources cannot make lift; a lattice has no
 * thickness for air to flow around; and the streamline you drew was the sum
 * of two fields that did not satisfy each other's boundary conditions. Lines
 * ran into the wing and stopped: 103 of 352 died on a surface they should
 * have gone around, and NONE of them died at a stagnation point, which is the
 * only place a streamline is allowed to end.
 *
 * This is the standard answer to that and it is what VSAERO, PANAIR and PMARC
 * are: ONE surface, panelled once, carrying a constant-strength source sheet
 * and a constant-strength normal-doublet sheet, with a wake shed from the
 * trailing edges and a Kutta condition that sets its strength. Lift comes out
 * of the doublet jump at the trailing edge rather than being computed by a
 * different program. The field you trace a streamline through is the gradient
 * of the same potential the boundary condition was applied to.
 *
 * THE FORMULATION (Morino)
 *
 * Write the total potential as the freestream plus what the sheets induce:
 *
 *     Phi(p) = Vinf . p + SUM_j [ sigma_j B_j(p) + mu_j C_j(p) ]
 *
 * and require that inside the body the total potential is exactly the
 * freestream continued -- the interior is "filled with undisturbed air",
 * which is a fiction with no physical content but pins the interior solution
 * so the exterior one is unique. That makes the induced part vanish at every
 * interior collocation point:
 *
 *     SUM_j [ sigma_j B_ij + mu_j C_ij ]  =  0
 *
 * The source strengths are then KNOWN, not solved for. The source sheet is
 * the only thing that jumps the normal velocity, by exactly sigma, and the
 * jump has to take the interior's n.Vinf to the exterior's zero:
 *
 *     sigma_j = -n_j . Vinf
 *
 * which leaves one unknown per panel -- mu -- and one equation per panel.
 *
 * THE KUTTA CONDITION
 *
 * Without one, every lifting surface comes out with zero circulation and the
 * aeroplane makes no lift. A wake doublet sheet leaves each trailing edge
 * carrying the jump between the panels either side of it:
 *
 *     mu_wake = mu_upper - mu_lower
 *
 * which is the statement that the potential is continuous round the trailing
 * edge -- so the flow leaves it smoothly instead of turning the corner at
 * infinite speed. The wake's influence folds into those two columns of the
 * matrix, so it costs no extra unknowns.
 *
 * WHAT IT STILL IS NOT
 *
 * Inviscid and incompressible. No boundary layer, so no skin friction, no
 * separation and no stall -- past the stall angle it will keep making lift
 * that the aeroplane would not. The wake is rigid rather than rolled up. The
 * panels are flat. Those are the assumptions of the method, they are the same
 * ones every code named above makes, and they are stated rather than hidden.
 */

import { sourcePotential, sourceVelocity, doubletPotential, doubletVelocity,
         panelFrame, solidAngle } from './panelkernel.js';

function luFactor(A, n){
  const piv = new Int32Array(n);
  for(let i = 0; i < n; i++) piv[i] = i;
  for(let k = 0; k < n; k++){
    let p = k, best = Math.abs(A[k*n+k]);
    for(let i = k+1; i < n; i++){
      const v = Math.abs(A[i*n+k]);
      if(v > best){ best = v; p = i; }
    }
    if(p !== k){
      for(let j = 0; j < n; j++){
        const t = A[k*n+j]; A[k*n+j] = A[p*n+j]; A[p*n+j] = t;
      }
      const t = piv[k]; piv[k] = piv[p]; piv[p] = t;
    }
    const d = A[k*n+k];
    if(Math.abs(d) < 1e-14) continue;
    for(let i = k+1; i < n; i++){
      const m = A[i*n+k]/d;
      A[i*n+k] = m;
      if(m === 0) continue;
      for(let j = k+1; j < n; j++) A[i*n+j] -= m*A[k*n+j];
    }
  }
  return piv;
}

function luSolve(LU, piv, b, n, out){
  const x = out || new Float64Array(n);
  for(let i = 0; i < n; i++) x[i] = b[piv[i]];
  for(let i = 1; i < n; i++){
    let s = x[i];
    for(let j = 0; j < i; j++) s -= LU[i*n+j]*x[j];
    x[i] = s;
  }
  for(let i = n-1; i >= 0; i--){
    let s = x[i];
    for(let j = i+1; j < n; j++) s -= LU[i*n+j]*x[j];
    const d = LU[i*n+i];
    x[i] = Math.abs(d) > 1e-14 ? s/d : 0;
  }
  return x;
}

/* How many of its own widths from an element the exact panel influence is
 * used rather than its point form.
 *
 * Swept against summing every element exactly, on an 800-panel wing, over
 * points in the field and over points in the band 0.6 to 3 panel widths off
 * the skin where the picture lives:
 *
 *   widths  near cells    field mean / worst     skin mean / worst    us
 *     1.6       1            0.112 / 10.20        1.330 / 13.15       15.7
 *     2.2       1            0.056 /  2.24        0.929 / 16.86       18.3
 *     2.2       2            0.048 /  3.85        0.774 /  8.13       19.7   <--
 *     3.0       2            0.036 /  0.63        0.424 /  5.12       26.6
 *
 * 3.0 is better and costs a third more; the streamlines do not show the
 * difference between a half and a third of a per cent, and they do show the
 * time. */
const EXACT_WIDTHS = 2.2;

/* And the same crossover for the influence MATRIX, where it is a potential
 * rather than a velocity. Checked against the exact build in
 * check_panelflow.mjs. */
const FAR_WIDTHS = 4.0;

const corners = (quads, j) => {
  const b = j*12;
  return [[quads[b],   quads[b+1],  quads[b+2]],
          [quads[b+3], quads[b+4],  quads[b+5]],
          [quads[b+6], quads[b+7],  quads[b+8]],
          [quads[b+9], quads[b+10], quads[b+11]]];
};

export class PanelFlow {
  /* `geom` is
   *   quads   Float64Array(n*12), four corners per panel, wound so the normal
   *           points OUT of the body
   *   te      [[upper, lower], ...] panel pairs meeting at a trailing edge
   *   patches [{start, ni, nj, wrapI, wrapJ}] structured blocks, used to find
   *           each panel's neighbours for the surface gradient
   *   ground  mirror everything in z = 0
   */
  constructor(geom, opts = {}){
    this.N = geom.quads.length / 12;
    this.quads = geom.quads;
    this.te = geom.te || [];
    this.patches = geom.patches || [];
    this.geomNb = geom.nb || null;
    this.ground = !!geom.ground;
    this.wakeLength = opts.wakeLength || 0;
    this.wakeSteps = opts.wakeSteps || 1;
    this.wakeDir = opts.wakeDir || [1, 0, 0];

    const N = this.N;
    this.c = new Float64Array(N*3);
    this.nrm = new Float64Array(N*3);
    this.area = new Float64Array(N);
    this.size = new Float64Array(N);
    this.frames = new Array(N);
    for(let j = 0; j < N; j++){
      const q = corners(this.quads, j);
      const F = panelFrame(q);
      this.frames[j] = F;
      this.c[j*3] = F.o[0]; this.c[j*3+1] = F.o[1]; this.c[j*3+2] = F.o[2];
      this.nrm[j*3] = F.n[0]; this.nrm[j*3+1] = F.n[1]; this.nrm[j*3+2] = F.n[2];
      this.area[j] = F.area;
      this.size[j] = Math.sqrt(F.area);
    }
    // how close a streamline may come to the skin before it is stopped; the
    // field's owner sets it from the model's own size
    this.nearStop = 0;
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity,
        z0 = Infinity, z1 = -Infinity;
    for(let j = 0; j < N; j++){
      const X = this.c[j*3], Y = this.c[j*3+1], Z = this.c[j*3+2];
      if(X < x0) x0 = X; if(X > x1) x1 = X;
      if(Y < y0) y0 = Y; if(Y > y1) y1 = Y;
      if(Z < z0) z0 = Z; if(Z > z1) z1 = Z;
    }
    this.box = [x0, x1, y0, y1, z0, z1];
    /* Control surfaces deflect by turning their panels' NORMALS, not by
     * moving them.
     *
     * The influence matrix depends only on the geometry, so a moved flap
     * means rebuilding and re-factoring it -- half a second, on every drag of
     * a slider. A turned normal changes the boundary condition by exactly
     * what the deflection changes it by to first order, which is the same
     * transpiration treatment a vortex lattice uses, and it is why the matrix
     * can be built once and every new attitude is a back-substitution. */
    this.controls = geom.controls || [];
    this.deflect = {};
    this.nEff = new Float64Array(N*3);
    this.mu = new Float64Array(N);
    this.sigma = new Float64Array(N);
    this.cp = new Float64Array(N);
    this.vs = new Float64Array(N*3);
    this._neighbours();
    this._wake();
    this._assemble();
    this._elements();
  }

  /* Each panel's four neighbours in the structured grid it came from.
   *
   * The surface velocity is the surface gradient of the potential, and a
   * gradient needs neighbours. Taking them from the structured block the
   * panel was generated in is exact and free; recovering them by searching
   * for shared edges is neither. -1 means the block ends there. */
  _neighbours(){
    /* Given directly when the geometry has been trimmed.
     *
     * A structured (i, j) block stops being one the moment panels are cut out
     * of it where another component passes through, and the gradient stencil
     * only ever wanted the neighbours, not the grid they came from. -1 is an
     * edge of the surface, which after trimming is a real thing a panel can
     * have. */
    if(this.geomNb){
      const nb = new Int32Array(this.N*4);
      for(let i = 0; i < this.N; i++)
        for(let k = 0; k < 4; k++) nb[i*4+k] = this.geomNb[i][k];
      this.nb = nb;
      this._stencils();
      return;
    }
    const nb = new Int32Array(this.N*4).fill(-1);
    for(const P of this.patches){
      const at = (i, j) => {
        if(P.wrapI) i = (i + P.ni) % P.ni;
        if(P.wrapJ) j = (j + P.nj) % P.nj;
        if(i < 0 || i >= P.ni || j < 0 || j >= P.nj) return -1;
        return P.start + i*P.nj + j;
      };
      for(let i = 0; i < P.ni; i++) for(let j = 0; j < P.nj; j++){
        const k = P.start + i*P.nj + j;
        nb[k*4]   = at(i-1, j); nb[k*4+1] = at(i+1, j);
        nb[k*4+2] = at(i, j-1); nb[k*4+3] = at(i, j+1);
      }
    }
    this.nb = nb;
    this._stencils();
  }

  /* The stencil each panel's surface gradient is fitted over.
   *
   * The four neighbours, and where there are not four of them -- which after
   * trimming is a third of the surface -- their neighbours as well, so a
   * panel on a seam still has points on both sides of it to fit a plane
   * through. */
  _stencils(){
    const nb = this.nb;
    this.stencil = new Array(this.N);
    for(let i = 0; i < this.N; i++){
      const set = new Set();
      for(let k = 0; k < 4; k++){ const j = nb[i*4+k]; if(j >= 0) set.add(j); }
      if(set.size < 4){
        for(const j of Array.from(set))
          for(let k = 0; k < 4; k++){
            const m = nb[j*4+k];
            if(m >= 0 && m !== i) set.add(m);
          }
      }
      this.stencil[i] = Int32Array.from(set);
    }
  }

  /* The shed wake: a strip of doublet panels leaving each trailing edge.
   *
   * Straight, along a fixed direction, and growing geometrically so a long
   * wake costs few panels. It is deliberately NOT aligned with the freestream:
   * the influence matrix then depends only on the geometry, so it is built and
   * factored once and every change of incidence is a back-substitution. The
   * error that buys is second order in incidence and is the standard "rigid
   * wake" of every code this method comes from. */
  _wake(){
    this.wake = [];
    if(!this.te.length) return;
    let span = 0;
    for(let j = 0; j < this.N; j++){
      const y = Math.abs(this.c[j*3+1]);
      if(y > span) span = y;
    }
    /* One panel, very long, rather than a chain of them.
     *
     * A constant-strength doublet strip running to infinity IS a horseshoe
     * vortex: the two side edges are its trailing legs and the far edge is at
     * infinity and induces nothing. So a single quad long enough to be
     * effectively infinite is not an approximation to a discretised wake, it
     * is the exact thing the discretised wake was approximating -- and it is
     * one element per trailing edge instead of fourteen. On the wing used for
     * validation that took the wake from 280 elements to 20 and it changed
     * the lift slope in the fourth decimal.
     */
    const L = this.wakeLength || Math.max(span*400, 1e-3);
    const d = this.wakeDir;
    const dl = Math.hypot(d[0], d[1], d[2]) || 1;
    const u = [d[0]/dl, d[1]/dl, d[2]/dl];
    for(const [up, lo] of this.te){
      // the shared edge is the one the two panels have in common
      const e = this._sharedEdge(up, lo);
      if(!e) continue;
      /* The strip has to be wound so its normal continues the UPPER surface's.
       *
       * Taken in the order the shared edge happened to be found in, it came
       * out pointing the other way, which silently applies the Kutta
       * condition with its sign reversed: the trailing-edge doublet jump on
       * an AR 6 wing at 4 degrees came out at -0.16 against a circulation of
       * about 3, and the lift slope was a thirtieth of what it should be.
       * There is no error message for this -- the solve converges and the
       * streamlines are smooth -- so the winding is derived rather than
       * assumed. */
      let a = e[0], b = e[1], t = 0;
      {
        const probe = [a, b,
                       [b[0]+u[0], b[1]+u[1], b[2]+u[2]],
                       [a[0]+u[0], a[1]+u[1], a[2]+u[2]]];
        const F = panelFrame(probe);
        const d = F.n[0]*this.nrm[up*3] + F.n[1]*this.nrm[up*3+1]
                + F.n[2]*this.nrm[up*3+2];
        if(d < 0){ const t2 = a; a = b; b = t2; }
      }
      const steps = this.wakeSteps;
      const r = 1.35;
      let s = steps === 1 ? L : L*(r - 1)/(Math.pow(r, steps) - 1);
      const strip = [];
      for(let k = 0; k < steps; k++){
        const a2 = [a[0]+u[0]*s, a[1]+u[1]*s, a[2]+u[2]*s];
        const b2 = [b[0]+u[0]*s, b[1]+u[1]*s, b[2]+u[2]*s];
        strip.push([a, b, b2, a2]);
        a = a2; b = b2; t += s; s *= r;
      }
      this.wake.push({up, lo, strip});
    }
  }

  _sharedEdge(i, j){
    const A = corners(this.quads, i), B = corners(this.quads, j);
    const near = (p, q) => (p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2;
    const tol = Math.pow(Math.min(this.size[i], this.size[j])*0.05, 2);
    for(let a = 0; a < 4; a++){
      const p1 = A[a], p2 = A[(a+1) & 3];
      for(let b = 0; b < 4; b++){
        const q1 = B[b], q2 = B[(b+1) & 3];
        if((near(p1, q1) < tol && near(p2, q2) < tol)
        || (near(p1, q2) < tol && near(p2, q1) < tol)) return [p1, p2];
      }
    }
    return null;
  }

  /* The influence matrices.
   *
   * A is the doublet potential at each interior collocation point, with the
   * wake folded into the trailing-edge columns. B is the source potential.
   * Both depend only on geometry, so this is paid once.
   *
   * The diagonal is the interior limit of a panel's own doublet potential.
   * The sheet jumps the potential by -mu across itself and is symmetric about
   * it, so just inside it is +mu/2: A_ii = 1/2 exactly, no quadrature. */
  /* The panel potentials, exact near and as a point far.
   *
   * The matrix is N-squared exact panel potentials, each an arctangent pair
   * and four logarithms, and that is where the whole build cost is: 2.3
   * seconds for a thousand panels. Past a few of its own widths a panel's
   * potential is its leading multipole with an error falling like (size/r)^2,
   * which for a matrix entry whose neighbours are a thousand times larger is
   * far below anything the solution can tell. */
  _srcPot(p, j, q, area, nrm){
    const a = j >= 0 ? this.area[j] : area;
    const cx = j >= 0 ? this.c[j*3] : (q[0][0]+q[1][0]+q[2][0]+q[3][0])/4;
    const cy = j >= 0 ? this.c[j*3+1] : (q[0][1]+q[1][1]+q[2][1]+q[3][1])/4;
    const cz = j >= 0 ? this.c[j*3+2] : (q[0][2]+q[1][2]+q[2][2]+q[3][2])/4;
    const dx = p[0]-cx, dy = p[1]-cy, dz = p[2]-cz;
    const r2 = dx*dx + dy*dy + dz*dz;
    if(r2 > FAR_WIDTHS*FAR_WIDTHS*a)
      return -a/(4*Math.PI*Math.sqrt(r2));
    return sourcePotential(p, q, 1, j >= 0 ? this.frames[j] : null);
  }

  _dblPot(p, j, q, area, nrm){
    const a = j >= 0 ? this.area[j] : area;
    const nx = j >= 0 ? this.nrm[j*3] : nrm[0];
    const ny = j >= 0 ? this.nrm[j*3+1] : nrm[1];
    const nz = j >= 0 ? this.nrm[j*3+2] : nrm[2];
    const cx = j >= 0 ? this.c[j*3] : (q[0][0]+q[1][0]+q[2][0]+q[3][0])/4;
    const cy = j >= 0 ? this.c[j*3+1] : (q[0][1]+q[1][1]+q[2][1]+q[3][1])/4;
    const cz = j >= 0 ? this.c[j*3+2] : (q[0][2]+q[1][2]+q[2][2]+q[3][2])/4;
    const dx = p[0]-cx, dy = p[1]-cy, dz = p[2]-cz;
    const r2 = dx*dx + dy*dy + dz*dz;
    if(r2 > FAR_WIDTHS*FAR_WIDTHS*a){
      const r = Math.sqrt(r2);
      return -a*(nx*dx + ny*dy + nz*dz)/(4*Math.PI*r2*r);
    }
    return doubletPotential(p, q, 1);
  }

  _assemble(){
    const N = this.N;
    const A = new Float64Array(N*N);
    const B = new Float64Array(N*N);
    const p = [0, 0, 0];
    const quads = [], mirrors = [], mnrm = [];
    for(let j = 0; j < N; j++){
      const q = corners(this.quads, j);
      quads.push(q);
      if(this.ground){
        const m = this._mirror(q);
        mirrors.push(m);
        const F = panelFrame(m);
        mnrm.push(F.n);
      }
    }
    for(let i = 0; i < N; i++){
      // the collocation point is the centroid; the self terms are taken as
      // their interior limits rather than evaluated there
      p[0] = this.c[i*3]; p[1] = this.c[i*3+1]; p[2] = this.c[i*3+2];
      for(let j = 0; j < N; j++){
        const q = quads[j];
        B[i*N+j] = this._srcPot(p, j, q);
        A[i*N+j] = i === j ? 0.5 : this._dblPot(p, j, q);
        if(this.ground){
          const m = mirrors[j];
          B[i*N+j] += this._srcPot(p, -1, m, this.area[j], mnrm[j]);
          A[i*N+j] += this._dblPot(p, -1, m, this.area[j], mnrm[j]);
        }
      }
      for(const w of this.wake){
        let c = 0;
        for(const s of w.strip){
          c += doubletPotential(p, s, 1);
          if(this.ground) c += doubletPotential(p, this._mirror(s), 1);
        }
        A[i*N + w.up] += c;
        A[i*N + w.lo] -= c;
      }
    }
    this.B = B;
    this.LU = A;
    this.piv = luFactor(A, N);
    this.rhs = new Float64Array(N);
  }

  _mirror(q){
    // a quad reflected in z = 0, wound the other way so its normal still
    // points out of the mirrored body
    return [[q[3][0], q[3][1], -q[3][2]], [q[2][0], q[2][1], -q[2][2]],
            [q[1][0], q[1][1], -q[1][2]], [q[0][0], q[0][1], -q[0][2]]];
  }

  /* Solve for the doublet distribution at this freestream, then read the
   * surface velocity and the pressure off it. */
  setControl(id, deg){ this.deflect[id] = deg; return this; }

  /* Rodrigues: turn n about `a` by `t`. */
  _turn(n, a, t, out){
    const c = Math.cos(t), s = Math.sin(t);
    const d = a[0]*n[0] + a[1]*n[1] + a[2]*n[2];
    const cx = a[1]*n[2] - a[2]*n[1];
    const cy = a[2]*n[0] - a[0]*n[2];
    const cz = a[0]*n[1] - a[1]*n[0];
    out[0] = n[0]*c + cx*s + a[0]*d*(1-c);
    out[1] = n[1]*c + cy*s + a[1]*d*(1-c);
    out[2] = n[2]*c + cz*s + a[2]*d*(1-c);
  }

  _effectiveNormals(){
    const N = this.N;
    this.nEff.set(this.nrm);
    const n = [0,0,0], o = [0,0,0];
    for(const c of this.controls){
      const deg = this.deflect[c.id] || 0;
      if(!deg) continue;
      const t = deg*Math.PI/180;
      for(let k = 0; k < c.panels.length; k++){
        const j = c.panels[k], a = c.axes[k];
        n[0] = this.nrm[j*3]; n[1] = this.nrm[j*3+1]; n[2] = this.nrm[j*3+2];
        this._turn(n, a, t, o);
        this.nEff[j*3] = o[0]; this.nEff[j*3+1] = o[1]; this.nEff[j*3+2] = o[2];
      }
    }
  }

  solve(vinf){
    const N = this.N;
    this.vinf = [vinf[0], vinf[1], vinf[2]];
    this.vfs = Math.hypot(vinf[0], vinf[1], vinf[2]) || 1;
    this._effectiveNormals();
    for(let j = 0; j < N; j++){
      this.sigma[j] = -(this.nEff[j*3]*vinf[0] + this.nEff[j*3+1]*vinf[1]
                      + this.nEff[j*3+2]*vinf[2]);
    }
    for(let i = 0; i < N; i++){
      let s = 0;
      for(let j = 0; j < N; j++) s += this.B[i*N+j]*this.sigma[j];
      this.rhs[i] = -s;
    }
    luSolve(this.LU, this.piv, this.rhs, N, this.mu);
    this._surface();
    this._strengths();
    return this;
  }

  /* Surface velocity and Cp.
   *
   * On the outer face the total potential is Vinf.p - mu, because the doublet
   * sheet jumps it by -mu and the interior was pinned at the freestream. So
   * the surface velocity is the freestream's tangential part minus the
   * surface gradient of mu -- an exact statement, evaluated by least squares
   * over each panel's neighbours in its own tangent plane.
   *
   * Reading it instead by summing every panel's influence at a point just off
   * the skin would be the same number in exact arithmetic and a much worse one
   * in practice: it costs O(N) per panel instead of O(1), and it is the
   * difference of two large nearly-cancelling sums right where the sheet is
   * singular. */
  _surface(){
    const N = this.N, nb = this.nb, vinf = this.vinf;
    for(let i = 0; i < N; i++){
      const F = this.frames[i];
      const st = this.stencil[i];
      let sxx = 0, sxy = 0, syy = 0, sxf = 0, syf = 0;
      for(let k = 0; k < st.length; k++){
        const j = st[k];
        const dx = this.c[j*3] - this.c[i*3];
        const dy = this.c[j*3+1] - this.c[i*3+1];
        const dz = this.c[j*3+2] - this.c[i*3+2];
        const a = dx*F.t[0] + dy*F.t[1] + dz*F.t[2];
        const b = dx*F.s[0] + dy*F.s[1] + dz*F.s[2];
        const d2 = a*a + b*b;
        /* A neighbour has to be far enough away to be a gradient sample.
         *
         * At a closed trailing edge the upper and lower surfaces meet, so
         * their last panels are microns apart in the tip cap's plane while
         * their potentials differ by nearly the whole circulation. Dividing
         * one by the other is not a gradient: the wingtip caps reported Cp
         * -3,000 and, with the cap joined right up to the trailing edge,
         * -19,000,000. Anything inside a sixth of a panel width is that, and
         * is dropped.
         */
        if(d2 < 0.027*this.area[i]) continue;
        const w = 1/Math.sqrt(d2);            // nearer neighbours weigh more
        const f = this.mu[j] - this.mu[i];
        sxx += w*a*a; sxy += w*a*b; syy += w*b*b; sxf += w*a*f; syf += w*b*f;
      }
      /* Ridge-regularised, because a trimmed surface has panels whose
       * neighbours all lie in one direction.
       *
       * A seam panel with two neighbours in a line has no information about
       * the gradient across that line, and a plain least-squares fit answers
       * with whatever divides by the near-zero determinant: two fuselage
       * panels at the root of the fin came out at Cp -10.7, which is not a
       * pressure, and the force integral wore it. The ridge says "and the
       * gradient is small unless the data says otherwise", which in the
       * unconstrained direction is the only honest thing available. */
      const ridge = 1e-3*(sxx + syy);
      const A11 = sxx + ridge, A22 = syy + ridge;
      const det = A11*A22 - sxy*sxy;
      let gt = 0, gs = 0;
      if(Math.abs(det) > 1e-24){
        gt = (sxf*A22 - syf*sxy)/det;
        gs = (syf*A11 - sxf*sxy)/det;
      }
      // the freestream's part in this panel's own surface, using the normal
      // the boundary condition was actually applied with
      const nx = this.nEff[i*3], ny = this.nEff[i*3+1], nz = this.nEff[i*3+2];
      const vn = vinf[0]*nx + vinf[1]*ny + vinf[2]*nz;
      const fx = vinf[0] - vn*nx, fy = vinf[1] - vn*ny, fz = vinf[2] - vn*nz;
      const vt = fx*F.t[0] + fy*F.t[1] + fz*F.t[2] - gt;
      const vs = fx*F.s[0] + fy*F.s[1] + fz*F.s[2] - gs;
      this.vs[i*3]   = vt*F.t[0] + vs*F.s[0];
      this.vs[i*3+1] = vt*F.t[1] + vs*F.s[1];
      this.vs[i*3+2] = vt*F.t[2] + vs*F.s[2];
      const q = (vt*vt + vs*vs)/(this.vfs*this.vfs);
      this.cp[i] = 1 - q;
    }
  }

  /* Force from the pressure on the skin. Non-dimensionalised outside. */
  forces(){
    const f = [0, 0, 0];
    for(let i = 0; i < this.N; i++){
      const k = -this.cp[i]*this.area[i];
      f[0] += k*this.nEff[i*3]; f[1] += k*this.nEff[i*3+1];
      f[2] += k*this.nEff[i*3+2];
    }
    return f;   // divided by q_inf; multiply by 0.5 rho V^2 for newtons
  }

  /* Induced drag, from the wake in the Trefftz plane.
   *
   * Not from the pressure on the skin. Integrating Cp.n over a low-order
   * panelling gives a "near-field" drag that is the small difference of two
   * large nearly-cancelling sums, and on the wings validated here it came out
   * at 0.42 to 0.59 of the value the same lift has to cost -- a factor of two
   * out, with no sign of being wrong. The Trefftz plane asks the question the
   * other way round: far downstream the wake is a two-dimensional vortex
   * sheet, and the kinetic energy left in it is the work the aeroplane did.
   *
   * Each wake strip leaves a trailing vortex at each of its side edges,
   * carrying +/- its own circulation. Summed over the strips the shared edges
   * cancel down to the spanwise gradient of the loading, which is what
   * actually trails.
   */
  trefftz(){
    if(!this.wake.length) return {di: 0, span: 0, lift: 0};
    /* Merge the trailing vortices at shared edges before doing anything
     * with them.
     *
     * Each strip trails +Gamma at one side edge and -Gamma at the other, and
     * where two strips meet those two are at the SAME place: physically they
     * combine into one vortex of strength Gamma_j - Gamma_j+1, which is the
     * spanwise gradient of the loading and is what actually trails. Left as
     * two separate vortices a strip's own pair does not cancel at its own
     * midpoint -- both induce the same way there -- and every strip picks up
     * a spurious 2.Gamma/pi.dy of self-downwash. Span efficiency came out
     * above one, which is impossible for a planar wing.
     */
    const merged = new Map();
    const at2 = (y, z, gam) => {
      const k = `${Math.round(y*1e6)},${Math.round(z*1e6)}`;
      merged.set(k, (merged.get(k) || 0) + gam);
    };
    for(const w of this.wake){
      const g = this.mu[w.up] - this.mu[w.lo];
      const s = w.strip[0];
      at2(s[0][1], s[0][2], g);
      at2(s[1][1], s[1][2], -g);
    }
    const vy = [], vz = [], vg = [];
    for(const [k, gam] of merged){
      if(Math.abs(gam) < 1e-12) continue;
      const [a, b] = k.split(',').map(Number);
      vy.push(a/1e6); vz.push(b/1e6); vg.push(gam);
    }
    const n = vg.length;
    /* In the Trefftz plane the trailing vortices are infinite straight lines,
     * so their induced velocity is the two-dimensional 1/2.pi.r, not 1/4.pi.r.
     * With a ground plane each one has an image of the opposite sign at -z. */
    const at = (y, z, skip) => {
      let wy = 0, wz = 0;
      for(let k = 0; k < n; k++){
        if(k === skip) continue;
        let dy = y - vy[k], dz = z - vz[k];
        let r2 = dy*dy + dz*dz;
        if(r2 > 1e-14){
          const c = vg[k]/(2*Math.PI*r2);
          wy += -c*dz; wz += c*dy;
        }
        if(this.ground){
          dz = z + vz[k];
          r2 = dy*dy + dz*dz;
          if(r2 > 1e-14){
            const c = -vg[k]/(2*Math.PI*r2);
            wy += -c*dz; wz += c*dy;
          }
        }
      }
      return [wy, wz];
    };
    let di = 0, lift = 0, ymin = Infinity, ymax = -Infinity;
    for(const w of this.wake){
      const g = this.mu[w.up] - this.mu[w.lo];
      const s = w.strip[0];
      const yc = 0.5*(s[0][1] + s[1][1]), zc = 0.5*(s[0][2] + s[1][2]);
      const dy = s[1][1] - s[0][1], dz = s[1][2] - s[0][2];
      const [, wz] = at(yc, zc, -1);
      // the sheet's self-induced velocity at its own centre is the mean of
      // the two sides, which the symmetric sum above already gives
      di += -0.5 * g * wz * dy;
      lift += g * dy;
      ymin = Math.min(ymin, s[0][1], s[1][1]);
      ymax = Math.max(ymax, s[0][1], s[1][1]);
    }
    return {di: Math.abs(di), lift: Math.abs(lift), span: ymax - ymin};
  }

  /* ---------------------------------------------------------------- field */

  /* Every sheet in the model as one flat list of elements.
   *
   * A body panel carries a source and a doublet; a wake panel carries a
   * doublet whose strength is a difference of two body panels'; and with a
   * ground plane each of those has an image. Rather than three loops with
   * three sets of special cases at every field point, they are flattened
   * once into elements that all look the same and differ only in where their
   * strength comes from.
   */
  _elements(){
    const els = [];
    const push = (q, sIdx, dUp, dLo, sgn) => {
      const F = panelFrame(q);
      const r = Math.sqrt(F.area);
      els.push({q, F, sIdx, dUp, dLo, sgn,
                c: F.o, n: F.n, a: F.area, r,
                // inside this the exact panel is used, outside it the point
                near2: (EXACT_WIDTHS*r)*(EXACT_WIDTHS*r)});
    };
    this.elOf = new Int32Array(this.N);
    for(let j = 0; j < this.N; j++){
      const q = corners(this.quads, j);
      this.elOf[j] = els.length;
      push(q, j, j, -1, 1);
      if(this.ground) push(this._mirror(q), j, j, -1, 1);
    }
    for(const w of this.wake){
      for(const s of w.strip){
        push(s, -1, w.up, w.lo, 1);
        if(this.ground) push(this._mirror(s), -1, w.up, w.lo, 1);
      }
    }
    this.els = els;
    const M = els.length;
    this.es = new Float64Array(M);
    this.ed = new Float64Array(M);

    /* Far-field lumping.
     *
     * Beyond a few panel widths a panel is a point source of strength
     * sigma.A and a point doublet of moment mu.A.n, and a whole cell of them
     * is one of each. Without this a streamline step costs an exact influence
     * from every element in the model, which on a thousand-panel aeroplane is
     * a hundred thousand of them per traced line.
     *
     * The cell is sized on the model, and the three-cell near radius is the
     * same trade the source-panel field it replaces was swept for: 0.12 %
     * mean error against summing every element, 0.45 % worst.
     */
    /* Sized on the BODY, not on the elements.
     *
     * The wake runs forty spans downstream, so sizing the cell on the whole
     * element list made it as big as the wake: eighteen cells for the entire
     * model, nothing ever far enough away to lump, and the "fast" field
     * evaluation exactly as slow as summing every panel. The body is the
     * thing whose panels need resolving; the wake is sparse and distant and
     * falls into whatever cells it reaches. */
    let ex = 0;
    for(let j = 0; j < this.N; j++)
      ex = Math.max(ex, Math.abs(this.c[j*3]), Math.abs(this.c[j*3+1]),
                        Math.abs(this.c[j*3+2]));
    this.cell = Math.max(ex*0.085, 1e-9);
    this.nearCells = 2;
    const key = (e) => `${Math.floor(e.c[0]/this.cell)},`
                     + `${Math.floor(e.c[1]/this.cell)},`
                     + `${Math.floor(e.c[2]/this.cell)}`;
    /* An element bigger than a cell cannot be cell-lumped.
     *
     * The wake stretches geometrically, so its last panels are twenty spans
     * long while a cell is a quarter of a chord. Binned by centroid, one of
     * those is a point doublet sitting in one cell while the sheet it stands
     * for crosses forty of them -- and a streamline passing right through it
     * reads a lumped value from a "distant" cell. That was 6.5 % of
     * freestream, in the wake, which is the part of the picture the wake is
     * there to make. They get their own list and their own near test, against
     * their OWN size rather than the cell's. */
    const bins = new Map();
    this.big = [];
    els.forEach((e, i) => {
      if(e.r > this.cell){ this.big.push(i); return; }
      const k = key(e);
      if(!bins.has(k)) bins.set(k, []);
      bins.get(k).push(i);
    });
    const C = bins.size;
    this.L = {
      M: C, ix: new Int32Array(C), iy: new Int32Array(C), iz: new Int32Array(C),
      cx: new Float64Array(C), cy: new Float64Array(C), cz: new Float64Array(C),
      q: new Float64Array(C), mx: new Float64Array(C), my: new Float64Array(C),
      mz: new Float64Array(C),
      start: new Int32Array(C+1), order: new Int32Array(els.length),
    };
    let at = 0, ci = 0;
    for(const [k, list] of bins){
      const [a, b, c] = k.split(',').map(Number);
      this.L.ix[ci] = a; this.L.iy[ci] = b; this.L.iz[ci] = c;
      let wx = 0, wy = 0, wz = 0, wa = 0;
      for(const i of list){
        const e = els[i];
        wx += e.c[0]*e.a; wy += e.c[1]*e.a; wz += e.c[2]*e.a; wa += e.a;
      }
      this.L.cx[ci] = wx/wa; this.L.cy[ci] = wy/wa; this.L.cz[ci] = wz/wa;
      this.L.start[ci] = at;
      for(const i of list) this.L.order[at++] = i;
      ci++;
    }
    this.L.start[C] = at;
  }

  /* The per-element strengths and the per-cell moments, for this solution. */
  _strengths(){
    const els = this.els, M = els.length;
    for(let i = 0; i < M; i++){
      const e = els[i];
      this.es[i] = e.sIdx >= 0 ? this.sigma[e.sIdx] : 0;
      this.ed[i] = e.sgn * (this.mu[e.dUp] - (e.dLo >= 0 ? this.mu[e.dLo] : 0));
    }
    const L = this.L;
    for(let c = 0; c < L.M; c++){
      let q = 0, mx = 0, my = 0, mz = 0;
      for(let k = L.start[c]; k < L.start[c+1]; k++){
        const i = L.order[k], e = els[i];
        q += this.es[i]*e.a;
        mx += this.ed[i]*e.a*e.n[0];
        my += this.ed[i]*e.a*e.n[1];
        mz += this.ed[i]*e.a*e.n[2];
      }
      L.q[c] = q; L.mx[c] = mx; L.my[c] = my; L.mz[c] = mz;
    }
  }

  /* Velocity anywhere in the field: the freestream plus every sheet. Exact
   * for the elements near the point, lumped for the rest. */
  velocity(p, out){
    out[0] = this.vinf[0]; out[1] = this.vinf[1]; out[2] = this.vinf[2];
    const L = this.L, cell = this.cell, near = this.nearCells;
    const kx = Math.floor(p[0]/cell), ky = Math.floor(p[1]/cell),
          kz = Math.floor(p[2]/cell);
    const inv4pi = 1/(4*Math.PI);
    const floor2 = (cell*0.5)*(cell*0.5);
    for(let c = 0; c < L.M; c++){
      const dx = L.ix[c]-kx, dy = L.iy[c]-ky, dz = L.iz[c]-kz;
      if(dx > near || dx < -near || dy > near || dy < -near
         || dz > near || dz < -near){
        const rx = p[0]-L.cx[c], ry = p[1]-L.cy[c], rz = p[2]-L.cz[c];
        let r2 = rx*rx + ry*ry + rz*rz;
        if(r2 < floor2) r2 = floor2;
        const r = Math.sqrt(r2), r3 = r2*r, r5 = r3*r2;
        const ks = L.q[c]*inv4pi/r3;
        out[0] += ks*rx; out[1] += ks*ry; out[2] += ks*rz;
        const mr = L.mx[c]*rx + L.my[c]*ry + L.mz[c]*rz;
        const k1 = 3*mr*inv4pi/r5, k2 = inv4pi/r3;
        out[0] += k1*rx - k2*L.mx[c];
        out[1] += k1*ry - k2*L.my[c];
        out[2] += k1*rz - k2*L.mz[c];
        continue;
      }
      for(let k = L.start[c]; k < L.start[c+1]; k++){
        const i = L.order[k], e = this.els[i];
        const rx = p[0]-e.c[0], ry = p[1]-e.c[1], rz = p[2]-e.c[2];
        const r2 = rx*rx + ry*ry + rz*rz;
        if(r2 > e.near2){ this._point(i, e, rx, ry, rz, r2, out); continue; }
        if(this.es[i]) sourceVelocity(p, e.q, this.es[i], out, e.F);
        if(this.ed[i]) doubletVelocity(p, e.q, this.ed[i], out, 0.04);
      }
    }
    /* The oversized elements are always exact.
     *
     * These are the wake strips, and a wake strip is four hundred spans long:
     * its centroid is out at infinity and its "own width" is meaningless, so
     * there is no distance at which a point source and a point doublet stand
     * for it. Treated as points from the wing, the field near the aeroplane
     * was 87 % wrong. There are one or two dozen of them and each is four
     * vortex filaments, so exact costs almost nothing. */
    for(const i of this.big){
      const e = this.els[i];
      if(this.es[i]) sourceVelocity(p, e.q, this.es[i], out, e.F);
      if(this.ed[i]) doubletVelocity(p, e.q, this.ed[i], out, 0.04);
    }
    return out;
  }

  /* One element seen from far enough away to be a point: a source of strength
   * sigma.A and a doublet of moment mu.A.n. This is the element's own leading
   * multipole, so its error falls like (size/r)^2 -- a couple of per cent of
   * that element's contribution at two and a half widths, and that element is
   * by then a small part of the sum.
   *
   * It exists because the exact form is expensive. A panel costs an
   * arctangent pair and four logarithms; at three cells of near radius a
   * point near the skin had 190 of them to evaluate and a field evaluation
   * cost 175 microseconds, which is 50 seconds of streamlines. The exact form
   * is kept where it is the whole point -- within a couple of widths of the
   * skin, where a point element would show its own singularity instead of the
   * sheet. */
  _point(i, e, rx, ry, rz, r2, out){
    const inv4pi = 1/(4*Math.PI);
    const r = Math.sqrt(r2), r3 = r2*r;
    if(this.es[i]){
      const ks = this.es[i]*e.a*inv4pi/r3;
      out[0] += ks*rx; out[1] += ks*ry; out[2] += ks*rz;
    }
    if(this.ed[i]){
      const w = this.ed[i]*e.a;
      const mr = w*(e.n[0]*rx + e.n[1]*ry + e.n[2]*rz);
      const k1 = 3*mr*inv4pi/(r3*r2), k2 = w*inv4pi/r3;
      out[0] += k1*rx - k2*e.n[0];
      out[1] += k1*ry - k2*e.n[1];
      out[2] += k1*rz - k2*e.n[2];
    }
  }

  /* Is p inside the body?
   *
   * The solid angle a closed surface subtends at a point is 4.pi inside it
   * and nothing outside, and the panels already carry everything the sum
   * needs. Same test the source-panel field used, on the same geometry.
   */
  inside(p, probe){
    const b = this.box;
    if(p[0] < b[0] || p[0] > b[1] || p[1] < b[2] || p[1] > b[3]
       || p[2] < b[4] || p[2] > b[5]){
      if(probe){ probe.near = Infinity; probe.size = 0; }
      return false;
    }
    /* The solid angle by its point approximation rather than the exact one.
     *
     * Each panel contributes A.(r.n)/|r|^3, which is the exact form's leading
     * term and costs no arctangents -- and this runs once per streamline step
     * per panel, where two atan2 apiece would cost more than the velocity the
     * step is being taken through. The test is a sign, not a value: the sum
     * is 4.pi inside a closed surface and nothing outside, so it is compared
     * against half of that and the approximation has room to be loose. */
    let om = 0, near = Infinity, ni = -1, nc = 0;
    const close = this._close || (this._close = new Int32Array(64));
    for(let j = 0; j < this.N; j++){
      const dx = p[0]-this.c[j*3], dy = p[1]-this.c[j*3+1], dz = p[2]-this.c[j*3+2];
      const d2 = dx*dx + dy*dy + dz*dz;
      if(d2 < near){ near = d2; ni = j; }
      const r = Math.sqrt(d2);
      if(r < 1e-12) continue;
      const c = this.area[j]*(dx*this.nrm[j*3] + dy*this.nrm[j*3+1]
                            + dz*this.nrm[j*3+2])/(d2*r);
      om += c;
      /* The nearby panels get the exact form.
       *
       * The point approximation is the leading term of a panel's solid angle
       * and it is only the leading term within a panel width or two of it --
       * which on a 12 % thick wing is the whole interior, because the wing is
       * barely thicker than its own panels. Inside/outside then came out
       * wrong for 4 % of points near the surface. Correcting just the close
       * ones costs a handful of arctangents rather than two thousand. */
      if(nc < close.length && d2 < 9*this.area[j]){
        close[nc++] = j;
        om -= c;
      }
    }
    /* Minus, not plus: `solidAngle` is signed the other way round from the
     * point sum here. The sum measures r from the surface to the point and
     * comes to -4.pi inside; the kernel's returns +4.pi there, because it is
     * written for the doublet potential. Added rather than subtracted, the
     * correction cancelled the term it was correcting and made the test
     * worse than the approximation it replaced -- 55 wrong out of 400 against
     * 17. */
    for(let k = 0; k < nc; k++)
      om -= solidAngle(p, this.els[this.elOf[close[k]]].q);
    if(probe && ni >= 0){
      probe.near = Math.sqrt(near);
      probe.size = this.size[ni];
      probe.nx = this.nrm[ni*3]; probe.ny = this.nrm[ni*3+1];
      probe.nz = this.nrm[ni*3+2];
    }
    if(om < -2*Math.PI) return true;
    const st = this.nearStop;
    return st > 0 && ni >= 0 && near < st*st;
  }

  /* Every element, no lumping. Kept so the lumping can be checked against
     the thing it approximates. */
  velocityExact(p, out){
    out[0] = this.vinf[0]; out[1] = this.vinf[1]; out[2] = this.vinf[2];
    for(let i = 0; i < this.els.length; i++){
      const e = this.els[i];
      if(this.es[i]) sourceVelocity(p, e.q, this.es[i], out, e.F);
      if(this.ed[i]) doubletVelocity(p, e.q, this.ed[i], out, 0.04);
    }
    return out;
  }
}
