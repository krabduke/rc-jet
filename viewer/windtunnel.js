/* Vortex-lattice method, in the browser.
 *
 * This is the same method as aero/vlm.py -- horseshoe vortices, flow tangency
 * at three-quarter chord, Kutta-Joukowski for lift, Trefftz plane for induced
 * drag, ground effect by images -- reimplemented so a slider can move a
 * control surface and the numbers change while you watch.
 *
 * It is a real solve, not a lookup. What it cannot do is anything viscous: no
 * boundary layer, no separation, no stall, no profile drag, no compressibility.
 * Past roughly 12 degrees of incidence a real wing has begun to stall and this
 * will happily keep predicting more lift; the viewer says so on screen.
 *
 * Control deflections enter through thin-aerofoil flap theory. A plain flap of
 * chord fraction E shifts the section's zero-lift angle by tau*delta, where
 *
 *     theta = acos(2E - 1),   tau = 1 - (theta - sin theta) / pi
 *
 * so a 26 % chord flaperon at 10 degrees is worth about 6.2 degrees of
 * incidence on the strips it covers. That is the standard result and it is the
 * honest way to feed a control angle into a lattice that carries no camber.
 */

const FAR = 1.0e4;      // how far downstream the trailing legs run, in chords
const RHO = 1.225;

/* ---------------------------------------------------------------- vectors */

function sub(a, b){ return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function add(a, b){ return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function cross(a, b){
  return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]];
}
function dot(a, b){ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function scale(a, k){ return [a[0]*k, a[1]*k, a[2]*k]; }
function norm(a){ return Math.sqrt(dot(a, a)); }
function mirrorZ(p){ return [p[0], p[1], -p[2]]; }

/* Velocity at p from a finite vortex segment a->b of unit strength.
 *
 * The singular-core cutoff is relative to BOTH the segment length and the
 * distance to it -- exactly as in aero/vlm.py. A cutoff relative to the
 * segment alone looks reasonable and is not: on a low aspect ratio wing the
 * tip trailing vortex sits right beside its neighbour's collocation point,
 * the contribution grows without bound as the lattice is refined, and the
 * lift slope walks downwards instead of converging. That is how this was
 * caught -- AR 8 and 12 agreed with the Python solver to four figures while
 * AR 4 was 9 per cent out and drifting.
 */
const CORE = 1e-10;

function segVel(p, a, b, out){
  const r1 = sub(p, a), r2 = sub(p, b);
  const n1 = norm(r1), n2 = norm(r2);
  const c = cross(r1, r2);
  const cc = dot(c, c);
  const r0 = sub(b, a);
  const sc = dot(r0, r0) * Math.max(n1*n1, n2*n2);
  if(cc < CORE * Math.max(sc, 1e-30) || n1 < 1e-12 || n2 < 1e-12){
    out[0]=out[1]=out[2]=0; return out;
  }
  const k = (dot(r0, r1)/n1 - dot(r0, r2)/n2) / (4 * Math.PI * cc);
  out[0] = c[0]*k; out[1] = c[1]*k; out[2] = c[2]*k;
  return out;
}

const _t1 = [0,0,0], _t2 = [0,0,0], _t3 = [0,0,0];

/* A horseshoe: trailing leg in from far upstream of a, bound a->b, trailing
 * leg out to far downstream of b. Wake runs along `w`. */
function horseshoe(p, a, b, w, out){
  const af = [a[0] + w[0]*FAR, a[1] + w[1]*FAR, a[2] + w[2]*FAR];
  const bf = [b[0] + w[0]*FAR, b[1] + w[1]*FAR, b[2] + w[2]*FAR];
  segVel(p, af, a, _t1);
  segVel(p, a, b, _t2);
  segVel(p, b, bf, _t3);
  out[0] = _t1[0] + _t2[0] + _t3[0];
  out[1] = _t1[1] + _t2[1] + _t3[1];
  out[2] = _t1[2] + _t2[2] + _t3[2];
  return out;
}

/* ------------------------------------------------------------ flap theory */

export function flapTau(chordFraction){
  const E = Math.min(0.95, Math.max(0.0, chordFraction));
  const th = Math.acos(2*E - 1);
  return 1 - (th - Math.sin(th)) / Math.PI;
}

/* ------------------------------------------------------------ the lattice */

/* One surface: a straight-tapered panel from root to tip. Sweep, dihedral and
 * height all come out of the two leading-edge points, so a table of these
 * describes a whole aircraft or a whole wing stack. */
function surfacePanels(s, refLen){
  const out = [];
  const nS = s.n_span || 8, nC = s.n_chord || 3;
  const sides = (s.mirror === false || s.axis === 'z') ? [1] : [1, -1];
  const band = (s.control && s.control_span) ? s.control_span : null;

  /* Spanwise stations, uniform.
   *
   * Clustering panels towards the tip is the textbook refinement and it is
   * wrong for this solver: the discrete Trefftz integral is only well behaved
   * while every strip is the same width, and clustering drove CDi to 282 on a
   * plain rectangular wing. Uniform is the arrangement aero/vlm.py was
   * validated at -- lift slope within 7.5 % of lifting-line theory across
   * AR 4 to 12, span efficiency 0.99 at AR 8 -- and matching that exactly is
   * worth more than a refinement whose error budget nobody has checked.
   */
  /* A planform table beats a root chord and a tip chord, because a real
   * delta's leading edge is a curve and two numbers cannot say that. The
   * geometry is lofted from the same table, so the lattice and the model are
   * the same wing rather than two approximations of one. */
  const fromTable = (f) => {
    const t = s.planform;
    if(f <= t[0][0]) return [t[0][1], t[0][2]];
    if(f >= t[t.length-1][0]) return [t[t.length-1][1], t[t.length-1][2]];
    for(let i = 0; i < t.length - 1; i++){
      if(f >= t[i][0] && f <= t[i+1][0]){
        const u = (f - t[i][0])/(t[i+1][0] - t[i][0]);
        return [t[i][1] + (t[i+1][1] - t[i][1])*u,
                t[i][2] + (t[i+1][2] - t[i][2])*u];
      }
    }
    return [t[t.length-1][1], t[t.length-1][2]];
  };

  const station = (f) => {
    const le = [
      s.le_root[0] + (s.le_tip[0] - s.le_root[0]) * f,
      s.le_root[1] + (s.le_tip[1] - s.le_root[1]) * f,
      s.le_root[2] + (s.le_tip[2] - s.le_root[2]) * f,
    ];
    let c = s.c_root + (s.c_tip - s.c_root) * f;
    if(s.planform){
      const [xle, cc] = fromTable(f);
      le[0] = xle; c = cc;
    }
    const tw = (s.twist_root || 0) + ((s.twist_tip || 0) - (s.twist_root || 0)) * f;
    return [le, c, tw * Math.PI / 180];
  };

  /* A fin's span runs in z and its section thickness in y -- the opposite of
   * a wing. Without this the vertical tail cannot be in the lattice at all,
   * which is why the rudder slider moved the geometry and changed nothing in
   * the solve. */
  const vert = s.axis === 'z';
  const point = (f, xc) => {
    const [le, c, tw] = station(f);
    const dx = (xc - 0.25) * c;
    const ct = Math.cos(tw), st = Math.sin(tw);
    if(vert) return [le[0] + 0.25*c + dx*ct, le[1] - dx*st, le[2]];
    return [le[0] + 0.25*c + dx*ct, le[1], le[2] - dx*st];
  };

  for(const sgn of sides){
    for(let i = 0; i < nS; i++){
      const f0 = i/nS, f1 = (i+1)/nS;
      const fm = 0.5*(f0 + f1);
      /* How much of this strip's deflection applies.
       *
       * Not all-or-nothing. A flap's shed vortex has a finite spanwise
       * extent, so the loading blends over roughly a strip either side of
       * the flap end rather than jumping; and numerically, a step between
       * neighbouring strips is a circulation discontinuity the lattice
       * cannot resolve -- it drove CDi to 10 and made CL fall as flap was
       * added. Ramping over the transition is both what happens and what the
       * solver can represent.
       */
      let frac = 1;
      if(band){
        const tr = Math.max(1.0/nS, 0.04);
        const up = Math.min(1, Math.max(0, (fm - band[0])/tr + 0.5));
        const dn = Math.min(1, Math.max(0, (band[1] - fm)/tr + 0.5));
        frac = Math.min(up, dn);
        frac = frac*frac*(3 - 2*frac);            // smoothstep
      }else if(!s.control){
        frac = 0;
      }
      const inBand = frac > 0.001;
      for(let j = 0; j < nC; j++){
        const x0 = j/nC, x1 = (j+1)/nC;
        let p00 = point(f0, x0), p10 = point(f1, x0);
        let p01 = point(f0, x1), p11 = point(f1, x1);
        if(sgn < 0){
          p00 = [p00[0], -p00[1], p00[2]]; p10 = [p10[0], -p10[1], p10[2]];
          p01 = [p01[0], -p01[1], p01[2]]; p11 = [p11[0], -p11[1], p11[2]];
          // negating y reverses the winding; swapping the inboard and outboard
          // corners restores it, so the normal and the bound-vortex direction
          // still agree and the mirrored half does not contribute lift with
          // the wrong sign
          let t = p00; p00 = p10; p10 = t;
          t = p01; p01 = p11; p11 = t;
        }
        const a = add(p00, scale(sub(p01, p00), 0.25));
        const b = add(p10, scale(sub(p11, p10), 0.25));
        const col = scale(add(add(p00, scale(sub(p01, p00), 0.75)),
                              add(p10, scale(sub(p11, p10), 0.75))), 0.5);
        let n = cross(sub(p11, p00), sub(p10, p01));
        const area = 0.5 * norm(n);
        const nn = norm(n);
        n = nn > 1e-12 ? scale(n, 1/nn) : (vert ? [0,1,0] : [0,0,1]);
        /* No "point the normal upwards" guard here.
         *
         * The winding is already made consistent above (the mirrored half has
         * its inboard and outboard corners swapped), so the normal that falls
         * out of the cross product is the right one. Forcing n[2] >= 0 on top
         * of that looks like a harmless safety net and is not: on a panel
         * inclined past 45 degrees -- which is every element of a racing car's
         * front wing -- it flips the normal that the winding correctly
         * produced, and the control surface then responds backwards. Adding
         * front flap took downforce off the front axle. */
        out.push({
          a: scale(a, 1/refLen), b: scale(b, 1/refLen),
          col: scale(col, 1/refLen), n,
          area: area/(refLen*refLen),
          // for a fin, "span" is z: the Trefftz strips key on it
          y: 0.5*(a[1] + b[1])/refLen,
          dy: vert ? Math.abs(b[2] - a[2])/refLen
                   : Math.abs(b[1] - a[1])/refLen,
          vert,
          surface: s.name,
          control: inBand ? s.control : null,
          cfrac: frac,
          tau: s.control_tau !== undefined ? s.control_tau
             : flapTau(s.control_chord !== undefined ? s.control_chord : 0.25),
          csign: s.control_sign === undefined ? 1 : s.control_sign,
          side: sgn,
        });
      }
    }
  }
  return out;
}

/* ------------------------------------------------------------- the solver */

/* LU with partial pivoting, factorised once and reused.
 *
 * The influence matrix depends only on the lattice, and the lattice does not
 * move: a control deflection enters through the boundary condition, not the
 * geometry. So the expensive part -- O(n^3) -- happens once, and every slider
 * movement is an O(n^2) back-substitution. That is the difference between a
 * readout that updates when you let go and one that updates while you drag.
 */
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

function luSolve(LU, piv, b, n){
  const x = new Float64Array(n);
  for(let i = 0; i < n; i++) x[i] = b[piv[i]];
  for(let i = 1; i < n; i++){
    let sum = x[i];
    for(let j = 0; j < i; j++) sum -= LU[i*n+j]*x[j];
    x[i] = sum;
  }
  for(let i = n-1; i >= 0; i--){
    let sum = x[i];
    for(let j = i+1; j < n; j++) sum -= LU[i*n+j]*x[j];
    const d = LU[i*n+i];
    x[i] = Math.abs(d) > 1e-14 ? sum/d : 0;
  }
  return x;
}

/* Induced drag from the Trefftz plane, in the full crossflow plane.
 * Working in y alone is exact for a single planar wing and wrong for anything
 * stacked -- a front wing, rear wing and beam wing shed at the same span
 * stations at different heights. */
function trefftzDrag(panels, gamma, vInf, sRef, ground){
  const strips = new Map();
  for(let i = 0; i < panels.length; i++){
    const p = panels[i];
    const key = p.surface + '|' + p.a[1].toFixed(7) + '|' + p.b[1].toFixed(7);
    let st = strips.get(key);
    if(!st){
      st = {G:0, yl:p.a[1], yr:p.b[1], zl:0, zr:0, n:0};
      strips.set(key, st);
    }
    st.G += gamma[i]; st.zl += p.a[2]; st.zr += p.b[2]; st.n++;
  }
  const items = [];
  for(const st of strips.values()){
    st.zl /= st.n; st.zr /= st.n;
    st.ym = 0.5*(st.yl + st.yr); st.zm = 0.5*(st.zl + st.zr);
    const dy = st.yr - st.yl, dz = st.zr - st.zl;
    const ds = Math.hypot(dy, dz);
    st.ds = ds;
    st.ny = ds > 1e-12 ? -dz/ds : 0;
    st.nz = ds > 1e-12 ?  dy/ds : 1;
    items.push(st);
  }
  const fil = [];
  for(let i = 0; i < items.length; i++){
    const st = items[i];
    fil.push([st.yl, st.zl,  st.G, i]);
    fil.push([st.yr, st.zr, -st.G, i]);
    if(ground){
      fil.push([st.yl, -st.zl, -st.G, -1]);
      fil.push([st.yr, -st.zr,  st.G, -1]);
    }
  }
  let d = 0;
  for(let i = 0; i < items.length; i++){
    const st = items[i];
    let vy = 0, vz = 0;
    for(const f of fil){
      // A strip's own two filaments are included, as they must be: each sits
      // half a strip width from the midpoint and the induced velocity goes
      // like G/ds, but it is then multiplied by ds, so the contribution is
      // 0.5 rho G^2 / pi and does not depend on the strip width at all.
      // Dropping them as a "principal value" turns the whole integral
      // negative -- they are the dominant positive term, not a singularity.
      const dy = st.ym - f[0], dz = st.zm - f[1];
      const r2 = dy*dy + dz*dz;
      if(r2 < 1e-18) continue;
      const k = f[2]/(2*Math.PI*r2);
      vy += -k*dz; vz += k*dy;
    }
    d += 0.5*RHO*st.G*(vy*st.ny + vz*st.nz)*st.ds;
  }
  return d / (0.5*RHO*vInf*vInf*sRef);
}

export class Tunnel {
  constructor(cfg){
    this.cfg = cfg;
    this.refLen = cfg.c_ref;
    this.last = null;
    this._cache = new Map();       // keyed on ground on/off
  }

  /* Build the lattice and factorise its influence matrix. The lattice does
   * not depend on the control positions, so this happens once. */
  _prepare(ground){
    const key = ground ? 'g' : 'f';
    let c = this._cache.get(key);
    if(c) return c;
    const panels = [];
    for(const s of this.cfg.surfaces) panels.push(...surfacePanels(s, this.refLen));
    const n = panels.length;
    const A = new Float64Array(n*n);
    const tmp = [0,0,0];
    const wake = [1, 0, 0];
    for(let i = 0; i < n; i++){
      const pi = panels[i];
      for(let j = 0; j < n; j++){
        const pj = panels[j];
        horseshoe(pi.col, pj.a, pj.b, wake, tmp);
        let vx = tmp[0], vy = tmp[1], vz = tmp[2];
        if(ground){
          horseshoe(pi.col, mirrorZ(pj.b), mirrorZ(pj.a), wake, tmp);
          vx += tmp[0]; vy += tmp[1]; vz += tmp[2];
        }
        A[i*n+j] = vx*pi.n[0] + vy*pi.n[1] + vz*pi.n[2];
      }
    }
    const piv = luFactor(A, n);
    c = {panels, n, LU: A, piv, ground};
    this._cache.set(key, c);
    return c;
  }

  /* Solve for one flight condition.
   *
   * A control deflection is applied to the boundary condition, not to the
   * geometry: a flap of chord fraction E deflected by delta changes the
   * section's zero-lift angle by tau*delta, which is an extra normal velocity
   * of |V|*tau*delta on the strips it covers. Rotating the panels instead
   * moves the lattice, and on a partial-span flap that puts a circulation
   * discontinuity in the middle of a strip: CDi came out at 1.85 for a CL of
   * -0.11, and refining made it worse. This way the matrix never changes, the
   * factorisation is reused, and the flap behaves.
   */
  /* Wind direction is a vector, not an angle.
   *
   * A tunnel blows from one fixed direction; what changes is how the model is
   * held in it. With only `alpha` the aircraft could pitch but never yaw, so
   * a fin could not see the flow and a rudder could not do anything. Passing
   * the freestream as a vector lets attitude -- pitch and sideslip together
   * -- set what the model sees.
   */
  solve(opts){
    const { alpha = 0, beta = 0, v = 20, controls = {},
            ground = false, xRef = null } = opts || {};
    const { panels, n, LU, piv } = this._prepare(ground);
    const L = this.refLen;
    const sRef = this.cfg.s_ref/(L*L);
    const bRef = this.cfg.b_ref/L;
    const xr = (xRef === null ? (this.cfg.x_ref_default || 0) : xRef)/L;

    const a = alpha*Math.PI/180, b = beta*Math.PI/180;
    const vv = [Math.cos(a)*Math.cos(b)*v, Math.sin(b)*v, Math.sin(a)*Math.cos(b)*v];
    const wake = [1, 0, 0];

    const rhs = new Float64Array(n);
    for(let i = 0; i < n; i++){
      const p = panels[i];
      let extra = 0;
      if(p.control && controls[p.control] !== undefined){
        // positive deflection is trailing edge down, which adds lift
        extra = controls[p.control]*Math.PI/180 * p.tau * p.csign
              * (p.cfrac === undefined ? 1 : p.cfrac);
      }
      rhs[i] = -dot(vv, p.n) - v*extra;
    }
    const gamma = luSolve(LU, piv, rhs, n);

    let F = [0,0,0], My = 0;
    const bySurface = {};
    const strips = [];
    const tmp = [0,0,0];
    for(let i = 0; i < n; i++){
      const pi = panels[i];
      const mid = scale(add(pi.a, pi.b), 0.5);
      const ind = [0,0,0];
      for(let j = 0; j < n; j++){
        const pj = panels[j];
        horseshoe(mid, pj.a, pj.b, wake, tmp);
        ind[0] += gamma[j]*tmp[0]; ind[1] += gamma[j]*tmp[1]; ind[2] += gamma[j]*tmp[2];
        if(ground){
          horseshoe(mid, mirrorZ(pj.b), mirrorZ(pj.a), wake, tmp);
          ind[0] += gamma[j]*tmp[0]; ind[1] += gamma[j]*tmp[1]; ind[2] += gamma[j]*tmp[2];
        }
      }
      const vt = add(vv, ind);
      const dl = sub(pi.b, pi.a);
      const df = scale(cross(vt, dl), RHO*gamma[i]);
      F = add(F, df);
      My += -(mid[0] - xr)*df[2] + mid[2]*df[0];
      bySurface[pi.surface] = (bySurface[pi.surface] || 0) + df[2];
      strips.push({y: pi.y*L, z: pi.col[2]*L, dL: df[2]});
    }

    const q = 0.5*RHO*v*v;
    const lift = F[2]*Math.cos(a) - F[0]*Math.sin(a);
    const side = F[1];
    const CDi = trefftzDrag(panels, gamma, v, sRef, ground);
    const CL = lift/(q*sRef);

    this.last = {
      CL, CDi, Cm: My/(q*sRef),
      lift_N: CL*q*this.cfg.s_ref,
      drag_N: CDi*q*this.cfg.s_ref,
      // side force back in newtons: the solve works in chords, so like lift
      // it has to be taken through the coefficient and the REAL area
      CY: side/(q*sRef),
      side_N: (side/(q*sRef))*q*this.cfg.s_ref,
      beta,
      panels: n, gamma, lattice: panels, alpha, v, ground,
      bySurface, strips,
      LD: Math.abs(CDi) > 1e-9 ? Math.abs(CL/CDi) : Infinity,
    };
    return this.last;
  }

  /* A closure giving the velocity the solved lattice induces at a point.
   *
   * The flow field needs this to make the body see the wings and to trace
   * streamlines through both. Chordwise panels in a strip are merged into one
   * filament first: that is the right far-field simplification and it cuts
   * the per-point cost by the chordwise panel count.
   */
  inducedVelocity(){
    const sol = this.last;
    if(!sol) return null;
    const L = this.refLen;
    const fils = filaments(sol, sol.ground);
    const wake = [1, 0, 0];
    const t = [0, 0, 0], q = [0, 0, 0];
    return (p, out) => {
      q[0] = p[0]/L; q[1] = p[1]/L; q[2] = p[2]/L;
      for(let i = 0; i < fils.length; i++){
        const f = fils[i];
        horseshoe(q, f.a, f.b, wake, t);
        out[0] += f.G*t[0]; out[1] += f.G*t[1]; out[2] += f.G*t[2];
      }
      return out;
    };
  }

  /* Neutral point as a fraction of MAC, from two solves. Moments are taken
   * about the MAC quarter-chord; referencing them to the origin makes Cm a
   * small difference between large numbers and the answer becomes noise. */
  neutralPoint(opts){
    const xLeMac = this.cfg.x_le_mac;
    const xRef = xLeMac + 0.25*this.cfg.c_ref;
    const s1 = this.solve({...opts, alpha: 0, xRef});
    const c1 = s1.CL, m1 = s1.Cm;
    const s2 = this.solve({...opts, alpha: 4, xRef});
    const dCL = s2.CL - c1;
    if(Math.abs(dCL) < 1e-9) return NaN;
    const dCmdCL = (s2.Cm - m1)/dCL;
    const xNp = xRef - dCmdCL*this.cfg.c_ref;
    return (xNp - xLeMac)/this.cfg.c_ref;
  }

  /* Lift slope per radian, from the same two solves. */
  liftSlope(opts){
    const c1 = this.solve({...opts, alpha: 0}).CL;
    const c2 = this.solve({...opts, alpha: 4}).CL;
    return (c2 - c1)/(4*Math.PI/180);
  }
}

/* ------------------------------------------------- streamlines for display
 *
 * Particles are advected by the freestream plus the velocity the solved vortex
 * system induces on them, so the smoke bends around the surfaces because the
 * circulation is there -- it is the solution being drawn, not a decoration.
 *
 * Chordwise panels in a strip are merged into one filament first. That is the
 * right far-field simplification and it cuts the per-particle cost by the
 * chordwise panel count.
 */
export function filaments(sol, ground){
  const map = new Map();
  const L = 1;
  for(let i = 0; i < sol.lattice.length; i++){
    const p = sol.lattice[i];
    const key = p.surface + '|' + p.a[1].toFixed(6) + '|' + p.b[1].toFixed(6);
    let f = map.get(key);
    if(!f){ f = {a:[0,0,0], b:[0,0,0], G:0, n:0}; map.set(key, f); }
    f.a[0] += p.a[0]; f.a[1] += p.a[1]; f.a[2] += p.a[2];
    f.b[0] += p.b[0]; f.b[1] += p.b[1]; f.b[2] += p.b[2];
    f.G += sol.gamma[i]; f.n++;
  }
  const out = [];
  for(const f of map.values()){
    out.push({a: scale(f.a, 1/f.n), b: scale(f.b, 1/f.n), G: f.G});
    if(ground){
      const a = scale(f.a, 1/f.n), b = scale(f.b, 1/f.n);
      out.push({a: mirrorZ(b), b: mirrorZ(a), G: f.G});
    }
  }
  return out;
}

export function velocityAt(p, fils, vv, wake, out){
  out[0] = vv[0]; out[1] = vv[1]; out[2] = vv[2];
  const t = [0,0,0];
  for(let i = 0; i < fils.length; i++){
    const f = fils[i];
    horseshoe(p, f.a, f.b, wake, t);
    out[0] += f.G*t[0]; out[1] += f.G*t[1]; out[2] += f.G*t[2];
  }
  return out;
}
