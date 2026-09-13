/* The flow field: solid body + lifting surfaces, and the streamlines through it.
 *
 * windtunnel.js solves the lifting surfaces. That gives forces, but a lattice
 * of wings is invisible to the air everywhere else -- streamlines traced from
 * it pass straight through the tub, the sidepods and the wheels. This adds the
 * solid body as source panels, so the flow has something to go around.
 *
 * Method: constant-strength source panels, Hess-Smith, at lowest order -- each
 * panel is a point source of strength sigma*area at its centroid. The
 * self-influence of a flat constant-source panel on its own normal velocity is
 * exactly sigma/2, which is the diagonal. Solve
 *
 *     0.5 sigma_i + sum_j!=i  (A_j / 4pi) (r.n_i)/|r|^3 sigma_j = -V.n_i
 *
 * and the body stops leaking. The point-source approximation is accurate while
 * panels are small compared with the distance between them; close to the
 * surface it is not, which is why the tracer keeps a standoff and why this is
 * for visualising the flow rather than for reading pressures off the skin.
 *
 * Body and wings are coupled by iteration: solve the body in the freestream,
 * solve the wings in freestream plus body, re-solve the body including the
 * wings. Two passes is enough to settle it, and it leaves the validated
 * vortex-lattice solve untouched.
 */

const RHO = 1.225;

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

export class BodyField {
  /* `panels` is {n, c:[x,y,z...], n_:[nx,ny,nz...], a:[area...]} in metres,
     in the model's own frame (x aft, y span, z up).

     `ground` mirrors every source in the plane z = 0. For a solid wall the
     image of a source is a source of the SAME sign at the reflected height,
     which makes the normal velocity vanish on the wall exactly. Without it
     the streamlines under a car simply carry on downwards through the track,
     which is both wrong and the most important part of the flow to get
     right on a ground-effect car. */
  constructor(panels, ground){
    this.ground = !!ground;
    this.N = panels.n;
    this.c = Float64Array.from(panels.c);
    this.nrm = Float64Array.from(panels.n_);
    this.area = Float64Array.from(panels.a);
    this.sigma = new Float64Array(this.N);
    // a length scale per panel, used to keep the point-source approximation
    // away from its own singularity while tracing
    this.rmin = new Float64Array(this.N);
    // a standoff for field evaluation only: a point source is singular at
    // its own centroid, and streamlines do come close to the skin
    for(let i = 0; i < this.N; i++) this.rmin[i] = Math.sqrt(this.area[i]) * 1.15;
    this._factor();
  }

  _factor(){
    const N = this.N, c = this.c, nrm = this.nrm, area = this.area;
    const A = new Float64Array(N*N);
    const inv4pi = 1/(4*Math.PI);
    for(let i = 0; i < N; i++){
      const cx = c[i*3], cy = c[i*3+1], cz = c[i*3+2];
      const nx = nrm[i*3], ny = nrm[i*3+1], nz = nrm[i*3+2];
      for(let j = 0; j < N; j++){
        if(this.ground){
          // the image of panel j, reflected in z = 0
          const rx = cx - c[j*3], ry = cy - c[j*3+1], rz = cz + c[j*3+2];
          const r2 = rx*rx + ry*ry + rz*rz;
          if(r2 > 1e-12){
            const r = Math.sqrt(r2);
            A[i*N+j] += area[j]*inv4pi*(rx*nx + ry*ny + rz*nz)/(r2*r);
          }
        }
        if(i === j){ A[i*N+j] += 0.5; continue; }
        // No near-field clamp here. The clamp belongs in field evaluation,
        // where a streamline can come arbitrarily close to a panel; applying
        // it inside the matrix destroys the neighbour terms, which are the
        // dominant ones in a source-panel method. With the clamp in, the
        // solved body leaked 15 m/s of a 69 m/s freestream through its own
        // skin -- 22 per cent transparent.
        const rx = cx - c[j*3], ry = cy - c[j*3+1], rz = cz - c[j*3+2];
        const r2 = rx*rx + ry*ry + rz*rz;
        const r = Math.sqrt(r2);
        A[i*N+j] += area[j]*inv4pi*(rx*nx + ry*ny + rz*nz)/(r2*r);
      }
    }
    this.LU = A;
    this.piv = luFactor(A, N);
    this.rhs = new Float64Array(N);
  }

  /* Solve the source strengths for a freestream plus whatever extra velocity
     the lifting surfaces induce at each panel centroid. */
  solve(vinf, extra){
    const N = this.N, nrm = this.nrm;
    for(let i = 0; i < N; i++){
      let vx = vinf[0], vy = vinf[1], vz = vinf[2];
      if(extra){ vx += extra[i*3]; vy += extra[i*3+1]; vz += extra[i*3+2]; }
      this.rhs[i] = -(vx*nrm[i*3] + vy*nrm[i*3+1] + vz*nrm[i*3+2]);
    }
    luSolve(this.LU, this.piv, this.rhs, N, this.sigma);
    return this.sigma;
  }

  /* Velocity the body's sources induce at a point. */
  add(p, out){
    const N = this.N, c = this.c, area = this.area, s = this.sigma;
    const inv4pi = 1/(4*Math.PI);
    let ax = 0, ay = 0, az = 0;
    for(let j = 0; j < N; j++){
      const rx = p[0] - c[j*3], ry = p[1] - c[j*3+1], rz = p[2] - c[j*3+2];
      let r2 = rx*rx + ry*ry + rz*rz;
      const rm = this.rmin[j];
      if(r2 < rm*rm) r2 = rm*rm;
      const r = Math.sqrt(r2);
      const k = s[j]*area[j]*inv4pi/(r2*r);
      ax += k*rx; ay += k*ry; az += k*rz;
      if(this.ground){
        const ix = rx, iy = ry, iz = p[2] + c[j*3+2];
        let i2 = ix*ix + iy*iy + iz*iz;
        if(i2 < rm*rm) i2 = rm*rm;
        const ir = Math.sqrt(i2);
        const ik = s[j]*area[j]*inv4pi/(i2*ir);
        ax += ik*ix; ay += ik*iy; az += ik*iz;
      }
    }
    out[0] += ax; out[1] += ay; out[2] += az;
    return out;
  }

  /* How close is p to the skin? Used to stop a streamline that has wandered
     inside the body, which the point-source approximation permits. */
  nearest(p){
    const N = this.N, c = this.c;
    let best = Infinity, bi = -1;
    for(let j = 0; j < N; j++){
      const rx = p[0] - c[j*3], ry = p[1] - c[j*3+1], rz = p[2] - c[j*3+2];
      const d = rx*rx + ry*ry + rz*rz;
      if(d < best){ best = d; bi = j; }
    }
    return {d: Math.sqrt(best), i: bi};
  }

  inside(p){
    const {d, i} = this.nearest(p);
    if(i < 0) return false;
    const rx = p[0] - this.c[i*3], ry = p[1] - this.c[i*3+1], rz = p[2] - this.c[i*3+2];
    const dn = rx*this.nrm[i*3] + ry*this.nrm[i*3+1] + rz*this.nrm[i*3+2];
    return dn < 0 && d < this.rmin[i]*4;
  }
}

/* --------------------------------------------------------------- tracing */

/* Trace one streamline forward through a velocity function.
 *
 * Second-order Runge-Kutta with a step set by the local speed, so the line is
 * resolved where the flow is doing something and does not waste points where
 * it is not. Streamlines are steady: traced once, they stand still and can be
 * looked at, which is the whole point of drawing them.
 */
export function traceLine(vel, seed, opts){
  const {maxSteps = 260, ds = 0.05, xEnd = 1e9, bounds = null,
         body = null} = opts || {};
  const pts = [], spd = [];
  const p = [seed[0], seed[1], seed[2]];
  const v = [0,0,0], k1 = [0,0,0], mid = [0,0,0];
  for(let s = 0; s < maxSteps; s++){
    vel(p, v);
    const m = Math.hypot(v[0], v[1], v[2]);
    if(!isFinite(m) || m < 1e-6) break;
    pts.push(p[0], p[1], p[2]);
    spd.push(m);
    if(p[0] > xEnd) break;
    if(bounds && (p[1] < bounds[0] || p[1] > bounds[1]
               || p[2] < bounds[2] || p[2] > bounds[3])) break;
    k1[0] = v[0]/m; k1[1] = v[1]/m; k1[2] = v[2]/m;
    mid[0] = p[0] + k1[0]*ds*0.5;
    mid[1] = p[1] + k1[1]*ds*0.5;
    mid[2] = p[2] + k1[2]*ds*0.5;
    vel(mid, v);
    const m2 = Math.hypot(v[0], v[1], v[2]) || 1;
    p[0] += v[0]/m2*ds; p[1] += v[1]/m2*ds; p[2] += v[2]/m2*ds;
    if(body && body.inside(p)) break;
  }
  return {pts, spd};
}

/* The colour map the reference CFD images use: blue through cyan, green and
 * yellow to red. It is not a good perceptual map, but it is the one everyone
 * reading a CFD plot already knows how to read, and a legend makes it exact. */
export function velocityColour(t, out){
  const x = Math.max(0, Math.min(1, t));
  let r, g, b;
  if(x < 0.25){ const f = x/0.25;         r = 0;        g = f*0.85;      b = 1; }
  else if(x < 0.5){ const f = (x-0.25)/0.25; r = 0;      g = 0.85+f*0.15; b = 1-f; }
  else if(x < 0.75){ const f = (x-0.5)/0.25; r = f;      g = 1;           b = 0; }
  else { const f = (x-0.75)/0.25;            r = 1;      g = 1-f*0.92;    b = f*0.25; }
  out[0] = r; out[1] = g; out[2] = b;
  return out;
}
