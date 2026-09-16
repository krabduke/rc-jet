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
    // the body's own bounding box, so the inside test can reject the great
    // majority of points without summing anything
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity,
        z0 = Infinity, z1 = -Infinity;
    for(let i = 0; i < this.N; i++){
      const X = this.c[i*3], Y = this.c[i*3+1], Z = this.c[i*3+2];
      if(X < x0) x0 = X; if(X > x1) x1 = X;
      if(Y < y0) y0 = Y; if(Y > y1) y1 = Y;
      if(Z < z0) z0 = Z; if(Z > z1) z1 = Z;
    }
    this.box = [x0, x1, y0, y1, z0, z1];
    // how close a streamline may come to the skin before it is stopped. The
    // field's owner sets this from the model's own size; until then, off.
    this.nearStop = 0;
    /* A length scale per panel, for field evaluation only.
     *
     * One point source per panel is a SAMPLING of a source sheet. Read from
     * further away than the sample spacing it is the sheet; read from closer
     * it is the samples, and what you see is 1,046 individual sources with
     * strengths that differ panel to panel. Drawn, that is not flow -- it is
     * scribble, and it was 34 % of the points on a traced streamline turning
     * more than 25 degrees from the step before.
     *
     * So the kernel is smoothed over a panel width, which reconstructs the
     * sheet from its samples. `rmin` is that width. */
    this.rmin = new Float64Array(this.N);
    for(let i = 0; i < this.N; i++) this.rmin[i] = Math.sqrt(this.area[i]) * 1.15;
    // far-field lumping: cell size from the model, and how many cells count
    // as "near" and are summed panel by panel
    let ex = 0;
    for(let i = 0; i < this.N; i++) ex = Math.max(ex, Math.abs(this.c[i*3]));
    // Measured, not guessed. Sweeping cell size against near-radius over 700
    // sample points in the region the streamlines actually visit:
    //
    //   cellFrac  near   mean err   worst err   speedup
    //     0.060     2      0.18%       1.37%      1.8x
    //     0.085     1      0.48%       5.57%      5.4x
    //     0.085     3      0.12%       0.45%      2.4x   <-- this
    //     0.140     1      0.91%       5.64%      4.8x
    //
    // The 5x options are tempting and wrong: 5 % of freestream bends a
    // streamline visibly over a five-metre trace. 0.45 % worst case does not.
    /* Far-field lumping cell, in the body's own units.
     *
     * The 0.08 m floor was swept and chosen on a car; on a 0.44 m aeroplane
     * it is 18 per cent of the model rather than 8, so the far field was
     * being approximated twice as coarsely on the smaller body. The sweep
     * that picked 0.085 was in units of the extent, and that is the part
     * that transfers between models. */
    this.lumpCell = Math.max(ex * 0.02, ex * 0.085);
    this.nearCells = 3;
    this.L = null;
    // Flow ports: engine intakes, exhausts and fan mouths. Each is a known
    // point source/sink (Q < 0 ingests, Q > 0 blows) softened by a core radius
    // so the velocity stays finite at the mouth. They enter the boundary
    // condition as a known field, so they cost no extra unknowns in the solve.
    this.ports = [];
    this._factor();
  }

  setPorts(ports){
    this.ports = ports || [];
  }

  /* Velocity of the port field at p (softened point sources, with ground
     images). out is accumulated into, like add(). */
  addPorts(p, out){
    const inv4pi = 1/(4*Math.PI);
    for(const q of this.ports){
      const rc2 = q.r*q.r;
      let rx = p[0] - q.p[0], ry = p[1] - q.p[1], rz = p[2] - q.p[2];
      let r2 = rx*rx + ry*ry + rz*rz + rc2;
      let k = q.Q*inv4pi/(r2*Math.sqrt(r2));
      out[0] += k*rx; out[1] += k*ry; out[2] += k*rz;
      if(this.ground){
        rz = p[2] + q.p[2];
        r2 = rx*rx + ry*ry + rz*rz + rc2;
        k = q.Q*inv4pi/(r2*Math.sqrt(r2));
        out[0] += k*rx; out[1] += k*ry; out[2] += k*rz;
      }
    }
    return out;
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
    const pv = [0, 0, 0];
    for(let i = 0; i < N; i++){
      let vx = vinf[0], vy = vinf[1], vz = vinf[2];
      if(extra){ vx += extra[i*3]; vy += extra[i*3+1]; vz += extra[i*3+2]; }
      if(this.ports.length){
        pv[0] = pv[1] = pv[2] = 0;
        this.addPorts([this.c[i*3], this.c[i*3+1], this.c[i*3+2]], pv);
        vx += pv[0]; vy += pv[1]; vz += pv[2];
      }
      this.rhs[i] = -(vx*nrm[i*3] + vy*nrm[i*3+1] + vz*nrm[i*3+2]);
    }
    luSolve(this.LU, this.piv, this.rhs, N, this.sigma);
    // the lumped far field is built from sigma, so it is stale now
    this.L = null;
    return this.sigma;
  }

  /* Velocity the body's sources induce at a point. */
  /* Lump the panels into cells once per solve, so the field can be evaluated
   * without visiting every panel.
   *
   * Tracing was the whole cost of a retrace -- 3.8 seconds of a 4.0 second
   * update -- because every one of roughly a hundred thousand velocity
   * evaluations summed all 1,188 panels and all 1,188 ground images. A source
   * falls off as 1/r^2, so beyond a few cell widths a panel is
   * indistinguishable from its share of a lumped source at its cell's centre
   * of strength. Near cells are still summed panel by panel, where the detail
   * matters and the approximation would be wrong.
   *
   * Everything here is a flat typed array and the arithmetic is inlined. The
   * first attempt used an array of cell objects and a helper closure, and came
   * out 1.8x SLOWER than the loop it replaced: the helper mutated `ax, ay, az`
   * from the enclosing scope, which forces the engine to allocate a context
   * for them and turns three register adds into three heap writes. An
   * optimisation that is not measured is a guess.
   *
   * This is a one-level Barnes-Hut. The solve that sets the panel strengths is
   * exact and untouched.
   */
  _lump(){
    const cell = this.lumpCell, N = this.N;
    const map = new Map();
    const kOf = new Int32Array(N);
    for(let j = 0; j < N; j++){
      const kx = Math.floor(this.c[j*3]/cell);
      const ky = Math.floor(this.c[j*3+1]/cell);
      const kz = Math.floor(this.c[j*3+2]/cell);
      const key = (kx + 512) * 1048576 + (ky + 512) * 1024 + (kz + 512);
      let id = map.get(key);
      if(id === undefined){ id = map.size; map.set(key, id); }
      kOf[j] = id;
    }
    const M = map.size;
    const count = new Int32Array(M);
    for(let j = 0; j < N; j++) count[kOf[j]]++;
    const start = new Int32Array(M + 1);
    for(let i = 0; i < M; i++) start[i+1] = start[i] + count[i];
    const fill = start.slice(0, M);
    const order = new Int32Array(N);
    for(let j = 0; j < N; j++) order[fill[kOf[j]]++] = j;

    const cw = new Float64Array(M), cx = new Float64Array(M);
    const cy = new Float64Array(M), cz = new Float64Array(M);
    const ix = new Int32Array(M), iy = new Int32Array(M), iz = new Int32Array(M);
    const aw = new Float64Array(M);
    for(let j = 0; j < N; j++){
      const id = kOf[j], w = this.sigma[j]*this.area[j];
      cw[id] += w;
      // weight the centre by |strength|, so a cell whose sources cancel does
      // not put its lumped source somewhere meaningless
      const a2 = Math.abs(w) + 1e-12;
      aw[id] += a2;
      cx[id] += this.c[j*3]*a2;
      cy[id] += this.c[j*3+1]*a2;
      cz[id] += this.c[j*3+2]*a2;
      ix[id] = Math.floor(this.c[j*3]/cell);
      iy[id] = Math.floor(this.c[j*3+1]/cell);
      iz[id] = Math.floor(this.c[j*3+2]/cell);
    }
    for(let i = 0; i < M; i++){ cx[i] /= aw[i]; cy[i] /= aw[i]; cz[i] /= aw[i]; }
    this.L = {M, cw, cx, cy, cz, ix, iy, iz, start, order};
    return this.L;
  }

  add(p, out){
    if(!this.L) this._lump();
    const L = this.L, M = L.M, cell = this.lumpCell, near = this.nearCells;
    const c = this.c, area = this.area, s = this.sigma, rmin = this.rmin;
    const inv4pi = 1/(4*Math.PI);
    const px = p[0], py = p[1], pz = p[2];
    const kx = Math.floor(px/cell), ky = Math.floor(py/cell), kz = Math.floor(pz/cell);
    const ground = this.ground;
    const farMin2 = (cell*0.5)*(cell*0.5);
    let ax = 0, ay = 0, az = 0;

    for(let i = 0; i < M; i++){
      const dx = L.ix[i] - kx, dy = L.iy[i] - ky, dz = L.iz[i] - kz;
      if(dx > near || dx < -near || dy > near || dy < -near
         || dz > near || dz < -near){
        let rx = px - L.cx[i], ry = py - L.cy[i], rz = pz - L.cz[i];
        let r2 = rx*rx + ry*ry + rz*rz;
        if(r2 < farMin2) r2 = farMin2;
        let k = L.cw[i]*inv4pi/(r2*Math.sqrt(r2));
        ax += k*rx; ay += k*ry; az += k*rz;
        if(ground){
          rz = pz + L.cz[i];
          r2 = rx*rx + ry*ry + rz*rz;
          if(r2 < farMin2) r2 = farMin2;
          k = L.cw[i]*inv4pi/(r2*Math.sqrt(r2));
          ax += k*rx; ay += k*ry; az += k*rz;
        }
        continue;
      }
      const e0 = L.start[i], e1 = L.start[i+1];
      for(let e = e0; e < e1; e++){
        const j = L.order[e];
        const w = s[j]*area[j]*inv4pi;
        const rm2 = rmin[j]*rmin[j];
        let rx = px - c[j*3], ry = py - c[j*3+1], rz = pz - c[j*3+2];
        // Softened, not clamped. See `rmin` above: clamping the radius leaves
        // every source at full strength right up to its own core and only
        // stops it going infinite, so the samples stay visible. Adding the
        // core to r-squared is the standard regularisation and it is what
        // turns the samples back into a sheet -- 34 % of traced points
        // kinking became 0.4 %.
        let r2 = rx*rx + ry*ry + rz*rz + rm2;
        let k = w/(r2*Math.sqrt(r2));
        ax += k*rx; ay += k*ry; az += k*rz;
        if(ground){
          rz = pz + c[j*3+2];
          r2 = rx*rx + ry*ry + rz*rz + rm2;
          k = w/(r2*Math.sqrt(r2));
          ax += k*rx; ay += k*ry; az += k*rz;
        }
      }
    }
    out[0] += ax; out[1] += ay; out[2] += az;
    if(this.ports.length) this.addPorts(p, out);
    return out;
  }

  /* Every panel, no approximation. Kept so the far-field lumping can be
     checked against the thing it approximates. */
  addExact(p, out){
    const N = this.N, c = this.c, area = this.area, s = this.sigma;
    const inv4pi = 1/(4*Math.PI);
    let ax = 0, ay = 0, az = 0;
    for(let j = 0; j < N; j++){
      const rx = p[0] - c[j*3], ry = p[1] - c[j*3+1], rz = p[2] - c[j*3+2];
      const rm = this.rmin[j];
      const r2 = rx*rx + ry*ry + rz*rz + rm*rm;
      const r = Math.sqrt(r2);
      const k = s[j]*area[j]*inv4pi/(r2*r);
      ax += k*rx; ay += k*ry; az += k*rz;
      if(this.ground){
        const ix = rx, iy = ry, iz = p[2] + c[j*3+2];
        const i2 = ix*ix + iy*iy + iz*iz + rm*rm;
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

  /* Is p inside the body?
   *
   * This used to ask whether p was behind the nearest panel AND within four
   * of that panel's own lengths of it. That catches a point which has just
   * slipped under the skin and nothing else: a streamline crossing the middle
   * of a 40 mm fuselage is 20 mm in, its nearest panel is 20 mm away, and
   * four panel lengths there is about 8 -- so the line was never stopped and
   * came out of the far side. Streamlines going straight through the
   * aeroplane is what that looked like.
   *
   * Gauss instead. The solid angle a closed surface subtends at a point is
   * 4.pi if the point is inside it and zero if it is not, and every panel
   * already carries the centroid, area and outward normal the sum needs. It
   * costs one more pass over the panels per step, which is the same order as
   * the velocity evaluation the step already does.
   */
  /* Is p inside the body, or close enough to it to stop?
   *
   * `probe`, if given, comes back carrying `near` -- the distance from p to
   * the nearest panel centroid -- and that panel's outward normal in `nx/ny/
   * nz`. The walk over the panels has to find both anyway, and a streamline
   * needs them to size its next step and to keep from being drawn into the
   * skin, so handing them back costs nothing and saves a second pass over
   * 1,450 panels for every step of every line.
   */
  inside(p, probe){
    if(probe){
      probe.near = Infinity; probe.size = 0;
      probe.nx = probe.ny = probe.nz = 0;
    }
    const b = this.box;
    if(b && (p[0] < b[0] || p[0] > b[1] || p[1] < b[2] || p[1] > b[3]
             || p[2] < b[4] || p[2] > b[5])) return false;
    const N = this.N, c = this.c, n = this.nrm, a = this.area;
    let omega = 0, near = Infinity, ni = -1;
    for(let j = 0; j < N; j++){
      const rx = p[0] - c[j*3], ry = p[1] - c[j*3+1], rz = p[2] - c[j*3+2];
      const r2 = rx*rx + ry*ry + rz*rz;
      if(r2 < 1e-14) return true;
      if(r2 < near){ near = r2; ni = j; }
      const r = Math.sqrt(r2);
      omega += a[j]*(rx*n[j*3] + ry*n[j*3+1] + rz*n[j*3+2])/(r2*r);
    }
    // outward normals, r measured from the surface to p: -4.pi inside, 0 out
    if(omega < -2*Math.PI) return true;
    /* And stop short of the skin as well as at it.
     *
     * A point source is singular at its own centroid, so the last few steps of
     * a line that runs right up to a panel are in a region where the velocity
     * is an artefact of the discretisation rather than the flow: on a 22 m/s
     * aeroplane those points read 185 m/s, and being the fastest in the field
     * they set the colour scale for everything else.
     *
     * The distance is set from the MODEL, not the panel. A panel-relative
     * stop is 2 mm on an aeroplane panelised at 3 mm and 80 mm on a car
     * panelised at 100 -- and 80 mm would delete every streamline under a
     * floor running 30 mm off the road, which is the one part of that car
     * worth looking at. `nearStop` is set by whoever built the field.
     */
    if(probe && ni >= 0){
      probe.near = Math.sqrt(near);
      probe.size = Math.sqrt(a[ni]);
      probe.nx = n[ni*3]; probe.ny = n[ni*3+1]; probe.nz = n[ni*3+2];
    }
    const st = this.nearStop;
    return st > 0 && ni >= 0 && near < st * st;
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
/* ------------------------------------------------------------ surface Cp */

/* Pressure coefficient at every panel centroid.
 *
 * This was missing entirely, and the header of this file used to say so: the
 * field was "for visualising the flow rather than for reading pressures off
 * the skin". So a car built around 2.55 CLA of downforce could not show where
 * any of it came from -- not the suction under the floor, not the stagnation
 * on the nose, not the peak on the wing. Streamlines tell you where the air
 * went. Only pressure tells you what it did.
 *
 * Method: evaluate the total velocity a short way off each panel along its own
 * normal -- far enough out that the panel's own point-source singularity does
 * not dominate, close enough that it is still the surface -- then take the
 * tangential component and use the incompressible Bernoulli relation
 *
 *     Cp = 1 - (Vt / Vinf)^2
 *
 * A solved body has no normal velocity on the skin, so dropping the normal
 * component costs nothing in principle and removes the residual the
 * point-source approximation leaves in practice. Cp = 1 at a stagnation
 * point; Cp < 0 wherever the flow has been accelerated, which on this car is
 * everywhere that matters.
 */
export function surfaceCp(body, vinf, latticeVel){
  const N = body.N, cp = new Float64Array(N);
  const vfs = Math.hypot(vinf[0], vinf[1], vinf[2]) || 1;
  const p = [0, 0, 0], v = [0, 0, 0];
  for(let i = 0; i < N; i++){
    const nx = body.nrm[i*3], ny = body.nrm[i*3+1], nz = body.nrm[i*3+2];
    // stand off by the panel's own length scale: inside that radius the
    // point-source model of the panel is not the panel
    const d = body.rmin[i] * 0.85;
    p[0] = body.c[i*3] + nx*d;
    p[1] = body.c[i*3+1] + ny*d;
    p[2] = body.c[i*3+2] + nz*d;
    v[0] = vinf[0]; v[1] = vinf[1]; v[2] = vinf[2];
    body.add(p, v);
    if(latticeVel) latticeVel(p, v);
    const vn = v[0]*nx + v[1]*ny + v[2]*nz;
    const tx = v[0] - vn*nx, ty = v[1] - vn*ny, tz = v[2] - vn*nz;
    const q = (tx*tx + ty*ty + tz*tz) / (vfs*vfs);
    cp[i] = 1 - q;
  }
  return cp;
}

/* A uniform grid over the panels, so a mesh vertex can find the panel nearest
 * to it without scanning all of them.
 *
 * The first version built its cell keys by string concatenation --
 * `i + ',' + j + ',' + k` -- inside the lookup, and searched up to six rings
 * outward. Six rings is 2,556 cells, so a vertex that happened to sit in empty
 * space cost two and a half thousand string allocations, and painting the car
 * took eighteen seconds. Integer keys into a Map, flat typed arrays for the
 * panel lists, and three rings instead of six: the same answer, without the
 * garbage.
 */
export function panelIndex(body, cell){
  const N = body.N;
  const K = (kx, ky, kz) => (kx + 1024)*4194304 + (ky + 1024)*2048 + (kz + 1024);
  const map = new Map();
  const kOf = new Int32Array(N);
  for(let j = 0; j < N; j++){
    const key = K(Math.floor(body.c[j*3]/cell),
                  Math.floor(body.c[j*3+1]/cell),
                  Math.floor(body.c[j*3+2]/cell));
    let id = map.get(key);
    if(id === undefined){ id = map.size; map.set(key, id); }
    kOf[j] = id;
  }
  const M = map.size;
  const count = new Int32Array(M);
  for(let j = 0; j < N; j++) count[kOf[j]]++;
  const start = new Int32Array(M + 1);
  for(let i = 0; i < M; i++) start[i+1] = start[i] + count[i];
  const fill = start.slice(0, M);
  const order = new Int32Array(N);
  for(let j = 0; j < N; j++) order[fill[kOf[j]]++] = j;

  return {
    cell, map, start, order,
    /* Nearest panel to (x,y,z), searching outward a ring at a time. Three
       rings covers everything on or just inside a skin; beyond that the answer
       would be meaningless anyway, so it gives up and says so. */
    nearest(x, y, z){
      const kx = Math.floor(x/cell), ky = Math.floor(y/cell), kz = Math.floor(z/cell);
      for(let r = 0; r <= 3; r++){
        let best = -1, bd = Infinity;
        for(let dx = -r; dx <= r; dx++){
          const ex = Math.abs(dx) === r;
          for(let dy = -r; dy <= r; dy++){
            const ey = ex || Math.abs(dy) === r;
            for(let dz = -r; dz <= r; dz++){
              if(r > 0 && !ey && Math.abs(dz) !== r) continue;
              const id = map.get(K(kx+dx, ky+dy, kz+dz));
              if(id === undefined) continue;
              for(let e = start[id]; e < start[id+1]; e++){
                const j = order[e];
                const ax = x - body.c[j*3], ay = y - body.c[j*3+1], az = z - body.c[j*3+2];
                const d2 = ax*ax + ay*ay + az*az;
                if(d2 < bd){ bd = d2; best = j; }
              }
            }
          }
        }
        if(best >= 0) return best;
      }
      return -1;
    },
  };
}

/* Trace one streamline through the field.
 *
 * The step shrinks as the line approaches the body. It used to be a constant
 * fraction of the model -- 8.4 mm on a 440 mm aeroplane -- while the lines
 * released from the skin start a panel width off it, about 11 mm. One step of
 * the wrong direction put them through the surface, so every surface line was
 * a two-point stub or a curl, and the model came out with a mat of dark
 * scribble lying on it instead of flow following its shape. Near the skin the
 * field varies over the standoff distance, so that is what sets the step.
 */
/* Take the normal component out of a velocity that is close to the skin.
 *
 * Solid surfaces do not let air through, and a source-panel body enforces that
 * exactly where its collocation points are and NOWHERE ELSE. Between them the
 * normal velocity is whatever the discretisation left over, and it is enough
 * to drive a line into the surface: a streamline released over the canopy came
 * out as a dark curl lying on the model rather than as flow following it.
 *
 * So the boundary condition is applied to the line as well as to the panels.
 * Inside half a band the normal component is removed in full, and it fades to
 * nothing by the band's edge, so a line leaves tangentially, follows the shape
 * while it is close, and is free as soon as it is clear. Nothing is invented:
 * this is the condition the body is already meant to satisfy, applied where
 * the body does not satisfy it.
 *
 * The band comes from the PANEL, not the model. How far out a panel's field
 * can be trusted is set by how big the panel is -- the aeroplane's median
 * panel is 8 mm, so its field is unreliable within a few millimetres of the
 * skin, and a band derived from the model's 284 mm diagonal was 1.7 mm: five
 * times smaller than the panel it was protecting. `cap` keeps a coarsely
 * panelled body from claiming a band wider than the gaps it has to leave open,
 * like the air under a car's floor.
 */
function slide(v, probe, cap){
  const band = Math.min(cap, probe.size * 1.25);
  const d = probe.near;
  if(!(d < band)) return;
  const w = Math.min(1, 2 * (1 - d / band));
  const vn = v[0]*probe.nx + v[1]*probe.ny + v[2]*probe.nz;
  v[0] -= w * vn * probe.nx;
  v[1] -= w * vn * probe.ny;
  v[2] -= w * vn * probe.nz;
}


export function traceLine(vel, seed, opts){
  const {maxSteps = 260, ds = 0.05, xEnd = 1e9, bounds = null,
         body = null, dsMin = ds*0.12, skin = 0} = opts || {};
  const pts = [], spd = [];
  // how close this line ever came to the skin, for free: the inside test is
  // run every step anyway and already knows
  let near = Infinity;
  const p = [seed[0], seed[1], seed[2]];
  const v = [0,0,0], k1 = [0,0,0], mid = [0,0,0];
  const probe = body ? {near: Infinity} : null;
  const ports = body && body.ports && body.ports.length ? body.ports : null;
  for(let s = 0; s < maxSteps; s++){
    if(body && body.inside(p, probe)) break;
    if(probe && probe.near < near) near = probe.near;
    vel(p, v);
    const m = Math.hypot(v[0], v[1], v[2]);
    if(!isFinite(m) || m < 1e-6) break;
    pts.push(p[0], p[1], p[2]);
    spd.push(m);
    if(p[0] > xEnd) break;
    if(bounds && (p[1] < bounds[0] || p[1] > bounds[1]
               || p[2] < bounds[2] || p[2] > bounds[3])) break;
    const h = probe && isFinite(probe.near)
      ? Math.min(ds, Math.max(dsMin, probe.near * 0.55)) : ds;
    if(skin > 0 && probe) slide(v, probe, skin);
    const mt = Math.hypot(v[0], v[1], v[2]) || m;
    k1[0] = v[0]/mt; k1[1] = v[1]/mt; k1[2] = v[2]/mt;
    mid[0] = p[0] + k1[0]*h*0.5;
    mid[1] = p[1] + k1[1]*h*0.5;
    mid[2] = p[2] + k1[2]*h*0.5;
    vel(mid, v);
    if(skin > 0 && probe) slide(v, probe, skin);
    const m2 = Math.hypot(v[0], v[1], v[2]) || 1;
    p[0] += v[0]/m2*h; p[1] += v[1]/m2*h; p[2] += v[2]/m2*h;
    if(ports){
      // a sink mouth swallows the line: this is air entering an engine or a
      // fan, and the right thing to draw is a line that ends at the lip
      for(const q of ports){
        if(q.Q >= 0) continue;
        const dx = p[0] - q.p[0], dy = p[1] - q.p[1], dz = p[2] - q.p[2];
        if(dx*dx + dy*dy + dz*dz < q.r*q.r*4){
          pts.push(p[0], p[1], p[2]); spd.push(m2);
          return {pts, spd, near, captured: true};
        }
      }
    }
  }
  return {pts, spd, near};
}

/* Colour maps.
 *
 * The first version of this was a rainbow -- blue, cyan, green, yellow, red --
 * with a comment admitting it is not a perceptual map and using it anyway
 * because it looks like the reference screenshots. A rainbow invents hard
 * edges where the data is smooth (the cyan-to-green step) and hides real
 * gradients where it is not (green through yellow spans a huge range and
 * reads as one colour). It makes a picture that looks like CFD and cannot be
 * read as data.
 *
 * `speedColour` is Google's Turbo: still blue-to-red so it is instantly
 * legible to anyone who has seen a CFD plot, but monotonic in lightness, so a
 * step in colour is a step in value everywhere on the bar.
 *
 * `pressureColour` is a diverging map, because Cp has a meaningful zero.
 * Cp = 0 is freestream static pressure: the white middle. Blue is suction
 * (Cp < 0, which is where downforce comes from) and red is above-ambient
 * (Cp > 0, stagnation). A sequential map on a diverging quantity hides the
 * sign of the thing you are looking for.
 */

const TURBO = [
  [0.190, 0.072, 0.232], [0.253, 0.265, 0.630], [0.268, 0.451, 0.865],
  [0.220, 0.624, 0.973], [0.130, 0.775, 0.859], [0.145, 0.874, 0.669],
  [0.365, 0.945, 0.447], [0.639, 0.986, 0.254], [0.847, 0.949, 0.184],
  [0.973, 0.826, 0.199], [0.997, 0.648, 0.164], [0.960, 0.446, 0.096],
  [0.858, 0.269, 0.042], [0.706, 0.146, 0.019], [0.480, 0.016, 0.011],
];

function ramp(stops, t, out){
  const x = Math.max(0, Math.min(1, t)) * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(x));
  const f = x - i;
  const a = stops[i], b = stops[i + 1];
  out[0] = a[0] + (b[0] - a[0]) * f;
  out[1] = a[1] + (b[1] - a[1]) * f;
  out[2] = a[2] + (b[2] - a[2]) * f;
  return out;
}

export function speedColour(t, out){ return ramp(TURBO, t, out); }

const COOLWARM = [
  [0.109, 0.259, 0.702], [0.259, 0.451, 0.843], [0.459, 0.639, 0.933],
  [0.671, 0.784, 0.976], [0.867, 0.867, 0.867], [0.969, 0.745, 0.655],
  [0.937, 0.573, 0.459], [0.843, 0.376, 0.298], [0.706, 0.016, 0.149],
];

/* t in 0..1, where 0.5 is Cp = 0. */
export function pressureColour(t, out){ return ramp(COOLWARM, t, out); }

/* Normalise a Cp onto 0..1 with the map's neutral colour landing exactly on
 * Cp = 0, using a separate scale either side of it.
 *
 * Cp is bounded above by 1 (you cannot stagnate harder than stagnation) and
 * unbounded below, so its range is always lopsided -- on this car, -2.5 to 1.
 * Forcing the scale symmetric to keep white at zero throws away the whole red
 * half of the map; letting it run lo-to-hi linearly moves white off zero, so
 * the colour stops meaning "ambient". Two half-scales fixes both: white is
 * ambient, blue saturates at the suction limit, red saturates at stagnation.
 */
export function cpNorm(cp, lo, hi){
  if(cp < 0) return 0.5 * (1 - Math.min(1, cp/Math.min(lo, -1e-6)));
  return 0.5 + 0.5 * Math.min(1, cp/Math.max(hi, 1e-6));
}

/* Kept so an existing caller does not break; it is the speed map now. */
export function velocityColour(t, out){ return speedColour(t, out); }
