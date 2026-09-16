/* Influence of one constant-strength quadrilateral panel.
 *
 * This is the arithmetic a low-order panel method is built from: given a flat
 * quadrilateral carrying a uniform source sheet of strength sigma, or a
 * uniform normal-doublet sheet of strength mu, what potential and what
 * velocity does it induce at a point?
 *
 * Both elements reduce to the same two pieces:
 *
 *   * the SOLID ANGLE the panel subtends at the point. A normal-doublet sheet
 *     of strength mu has potential -mu.Omega/4pi and nothing else, and the
 *     out-of-plane velocity of a source sheet is sigma.Omega/4pi -- which is
 *     where the +/- sigma/2 jump across the sheet comes from, because Omega
 *     goes to -/+ 2pi as you cross it.
 *   * a LINE INTEGRAL along the four edges, which gives the in-plane velocity
 *     of the source sheet and the logarithmic part of its potential.
 *
 * and the doublet's VELOCITY is that of a vortex ring of circulation mu run
 * round the panel's edges, which is the same field and is far better behaved
 * to evaluate.
 *
 * The solid angle is Van Oosterom and Strackee's formula on the two triangles
 * the quadrilateral splits into, rather than the arctangent-per-edge form in
 * the textbooks. The textbook form divides by the in-plane slope of each edge
 * and by the out-of-plane distance, so it has to be special-cased for an edge
 * parallel to the local y axis and again for a point in the panel's own
 * plane -- which is exactly where every collocation point sits. Van Oosterom
 * has no such cases.
 *
 * Nothing here is taken on trust: tools/check_panelkernel.mjs evaluates every
 * one of these against brute-force quadrature over the same panel, which is a
 * statement about the maths rather than about a transcription of it.
 */

const INV4PI = 1 / (4 * Math.PI);

/* Signed solid angle subtended at p by the triangle (a, b, c), positive when
 * the triangle is seen anticlockwise. Van Oosterom & Strackee (1983):
 *
 *     tan(Omega/2) = (a x b . c) / (|a||b||c| + (a.b)|c| + (a.c)|b| + (b.c)|a|)
 *
 * with a, b, c measured from p. atan2 of the two halves keeps it continuous
 * through the full +/- 2pi range, which a plain atan does not.
 */
function triSolidAngle(px, py, pz, a, b, c){
  const ax = a[0]-px, ay = a[1]-py, az = a[2]-pz;
  const bx = b[0]-px, by = b[1]-py, bz = b[2]-pz;
  const cx = c[0]-px, cy = c[1]-py, cz = c[2]-pz;
  const la = Math.hypot(ax, ay, az);
  const lb = Math.hypot(bx, by, bz);
  const lc = Math.hypot(cx, cy, cz);
  if(la < 1e-14 || lb < 1e-14 || lc < 1e-14) return 0;
  const num = ax*(by*cz - bz*cy) + ay*(bz*cx - bx*cz) + az*(bx*cy - by*cx);
  const den = la*lb*lc
            + (ax*bx + ay*by + az*bz)*lc
            + (ax*cx + ay*cy + az*cz)*lb
            + (bx*cx + by*cy + bz*cz)*la;
  return 2 * Math.atan2(num, den);
}

/* Solid angle of a quadrilateral (q is 4 points). Split on the 0-2 diagonal.
 * A quadrilateral panel from a curved surface is not exactly planar, and
 * splitting it this way is consistent about which surface is meant. */
export function solidAngle(p, q){
  return triSolidAngle(p[0], p[1], p[2], q[0], q[1], q[2])
       + triSolidAngle(p[0], p[1], p[2], q[0], q[2], q[3]);
}

/* The same, for a point (x, y, z) and corners already in the panel's own
 * frame and flattened onto z = 0. The source element is a FLAT panel -- its
 * edge integrals are line integrals in the panel plane -- so its solid angle
 * has to be the flat one too, or the velocity stops being the gradient of the
 * potential. On a quad warped by 5 % of its width that inconsistency was
 * worth 1.5 %. */
function flatSolidAngle(x, y, z, loc){
  const a = [loc[0][0], loc[0][1], 0], b = [loc[1][0], loc[1][1], 0];
  const c = [loc[2][0], loc[2][1], 0], d = [loc[3][0], loc[3][1], 0];
  return triSolidAngle(x, y, z, a, b, c) + triSolidAngle(x, y, z, a, c, d);
}

/* Velocity of a straight vortex filament from a to b, circulation gamma.
 * `core` keeps it finite on the filament itself. */
function filament(p, a, b, gamma, core, out){
  const r1x = p[0]-a[0], r1y = p[1]-a[1], r1z = p[2]-a[2];
  const r2x = p[0]-b[0], r2y = p[1]-b[1], r2z = p[2]-b[2];
  const cx = r1y*r2z - r1z*r2y;
  const cy = r1z*r2x - r1x*r2z;
  const cz = r1x*r2y - r1y*r2x;
  const c2 = cx*cx + cy*cy + cz*cz;
  if(c2 < 1e-24) return out;
  const l1 = Math.hypot(r1x, r1y, r1z);
  const l2 = Math.hypot(r2x, r2y, r2z);
  if(l1 < 1e-14 || l2 < 1e-14) return out;
  const r0x = b[0]-a[0], r0y = b[1]-a[1], r0z = b[2]-a[2];
  const k = gamma * INV4PI
          * ((r0x*r1x + r0y*r1y + r0z*r1z)/l1 - (r0x*r2x + r0y*r2y + r0z*r2z)/l2)
          / Math.max(c2, core*core*(r0x*r0x + r0y*r0y + r0z*r0z));
  out[0] += k*cx; out[1] += k*cy; out[2] += k*cz;
  return out;
}

/* Velocity of a constant-strength normal-doublet quadrilateral, as the ring
 * vortex of circulation mu that is its exact equivalent. */
export function doubletVelocity(p, q, mu, out, core = 0){
  filament(p, q[0], q[1], mu, core, out);
  filament(p, q[1], q[2], mu, core, out);
  filament(p, q[2], q[3], mu, core, out);
  filament(p, q[3], q[0], mu, core, out);
  return out;
}

/* Potential of a constant-strength normal-doublet quadrilateral.
 *
 * The doublet axis is the panel normal given by the corner order, so the sign
 * follows the winding of `q` and not a separate convention to get wrong. */
export function doubletPotential(p, q, mu){
  return mu * INV4PI * solidAngle(p, q);
}

/* Panel frame: origin at the centroid, z along the normal from the corner
 * order, x along the first edge. Returned as three unit vectors and the
 * corners expressed in it. */
export function panelFrame(q){
  const ox = (q[0][0]+q[1][0]+q[2][0]+q[3][0])/4;
  const oy = (q[0][1]+q[1][1]+q[2][1]+q[3][1])/4;
  const oz = (q[0][2]+q[1][2]+q[2][2]+q[3][2])/4;
  // normal from the diagonals, which is the right one for a warped quad
  const d1 = [q[2][0]-q[0][0], q[2][1]-q[0][1], q[2][2]-q[0][2]];
  const d2 = [q[3][0]-q[1][0], q[3][1]-q[1][1], q[3][2]-q[1][2]];
  let nx = d1[1]*d2[2] - d1[2]*d2[1];
  let ny = d1[2]*d2[0] - d1[0]*d2[2];
  let nz = d1[0]*d2[1] - d1[1]*d2[0];
  const area = 0.5*Math.hypot(nx, ny, nz);
  const ln = Math.hypot(nx, ny, nz) || 1;
  nx /= ln; ny /= ln; nz /= ln;
  let tx = q[1][0]-q[0][0], ty = q[1][1]-q[0][1], tz = q[1][2]-q[0][2];
  const dot = tx*nx + ty*ny + tz*nz;
  tx -= dot*nx; ty -= dot*ny; tz -= dot*nz;
  const lt = Math.hypot(tx, ty, tz) || 1;
  tx /= lt; ty /= lt; tz /= lt;
  const sx = ny*tz - nz*ty, sy = nz*tx - nx*tz, sz = nx*ty - ny*tx;
  const loc = [];
  for(let k = 0; k < 4; k++){
    const rx = q[k][0]-ox, ry = q[k][1]-oy, rz = q[k][2]-oz;
    loc.push([rx*tx + ry*ty + rz*tz, rx*sx + ry*sy + rz*sz, rx*nx + ry*ny + rz*nz]);
  }
  return {o: [ox, oy, oz], t: [tx, ty, tz], s: [sx, sy, sz], n: [nx, ny, nz],
          area, loc};
}

/* The edge line integral both source formulas need.
 *
 *   L_k = ln( (r_k + r_k+1 + d_k) / (r_k + r_k+1 - d_k) )
 *
 * evaluated in the panel's own plane. Degenerate when the point is on the
 * edge's own line beyond its ends, where the logarithm's argument goes to
 * zero; the guard is the only special case in the file. */
function edgeLog(x, y, z, x1, y1, x2, y2){
  const dx = x2-x1, dy = y2-y1;
  const d = Math.hypot(dx, dy);
  if(d < 1e-14) return {d: 0, L: 0};
  const r1 = Math.hypot(x-x1, y-y1, z);
  const r2 = Math.hypot(x-x2, y-y2, z);
  const lo = r1 + r2 - d;
  if(lo < 1e-14) return {d, L: 0};
  return {d, L: Math.log((r1 + r2 + d) / lo)};
}

/* Velocity of a constant-strength source quadrilateral.
 *
 * In-plane from the edge integrals, out-of-plane from the solid angle. The
 * jump of sigma/2 across the sheet is carried entirely by the solid angle
 * swinging through 4pi, so it comes out right without a side test. */
export function sourceVelocity(p, q, sigma, out, frame = null){
  const F = frame || panelFrame(q);
  const rx = p[0]-F.o[0], ry = p[1]-F.o[1], rz = p[2]-F.o[2];
  const x = rx*F.t[0] + ry*F.t[1] + rz*F.t[2];
  const y = rx*F.s[0] + ry*F.s[1] + rz*F.s[2];
  const z = rx*F.n[0] + ry*F.n[1] + rz*F.n[2];
  let u = 0, v = 0;
  for(let k = 0; k < 4; k++){
    const a = F.loc[k], b = F.loc[(k+1) & 3];
    const e = edgeLog(x, y, z, a[0], a[1], b[0], b[1]);
    if(!e.d) continue;
    u += (b[1]-a[1]) / e.d * e.L;
    v += (a[0]-b[0]) / e.d * e.L;
  }
  u *= sigma * INV4PI;
  v *= sigma * INV4PI;
  const w = -sigma * INV4PI * flatSolidAngle(x, y, z, F.loc);
  out[0] += u*F.t[0] + v*F.s[0] + w*F.n[0];
  out[1] += u*F.t[1] + v*F.s[1] + w*F.n[1];
  out[2] += u*F.t[2] + v*F.s[2] + w*F.n[2];
  return out;
}

/* Potential of a constant-strength source quadrilateral. */
export function sourcePotential(p, q, sigma, frame = null){
  const F = frame || panelFrame(q);
  const rx = p[0]-F.o[0], ry = p[1]-F.o[1], rz = p[2]-F.o[2];
  const x = rx*F.t[0] + ry*F.t[1] + rz*F.t[2];
  const y = rx*F.s[0] + ry*F.s[1] + rz*F.s[2];
  const z = rx*F.n[0] + ry*F.n[1] + rz*F.n[2];
  let sum = 0;
  for(let k = 0; k < 4; k++){
    const a = F.loc[k], b = F.loc[(k+1) & 3];
    const e = edgeLog(x, y, z, a[0], a[1], b[0], b[1]);
    if(!e.d) continue;
    sum += ((x-a[0])*(b[1]-a[1]) - (y-a[1])*(b[0]-a[0])) / e.d * e.L;
  }
  /* The solid-angle term carries z, not |z|.
   *
   * A source sheet's potential is even in z -- the sheet does not know which
   * side you are on -- and `sum` is even while Omega is odd, so only z.Omega
   * is even. Written with |z|, as the textbook form is when its arctangents
   * are collapsed into a solid angle, it is right above the panel and wrong
   * below it: at (0, 0, -1.1) off a unit square it gave -0.1885 against a
   * true -0.0680. */
  return sigma * INV4PI * (sum - z * flatSolidAngle(x, y, z, F.loc));
}
