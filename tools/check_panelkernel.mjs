/* Are the panel influence coefficients the ones they claim to be?
 *
 * A panel method is four formulas and a linear solve. If one of the four has a
 * sign wrong the solve still converges, the streamlines still come out smooth,
 * and the answer is wrong -- there is no failure to notice. The published
 * forms are also easy to mistranscribe: they are written per edge with an
 * arctangent divided by the edge slope and by the out-of-plane distance, so
 * they carry special cases for a vertical edge and for a point in the panel's
 * own plane, which is where every collocation point sits.
 *
 * So nothing here is taken from a transcription. Each closed form is measured
 * against brute-force quadrature over the same panel -- the quad subdivided
 * into up to 160,000 point elements and summed -- which is a statement about
 * the mathematics rather than about someone's typing. Two errors were found
 * this way and neither would have shown up downstream:
 *
 *   * the doublet potential was negated;
 *   * the source potential's solid-angle term needs z.Omega, not |z|.Omega.
 *     A source sheet's potential is even in z, `sum` is even and Omega is odd,
 *     so only z.Omega is even. With |z| it was right above the panel and
 *     wrong below it -- -0.1885 against a true -0.0680.
 *
 * Run:  node tools/check_panelkernel.mjs
 */
import { solidAngle, doubletPotential, doubletVelocity,
         sourcePotential, sourceVelocity, panelFrame }
  from '../viewer/panelkernel.js';

const I4 = 1/(4*Math.PI);

/* The quad subdivided bilinearly into n*n point elements. */
function cells(q, n){
  const P = (u, v) => [0,1,2].map(k => q[0][k]*(1-u)*(1-v) + q[1][k]*u*(1-v)
                                     + q[2][k]*u*v + q[3][k]*(1-u)*v);
  const out = [];
  for(let i = 0; i < n; i++) for(let j = 0; j < n; j++){
    const a = P(i/n, j/n), b = P((i+1)/n, j/n),
          c = P((i+1)/n, (j+1)/n), d = P(i/n, (j+1)/n);
    const e1 = [c[0]-a[0], c[1]-a[1], c[2]-a[2]];
    const e2 = [d[0]-b[0], d[1]-b[1], d[2]-b[2]];
    const nx = e1[1]*e2[2]-e1[2]*e2[1], ny = e1[2]*e2[0]-e1[0]*e2[2],
          nz = e1[0]*e2[1]-e1[1]*e2[0];
    const m = Math.hypot(nx, ny, nz);
    out.push([(a[0]+b[0]+c[0]+d[0])/4, (a[1]+b[1]+c[1]+d[1])/4,
              (a[2]+b[2]+c[2]+d[2])/4, 0.5*m, nx/m, ny/m, nz/m]);
  }
  return out;
}
const srcPotQ = (p, C, s) => { let f = 0;
  for(const c of C) f += c[3]/Math.hypot(p[0]-c[0], p[1]-c[1], p[2]-c[2]);
  return -s*I4*f; };
const srcVelQ = (p, C, s) => { const o = [0,0,0];
  for(const c of C){ const dx = p[0]-c[0], dy = p[1]-c[1], dz = p[2]-c[2];
    const r = Math.hypot(dx, dy, dz), k = s*I4*c[3]/(r*r*r);
    o[0] += k*dx; o[1] += k*dy; o[2] += k*dz; }
  return o; };
const dblPotQ = (p, C, m) => { let f = 0;
  for(const c of C){ const dx = p[0]-c[0], dy = p[1]-c[1], dz = p[2]-c[2];
    const r = Math.hypot(dx, dy, dz);
    f += c[3]*(c[4]*dx + c[5]*dy + c[6]*dz)/(r*r*r); }
  return -m*I4*f; };
const dblVelQ = (p, C, m) => { const h = 1e-6, o = [0,0,0];
  for(let k = 0; k < 3; k++){ const a = p.slice(), b = p.slice();
    a[k] += h; b[k] -= h; o[k] = (dblPotQ(a, C, m) - dblPotQ(b, C, m))/(2*h); }
  return o; };

const rel  = (a, b, sc) => Math.abs(a-b)/Math.max(Math.abs(b), sc);
const relV = (a, b, sc) => Math.hypot(a[0]-b[0], a[1]-b[1], a[2]-b[2])
                         / Math.max(Math.hypot(b[0], b[1], b[2]), sc);

/* A planar quad has to be BUILT planar, not written down.
 *
 * Four coordinates typed out are four points in space, and four points in
 * space are not coplanar. The quad first used here as a "tilted flat panel"
 * was warped by 1.4 % of its own width, and it was that warp -- not the
 * formula -- that the sweep was reporting as a 1.5 % source-velocity error.
 * So a tilted panel is a flat one put through a rotation. */
function rotate(q, ax, ay, az){
  const R = (c, s, i, j) => (p) => { const a = p.slice();
    a[i] = p[i]*c - p[j]*s; a[j] = p[i]*s + p[j]*c; return a; };
  const rx = R(Math.cos(ax), Math.sin(ax), 1, 2);
  const ry = R(Math.cos(ay), Math.sin(ay), 2, 0);
  const rz = R(Math.cos(az), Math.sin(az), 0, 1);
  return q.map(p => rz(ry(rx(p))).map((v, k) => v + [0.4, 0.25, 0.6][k]));
}
const SKEW = [[-0.6,-0.4,0],[0.7,-0.5,0],[0.5,0.6,0],[-0.4,0.45,0]];
const FLAT = {
  'unit square': [[-0.5,-0.5,0],[0.5,-0.5,0],[0.5,0.5,0],[-0.5,0.5,0]],
  'skewed':      SKEW,
  'tilted':      rotate(SKEW, 0.37, -0.62, 1.1),
};
// Out of plane by a fraction of its own width. A panel taken off a real
// surface is warped by well under a per cent; both of these are worse.
const WARPED = {
  'warped 1.4%': [[0.1,0.2,0.3],[1.05,0.1,0.42],[1.2,1.0,0.25],[0.2,1.1,0.18]],
  'warped 3.5%': [[-0.5,-0.5,0.04],[0.5,-0.5,-0.03],[0.5,0.5,0.05],[-0.5,0.5,-0.02]],
};
const SIGMA = 1.7, MU = -0.9;

function sweep(panels){
  const w = {sp: 0, sv: 0, dp: 0, dv: 0}, notes = [];
  for(const [name, q] of Object.entries(panels)){
    const F = panelFrame(q), size = Math.sqrt(F.area);
    for(const mult of [0.08, 0.25, 0.7, 2.0, 6.0]){
      for(let a = 0; a < 7; a++){
        const th = a/7*Math.PI*2;
        const dir = [0,1,2].map(k => Math.cos(th)*F.t[k] + Math.sin(th)*F.n[k]);
        let p = [0,1,2].map(k => F.o[k] + dir[k]*size*mult + F.s[k]*size*0.13);
        /* Keep off the sheet. A point in the panel's plane and inside its
         * outline is ON the singularity: the closed form gives the one-sided
         * limit, the quadrature gives the mean of the two sides, and they
         * differ by sigma/2 by construction rather than by error. */
        const off = [0,1,2].reduce((s2, k) => s2 + (p[k]-F.o[k])*F.n[k], 0);
        if(Math.abs(off) < size*0.02) p = p.map((v, k) => v + F.n[k]*size*0.05);
        const C = cells(q, mult < 0.3 ? 400 : (mult < 1 ? 200 : 60));
        const sc = Math.abs(SIGMA)*size*0.02;
        const e = {
          sp: rel(sourcePotential(p, q, SIGMA), srcPotQ(p, C, SIGMA), sc),
          sv: relV(sourceVelocity(p, q, SIGMA, [0,0,0]), srcVelQ(p, C, SIGMA), sc/size),
          dp: rel(doubletPotential(p, q, MU), dblPotQ(p, C, MU), Math.abs(MU)*0.02),
          dv: relV(doubletVelocity(p, q, MU, [0,0,0]), dblVelQ(p, C, MU), Math.abs(MU)*0.02/size),
        };
        for(const k of ['sp','sv','dp','dv']) w[k] = Math.max(w[k], e[k]);
        if(Math.max(...Object.values(e)) > 0.02)
          notes.push(`${name} at ${mult} widths, direction ${a}: `
            + Object.entries(e).map(([k, v]) => `${k} ${(100*v).toFixed(2)}%`).join('  '));
      }
    }
  }
  return {w, notes};
}

const flat = sweep(FLAT);
const warp = sweep(WARPED);

console.log('\nPANEL INFLUENCE COEFFICIENTS vs BRUTE-FORCE QUADRATURE');
console.log('  flat panels, worst relative error over 105 field points');
console.log(`    source potential   ${(100*flat.w.sp).toFixed(4)} %`);
console.log(`    source velocity    ${(100*flat.w.sv).toFixed(4)} %`);
console.log(`    doublet potential  ${(100*flat.w.dp).toFixed(4)} %`);
console.log(`    doublet velocity   ${(100*flat.w.dv).toFixed(4)} %`);
for(const n of flat.notes.slice(0, 8)) console.log('     x ' + n);

/* The warped quad is not a failure, it is the flat-panel approximation.
 *
 * The doublet element is a ring vortex on the true corners, so it follows a
 * warped quad exactly. The source element is FLAT by construction -- its edge
 * terms are line integrals in the panel plane -- so against a quad warped by
 * 5 % of its width it differs from the warped surface by a few per cent in
 * the near field. That is the modelling error of using flat panels, and it is
 * measured here rather than assumed away. */
console.log('\n  quads warped 1.4 % and 3.5 % of their width out of plane,');
console.log('  which is more warp than a panel off a real surface has:');
console.log(`    flat source panel differs by  ${(100*warp.w.sp).toFixed(2)} % in potential,`
          + ` ${(100*warp.w.sv).toFixed(2)} % in velocity`);
console.log(`    ring-vortex doublet follows the warp to `
          + `${(100*Math.max(warp.w.dp, warp.w.dv)).toFixed(4)} %`);

const q = FLAT['unit square'], F = panelFrame(q), eps = 1e-5;
const up = [0,1,2].map(k => F.o[k] + F.n[k]*eps);
const dn = [0,1,2].map(k => F.o[k] - F.n[k]*eps);
const jumpPhi = doubletPotential(up, q, MU) - doubletPotential(dn, q, MU);
const vu = sourceVelocity(up, q, SIGMA, [0,0,0]);
const vd = sourceVelocity(dn, q, SIGMA, [0,0,0]);
const jumpVn = [0,1,2].reduce((s, k) => s + (vu[k]-vd[k])*F.n[k], 0);
console.log('\n  the jumps across the sheet, which are what the elements are for');
console.log(`    doublet potential jumps by  ${jumpPhi.toFixed(6)}   (-mu = ${(-MU).toFixed(6)})`);
console.log(`    source normal velocity by   ${jumpVn.toFixed(6)}   (sigma = ${SIGMA.toFixed(6)})`);

let worstG = 0;
for(const [, qq] of Object.entries({...FLAT, ...WARPED})){
  const Ff = panelFrame(qq), sz = Math.sqrt(Ff.area);
  for(const mult of [0.4, 1.5, 4.0]){
    const p = [0,1,2].map(k => Ff.o[k] + Ff.n[k]*sz*mult + Ff.t[k]*sz*0.3);
    const h = sz*1e-5, g = [0,0,0], gd = [0,0,0];
    for(let k = 0; k < 3; k++){
      const a = p.slice(), b = p.slice(); a[k] += h; b[k] -= h;
      g[k]  = (sourcePotential(a, qq, SIGMA) - sourcePotential(b, qq, SIGMA))/(2*h);
      gd[k] = (doubletPotential(a, qq, MU)  - doubletPotential(b, qq, MU))/(2*h);
    }
    worstG = Math.max(worstG,
      relV(sourceVelocity(p, qq, SIGMA, [0,0,0]), g, 1e-6),
      relV(doubletVelocity(p, qq, MU, [0,0,0]), gd, 1e-6));
  }
}
console.log(`\n  each element's velocity is the gradient of its own potential`);
console.log(`  to ${(100*worstG).toFixed(4)} % -- so the solve and the picture`);
console.log('  are the same flow field, not two descriptions of it');

const ok = flat.w.sp < 0.005 && flat.w.sv < 0.005
        && flat.w.dp < 0.005 && flat.w.dv < 0.005
        // the jumps are read a micron off the sheet rather than on it, so
        // they carry that offset's own error and not only the formula's
        && Math.abs(jumpPhi + MU) < 1e-4 && Math.abs(jumpVn - SIGMA) < 1e-4
        && worstG < 1e-4 && warp.w.sv < 0.06;
console.log('\n' + '='.repeat(62));
console.log(ok ? 'PASS  the influence coefficients are the ones they claim to be'
               : 'FAIL  a panel influence coefficient does not match quadrature');
process.exit(ok ? 0 : 1);
