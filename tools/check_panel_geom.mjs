/* Is this aeroplane something the panel method can actually solve?
 *
 * The solver's own validation (check_panelkernel, check_panelflow) says the
 * mathematics is right on shapes with known answers. This says the AEROPLANE
 * handed to it is a shape it can answer about, and that the answer behaves.
 *
 * The failures it is here for all happened:
 *
 *   * a mirrored surface wound inside out, so its normals pointed into it and
 *     the boundary condition was applied with its sign reversed;
 *   * components grazing each other -- the stabilator's panels a tenth of
 *     their own width off the tail boom -- which makes the influence matrix
 *     near-singular exactly there: minimum Cp -158, lift slope three times
 *     what it should be, and nothing that looks like an error;
 *   * the surface gradient taken across a trailing edge, where the potential
 *     is discontinuous by the circulation: Cp -222,000;
 *   * a wing capped at the side of the fuselage, which sheds a root vortex a
 *     real wing-body does not have and cost 40 % of the lift slope.
 *
 * None of those announce themselves. Each one leaves a solve that converges
 * and a picture that looks like flow.
 *
 *     node tools/check_panel_geom.mjs
 */
import { PanelFlow } from '../viewer/panelflow.js';
import { solidAngle } from '../viewer/panelkernel.js';
import { PanelTunnel } from '../viewer/paneltunnel.js';
import { traceLine } from '../viewer/flowfield.js';
import { readFileSync } from 'node:fs';

const data = JSON.parse(readFileSync(new URL('../viewer/parts.json', import.meta.url)));
const cfg = data.tunnel, geom = data.panel_body;
if(!geom){
  console.log('FAIL  viewer/parts.json has no panel_body');
  process.exit(1);
}
geom.quads = Float64Array.from(geom.quads);
const pf = new PanelFlow(Object.assign({}, geom, {ground: !!cfg.ground}));
const quadOf = (j) => pf.els[pf.elOf[j]].q;
let ok = true;
const fail = (why) => { ok = false; console.log('   x  ' + why); };

console.log(`\nPANEL SURFACE  (${pf.N} panels, ${pf.te.length} trailing edges)`);
for(const p of geom.parts)
  console.log(`   ${p.name.padEnd(14)} ${String(p.count).padStart(4)}`);

/* ---- closed, and the right way out.
 *
 * Not per component: the components are TRIMMED into each other, so the wing
 * has no root and the fin has no bottom and neither is closed on its own. The
 * union is, to within the seam a trim leaves. Two tests: the areas' normals
 * cancel, which any closed surface's do, and a point deep inside the body
 * sees the whole surface subtending 4.pi, which only an outward-wound closed
 * one does. */
let sx = 0, sy = 0, sz = 0, A = 0;
for(let i = 0; i < pf.N; i++){
  sx += pf.area[i]*pf.nrm[i*3]; sy += pf.area[i]*pf.nrm[i*3+1];
  sz += pf.area[i]*pf.nrm[i*3+2]; A += pf.area[i];
}
const closure = Math.hypot(sx, sy, sz)/A;
/* The probe point has to be somewhere the body actually IS.
 *
 * The centre of gravity is a mass property, not a place: at y = 0, z = 0 it
 * lands inside the wing's centre section, which is the one part of the wing
 * that gets trimmed away. Probe from the middle of the fuselage's own panels
 * instead, which is inside the fuselage by construction. */
let om = 0, px = 0, py = 0, pz = 0, pa = 0;
for(const p of geom.parts){
  if(!/^(fuselage)$/.test(p.name)) continue;
  for(let i = p.start; i < p.start + p.count; i++){
    px += pf.c[i*3]*pf.area[i]; py += pf.c[i*3+1]*pf.area[i];
    pz += pf.c[i*3+2]*pf.area[i]; pa += pf.area[i];
  }
}
const deep = [px/pa, py/pa, pz/pa];
for(let i = 0; i < pf.N; i++) om += solidAngle(deep, quadOf(i));
console.log(`\n   sum of area times normal   ${closure.toExponential(2)} of the wetted area`);
console.log(`   solid angle inside the body   ${(om/Math.PI).toFixed(3)} pi   (4 = closed and outward)`);
if(closure > 0.02) fail(`the surface does not close: ${closure.toExponential(2)}`);
if(Math.abs(om/Math.PI - 4) > 0.25)
  fail(`the surface is open or inside out: ${(om/Math.PI).toFixed(3)} pi`);

/* ---- components keep clear of each other */
const bodyOf = new Array(pf.N).fill('?');
for(const p of geom.parts)
  for(let i = p.start; i < p.start + p.count; i++)
    bodyOf[i] = p.name.replace(/_(root|tip)$/, '').replace(/^(nose|tail)_cap$/, 'fuselage');
let worst = Infinity, pair = '';
for(let i = 0; i < pf.N; i++) for(let j = i+1; j < pf.N; j++){
  if(bodyOf[i] === bodyOf[j]) continue;
  const d = Math.hypot(pf.c[i*3]-pf.c[j*3], pf.c[i*3+1]-pf.c[j*3+1],
                       pf.c[i*3+2]-pf.c[j*3+2]);
  const s = 0.5*(pf.size[i] + pf.size[j]);
  if(d/s < worst){ worst = d/s; pair = `${bodyOf[i]} / ${bodyOf[j]}`; }
}
console.log(`   closest two panels of different components   ${worst.toFixed(2)} panel widths  (${pair})`);
/* 0.3, which is what the trim guarantees.
 *
 * Measured rather than chosen: at 0.12 panel widths the stabilator grazing
 * the tail boom made the influence matrix near-singular and put the minimum
 * Cp at -158 with the lift slope three times what it should be. Trimming to
 * 0.35 gives a closed surface, a sane matrix, and LESS of the skin reading an
 * impossible pressure than trimming to 0.75 did -- because 0.75 cut a seam
 * six per cent of the wetted area wide and the body stopped being closed. */
if(worst < 0.3) fail(`${pair} are ${worst.toFixed(2)} panel widths apart -- too close to panel against`);

/* ---- the trailing edges are edges */
let teBad = 0;
for(const [u, l] of pf.te){
  const a = quadOf(u), b = quadOf(l);
  const tol = Math.pow(Math.min(pf.size[u], pf.size[l])*0.1, 2);
  let shared = false;
  for(let k = 0; k < 4 && !shared; k++) for(let m = 0; m < 4; m++){
    const p1 = a[k], p2 = a[(k+1)&3], q1 = b[m], q2 = b[(m+1)&3];
    const d = (x, y) => (x[0]-y[0])**2 + (x[1]-y[1])**2 + (x[2]-y[2])**2;
    if((d(p1,q1) < tol && d(p2,q2) < tol) || (d(p1,q2) < tol && d(p2,q1) < tol)){
      shared = true; break;
    }
  }
  if(!shared) teBad++;
}
console.log(`   trailing-edge pairs that really share an edge   ${pf.te.length - teBad}/${pf.te.length}`);
if(teBad) fail(`${teBad} trailing-edge pairs do not share an edge, so no wake leaves them`);
console.log(`   wake strips shed   ${pf.wake.length}`);
if(pf.wake.length !== pf.te.length) fail('a trailing edge shed no wake');

/* ---- the solution behaves */
const T = new PanelTunnel(Object.assign({}, cfg, {_parts: geom.parts}), pf);
const base = {v: cfg.v_default, beta: 0, controls: {}};
console.log('\nSOLUTION');
console.log('   alpha     CL      Cp min    Cp max   |mu| max');
let lastCL = -1e9, cpWorst = 0;
for(const alpha of [0, 4, 8, 12]){
  const r = T.solve({...base, alpha});
  let lo = 1e9, hi = -1e9, mm = 0;
  for(let i = 0; i < pf.N; i++){
    if(pf.cp[i] < lo) lo = pf.cp[i];
    if(pf.cp[i] > hi) hi = pf.cp[i];
    mm = Math.max(mm, Math.abs(pf.mu[i]));
  }
  cpWorst = Math.min(cpWorst, lo);
  console.log(`   ${String(alpha).padStart(5)}  ${r.CL.toFixed(4).padStart(7)}`
    + `  ${lo.toFixed(2).padStart(8)}  ${hi.toFixed(2).padStart(7)}  ${mm.toFixed(3).padStart(9)}`);
  if(r.CL <= lastCL) fail(`CL did not rise between ${alpha-4} and ${alpha} degrees`);
  lastCL = r.CL;
  if(hi > 1.02) fail(`Cp ${hi.toFixed(2)} above stagnation, which is not possible`);
}
/* By AREA, not by the worst panel.
 *
 * A flat wingtip cap meets the trailing edge in a sharp corner, and a sharp
 * corner in potential flow has an unbounded velocity: the flow turns from the
 * lower surface to the upper around zero radius. It is where the tip vortex
 * comes from and it is genuinely singular, so the Cp on the panel nearest it
 * is set by how small that panel is and by nothing else -- there is no value
 * to gate on. What can be gated is how MUCH of the aeroplane is doing it. A
 * few square millimetres of tip corner is the corner; a per cent of the
 * wetted area is a panelling fault.
 */
let bad = 0, wet = 0;
for(let i = 0; i < pf.N; i++){
  wet += pf.area[i];
  if(pf.cp[i] < -8) bad += pf.area[i];
}
console.log(`   surface below Cp -8   ${(100*bad/wet).toFixed(3)} % of the wetted area`);
if(bad/wet > 0.004)
  fail(`${(100*bad/wet).toFixed(2)} % of the surface is below Cp -8 -- that is panelling, not a suction peak`);

/* The two ways of getting the lift have to agree.
 *
 * One integrates the pressure over the skin; the other is Kutta-Joukowski on
 * the circulation the wake carries. They are different readings of the same
 * solution and they must land in the same place -- the wake's is a little
 * lower because the fuselage sheds none and so appears only in the pressures.
 * They came out with OPPOSITE SIGNS once: the doublet jump and the
 * circulation are signed the other way round from each other, and the span
 * load plot had every station of the wing pulling the aeroplane down while
 * the readout beside it said it was flying.
 */
{
  const r = T.solve({...base, alpha: 8});
  const sum = r.strips.reduce((a, st) => a + st.dL, 0);
  console.log(`\n   lift from the pressures      ${r.lift_N.toFixed(3)} N`);
  console.log(`   lift from the wake           ${sum.toFixed(3)} N`
            + `   (${(100*sum/r.lift_N).toFixed(0)} % -- the fuselage sheds no wake)`);
  if(!(sum/r.lift_N > 0.55 && sum/r.lift_N < 1.05))
    fail(`the wake says ${sum.toFixed(2)} N and the pressures say ${r.lift_N.toFixed(2)} N`);
}

console.log('\n   control      -max        0        +max');
for(const c of cfg.controls){
  /* A rudder moves the aeroplane sideways, so asking whether it changed the
   * LIFT is asking the wrong question -- it answered "by 0.0001", which read
   * as a broken hinge. Each control is measured on the force it is for. */
  const vertical = /rudder/.test(c.id);
  const get = (v) => { const r = T.solve({...base, alpha: 6, controls: {[c.id]: v}});
                       return vertical ? r.CY : r.CL; };
  const lo = get(c.min), md = get(0), hi = get(c.max);
  console.log(`   ${c.id.padEnd(12)} ${lo.toFixed(4).padStart(8)}  ${md.toFixed(4).padStart(8)}`
            + `  ${hi.toFixed(4).padStart(8)}   ${vertical ? 'side force' : 'lift'}`);
  if(!(lo < md && md < hi) && !(lo > md && md > hi))
    fail(`${c.id} does not change ${vertical ? 'side force' : 'lift'} monotonically`
       + ' -- its hinge axis or its panels are wrong');
}

/* ---- and the field is one a streamline can be drawn in */
const a = (cfg.alpha_default || 4)*Math.PI/180;
const V = cfg.v_default;
pf.solve([V*Math.cos(a), 0, V*Math.sin(a)]);
const b = pf.box, span = b[1]-b[0];
const diag = Math.hypot(b[1]-b[0], b[3]-b[2], b[5]-b[4]);
pf.nearStop = diag*0.006;
const vel = (p, out) => pf.velocity(p, out);
const opts = {maxSteps: 900, ds: span/58, xEnd: b[1]+span*1.15, body: pf,
              skin: diag*0.03,
              bounds: [-diag*0.95, diag*0.95, -diag*0.95, diag*0.95]};
let kinks = 0, tot = 0, died = 0, vmax = 0, n = 0;
const pr = {near: Infinity};
for(let j = 0; j < 20; j++) for(let k = 0; k < 14; k++){
  const y = b[2] - 0.15*(b[3]-b[2]) + 1.3*(b[3]-b[2])*j/19;
  const z = b[4] - 0.6*(b[5]-b[4]) + 2.2*(b[5]-b[4])*k/13;
  const x0 = b[0] - 0.45*span;
  const L = traceLine(vel, [x0, y, z - (0.5*(b[0]+b[1]) - x0)*Math.tan(a)], opts);
  n++;
  const m = L.spd.length; tot += m;
  for(const v of L.spd) vmax = Math.max(vmax, v/V);
  if(m > 2 && !L.captured){
    const e = [L.pts[(m-1)*3], L.pts[(m-1)*3+1], L.pts[(m-1)*3+2]];
    pf.inside(e, pr);
    if(pr.near < diag*0.02) died++;
  }
  for(let i = 2; i < m; i++){
    const ax = L.pts[i*3]-L.pts[(i-1)*3], ay = L.pts[i*3+1]-L.pts[(i-1)*3+1],
          az = L.pts[i*3+2]-L.pts[(i-1)*3+2];
    const bx = L.pts[(i-1)*3]-L.pts[(i-2)*3], by = L.pts[(i-1)*3+1]-L.pts[(i-2)*3+1],
          bz = L.pts[(i-1)*3+2]-L.pts[(i-2)*3+2];
    const la = Math.hypot(ax,ay,az), lb = Math.hypot(bx,by,bz);
    if(la < 1e-9 || lb < 1e-9) continue;
    if((ax*bx+ay*by+az*bz)/(la*lb) < Math.cos(25*Math.PI/180)) kinks++;
  }
}
console.log('\nSTREAMLINES');
console.log(`   ${n} lines, ${tot} points`);
console.log(`   turning more than 25 degrees in one step   ${(100*kinks/tot).toFixed(2)} %`);
console.log(`   ending on the skin                        ${died}/${n}`);
console.log(`   fastest point in the field                ${vmax.toFixed(2)} x freestream`);
if(kinks/tot > 0.005) fail(`${(100*kinks/tot).toFixed(1)} % of points kink -- the field is not smooth`);
if(died > n*0.12) fail(`${died} of ${n} lines end on the skin instead of going round it`);
if(vmax > 2.0) fail(`${vmax.toFixed(1)} x freestream somewhere -- a potential flow round this does not do that`);

console.log('\n' + '='.repeat(66));
console.log(ok ? 'PASS  the aeroplane is a surface the panel method can answer about'
               : 'FAIL  the panelled aeroplane is not solvable as it stands');
process.exit(ok ? 0 : 1);
