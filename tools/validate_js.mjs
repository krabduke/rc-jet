/* Check the browser solver against the Python one.
 *
 * Both solve the same lattice from the same spec, so they must agree. They
 * will not agree exactly: the browser runs a coarser lattice on purpose, and
 * a VLM's answer depends on panel count. The point of this check is that the
 * disagreement is small and that it shrinks as the browser lattice is
 * refined -- if it does not, the two are not the same method.
 */
import { Tunnel, flapTau } from '../viewer/windtunnel.js';
import { readFileSync } from 'node:fs';

const cfg = JSON.parse(readFileSync(new URL('./tunnel.json', import.meta.url)));
const controls = {};
for(const c of cfg.controls) controls[c.id] = c.value;

console.log('\nBROWSER SOLVER CHECK');
console.log(`  s_ref ${cfg.s_ref.toFixed(5)} m2   c_ref ${cfg.c_ref.toFixed(4)} m`
          + `   b_ref ${cfg.b_ref.toFixed(3)} m`);
console.log(`  flap tau at 26% chord: ${flapTau(0.26).toFixed(3)}`
          + '  (thin-aerofoil flap effectiveness)');

const t = new Tunnel(cfg);
const t0 = performance.now();
const s = t.solve({alpha: 4, v: 22, controls});
const dt = performance.now() - t0;
console.log(`\n  panels ${s.panels}   solve ${dt.toFixed(1)} ms`);
console.log(`  CL at 4 deg      ${s.CL.toFixed(4)}`);
console.log(`  CDi              ${s.CDi.toFixed(5)}`);
console.log(`  lift slope       ${t.liftSlope({v:22, controls}).toFixed(3)} /rad`);
const np = t.neutralPoint({v:22, controls});
console.log(`  neutral point    ${(np*100).toFixed(1)} % MAC`);
console.log(`  CG               ${(cfg.cg_frac*100).toFixed(1)} % MAC`);
console.log(`  static margin    ${((np - cfg.cg_frac)*100).toFixed(1)} % MAC`);

// The browser must reproduce the validated Python solver. These are the
// numbers aero/analyse.py prints for the same aircraft.
const REF = {slope: 3.533, np: 0.376};
const slope = t.liftSlope({v:22, controls:{...controls, flaperon:0, stabilator:0}});
const npc = t.neutralPoint({v:22, controls:{...controls, flaperon:0, stabilator:0}});
const dS = Math.abs(slope/REF.slope - 1)*100;
const dN = Math.abs(npc - REF.np)*100;
let ok = dS < 4.0 && dN < 2.5;
console.log('\n  AGAINST THE PYTHON SOLVER (controls neutral)');
console.log(`    lift slope    ${slope.toFixed(3)} vs ${REF.slope}  (${dS.toFixed(1)} % , limit 4 %)`);
console.log(`    neutral point ${(npc*100).toFixed(1)} % vs ${(REF.np*100).toFixed(1)} %`
          + `  (${dN.toFixed(1)} pts, limit 2.5)`);

console.log('\n  lattice convergence (lift slope, per rad):');
for(const [ns, nc] of [[6,3],[10,4],[16,6],[22,8]]){
  const c2 = JSON.parse(JSON.stringify(cfg));
  for(const su of c2.surfaces){
    su.n_span = Math.max(3, Math.round(su.n_span * ns/10));
    su.n_chord = Math.max(2, Math.round(su.n_chord * nc/4));
  }
  const tt = new Tunnel(c2);
  const a = performance.now();
  const sl = tt.liftSlope({v:22, controls});
  const npx = tt.neutralPoint({v:22, controls});
  const ms = performance.now() - a;
  const n = tt.solve({alpha:4, v:22, controls}).panels;
  console.log(`    ${String(n).padStart(4)} panels  slope ${sl.toFixed(3)}`
            + `   NP ${(npx*100).toFixed(1)}%   ${ms.toFixed(0)} ms`);
}

console.log('\n  control response (lift slope is unchanged; CL shifts):');
for(const d of [-20, -12, 0, 12, 20]){
  const s2 = t.solve({alpha: 4, v: 22, controls: {...controls, flaperon: d}});
  console.log(`    flaperon ${String(d).padStart(4)} deg   CL ${s2.CL.toFixed(4)}`
            + `   CDi ${s2.CDi.toFixed(5)}`);
}
