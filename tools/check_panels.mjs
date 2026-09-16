/* Is the body panelisation something a source-panel solve can answer?
 *
 * A source panel's strength should come out the order of the freestream. When
 * a pair of panels sits a fraction of their own size apart and faces opposite
 * ways -- which is what closing a 4 mm thick wingtip with four rings of cap
 * does -- the influence matrix is near-singular there, the strengths run away,
 * and the field a millimetre off them is not flow. On this aeroplane the
 * wingtip carried sigma = 1,628 against a 22 m/s freestream, and the
 * streamline that passed 2 mm from it read 295 m/s: the fastest thing in the
 * picture, so it set the colour scale for everything else and painted the
 * model with dark scribble.
 *
 * The solve does not fail, and nothing downstream says anything. So check it.
 */
import { BodyField } from '../viewer/flowfield.js';
import { readFileSync } from 'node:fs';

const man = JSON.parse(readFileSync(new URL('../viewer/parts.json', import.meta.url)));
const bp = man.body_panels;
const cfg = man.tunnel;
const V = cfg.v_default;

const body = new BodyField(bp, {ground: !!cfg.ground});
body.solve([V, 0, 0], null);

const s = body.sigma, a = body.area;
const abs = Array.from(s, Math.abs).sort((x, y) => x - y);
const q = f => abs[Math.floor(f * (abs.length - 1))];
const worst = [...s.keys()].sort((i, j) => Math.abs(s[j]) - Math.abs(s[i]));

console.log(`\nBODY PANELISATION  (${body.N} panels, freestream ${V} m/s)`);
console.log(`  wetted area      ${a.reduce((x, y) => x + y, 0).toFixed(4)} m2`);
console.log(`  panel size       median ${(Math.sqrt(q(0.5) * 0 + median(a)) * 1000).toFixed(1)} mm`
          + `   smallest ${(Math.sqrt(Math.min(...a)) * 1000).toFixed(2)} mm`);
console.log(`  source strength  median ${q(0.5).toFixed(1)}`
          + `   p99 ${q(0.99).toFixed(0)}   max ${q(1).toFixed(0)}`);
console.log('\n  strongest panels:');
for (const i of worst.slice(0, 5)) {
  console.log(`    sigma ${s[i].toFixed(0).padStart(7)}`
            + `   ${(Math.sqrt(a[i]) * 1000).toFixed(2).padStart(6)} mm panel`
            + `   at ${[0, 1, 2].map(k => (body.c[i * 3 + k] * 1000).toFixed(0).padStart(5)).join(',')} mm`);
}

function median(arr) {
  const v = Array.from(arr).sort((x, y) => x - y);
  return v[Math.floor(v.length / 2)];
}

/* The limit is 12 x freestream. A closed body in a uniform stream has source
 * strengths of the order of the stream itself; an order of magnitude covers
 * the genuinely awkward places -- a nose, a lip, the join between a wing and a
 * fuselage -- and anything past it is the panelisation, not the aeroplane. */
const ratio = q(1) / V;
const ok = ratio < 12;
console.log('\n' + '='.repeat(62));
console.log(`  strongest source is ${ratio.toFixed(1)} x freestream (limit 12)`);
console.log(ok ? 'PASS  the panelisation is one the source solve can answer'
               : 'FAIL  panels too close together for the solve: the field near '
                 + 'them is discretisation noise, not flow');
process.exit(ok ? 0 : 1);
