/* What an inviscid solve cannot tell you, worked out beside it.
 *
 * A panel method gives a correct potential flow, and a correct potential flow
 * has no skin friction, no separation, and -- by d'Alembert -- no pressure
 * drag on any closed body at all. On a 440 mm model at Reynolds 262,000 that
 * is not a detail. Measured against this aeroplane:
 *
 *   the nozzle base, one flat 7.6 cm2 disc, was given Cp +0.77 where a real
 *   blunt base sits near -0.15. That single face is worth more drag than the
 *   entire induced drag the solver does compute.
 *
 * So three things are added, each from a stated source, and each of them a
 * correction TO the solution rather than a replacement for it -- they are
 * computed from what the panel solve actually found, not from a table.
 *
 * SKIN FRICTION AND FORM. The ordinary component build-up: flat-plate
 * friction at each part's own Reynolds number, times a form factor for its
 * thickness, times its wetted area. Laminar below transition and turbulent
 * above, because at this size most of the aeroplane is laminar.
 *
 * THE LAMINAR SEPARATION BUBBLE. Below about Re 500,000 a laminar boundary
 * layer separates before it transitions, and what follows is a bubble that
 * costs far more than the friction underneath it. Selig's low-Reynolds work
 * at Illinois finds that at Re 200,000 the flow does not reattach at all.
 * Flat-plate friction alone gives a section drag of 0.0066 at Re 262,000
 * where the measured value for an ordinary section is about 0.016, so the
 * bubble is more than half the drag and leaving it out is not conservative,
 * it is wrong. The correlation below is calibrated to that.
 *
 * THE LEADING-EDGE SUCTION THAT IS NOT THERE. Potential flow takes the air
 * round a leading edge and books the suction; a boundary layer that cannot
 * negotiate the pressure recovery separates instead and the suction is lost.
 * Polhamus (NASA TN D-3767, D-4739): "With leading-edge vortex flow, a
 * Kutta-type condition exists at the leading edge and the leading-edge
 * suction is lost." What is lost as suction reappears as a normal force --
 * vortex lift -- and as drag, because the resultant is no longer tilted
 * forward. Both are derived here from the solve's OWN suction, and how much
 * of it survives is decided by the solve's own leading-edge loading.
 */

/* Flat-plate skin friction.
 *
 * Above transition it is the MIXED form -- laminar to the transition point,
 * turbulent after -- not the fully-turbulent one. Switching between the two
 * pure forms puts a step in the drag at the transition Reynolds number:
 * laminar gives 0.00188 there and fully turbulent 0.00536, so the aeroplane's
 * profile drag jumped by a third between 36 and 38 m/s and the drag polar had
 * a cliff in the middle of the cruise range. The mixed form is 0.00196 at the
 * same point, which meets the laminar value, because it is the same boundary
 * layer described continuously. */
const cfLaminar = (re) => 1.328 / Math.sqrt(Math.max(re, 1));
const cfMixed   = (re) => Math.max(0.074 / Math.pow(Math.max(re, 1), 0.2)
                                   - 1700 / Math.max(re, 1), 1e-4);

/* Transition, as a Reynolds number based on run length. Below it the plate is
 * laminar all the way. 5 x 10^5 is the textbook figure for a smooth surface
 * in low-turbulence air, which is what a model in still air is. */
const RE_TRANSITION = 5e5;

/* The bubble penalty, as a section drag increment.
 *
 * Zero at and above transition, rising linearly as the Reynolds number falls
 * below it. The constant is set so that an ordinary section at Re 262,000 --
 * this aeroplane's mean chord at 22 m/s -- comes out at about 0.016 total,
 * which is what the low-Reynolds measurements give. It is a correlation, not
 * a computation, and it is the least defensible number in this file; it is
 * also worth more than everything else in it, so it is stated rather than
 * quietly folded into a form factor.
 */
const BUBBLE = 0.020;

function bubbleDrag(re){
  if(re >= RE_TRANSITION) return 0;
  return BUBBLE * (1 - re / RE_TRANSITION);
}

/* Form factors. Body: Hoerner's slenderness form. Surface: the usual
 * thickness form. */
const ffBody    = (ld) => 1 + 60 / Math.pow(Math.max(ld, 2), 3) + 0.0025 * ld;
const ffSurface = (tc) => 1 + 2.7 * tc + 100 * Math.pow(tc, 4);

/* Section drag off the table, by thickness, Reynolds number and loading.
 *
 * Trilinear in (t/c, log Re, CL). Log in Reynolds because that is how drag
 * varies with it and how the table is spaced; linear in the other two. Held
 * at the ends rather than extrapolated -- past the table's Reynolds range the
 * answer stops changing much, and past its CL range it would be extrapolating
 * a stall, which is not a thing to guess at. */
function sectionCd(tab, tc, re, cl){
  if(!tab) return null;
  const at = (arr, v) => {
    if(v <= arr[0]) return [0, 0];
    if(v >= arr[arr.length-1]) return [arr.length-2, 1];
    let i = 0;
    while(i < arr.length-2 && arr[i+1] < v) i++;
    return [i, (v - arr[i])/(arr[i+1] - arr[i])];
  };
  const lre = tab.re.map(Math.log);
  const [i, fi] = at(tab.tc, tc);
  const [j, fj] = at(lre, Math.log(Math.max(re, 1)));
  const [k, fk] = at(tab.cl, Math.abs(cl));
  const g = (a, b, c) => tab.cd[a][b][c];
  const lerp = (p, q, t) => p + (q - p)*t;
  const c00 = lerp(g(i,j,k),     g(i,j,k+1),     fk);
  const c01 = lerp(g(i,j+1,k),   g(i,j+1,k+1),   fk);
  const c10 = lerp(g(i+1,j,k),   g(i+1,j,k+1),   fk);
  const c11 = lerp(g(i+1,j+1,k), g(i+1,j+1,k+1), fk);
  return lerp(lerp(c00, c01, fj), lerp(c10, c11, fj), fi);
}

/* Per-component geometry: wetted area, streamwise length, and how thick it is.
 *
 * Caps are folded into the thing they cap. On their own they have no
 * streamwise extent at all, so a Reynolds number based on their length is
 * zero and the friction coefficient goes to infinity -- the nose cap came out
 * at Cf = 1.1. They are not separate objects; they are the ends of one.
 */
function parts(pf, geom){
  const by = new Map();
  for(const p of (geom.parts || [])){
    const base = p.name.replace(/_(root|tip|cap|front|back|in|out)$/, '')
                       .replace(/^(nose|tail)$/, 'fuselage');
    if(!by.has(base))
      by.set(base, {name: base, area: 0,
                    lo: [Infinity, Infinity, Infinity],
                    hi: [-Infinity, -Infinity, -Infinity]});
    const c = by.get(base);
    for(let i = p.start; i < p.start + p.count; i++){
      c.area += pf.area[i];
      for(let k = 0; k < 3; k++){
        const v = pf.c[i*3+k];
        if(v < c.lo[k]) c.lo[k] = v;
        if(v > c.hi[k]) c.hi[k] = v;
      }
    }
  }
  const out = [];
  for(const c of by.values()){
    if(!(c.area > 0)) continue;
    const ext = [c.hi[0]-c.lo[0], c.hi[1]-c.lo[1], c.hi[2]-c.lo[2]]
      .map(v => Math.max(v, 1e-6));
    // span is the wider of the two transverse extents: a wing spans in y, a
    // fin in z, and the mean chord is planform over span
    c.span = Math.max(ext[1], ext[2]);
    /* Thin in its OWN thin direction. A fin is thin in y and a wing is thin
     * in z; measuring "thin" as height over length made the fin, which is
     * tall and short, look like a very stubby body and gave it a form factor
     * of 8.5. */
    const thin = Math.min(...ext) / Math.max(...ext);
    out.push({name: c.name, area: c.area, len: ext[0], thin,
              span: c.span, planform: c.area/2});
  }
  return out;
}

/* The base: panels whose normal faces downstream.
 *
 * A base is where the body stops rather than closes, and potential flow
 * always closes it -- the rear stagnation is the whole of d'Alembert. A real
 * one sits at about Cp -0.15, and with a jet filling it, nearer zero. */
function baseArea(pf, u){
  let a = 0;
  for(let i = 0; i < pf.N; i++){
    const d = pf.nrm[i*3]*u[0] + pf.nrm[i*3+1]*u[1] + pf.nrm[i*3+2]*u[2];
    /* Engine faces are INCLUDED here, unlike everywhere else: a nozzle with
     * the engine off is a base, and it is the biggest one on the aeroplane.
     * What decides whether it behaves like one is how much gas is coming out
     * of it, which is what `jetFill` is for. */
    if(d > 0.85) a += pf.area[i] * d;
  }
  return a;
}

/* How much of the leading-edge suction survives.
 *
 * Decided by the solve's own leading-edge loading, which is the physical
 * argument: the suction is lost when the boundary layer cannot climb back out
 * of the peak. Fully attached below Cp -1.5, fully separated past -5, and a
 * straight line between. On this wing that is all the suction at 4 degrees,
 * three quarters of it at 8, and none by 12 -- which is where a 40-degree
 * delta with a strake starts flying on its vortices.
 */
export function suctionKept(cpPeak){
  const m = Math.abs(cpPeak);
  if(m <= 1.5) return 1;
  if(m >= 5.0) return 0;
  return 1 - (m - 1.5) / 3.5;
}

/* The whole build-up.
 *
 * `pot` is what the panel solve found: CL, the Trefftz induced drag, and the
 * lowest Cp anywhere on a lifting surface. Everything else is derived.
 */
export function buildUp(pf, geom, o){
  const {rho = 1.225, nu = 1.5e-5, v, sRef, alpha, cpPeak = 0,
         CL: CLpot, CDi, jetFill = 0, extra = [], sections = null,
         tc = {}} = o;
  const u = [Math.cos(alpha), 0, Math.sin(alpha)];
  const comps = [];
  let cd0 = 0;
  for(const c of parts(pf, geom)){
    /* A part is a body or a surface by its shape, not by its name: a fin and
     * a fuselage are told apart by how thick they are for their length. */
    const thin = c.thin;
    const surface = thin < 0.35;
    const thick = tc[c.name];
    if(surface && sections && thick){
      /* A LIFTING SURFACE gets its drag from the table.
       *
       * Reynolds on its own mean chord, which is planform over span, not on
       * the bounding box -- a swept wing's box is longer than its chord and
       * a wing is not a plate the length of its own footprint. The table is
       * NeuralFoil, which is XFOIL with a real boundary layer, so the
       * laminar bubble is computed rather than correlated. That matters: the
       * correlation it replaces was a factor of two high at this aeroplane's
       * Reynolds number, on the largest single piece of its drag.
       */
      const chord = c.planform / Math.max(c.span, 1e-6);
      const re = v * chord / nu;
      const cd = sectionCd(sections, thick, re, CLpot);
      const d = cd * c.planform / sRef;
      comps.push({name: c.name, re, cf: cd, ff: 1, bubble: 0,
                  area: c.planform, cd: d, from: 'section table'});
      cd0 += d;
      continue;
    }
    const re = v * c.len / nu;
    const cf = re < RE_TRANSITION ? cfLaminar(re) : cfMixed(re);
    const ff = surface ? ffSurface(Math.min(thin, 0.3))
                       : ffBody(c.len / Math.max(Math.sqrt(c.area/Math.PI), 1e-6));
    const bub = surface ? bubbleDrag(re) * 0.5 : 0;   // per wetted side
    const d = (cf * ff + bub) * c.area / sRef;
    comps.push({name: c.name, re, cf, ff, bubble: bub, area: c.area, cd: d,
                from: 'flat plate and form'});
    cd0 += d;
  }
  /* Surfaces the aeroplane has and the panel model does not.
   *
   * A leading-edge root extension is 74 mm long and 13 mm wide on this
   * aeroplane: thin enough that panelling it would put a sliver alongside the
   * fuselage, which is the near-singular configuration everything here works
   * to avoid, and it would represent nothing anyway. What a strake is FOR is
   * shedding a vortex, which is the one thing a potential flow cannot do; its
   * effect is in the vortex-lift model below. What it also is, though, is
   * wetted area, and that is a thing this build-up can count.
   */
  for(const e of extra){
    const re = v * e.len / nu;
    const cf = re < RE_TRANSITION ? cfLaminar(re) : cfMixed(re);
    const ff = ffSurface(Math.min(e.thin || 0.08, 0.3));
    const d = (cf * ff + bubbleDrag(re) * 0.5) * e.area / sRef;
    comps.push({name: e.name, re, cf, ff, bubble: bubbleDrag(re) * 0.5,
                area: e.area, cd: d});
    cd0 += d;
  }
  const aBase = baseArea(pf, u);
  const cpBase = -0.15 * (1 - Math.min(1, jetFill));
  const cdBase = -cpBase * aBase / sRef;

  /* The leading-edge suction the potential solve claimed:
   * zero-suction drag is CL.tan(alpha); full-suction drag is the Trefftz
   * value; the difference is the suction. */
  const cs = Math.max(0, CLpot * Math.tan(alpha) - CDi);
  const kept = suctionKept(cpPeak);
  const clVortex = (1 - kept) * cs * Math.cos(alpha);
  const CL = CLpot + clVortex;
  const cdLift = kept * CDi + (1 - kept) * CL * Math.tan(alpha);

  return {
    CL, CLpot, clVortex, kept, cs,
    cd0, cdBase, cdLift, cdi: CDi,
    CD: cd0 + cdBase + cdLift,
    comps, aBase, cpBase,
  };
}
