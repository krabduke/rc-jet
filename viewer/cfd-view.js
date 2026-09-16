/* Streamlines, surface pressure and a cutting plane: a flow field you can read.
 *
 * The first version of this drew 230 one-pixel streamlines, traced once and
 * then left alone, coloured on a rainbow map, with no surface pressure and no
 * indication of which way the air was going. It looked like a CFD screenshot
 * and told you almost nothing: you could not tell inflow from wake, you could
 * not see where the downforce came from, and a `this.arrows = null` sat in the
 * constructor as an admission that the direction cue had been planned and
 * never built.
 *
 * What is here now:
 *
 *   ribbons   Streamlines as camera-facing ribbons a few millimetres wide, so
 *             they are visible, and with a travelling pulse along each one so
 *             the flow moves at its own local speed. Direction and speed are
 *             legible without reading a legend.
 *   arrows    Cone glyphs along each line, so direction is still unambiguous
 *             with the animation paused or in a screenshot.
 *   surface   Pressure coefficient painted on the actual model geometry, which
 *             is the only view that answers "where does the load come from".
 *   plane     A cutting plane through the flow with filled colour, which is the
 *             single most readable thing a post-processor produces.
 *
 * The field underneath is unchanged and still solved: freestream, plus the
 * body's source panels, plus the lifting surfaces' horseshoe vortices.
 */

import * as THREE from 'three';
import {
  BodyField, traceLine, speedColour, pressureColour, cpNorm, surfaceCp,
  panelIndex,
} from './flowfield.js';
import { PanelFlow } from './panelflow.js';
import { fadeObject } from './anim.js';

const TRACE_STEPS = 240;

/* Scene (glTF Y-up) from solver frame (x aft, y span, z up), and back. */
const toScene = (x, y, z) => [x, z, -y];
const toSolver = (X, Y, Z) => [X, -Z, Y];

export class CFDView {
  constructor(opts){
    this.cfg = opts.cfg;
    this.root = opts.root;
    this.bounds = opts.bounds;
    this.bodyPanels = opts.bodyPanels;
    this.panelBody = opts.panelBody || null;
    this.nLines = opts.nLines || 560;
    this.group = new THREE.Group();
    this.group.name = 'cfd';
    this.root.add(this.group);
    /* One solver or the other.
     *
     * `panelBody` is the aeroplane as a closed quadrilateral surface and gets
     * the real thing: source and doublet panels, a wake and a Kutta
     * condition, solved once at build and back-substituted for each new
     * incidence. `bodyPanels` is the older centroid-normal-area form, which
     * can only carry sources -- no circulation, so no lift of its own, and it
     * needs a vortex lattice beside it to make any. Both are here while the
     * models are converted one at a time; they are not combined.
     */
    if(this.panelBody){
      const g = Object.assign({}, this.panelBody,
                              {quads: Float64Array.from(this.panelBody.quads),
                               ground: !!this.cfg.ground});
      this.pf = new PanelFlow(g);
      this.body = this.pf;
      // the seeding wants centroids, normals and areas, which the solved
      // surface has anyway
      const N = this.pf.N;
      const c = new Float64Array(N*3), nn = new Float64Array(N*3),
            a = new Float64Array(N);
      for(let i = 0; i < N*3; i++){ c[i] = this.pf.c[i]; nn[i] = this.pf.nrm[i]; }
      for(let i = 0; i < N; i++) a[i] = this.pf.area[i];
      this.bodyPanels = {n: N, c, n_: nn, a};
    } else {
      this.pf = null;
      this.body = this.bodyPanels
        ? new BodyField(this.bodyPanels, !!this.cfg.ground) : null;
    }

    this.lines = null;
    this.arrows = null;       // built now, rather than declared and forgotten
    this.plane = null;
    this.cp = null;
    this.index = null;
    this.painted = [];        // meshes whose colours we replaced

    this.show = {ribbons: true, arrows: true, plane: false, surface: false,
                 undisturbed: false};
    /* How much a streamline has to be changed by the machine before it is
     * worth drawing. A rake wide enough to show the flow around the model
     * also fires a lot of lines straight past it, and a line that arrives
     * parallel and at freestream speed and leaves the same way carries no
     * information -- it is the grid, drawn in the same ink as the answer.
     * Both thresholds are relative, so they mean the same thing on a 4.6 m
     * car and a 0.44 m model: a fraction of freestream speed, and a fraction
     * of model length of lateral deflection. Either one qualifies.
     *
     * Measured rather than guessed. On the car the two criteria do different
     * jobs: deflection decides almost every line, and moving the speed cut
     * anywhere between 2% and 15% changes the retained count by at most eight
     * -- but those eight are the lines that run straight into a stagnation
     * region, losing all their speed without ever leaving their path, which
     * deflection alone would throw away. What comes out are the lines that
     * pass wide: at this cut the discarded seeds sit 3.1 m off the centreline
     * on average against 0.9 m for the ones kept, on a car 4.6 m long. */
    this.disturb = {speed: 0.35, deflect: 0.55, near: 0.20};
    /* `dash` is the arc length of one travelling pulse, and it has to be a
     * fraction of the MODEL.
     *
     * It was 0.9 m, chosen on a car 4.98 m long whose lines run about ten
     * metres -- eleven pulses a line, which reads as flow moving. The same
     * 0.9 m on a 0.44 m aeroplane whose lines are 1.2 m long is 1.4 pulses a
     * line, and since the pulse sits at 34 % brightness for about seventy
     * per cent of its period, what that draws is one bright head and a line
     * that has mostly vanished. Set in _buildSeeds from the model's length. */
    this.anim = {on: true, rate: 0.55, dash: 0.9, pulse: 1.0};
    this.range = {auto: true, lo: 0.35, hi: 1.45};   // x freestream
    this.cpRange = {lo: -3.0, hi: 1.0};
    this.clock = 0;
    this._buildSeeds();
  }

  /* --------------------------------------------------------------- seeding */

  /* Seed a rake big enough to contain the model at any attitude, and put more
   * of the lines where the model is.
   *
   * The rake used to be sized on the body's bounding box, so a pitched or
   * yawed model rotated out of it. It is sized on the model's diagonal now.
   * The spacing is no longer uniform either: a uniform grid spends most of its
   * lines on undisturbed air that looks the same everywhere, so the rows and
   * columns are pulled towards the centre of the frontal area, where the
   * model actually is.
   */
  _buildSeeds(){
    const P = this.bodyPanels;
    let x0 = 1e9, x1 = -1e9, y1 = 0, z0 = 1e9, z1 = -1e9;
    for(let i = 0; i < P.n; i++){
      x0 = Math.min(x0, P.c[i*3]); x1 = Math.max(x1, P.c[i*3]);
      y1 = Math.max(y1, Math.abs(P.c[i*3+1]));
      z0 = Math.min(z0, P.c[i*3+2]); z1 = Math.max(z1, P.c[i*3+2]);
    }
    this.ext = {x0, x1, y1, z0, z1};

    const L = x1 - x0, W = 2*y1, Hh = z1 - z0;
    const diag = 0.5*Math.hypot(L, W, Hh);
    const zc = 0.5*(z0 + z1);
    /* The rake covers the model and a margin, not the whole domain.
     *
     * At 1.9 and 1.7 diagonals it threw most of its lines into clear air
     * either side, where they run straight and tell you nothing: a third of
     * the frame was undisturbed sheets. Tighter, and bunched harder towards
     * the centre, so the same budget buys resolution where the flow is
     * actually doing something. */
    /* Sized on the frontal box, not the diagonal.
     *
     * On a slender body the diagonal is nearly all length. A 463 mm aeroplane
     * 300 mm across and 132 mm tall got a rake 312 mm tall -- two and a half
     * times the model -- so half the lines were released above or below it
     * and ran dead straight the whole way past. A frontal box with a margin
     * spends the budget on air that the model is actually in. */
    const spanY = W * 1.25 + diag * 0.10;
    const spanZ = Hh * 1.70 + diag * 0.10;
    const xs = x0 - L * 0.55;

    // bunch f towards the middle without leaving the edges empty
    const bunch = (f) => 0.5 + (f - 0.5) * (0.42 + 0.58*Math.abs(2*f - 1));

    const seeds = [];
    /* More of the budget upstream than on the skin.
     *
     * It used to be 55 / 45. Surface seeds start a panel width off the body
     * and hug it, so at 45 % of the lines they cover the aeroplane in ribbons
     * -- which is the one thing the picture must not do, because the reason
     * to draw the flow at all is to see it against the shape making it. They
     * are still worth having: they are the only lines that show where the
     * flow attaches and where it leaves. There are just fewer of them. */
    const nRake = Math.round(this.nLines * 0.72);
    const rows = Math.max(8, Math.round(Math.sqrt(nRake * 0.62)));
    const cols = Math.max(8, Math.round(nRake / rows));
    for(let i = 0; i < rows; i++){
      for(let j = 0; j < cols; j++){
        const fz = bunch(rows > 1 ? i/(rows - 1) : 0.5);
        const fy = bunch(cols > 1 ? j/(cols - 1) : 0.5);
        let z = zc + (fz - 0.5) * spanZ;
        if(this.cfg.ground){
          // there is nothing below a track, so fill the space above it -- and
          // put extra rows in the first 15 % of the height, because on a
          // ground-effect car that is where the whole story is
          const g = fz < 0.38 ? Math.pow(fz/0.38, 1.7)*0.16 : 0.16 + (fz - 0.38)/0.62*0.84;
          z = 0.012 + g * spanZ * 1.05;
        }
        seeds.push([xs, (fy - 0.5) * spanY, z]);
      }
    }
    // the upstream rake is kept on its own: _prepare shears it onto whatever
    // freestream direction the solve is running at
    this.rake = seeds.slice();

    /* Surface seeding: the lines that hug the body.
     *
     * A rake upstream can only ever show what passes near the model; the
     * flow that matters -- over the nose, across the roof, under the floor
     * and into the intakes -- needs to start ON the wetted surface. Each
     * seed sits a panel width off a panel centroid along its outward normal,
     * so the line leaves the skin tangentially and follows the curvature
     * that the solved source field already encodes. This is the difference
     * between a picture of flow going past a car and a picture of the flow
     * around it.
     */
    const nSurf = Math.max(1, this.nLines - seeds.length);
    const stride = Math.max(1, Math.floor(P.n / nSurf));
    for(let i = 0; i < P.n && seeds.length < this.nLines; i += stride){
      const nx = P.n_[i*3], ny = P.n_[i*3+1], nz = P.n_[i*3+2];
      // skip the base: a panel whose normal is very nearly +x faces straight
      // downstream, so a seed pushed off it starts inside the wake, where the
      // line is neither attached to the body nor telling you about the flow
      // that got it there
      if(nx >= -0.15 && Math.abs(nz) <= 0.15 && Math.abs(ny) <= 0.15) continue;
      const px = P.c[i*3], py = P.c[i*3+1], pz = P.c[i*3+2];
      /* Stand off by the panel's own size, floored on the MODEL's size.
       *
       * The floor used to be a flat 20 mm, which is a sensible fraction of a
       * five-metre car and a third of the width of a 440 mm aeroplane: every
       * surface seed on the jet was launched 20 mm off the skin, which on the
       * tailcone is outside the far side of it. */
      /* Far enough off the skin to read as flow rather than as paint.
       *
       * A panel width and a half put the line inside the model's own
       * silhouette from most angles, so it drew over the aeroplane instead of
       * round it. */
      const d = Math.max(diag * 0.035, Math.sqrt(P.a[i]) * 2.2);
      let sx = px + nx*d, sy = py + ny*d, sz = pz + nz*d;
      if(this.cfg.ground && sz < 0.006) sz = 0.006;
      seeds.push([sx, sy, sz]);
    }
    // a streamline may come this close to the skin and no closer: about half
    // a per cent of the model, which is under an aeroplane's skin thickness
    // and well inside a car's ride height
    if(this.body) this.body.nearStop = diag * 0.006;
    this.surf = seeds.slice(this.rake.length);
    this.seeds = seeds;
    this.domain = {spanY, spanZ, xs, diag, L};
    // Just under half the model's length, which puts about eleven pulses on a
    // line that runs a few model lengths -- the density the car had, and the
    // density at which a pulse reads as a travelling wave rather than as a
    // dotted line.
    this.anim.dash = L * 0.45;
  }

  /* ------------------------------------------------------------------ solve */

  /* Solve the body, couple it to the lifting surfaces, and set up the field.
   *
   * Two passes: the body in the freestream, then the body seeing the wings.
   * That leaves the validated lattice solve untouched and settles the coupling.
   */
  _prepare(vinf, latticeVel, ports){
    if(this.pf) return this._preparePanel(vinf, ports);
    const N = this.body.N;
    this.vinf = vinf;
    this.body.setPorts(ports || null);
    this.body.solve(vinf, null);
    if(latticeVel){
      const extra = new Float64Array(N*3);
      const p = [0,0,0], v = [0,0,0];
      for(let i = 0; i < N; i++){
        p[0] = this.body.c[i*3]; p[1] = this.body.c[i*3+1]; p[2] = this.body.c[i*3+2];
        v[0] = v[1] = v[2] = 0;
        latticeVel(p, v);
        extra[i*3] = v[0]; extra[i*3+1] = v[1]; extra[i*3+2] = v[2];
      }
      this.body.solve(vinf, extra);
    }
    this.vel = (p, out) => {
      out[0] = vinf[0]; out[1] = vinf[1]; out[2] = vinf[2];
      this.body.add(p, out);
      if(latticeVel) latticeVel(p, out);
      return out;
    };
    this.vfs = Math.hypot(vinf[0], vinf[1], vinf[2]) || 1;

    this._shearRake(vinf);

    const span = this.ext.x1 - this.ext.x0;
    const zc = 0.5*(this.ext.z0 + this.ext.z1);
    /* The trace roams further than the rake seeds it.
     *
     * The bounds used to be the rake's own extents, so tightening the rake
     * would have cut every line short the moment it was deflected. A line is
     * allowed most of a model diagonal either side; what the rake decides is
     * where lines start, not how far they may go. */
    const mY = this.domain.diag * 0.95, mZ = this.domain.diag * 0.95;
    this._traceOpts = {
      maxSteps: TRACE_STEPS, ds: span/58, xEnd: this.ext.x1 + span*1.15,
      body: this.body,
      // the widest a line may be held tangential to the skin. The band
      // itself is the local panel's own size; this only stops a coarsely
      // panelled body -- a car's floor at 100 mm a panel -- from claiming a
      // band wider than the gap it has to leave open underneath it.
      skin: this.domain.diag * 0.045,
      back: TRACE_STEPS,
      xStart: this.domain.xs - span * 0.15,
      bounds: [-mY, mY, this.cfg.ground ? 0.0 : zc - mZ, zc + mZ],
    };
  }

  /* The panel-method path: one solve, and the field is its gradient.
   *
   * No second pass and no coupling. There is one set of singularities and one
   * boundary condition, so the velocity a streamline is traced through is the
   * gradient of the same potential the pressures came off -- which is the
   * whole reason for the method and the thing the two-solver arrangement
   * could not say.
   */
  _preparePanel(vinf, ports){
    this.vinf = vinf;
    /* The engine, as a boundary condition rather than as a point sink.
     *
     * The ports callback already says how much the machine swallows and
     * blows -- it was written for the source-panel field, where an intake
     * was a point sink sitting in front of a solid nose. On a closed panel
     * surface the same numbers say something better: this much volume
     * crosses these panels. The capture streamtube then forms because the
     * skin is permeable there, not because a sink is pulling at it.
     */
    let qIn = 0, qOut = 0;
    for(const q of (ports || [])){
      if(q.Q < 0) qIn -= q.Q; else qOut += q.Q;
    }
    this.pf.setFlow(qIn, qOut);
    this.pf.solve(vinf);
    this.vel = (p, out) => this.pf.velocity(p, out);
    this.vfs = Math.hypot(vinf[0], vinf[1], vinf[2]) || 1;
    this._shearRake(vinf);
    const span = this.ext.x1 - this.ext.x0;
    const zc = 0.5*(this.ext.z0 + this.ext.z1);
    const mY = this.domain.diag * 0.95, mZ = this.domain.diag * 0.95;
    this._traceOpts = {
      maxSteps: TRACE_STEPS, ds: span/58, xEnd: this.ext.x1 + span*1.15,
      body: this.body,
      /* A narrow skin band, kept because it measurably helps and not
       * because the argument for it survived.
       *
       * The argument was that a solved panel body has no normal velocity on
       * its own skin by construction, so a line arriving at it turns and goes
       * round without being made to. That is true of the exact field and not
       * quite true of the one being traced, which is integrated in finite
       * steps through a lumped far field. Measured over 352 lines: without
       * the band 24 of them end on the skin, with it 6, and the kink rate is
       * zero either way. So: 6.
       */
      skin: this.domain.diag * 0.03,
      /* Walked upstream from the seed as well.
       *
       * A line seeded on the skin and walked only downstream is drawn
       * beginning ON the aeroplane, which is air appearing out of the
       * surface. Those seeds are 28 % of the budget and nearly all of them
       * survive the proximity filter, so over half the lines on screen
       * started that way. */
      back: TRACE_STEPS,
      xStart: this.domain.xs - span * 0.15,
      bounds: [-mY, mY, this.cfg.ground ? 0.0 : zc - mZ, zc + mZ],
    };
    return this;
  }

  /* Put the rake where the model is, for the freestream it is being flown at.
   *
   * The seeds sit on a plane upstream and the flow is tilted by the angle of
   * attack, so a line released level with the model has climbed
   * (x_mid - x_seed) * tan(alpha) by the time it arrives. At eight degrees on
   * a 463 mm aeroplane that is 67 mm against a 132 mm tall model: the entire
   * rake sailed over the top, and what the picture showed was undisturbed
   * air with an aeroplane behind it. Offsetting each seed back along the
   * freestream puts the rake's image at the model rather than above it.
   *
   * Only the upstream rake moves. The surface seeds are already on the skin
   * and must stay there.
   */
  _shearRake(vinf){
    if(!this.rake) return;
    const vx = vinf[0] || 1;
    const drift = 0.5*(this.ext.x0 + this.ext.x1) - this.domain.xs;
    const dy = -drift * (vinf[1] / vx);
    const dz = -drift * (vinf[2] / vx);
    const out = [];
    for(const s of this.rake){
      let z = s[2] + dz;
      if(this.cfg.ground) z = Math.max(0.006, z);
      out.push([s[0], s[1] + dy, z]);
    }
    this.seeds = out.concat(this.surf || []);
  }

  /* The colour ranges, from the field rather than from a constant.
   *
   * Speed: not the observed maximum -- a point-source panel is singular at its
   * own centroid, so a line grazing the skin picks up a spike (672 m/s against
   * a 69 m/s freestream once) that normalises everything else to one colour.
   * Not zero-based either, because nearly all of the flow is at freestream
   * speed. So: a window around freestream, set at the 2nd and 98th percentile
   * of the traced field so one singular spike cannot set it. It used to be
   * hard-coded at 0.35-1.45x and unreachable, which clipped the fastest and
   * most interesting flow under the floor to flat red.
   *
   * Pressure: to the 2nd percentile of suction, not the single worst panel --
   * one panel at Cp -4.7 beside a hundred between -0.4 and -1.2 compresses the
   * whole underfloor into one shade of blue. The top stays at Cp = 1, which is
   * the physical limit, so two runs are comparable by eye.
   */
  _setRanges(traced, vinf, latticeVel){
    traced = this._shown(traced);
    if(this.range.auto){
      const all = [];
      for(const L of traced) for(const m of L.spd) all.push(m);
      all.sort((a, b) => a - b);
      const q = (f) => all.length ? all[Math.floor(f*(all.length - 1))] : this.vfs;
      /* and clamped to a physical window around freestream.
       *
       * A percentile alone is not enough when the tail of the distribution is
       * singular rather than fast: on the car two per cent of sampled points
       * sat above five times freestream, so the 98th percentile put the top of
       * the scale at 1,290 km/h and painted the entire car one colour. Nothing
       * in a potential-flow solution goes twice freestream except a point
       * standing on a singularity, so the window stops there. */
      const lo = this.vfs * 0.35, hi = this.vfs * 2.2;
      this.vmin = Math.max(lo, Math.min(q(0.02), this.vfs*0.85));
      this.vmax = Math.min(hi, Math.max(q(0.98), this.vfs*1.12));
    } else {
      this.vmin = this.vfs*this.range.lo;
      this.vmax = this.vfs*this.range.hi;
    }
    this.cp = this.pf ? this.pf.cp : surfaceCp(this.body, vinf, latticeVel);
    let lo = 1e9, hi = -1e9;
    for(const v of this.cp){ if(v < lo) lo = v; if(v > hi) hi = v; }
    const sorted = Array.from(this.cp).sort((a, b) => a - b);
    const pct = (f) => sorted[Math.floor(f*(sorted.length - 1))];
    /* Both ends follow the data, with a floor on the span.
     *
     * A fixed -0.5 to 1.0 scale fitted the car, whose underfloor runs to
     * Cp -1.5, and badly misrepresented the aeroplane, whose fuselage panels
     * live between -0.08 and +0.12: on that scale a body with almost no
     * pressure variation at all rendered as saturated red over nearly the
     * whole model, which is a picture of the colour map rather than of the
     * flow. Percentiles at both ends fix that; the +/-0.25 floor stops the
     * opposite failure, where a genuinely flat field gets stretched across the
     * full map and noise is dressed up as signal. `cpNorm` keeps the neutral
     * colour on Cp = 0 whatever the two ends are.
     */
    this.cpRange = {lo: Math.min(pct(0.02), -0.25),
                    hi: Math.max(pct(0.98), 0.25)};
    this.cpObserved = {lo, hi};
  }

  /* Blocking trace. Kept for the offline harness, which has no frames to
     spread work across; the viewer uses `runProgressive`. */
  run(vinf, latticeVel, ports){
    if(!this.body) return null;
    const T = {};
    let t = performance.now();
    this._prepare(vinf, latticeVel, ports);
    T.solve = performance.now() - t; t = performance.now();
    const traced = [];
    let vpeak = 0;
    for(const s of this.seeds){
      const L = traceLine(this.vel, s, this._traceOpts);
      if(L.spd.length > 6){
        traced.push(this._score(L));
        for(const m of L.spd) if(m > vpeak) vpeak = m;
      }
    }
    T.trace = performance.now() - t; t = performance.now();
    this._setRanges(traced, vinf, latticeVel);
    T.cp = performance.now() - t; t = performance.now();
    this.traced = traced;
    this._drawRibbons(traced);
    T.ribbons = performance.now() - t; t = performance.now();
    this._drawArrows(traced);
    T.arrows = performance.now() - t;
    if(this.show.plane) this._drawPlane();
    if(this.show.surface) this.paintSurface();
    this.timing = T;
    const shown = this._shown(traced);
    return {lines: shown.length, traced: traced.length,
            hidden: traced.length - shown.length,
            vmin: this.vmin, vmax: this.vmax,
            vpeak, vfs: this.vfs, cpLo: this.cpObserved.lo,
            cpHi: this.cpObserved.hi, timing: T};
  }

  /* Solve, then trace in slices so the page stays responsive.
   *
   * `onProgress(done, total)` is called after each slice; the ribbons are
   * redrawn as the lines arrive, so the flow field appears progressively
   * rather than after a multi-second stall. Returns the same result object
   * `run` does.
   */
  async runProgressive(vinf, latticeVel, onProgress, opts = {}){
    /* ports and sliceMs ride in an options object rather than as two more
       positional arguments: `ports` arrived after `sliceMs` already had a
       default, and the next caller to pass a slice length positionally
       would have handed it over as the port list. */
    const {ports = null, sliceMs = 26} = opts;
    if(!this.body) return null;
    const T = {};
    let t0 = performance.now();
    this._prepare(vinf, latticeVel, ports);
    T.solve = performance.now() - t0;

    const traced = [];
    let vpeak = 0, i = 0, slice = 0;
    const total = this.seeds.length;
    /* Yield as a macrotask, not on the next animation frame.
     *
     * Awaiting requestAnimationFrame between slices pins the work to the frame
     * rate: a 26 ms slice followed by a wait for the next frame is a duty cycle
     * of about a half in the best case, and far worse wherever the browser
     * throttles frames -- measured at nine times slower overall than the
     * blocking version it replaced, which is not a fix, it is a different
     * problem. A macrotask yield lets input and paint in without handing the
     * whole thread back, so the work proceeds at close to full speed.
     */
    const yieldToPage = () => (globalThis.scheduler && scheduler.yield)
      ? scheduler.yield()
      : new Promise(r => setTimeout(r, 0));
    t0 = performance.now();
    while(i < total){
      const until = performance.now() + sliceMs;
      while(i < total && performance.now() < until){
        const L = traceLine(this.vel, this.seeds[i++], this._traceOpts);
        if(L.spd.length > 6){
          traced.push(this._score(L));
          for(const m of L.spd) if(m > vpeak) vpeak = m;
        }
      }
      slice++;
      if(onProgress) onProgress(i, total);
      // Draw what there is every so often, so the flow field fills in instead
      // of appearing all at once at the end. Ribbon building is 16 ms, so a
      // redraw every eighth slice is cheap next to the tracing.
      if(slice % 8 === 0 && traced.length > 4){
        this._setRanges(traced, vinf, latticeVel);
        this._drawRibbons(traced, Math.min(1, (i + 1) / Math.max(total, 1)));
      }
      await yieldToPage();
    }
    T.trace = performance.now() - t0;

    t0 = performance.now();
    this._setRanges(traced, vinf, latticeVel);
    T.cp = performance.now() - t0;
    t0 = performance.now();
    this.traced = traced;
    this._drawRibbons(traced);
    T.ribbons = performance.now() - t0;
    t0 = performance.now();
    this._drawArrows(traced);
    T.arrows = performance.now() - t0;
    if(this.show.plane) this._drawPlane();
    if(this.show.surface) this.paintSurface();
    this.timing = T;
    const shown = this._shown(traced);
    return {lines: shown.length, traced: traced.length,
            hidden: traced.length - shown.length,
            vmin: this.vmin, vmax: this.vmax,
            vpeak, vfs: this.vfs, cpLo: this.cpObserved.lo,
            cpHi: this.cpObserved.hi, timing: T};
  }

  /* ------------------------------------------------------ what changed */

  /* Score one traced line against the flow it would have been in if the
   * machine were not there: a straight ray from the seed along the
   * freestream, everywhere at freestream speed.
   *
   *   dv  the largest fractional change in speed anywhere along the line
   *   dn  the largest lateral departure from that ray, as a fraction of
   *       model length
   *
   * Speed alone would miss a line bent around the body without slowing;
   * deflection alone would miss one that slows to a stop dead ahead of a
   * stagnation point without ever leaving the ray. A line is interesting if
   * either says so, and a line swallowed by an intake always is -- ending at
   * the lip is the whole thing it has to say.
   */
  _score(L){
    const v = this.vinf || [1, 0, 0];
    const vfs = Math.hypot(v[0], v[1], v[2]) || 1;
    const ux = v[0]/vfs, uy = v[1]/vfs, uz = v[2]/vfs;
    const p = L.pts, n = L.spd.length;
    const x0 = p[0], y0 = p[1], z0 = p[2];
    let dv = 0, dn = 0;
    for(let i = 0; i < n; i++){
      const ds = Math.abs(L.spd[i] - vfs);
      if(ds > dv) dv = ds;
      const rx = p[i*3] - x0, ry = p[i*3+1] - y0, rz = p[i*3+2] - z0;
      const t = rx*ux + ry*uy + rz*uz;
      const ex = rx - t*ux, ey = ry - t*uy, ez = rz - t*uz;
      const e = Math.hypot(ex, ey, ez);
      if(e > dn) dn = e;
    }
    L.dv = dv/vfs;
    L.dn = dn/Math.max(this.domain.L, 1e-6);
    /* What makes a line worth drawing: it came near the aeroplane.
     *
     * The speed and deflection thresholds were set against a field that
     * spiked to three times freestream near the skin, where "this line was
     * changed" was a thing you could measure a long way off. A solved panel
     * body has no such spikes -- nothing in it exceeds 1.17 times freestream
     * -- so a threshold on the change keeps a wall of lines that pass a
     * diagonal away and were bent by half a per cent. A rake wide enough to
     * cover the model at any attitude fires most of its lines into clear
     * air, and clear air drawn in the same ink as the answer is the grid.
     *
     * Proximity says it directly, and the tracer already knows it: the inside
     * test runs every step and reports the distance to the nearest panel, so
     * the closest approach costs nothing to keep.
     */
    const near = isFinite(L.near) ? L.near : Infinity;
    L.disturbed = !!L.captured
      || near < this.disturb.near * this.domain.diag
      || L.dv >= this.disturb.speed || L.dn >= this.disturb.deflect;
    return L;
  }

  /* The lines actually drawn. Everything downstream -- colour ranges, ribbons,
     arrows -- reads this rather than the full traced set, so the scale keys
     the picture on screen and not on lines nobody can see. */
  _shown(traced){
    if(this.show.undisturbed) return traced;
    const out = [];
    for(const L of traced) if(L.disturbed) out.push(L);
    return out.length ? out : traced;
  }

  /* ---------------------------------------------------------------- ribbons */

  _drawRibbons(traced, progress = 1){
    traced = this._shown(traced);
    let verts = 0, tris = 0;
    for(const L of traced){
      const n = L.spd.length;
      verts += n*2;
      tris += (n - 1)*2;
    }
    const pos = new Float32Array(verts*3);
    const tan = new Float32Array(verts*3);
    const col = new Float32Array(verts*3);
    const side = new Float32Array(verts);
    const arc = new Float32Array(verts);
    const spdN = new Float32Array(verts);
    const idx = new Uint32Array(tris*3);

    const rgb = [0,0,0];
    const sc = 1/Math.max(this.vmax - this.vmin, 1e-6);
    let vi = 0, ii = 0;
    for(const L of traced){
      const n = L.spd.length;
      let s = 0;
      const base = vi;
      for(let i = 0; i < n; i++){
        // tangent from the neighbours, so the ribbon does not kink
        const a = Math.max(0, i - 1), b = Math.min(n - 1, i + 1);
        let tx = L.pts[b*3] - L.pts[a*3];
        let ty = L.pts[b*3+1] - L.pts[a*3+1];
        let tz = L.pts[b*3+2] - L.pts[a*3+2];
        const tm = Math.hypot(tx, ty, tz) || 1;
        tx /= tm; ty /= tm; tz /= tm;
        if(i > 0){
          s += Math.hypot(L.pts[i*3] - L.pts[(i-1)*3],
                          L.pts[i*3+1] - L.pts[(i-1)*3+1],
                          L.pts[i*3+2] - L.pts[(i-1)*3+2]);
        }
        speedColour((L.spd[i] - this.vmin)*sc, rgb);
        const P = toScene(L.pts[i*3], L.pts[i*3+1], L.pts[i*3+2]);
        const T = toScene(tx, ty, tz);
        for(const sg of [-1, 1]){
          pos[vi*3] = P[0]; pos[vi*3+1] = P[1]; pos[vi*3+2] = P[2];
          tan[vi*3] = T[0]; tan[vi*3+1] = T[1]; tan[vi*3+2] = T[2];
          col[vi*3] = rgb[0]; col[vi*3+1] = rgb[1]; col[vi*3+2] = rgb[2];
          side[vi] = sg;
          arc[vi] = s;
          spdN[vi] = L.spd[i]/this.vfs;
          vi++;
        }
      }
      for(let i = 0; i < n - 1; i++){
        const a = base + i*2, b = a + 2;
        idx[ii++] = a;   idx[ii++] = a+1; idx[ii++] = b+1;
        idx[ii++] = a;   idx[ii++] = b+1; idx[ii++] = b;
      }
    }

    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('aTan', new THREE.BufferAttribute(tan, 3));
    g.setAttribute('aCol', new THREE.BufferAttribute(col, 3));
    g.setAttribute('aSide', new THREE.BufferAttribute(side, 1));
    g.setAttribute('aArc', new THREE.BufferAttribute(arc, 1));
    g.setAttribute('aSpd', new THREE.BufferAttribute(spdN, 1));
    g.setIndex(new THREE.BufferAttribute(idx, 1));

    /* Ribbon width has to fall as the line count rises.
     *
     * The seeding went from 170 lines to 420 without the width moving, so
     * two and a half times as much ink went onto the same picture and the
     * field closed up into a mass you cannot see the car through. Total ink
     * is roughly lines x width, so holding that constant means scaling by
     * sqrt(170/n) -- and a floor, because a ribbon thinner than a pixel
     * stops reading as a ribbon and starts flickering.
     */
    /* Scaled by the RIBBON actually drawn, not by the number of lines.
     *
     * Ink on the picture is length times width, so the thing to hold constant
     * is length times width -- and the length is not a property of the line
     * count. Two changes broke that assumption in turn: the proximity filter,
     * which made the drawn count vary between 90 and 430 with attitude while
     * the seeded count stayed at 560; and walking each line upstream as well
     * as down, which doubled every line's length at a stroke and doubled the
     * ink without changing the count at all. So it is measured: total arc
     * length, in model lengths, against what read well before.
     */
    let drawn = 0;
    for(const L of traced){
      const n = L.spd.length;
      for(let i = 1; i < n; i++)
        drawn += Math.hypot(L.pts[i*3] - L.pts[(i-1)*3],
                            L.pts[i*3+1] - L.pts[(i-1)*3+1],
                            L.pts[i*3+2] - L.pts[(i-1)*3+2]);
    }
    /* Scaled to the FINISHED picture, not to the part of it drawn so far.
     *
     * Tracing runs progressively -- a slice of lines per frame, redrawn as
     * they arrive -- and the ribbons were being re-sized on each partial set.
     * At four lines in, `ink` came out at 6.3 and the screen filled with
     * ribbons fifteen millimetres wide on a 440 mm aeroplane. It settled to
     * 0.9 by the end, so the finished picture was right and every slider
     * move flashed a wall of green on the way there. Dividing by the progress
     * fraction estimates what the total will be, which is stable from the
     * first slice because every line is about as long as every other. */
    const est = drawn / Math.max(progress, 0.08);
    const ink = Math.sqrt(460 / Math.max(est / Math.max(this.domain.L, 1e-6), 1));
    /* The floor is a fraction of the model, not a number of metres.
     *
     * It used to be 0.0013 m, chosen on a car 4.98 m long. On a 0.44 m model
     * that floor is what sets the width, so the ribbons came out 3.4 times
     * fatter relative to their subject than the same code drew on the car --
     * which is most of why one of these two reads clearly and the other does
     * not. The floor only exists to stop a sub-pixel ribbon flickering, so it
     * belongs in the model's own units. */
    const width = Math.max(this.domain.L * 0.0005, this.domain.L * 0.0016 * ink);
    /* Opacity falls with the count too.
     *
     * Width alone is not enough: a hundred thin opaque ribbons stacked along
     * the line of sight are still opaque, and the model disappears behind
     * its own flow field. Letting each line be more transparent as the
     * seeding gets denser keeps the total the eye receives about constant,
     * so raising the line count buys resolution instead of fog. */
    const alpha = Math.max(0.30, Math.min(0.96, 0.96 * ink));
    const mat = new THREE.ShaderMaterial({
      uniforms: {
        uTime:  {value: 0},
        uWidth: {value: width},
        uDash:  {value: this.anim.dash},
        uRate:  {value: this.anim.rate},
        uPulse: {value: this.anim.on ? this.anim.pulse : 0},
        uOpacity: {value: alpha},
        uFogNear: {value: this.domain.L * 1.3},
        uFogFar:  {value: this.domain.L * 3.6},
      },
      vertexShader: `
        attribute vec3 aTan;
        attribute vec3 aCol;
        attribute float aSide;
        attribute float aArc;
        attribute float aSpd;
        uniform float uWidth;
        varying vec3 vCol;
        varying float vArc;
        varying float vSpd;
        varying float vEdge;
        varying float vDepth;
        void main(){
          vec4 mv = modelViewMatrix * vec4(position, 1.0);
          vec3 t = normalize((modelViewMatrix * vec4(aTan, 0.0)).xyz);
          vec3 look = normalize(-mv.xyz);
          vec3 off = cross(t, look);
          float m = length(off);
          // a ribbon seen exactly edge-on has no width to expand into; fall
          // back to any perpendicular so it never collapses to nothing
          off = m > 1e-4 ? off/m : normalize(cross(t, vec3(0.0, 0.0, 1.0)));
          mv.xyz += off * aSide * uWidth;
          vCol = aCol; vArc = aArc; vSpd = aSpd;
          vEdge = abs(aSide); vDepth = -mv.z;
          gl_Position = projectionMatrix * mv;
        }`,
      fragmentShader: `
        uniform float uTime, uDash, uRate, uPulse, uOpacity, uFogNear, uFogFar;
        varying vec3 vCol;
        varying float vArc;
        varying float vSpd;
        varying float vEdge;
        varying float vDepth;
        void main(){
          // The pulse travels downstream at the local flow speed, so fast air
          // visibly moves faster than slow air and a stagnation region sits
          // almost still. That is the direction cue the first version had no
          // way of giving.
          float ph = fract(vArc/uDash - uTime*uRate*max(vSpd, 0.12));
          float head = smoothstep(0.0, 0.14, ph) * (1.0 - smoothstep(0.14, 0.85, ph));
          /* The trough stays bright enough to be a line.
           *
           * At 0.34 the pulse spends about seventy per cent of its period
           * nearly invisible, so with many pulses on a line the line reads as
           * a row of dots and with few it reads as one comet and a gap. The
           * point of the pulse is to show WHICH WAY and HOW FAST the air is
           * going, and a travelling brightening does that without the line
           * having to disappear between heads. */
          float b = mix(1.0, 0.72 + 0.85*head, uPulse);
          // Fake the round section: the middle of the ribbon reads as the lit
          // crown of a tube and the edges fall away, so a dense field of lines
          // reads as depth instead of as flat paint.
          b *= 1.18 - 0.55*vEdge*vEdge;
          // Distance fade keeps the far side of the model from fighting the
          // near side for attention -- the single biggest readability win when
          // the line count goes up.
          b *= mix(1.0, 0.22, smoothstep(uFogNear, uFogFar, vDepth));
          gl_FragColor = vec4(vCol*b, uOpacity * mix(1.0, 0.55, smoothstep(uFogNear, uFogFar, vDepth)));
        }`,
      transparent: true,
      depthWrite: false,
      side: THREE.DoubleSide,
    });

    if(this.lines){
      this.group.remove(this.lines);
      this.lines.geometry.dispose();
      this.lines.material.dispose();
    }
    this.lines = new THREE.Mesh(g, mat);
    this.lines.frustumCulled = false;
    this.lines.raycast = () => {};
    this.lines.visible = this.show.ribbons;
    this.group.add(this.lines);
  }

  /* ----------------------------------------------------------------- arrows */

  /* Cone glyphs along each streamline. The animation carries direction while
   * it is running; these carry it when it is paused, in a screenshot, or for
   * anyone who has motion reduced. */
  _drawArrows(traced){
    traced = this._shown(traced);
    /* A fixed budget of arrows, not a fixed spacing along each line.
     *
     * At a stride of 22 the arrow count rose with the line count, so seeding
     * more densely produced thousands of cones and the glyphs became the
     * picture -- a swarm with a model somewhere inside it. The job of an
     * arrow here is only to say which way the flow goes when the animation
     * is paused, and about four hundred of them does that at any density.
     */
    const BUDGET = 230;
    let total = 0;
    for(const L of traced) total += Math.max(0, L.spd.length - 8);
    const stride = Math.max(18, Math.round(total / Math.max(BUDGET, 1)));
    const picks = [];
    for(const L of traced){
      const n = L.spd.length;
      for(let i = 6; i < n - 2; i += stride) picks.push([L, i]);
    }
    if(this.arrows){
      this.group.remove(this.arrows);
      this.arrows.geometry.dispose();
      this.arrows.material.dispose();
      this.arrows = null;
    }
    if(!picks.length) return;

    // Smaller than they were. A direction glyph has to be readable, not
    // dominant: at 0.0022 of model length they were wider than the ribbons
    // they sit on and the picture became a field of cones with an aircraft
    // somewhere inside it.
    // same scale trap: 0.0022 m was 1.3 per mille of the car and 5.0 of the
    // aeroplane, so the glyphs were four times the size relative to the model
    const r = Math.max(this.domain.L * 0.0004, this.domain.L * 0.0013);
    const geo = new THREE.ConeGeometry(r, r*2.8, 7, 1);
    geo.translate(0, -r*0.4, 0);          // pivot nearer the base
    const mat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.72, depthWrite: false,
    });
    const inst = new THREE.InstancedMesh(geo, mat, picks.length);
    inst.frustumCulled = false;
    inst.raycast = () => {};

    const m4 = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const up = new THREE.Vector3(0, 1, 0);
    const dir = new THREE.Vector3();
    const at = new THREE.Vector3();
    const one = new THREE.Vector3(1, 1, 1);
    const rgb = [0,0,0];
    const sc = 1/Math.max(this.vmax - this.vmin, 1e-6);
    const colours = new Float32Array(picks.length*3);
    picks.forEach(([L, i], k) => {
      const a = i, b = Math.min(L.spd.length - 1, i + 1);
      const D = toScene(L.pts[b*3] - L.pts[a*3],
                        L.pts[b*3+1] - L.pts[a*3+1],
                        L.pts[b*3+2] - L.pts[a*3+2]);
      dir.set(D[0], D[1], D[2]).normalize();
      q.setFromUnitVectors(up, dir);
      const P = toScene(L.pts[a*3], L.pts[a*3+1], L.pts[a*3+2]);
      at.set(P[0], P[1], P[2]);
      m4.compose(at, q, one);
      inst.setMatrixAt(k, m4);
      speedColour((L.spd[a] - this.vmin)*sc, rgb);
      colours[k*3] = rgb[0]; colours[k*3+1] = rgb[1]; colours[k*3+2] = rgb[2];
    });
    inst.instanceMatrix.needsUpdate = true;
    inst.geometry.setAttribute(
      'color', new THREE.InstancedBufferAttribute(colours, 3));
    inst.visible = this.show.arrows;
    this.arrows = inst;
    this.group.add(inst);
  }

  /* ----------------------------------------------------------- cutting plane */

  /* A plane through the flow, filled with colour.
   *
   * This is the view a post-processor opens with, and the one that was missing
   * entirely: streamlines show you where air went, a filled plane shows you
   * the whole field at once -- the stagnation ahead of the nose, the
   * acceleration over the shoulder, the low-energy wake behind. Sampled on a
   * grid and coloured per vertex, which is enough at this resolution and costs
   * one pass.
   */
  _drawPlane(axis){
    const ax = axis || this.planeAxis || 'y';
    this.planeAxis = ax;
    const {x0, x1, z0, z1} = this.ext;
    const L = x1 - x0;
    const zc = 0.5*(z0 + z1);
    const nu = 150, nv = 96;
    const u0 = x0 - L*0.5, u1 = x1 + L*0.9;
    const vSpan = this.domain.spanZ * 0.95;
    const v0 = this.cfg.ground ? 0.004 : zc - vSpan*0.5;
    const v1 = this.cfg.ground ? v0 + vSpan : zc + vSpan*0.5;

    const pos = new Float32Array(nu*nv*3);
    const col = new Float32Array(nu*nv*3);
    const idx = [];
    const vel = this.vel, out = [0,0,0], rgb = [0,0,0];
    const sc = 1/Math.max(this.vmax - this.vmin, 1e-6);
    for(let j = 0; j < nv; j++){
      for(let i = 0; i < nu; i++){
        const k = j*nu + i;
        const u = u0 + (u1 - u0)*i/(nu - 1);
        const w = v0 + (v1 - v0)*j/(nv - 1);
        let sx, sy, sz;
        if(ax === 'y'){ sx = u; sy = 0.0; sz = w; }
        else { sx = u; sy = w - (v0 + v1)*0.5; sz = zc; }
        out[0] = out[1] = out[2] = 0;
        vel([sx, sy, sz], out);
        const m = Math.hypot(out[0], out[1], out[2]);
        const inBody = this.body.inside([sx, sy, sz]);
        speedColour((m - this.vmin)*sc, rgb);
        const P = toScene(sx, sy, sz);
        pos[k*3] = P[0]; pos[k*3+1] = P[1]; pos[k*3+2] = P[2];
        // inside the body there is no flow; grey it out rather than show the
        // meaningless field the panel model produces in there
        const f = inBody ? 0.12 : 1.0;
        col[k*3] = rgb[0]*f + (inBody ? 0.10 : 0);
        col[k*3+1] = rgb[1]*f + (inBody ? 0.11 : 0);
        col[k*3+2] = rgb[2]*f + (inBody ? 0.12 : 0);
      }
    }
    for(let j = 0; j < nv - 1; j++){
      for(let i = 0; i < nu - 1; i++){
        const a = j*nu + i, b = a + 1, c = a + nu, d = c + 1;
        idx.push(a, c, d, a, d, b);
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    g.setIndex(idx);
    const mat = new THREE.MeshBasicMaterial({
      vertexColors: true, side: THREE.DoubleSide,
      transparent: true, opacity: 0.9, depthWrite: false,
    });
    if(this.plane){
      this.group.remove(this.plane);
      this.plane.geometry.dispose();
      this.plane.material.dispose();
    }
    this.plane = new THREE.Mesh(g, mat);
    this.plane.frustumCulled = false;
    this.plane.raycast = () => {};
    this.plane.visible = this.show.plane;
    this.group.add(this.plane);
  }

  /* --------------------------------------------------------- surface pressure */

  /* Paint Cp onto the real geometry.
   *
   * Each model vertex is put back into the solver frame, the nearest solved
   * panel is found through a grid index, and its Cp becomes a vertex colour on
   * a diverging map. Blue is suction and red is above ambient, so the
   * underfloor, the wing undersides and the diffuser go blue and the nose and
   * the wing leading edges go red -- which is the whole of the car's aero in
   * one picture.
   *
   * The original material is kept so `clearSurface` can put it back.
   */
  paintSurface(meshes){
    if(!this.cp) return 0;
    if(meshes) this._meshes = meshes;
    let list = this._meshes || [];
    /* Only the surfaces the air touches.
     *
     * Painting every mesh meant painting the engine, the gearbox and the whole
     * power unit -- 437,000 of the car's 741,000 vertices, all of it under the
     * bodywork where no air goes and nobody can see it. Pressure belongs on
     * the wetted surface, so anything whose nearest panel is further away than
     * a panel is wide gets skipped: that is the test for "the air never
     * touched this".
     */
    if(!this.index) this.index = panelIndex(this.body, this.domain.diag*0.07);
    const reach = this.domain.diag * 0.10;
    /* Into the flow group's frame, not the world's.
     *
     * The streamlines are built in solver coordinates and added to this group,
     * so the group's local space IS the solver frame (mapped Y-up). The model
     * root is translated, so a vertex's world position is that translation away
     * from anything the panels know about -- which made every vertex look
     * infinitely far from every panel, the mesh filter reject all 219 meshes,
     * and the pressure map silently paint nothing at all.
     */
    this.group.updateWorldMatrix(true, false);
    const toGroup = new THREE.Matrix4().copy(this.group.matrixWorld).invert();
    const wp0 = new THREE.Vector3();
    list = list.filter(m => {
      const p = m.geometry && m.geometry.getAttribute('position');
      if(!p || !p.count) return false;
      // sample a few vertices rather than all of them
      let near = 0, tries = 0;
      m.updateWorldMatrix(true, false);
      for(let k = 0; k < p.count && tries < 12; k += Math.max(1, p.count >> 3)){
        tries++;
        wp0.set(p.getX(k), p.getY(k), p.getZ(k))
           .applyMatrix4(m.matrixWorld).applyMatrix4(toGroup);
        const S = toSolver(wp0.x, wp0.y, wp0.z);
        const j = this.index.nearest(S[0], S[1], S[2]);
        if(j < 0) continue;
        const d = Math.hypot(S[0] - this.body.c[j*3], S[1] - this.body.c[j*3+1],
                             S[2] - this.body.c[j*3+2]);
        if(d < reach) near++;
      }
      return near * 2 >= tries;
    });
    this.clearSurface();
    const lo = this.cpRange.lo, hi = this.cpRange.hi;
    const rgb = [0,0,0];
    const wp = new THREE.Vector3();
    let painted = 0, verts = 0;
    const t0 = performance.now();
    for(const m of list){
      const g = m.geometry;
      const p = g.getAttribute('position');
      if(!p) continue;
      const n = p.count;
      const cols = new Float32Array(n*3);
      m.updateWorldMatrix(true, false);
      for(let i = 0; i < n; i++){
        wp.set(p.getX(i), p.getY(i), p.getZ(i))
          .applyMatrix4(m.matrixWorld).applyMatrix4(toGroup);
        const S = toSolver(wp.x, wp.y, wp.z);
        const j = this.index.nearest(S[0], S[1], S[2]);
        const cp = j >= 0 ? this.cp[j] : 0;
        pressureColour(cpNorm(cp, lo, hi), rgb);
        cols[i*3] = rgb[0]; cols[i*3+1] = rgb[1]; cols[i*3+2] = rgb[2];
      }
      g.setAttribute('color', new THREE.BufferAttribute(cols, 3));
      /* Unlit.
       *
       * The first version used a lit standard material, and the scene lighting
       * multiplied every vertex colour down towards grey -- so a car that was
       * genuinely blue under the floor and red on the nose rendered as two
       * shades of nothing. On a contour plot the colour IS the datum: it must
       * not be modulated by where the lights happen to be. This is why every
       * post-processor draws its pressure surfaces flat.
       */
      const mat = new THREE.MeshBasicMaterial({vertexColors: true});
      this.painted.push({mesh: m, material: m.material});
      m.material = mat;
      painted++;
      verts += n;
    }
    this.show.surface = true;
    this.paintStats = {meshes: painted, verts,
                       ms: performance.now() - t0};
    return painted;
  }

  clearSurface(){
    for(const {mesh, material} of this.painted){
      if(mesh.material && mesh.material !== material) mesh.material.dispose();
      mesh.material = material;
      if(mesh.geometry.getAttribute('color')) mesh.geometry.deleteAttribute('color');
    }
    this.painted = [];
  }

  /* -------------------------------------------------------------- the clock */

  /* Advance the travelling pulse. Called from the render loop; costs one
   * uniform write, which is why the flow can move without retracing. */
  tick(dt){
    this.clock += dt;
    if(this.lines) this.lines.material.uniforms.uTime.value = this.clock;
  }

  setAnim(on){
    this.anim.on = !!on;
    if(this.lines){
      this.lines.material.uniforms.uPulse.value = on ? this.anim.pulse : 0;
    }
  }

  setRate(r){
    this.anim.rate = r;
    if(this.lines) this.lines.material.uniforms.uRate.value = r;
  }

  /* Layers fade rather than pop.
   *
   * Two hundred streamlines appearing between one frame and the next reads as
   * a glitch, not as a layer arriving -- and with four layers available, a
   * pop gives you no clue which one just changed. A third of a second of fade
   * does. */
  setLayer(which, on){
    this.show[which] = !!on;
    if(which === 'undisturbed'){
      // a filter, not a layer: the lines are already traced, so this is a
      // redraw at the current solve rather than another second of tracing
      if(this.traced){
        this._setRanges(this.traced, this.vinf, null);
        this._drawRibbons(this.traced);
        this._drawArrows(this.traced);
      }
      return;
    }
    if(which === 'ribbons' && this.lines) fadeObject(this.lines, on, {max: 0.96});
    if(which === 'arrows' && this.arrows) fadeObject(this.arrows, on, {max: 0.95});
    if(which === 'plane'){
      if(on && !this.plane && this.vel) this._drawPlane();
      if(this.plane) fadeObject(this.plane, on, {max: 0.9});
    }
    if(which === 'surface'){
      if(on) this.paintSurface();
      else { this.clearSurface(); this.show.surface = false; }
    }
  }

  setVisible(on){
    this.group.visible = on;
    if(!on && this.show.surface) this.clearSurface();
  }

  dispose(){
    this.clearSurface();
    for(const k of ['lines', 'arrows', 'plane']){
      const o = this[k];
      if(o){
        this.group.remove(o);
        o.geometry.dispose();
        o.material.dispose();
        this[k] = null;
      }
    }
  }
}

/* ------------------------------------------------------------------ legend */

/* A colour bar with real numbers on it, and a mark showing where freestream
 * sits, because "is this air faster or slower than the wind" is the question
 * every colour on the bar is answering. The first version had ticks and a
 * label and no reference, so a colour told you a number and the number told
 * you nothing. */
export function drawLegend(canvas, lo, hi, unitLabel, opts){
  const {ref = null, refLabel = '', map = speedColour,
         zeroAt = null} = opts || {};
  const g = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  g.clearRect(0, 0, W, H);
  const barH = 13, top = 17, left = 4, right = W - 4;
  const rgb = [0,0,0];
  for(let x = left; x < right; x++){
    map((x - left)/(right - left), rgb);
    g.fillStyle = `rgb(${Math.round(rgb[0]*255)},${Math.round(rgb[1]*255)},${Math.round(rgb[2]*255)})`;
    g.fillRect(x, top, 1, barH);
  }
  g.strokeStyle = '#2C383E';
  g.strokeRect(left, top, right - left, barH);
  g.fillStyle = '#96A3AB';
  g.font = '10px ui-monospace, monospace';
  g.textBaseline = 'top';
  g.fillText(unitLabel, left, 2);
  // the unit label owns this much of the top row; a mark's label starts after
  // it, or the two are drawn one on top of the other -- which is how the car's
  // bar came to read "VelocVity [km/h]" whenever the freestream sat near the
  // bottom of a clamped range
  const unitEnd = left + g.measureText(unitLabel).width + 9;

  const xOf = (v) => left + (right - left)*(v - lo)/Math.max(hi - lo, 1e-9);

  // the reference mark: freestream on a speed bar, Cp = 0 on a pressure bar
  const marks = [];
  if(ref !== null && ref > lo && ref < hi) marks.push([ref, refLabel || 'V∞']);
  if(zeroAt !== null && zeroAt > lo && zeroAt < hi) marks.push([zeroAt, 'Cp 0']);
  for(const [v, label] of marks){
    const x = Math.round(xOf(v));
    g.fillStyle = '#EAF2F6';
    g.fillRect(x, top - 4, 1, barH + 8);
    g.textBaseline = 'top';
    const tw = g.measureText(label).width;
    g.fillStyle = '#EAF2F6';
    g.fillText(label,
               Math.max(unitEnd, Math.min(x - tw/2, right - tw)), 2);
  }

  g.textBaseline = 'alphabetic';
  for(let i = 0; i <= 4; i++){
    const x = left + (right - left) * i/4;
    const v = lo + (hi - lo) * i/4;
    g.fillStyle = '#2C383E';
    g.fillRect(Math.min(x, right-1), top + barH, 1, 3);
    g.fillStyle = '#96A3AB';
    const t = Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(v === Math.round(v) ? 1 : 2);
    const tw = g.measureText(t).width;
    g.fillText(t, Math.max(left, Math.min(x - tw/2, right - tw)), top + barH + 13);
  }
}

export { speedColour, pressureColour };
