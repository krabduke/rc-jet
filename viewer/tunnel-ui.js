/* Wind tunnel: the controls, the instruments and the smoke.
 *
 * Everything on screen comes out of the solve in windtunnel.js. The smoke
 * filaments are advected by the freestream plus the velocity the solved vortex
 * system induces on them, so they bend around the model because the
 * circulation is there. The spanwise load plot is the panel forces. Move a
 * control surface and every one of them changes, because the solver is re-run,
 * not because a curve was faked.
 */

import * as THREE from 'three';
import { Tunnel } from './windtunnel.js';
import { CFDView, drawLegend } from './cfd-view.js';
import { tweenNumber, revealRows } from './anim.js';

const MM = 0.001;

/* A smoke rake, as a real tunnel has: a row of nozzles upstream releasing
 * filaments into the flow. Seeding uniformly through the volume looks like
 * fog and shows nothing; a rake shows exactly where the flow goes. */
const RAKE_ROWS = 7;
const RAKE_COLS = 13;
const TRAIL = 26;          // points kept per filament

export class WindTunnel {
  constructor(opts){
    this.cfg = opts.cfg;                 // data.tunnel
    this.scene = opts.scene;
    this.root = opts.root;               // the loaded model root
    this.parts = opts.parts;             // name -> THREE.Object3D
    this.manifest = opts.manifest;       // data.parts (for pivots)
    this.bounds = opts.bounds;           // THREE.Box3 of the model, metres
    this.onSolve = opts.onSolve || (() => {});
    // A callback returning the current flow ports (intake sinks, exhaust and
    // fan sources) in solver frame, metres. The viewer's throttle and fan
    // state feed it; the tunnel re-solves through them.
    this.portsFn = opts.ports || null;
    this.solver = new Tunnel(this.cfg);
    this.slowmo = opts.slowmo || 0.02;

    this.state = {
      on: false,
      v: this.cfg.v_default,
      alpha: this.cfg.alpha_default,
      beta: this.cfg.beta_default || 0,
      controls: {},
      smoke: true,
    };
    for(const c of this.cfg.controls) this.state.controls[c.id] = c.value;

    this._buildTunnelBox();
    if(opts.bodyPanels){
      this.cfd = new CFDView({
        cfg: this.cfg, root: this.root, bounds: this.bounds,
        bodyPanels: opts.bodyPanels, nLines: opts.nLines || 560,
      });
    }
    this.group.visible = false;
    // Open at the incidence that actually flies, so the first thing on screen
    // is the aircraft in trim. A car has no such condition -- it is pressed
    // down, not held up -- so it opens at its ride attitude instead.
    if(this.cfg.kind !== 'car' && this.cfg.mass_kg)
      this.state.alpha = this.trimAlpha();
    this.solve();
  }

  /* The incidence at which lift equals weight, at the current speed and
   * control positions. Two solves and a straight line: CL is linear in alpha
   * in a lattice, so there is nothing to iterate. */
  trimAlpha(){
    const W = this.cfg.mass_kg * 9.81;
    const q = 0.5 * 1.225 * this.state.v * this.state.v;
    const need = W / (q * this.cfg.s_ref);
    const o = {v: this.state.v, controls: this.state.controls,
               ground: !!this.cfg.ground};
    const c0 = this.solver.solve({...o, alpha: 0}).CL;
    const c4 = this.solver.solve({...o, alpha: 4}).CL;
    const slope = (c4 - c0) / 4;
    if(Math.abs(slope) < 1e-6) return this.cfg.alpha_default;
    const a = (need - c0) / slope;
    return Math.max(this.cfg.alpha_min, Math.min(this.cfg.alpha_max, a));
  }

  /* ------------------------------------------------------------ geometry */

  _buildTunnelBox(){
    // A working section: floor grid and a light cage, so the model reads as
    // being inside something rather than floating.
    const g = new THREE.Group();
    const b = this.bounds;
    const len = (b.max.x - b.min.x) * 2.6;
    const wid = Math.max((b.max.z - b.min.z), (b.max.x - b.min.x) * 0.7) * 2.2;
    const hgt = Math.max((b.max.y - b.min.y) * 2.6, wid * 0.5);
    const cx = (b.max.x + b.min.x) / 2, cz = (b.max.z + b.min.z) / 2;
    const floorY = b.min.y - hgt * 0.22;

    const grid = new THREE.GridHelper(Math.max(len, wid), 26, 0x3A474D, 0x2A353A);
    grid.position.set(cx, floorY, cz);
    g.add(grid);

    const cage = new THREE.Box3Helper(
      new THREE.Box3(
        new THREE.Vector3(cx - len/2, floorY, cz - wid/2),
        new THREE.Vector3(cx + len/2, floorY + hgt, cz + wid/2)),
      0x3F4E55);
    cage.material.transparent = true;
    cage.material.opacity = 0.5;
    g.add(cage);

    this.box = {len, wid, hgt, cx, cz, floorY};
    this.group = new THREE.Group();
    this.group.add(g);
    this.scene.add(this.group);
  }

  /* ------------------------------------------------------------- solving */

  solve(){
    const s = this.state;
    const o = {v: s.v, beta: s.beta, controls: s.controls,
               ground: !!this.cfg.ground};
    this.np = this.solver.neutralPoint(o);
    // neutralPoint leaves the solver at alpha 4; re-solve at the real one
    this.sol = this.solver.solve({...o, alpha: s.alpha});
    this.onSolve(this.report());
    return this.sol;
  }

  report(){
    const s = this.sol, st = this.state;
    const W = this.cfg.mass_kg * 9.81;
    return {
      CL: s.CL, CDi: s.CDi, LD: s.LD,
      lift_N: s.lift_N, drag_N: s.drag_N,
      liftFrac: s.lift_N / W,
      np: this.np, cg: this.cfg.cg_frac,
      margin: this.np - this.cfg.cg_frac,
      alpha: st.alpha, beta: st.beta, v: st.v,
      side_N: s.side_N, CY: s.CY,
      stalled: st.alpha > this.cfg.stall_alpha,
      stall_alpha: this.cfg.stall_alpha,
      strips: s.strips,
      panels: s.panels,
      bySurface: s.bySurface,
    };
  }

  /* ---------------------------------------------------------- appearance */

  setOn(on){
    this.state.on = on;
    this.group.visible = on;
    if(this.cfd) this.cfd.setVisible(on);
    if(on) this.solve();
    this.applyAttitude();
  }

  set(key, value){
    if(key === 'v' || key === 'alpha' || key === 'beta') this.state[key] = value;
    else this.state.controls[key] = value;
    this.solve();
    this.applyAttitude();
  }

  /* Pitch the model to the solved incidence and put every control surface
   * where its slider says. The hinge comes from the manifest, so a surface
   * turns about its own hinge line rather than about the model origin. */
  /* The wind blows from one fixed direction and the model is held at an
   * attitude in it -- which is what a tunnel does, and what makes rotating
   * the model change the aerodynamics rather than just the view. Pitch is
   * alpha, yaw is sideslip, and the solver is given the same pair. */
  applyAttitude(){
    if(this.root){
      this.root.rotation.z = this.state.on
        ? -this.state.alpha * Math.PI/180 : 0;
      this.root.rotation.y = this.state.on
        ? this.state.beta * Math.PI/180 : 0;
    }
    for(const c of this.cfg.controls){
      // rotate by the difference from the angle the mesh was built at
      const baked = c.baked || 0;
      const want = this.state.on ? this.state.controls[c.id] : baked;
      const ang = (want - baked) * Math.PI/180;
      c.objects.forEach((name, i) => {
        const o = this.parts.get(name);
        const m = this.manifest[name];
        if(!o || !m || !m.pivot) return;
        const sign = (c.sign && c.sign[i] !== undefined) ? c.sign[i] : 1;
        const ax = m.pivot.axis;
        // model axes are Blender's: x aft, y span, z up. The GLB is exported
        // Y-up, so Blender z -> three y and Blender y -> three -z.
        o.rotation.set(0, 0, 0);
        if(Math.abs(ax[2]) > 0.5) o.rotation.y = ang * sign;      // vertical hinge
        else o.rotation.z = -ang * sign;                          // spanwise hinge
      });
    }
  }

  /* Retrace the streamlines through the freshly solved field.
   *
   * It costs a few hundred milliseconds, so it is not done while a slider is
   * moving: the numbers update on every input, the flow picture updates when
   * the slider is released. The picture itself is not static once it is drawn
   * -- the pulse travelling along each line runs off the render clock, so the
   * flow keeps moving between retraces for the cost of one uniform write.
   */
  retrace(){
    if(!this.cfg || !this.cfd || !this.state.on) return null;
    const a = this.state.alpha * Math.PI/180;
    const v = this.state.v;
    const b = this.state.beta * Math.PI/180;
    const vinf = [Math.cos(a)*Math.cos(b)*v, Math.sin(b)*v, Math.sin(a)*Math.cos(b)*v];
    const lat = this.solver.inducedVelocity();
    const ports = this.portsFn ? this.portsFn(this.state) : null;
    const t0 = performance.now();
    const r = this.cfd.run(vinf, lat, ports);
    return r ? {...r, ms: performance.now() - t0} : null;
  }

  /* The same retrace, spread across frames.
   *
   * The field costs about 45 microseconds per evaluation -- nearly all of it
   * the vortex lattice -- and a full set of lines needs of the order of a
   * hundred thousand of them. Done in one call that is a multi-second frozen
   * tab. Done in slices it is the same arithmetic with the page still
   * responding and the streamlines appearing as they are traced.
   */
  async retraceProgressive(onProgress){
    if(!this.cfg || !this.cfd || !this.state.on) return null;
    const a = this.state.alpha * Math.PI/180;
    const v = this.state.v;
    const b = this.state.beta * Math.PI/180;
    const vinf = [Math.cos(a)*Math.cos(b)*v, Math.sin(b)*v, Math.sin(a)*Math.cos(b)*v];
    const lat = this.solver.inducedVelocity();
    const ports = this.portsFn ? this.portsFn(this.state) : null;
    const t0 = performance.now();
    const r = await this.cfd.runProgressive(vinf, lat, onProgress, {ports});
    return r ? {...r, ms: performance.now() - t0} : null;
  }

  /* Advance the travelling pulse. This used to be a no-op with a comment
   * saying streamlines are steady and there was nothing to animate -- which
   * is true of the geometry and beside the point: a wind tunnel where you
   * cannot see which way the air moves is not showing you the air. */
  update(dt){ if(this.cfd) this.cfd.tick(dt); }

  /* Which layers are drawn: streamlines, direction arrows, surface pressure,
   * the cutting plane. `meshes` has to be handed over before surface pressure
   * can be painted, because it is painted onto the real model geometry. */
  setMeshes(meshes){ if(this.cfd) this.cfd._meshes = meshes; }

  setLayer(which, on){
    if(!this.cfd) return;
    if(which === 'anim') this.cfd.setAnim(on);
    else this.cfd.setLayer(which, on);
  }

  layerState(){ return this.cfd ? this.cfd.show : {}; }
  cpRange(){ return this.cfd ? this.cfd.cpRange : null; }
}

/* ---------------------------------------------------------------- the panel */

export function buildPanel(host, cfg, onChange, onCommit){
  const el = document.createElement('div');
  el.id = 'tunnel';
  el.innerHTML = `
    <div class="twrap">
      <div class="thead">
        <span class="tt">Wind tunnel</span>
        <span class="ts" id="t-panels">—</span>
      </div>
      <div class="tstatus" id="t-status">
      </div>
      <div class="tsliders" id="t-sliders"></div>
      <div class="tviews" id="t-views"></div>
      <div class="treadout" id="t-readout"></div>
      <div class="tplot">
        <div class="tcap" id="t-legcap">Local speed</div>
        <canvas id="t-legend" width="276" height="50"></canvas>
        <p class="thow" id="t-how"></p>
      </div>
      <div class="tplot">
        <div class="tcap">Spanwise load</div>
        <canvas id="t-span" width="276" height="70"></canvas>
      </div>
      <p class="tnote" id="t-note"></p>
      <p class="tabout" id="t-about"></p>
    </div>`;
  host.appendChild(el);

  const mk = (id, label, min, max, value, unit, step) => `
    <label class="trow" for="ts-${id}">
      <span class="tlab">${label}</span>
      <input type="range" id="ts-${id}" min="${min}" max="${max}"
             step="${step || 0.5}" value="${value}">
      <output id="to-${id}">${(+value).toFixed(1)}${unit}</output>
    </label>`;

  const rows = [
    mk('v', 'Airspeed', cfg.v_min, cfg.v_max, cfg.v_default, ' m/s', 0.5),
    mk('alpha', 'Angle of attack', cfg.alpha_min, cfg.alpha_max,
       cfg.alpha_default, '°', 0.25),
  ];
  if(cfg.beta_max !== undefined){
    rows.push(mk('beta', 'Sideslip', cfg.beta_min, cfg.beta_max,
                 cfg.beta_default, '°', 0.5));
  }
  for(const c of cfg.controls){
    rows.push(mk(c.id, c.label, c.min, c.max, c.value, '°', 0.5));
  }
  el.querySelector('#t-sliders').innerHTML = rows.join('');

  /* What you are looking at, as switches rather than as a fixed picture.
   *
   * The old panel had airspeed, incidence and the control surfaces and nothing
   * else: one view, always on, no way to ask a different question. These are
   * the four things a post-processor lets you turn on, plus the animation --
   * which is the difference between a flow field you watch and a picture of
   * one.
   */
  const views = [
    ['ribbons', 'Streamlines', true, 'lines through the flow, coloured by speed'],
    ['arrows', 'Direction arrows', true, 'which way the air is going'],
    ['surface', 'Surface pressure', false,
     'Cₚ on the car: blue is suction, which is where the load comes from'],
    ['plane', 'Cutting plane', false, 'the whole field on one slice'],
    ['undisturbed', 'Undisturbed air', false,
     'also draw the lines that pass by unchanged'],
    ['anim', 'Animate', true, 'the pulse travels at the local flow speed'],
  ];
  el.querySelector('#t-views').innerHTML = views.map(
    ([id, label, on, hint]) => `
      <label class="tview" for="tv-${id}" title="${hint}">
        <input type="checkbox" id="tv-${id}" ${on ? 'checked' : ''}>
        <span>${label}</span>
      </label>`).join('');

  const wire = (id) => {
    const inp = el.querySelector('#ts-' + id);
    const out = el.querySelector('#to-' + id);
    const unit = id === 'v' ? ' m/s' : '°';
    inp.addEventListener('input', () => {
      out.textContent = (+inp.value).toFixed(1) + unit;
      onChange(id, +inp.value);
    });
    // Retracing the flow costs about a second, so it happens on release.
    // The numbers move while you drag; the picture settles when you stop.
    inp.addEventListener('change', () => { if(onCommit) onCommit(id, +inp.value); });
  };
  wire('v'); wire('alpha');
  if(cfg.beta_max !== undefined) wire('beta');
  for(const c of cfg.controls) wire(c.id);

  const setSlider = (id, v) => {
    const inp = el.querySelector('#ts-' + id);
    const out = el.querySelector('#to-' + id);
    if(!inp) return;
    inp.value = v;
    out.textContent = (+v).toFixed(1) + (id === 'v' ? ' m/s' : '\u00B0');
  };

  const viewState = {};
  for(const [id, , on] of views) viewState[id] = on;
  const onView = (fn) => {
    for(const [id] of views){
      const box = el.querySelector('#tv-' + id);
      box.addEventListener('change', () => {
        viewState[id] = box.checked;
        fn(id, box.checked);
      });
    }
  };

  return {
    el,
    views: viewState,
    onView,
    setLegendCaption(t){ el.querySelector('#t-legcap').textContent = t; },
    setHow(t){ el.querySelector('#t-how').textContent = t; },
    setSlider,
    /* Show a state the tunnel arrived at on its own -- the trim incidence,
       say -- without firing the change handler back into the solver. */
    sync(state){
      setSlider('v', state.v);
      setSlider('alpha', state.alpha);
      if(cfg.beta_max !== undefined) setSlider('beta', state.beta);
      for(const c of cfg.controls) setSlider(c.id, state.controls[c.id]);
    },
    reset(){
      for(const [id, v] of [['v', cfg.v_default], ['alpha', cfg.alpha_default],
                            ...cfg.controls.map(c => [c.id, c.value])]){
        const inp = el.querySelector('#ts-' + id);
        inp.value = v;
        inp.dispatchEvent(new Event('input'));
      }
    },
  };
}

export function renderReadout(el, r, cfg, kind){
  const rows = kind === 'car' ? [
    ['Downforce', `${Math.abs(r.lift_N/9.81).toFixed(0)} kg`],
    ['As % of car mass', `${(Math.abs(r.liftFrac)*100).toFixed(0)} %`],
    ['C<sub>L</sub>A', `${Math.abs(r.CL*cfg.s_ref).toFixed(2)}`],
    ['Induced drag', `${(r.drag_N/9.81).toFixed(0)} kg-force`],
    ['Downforce / drag', `${r.LD.toFixed(1)}`],
    ['Aero balance', `${(r.frontFrac*100).toFixed(0)} % front`],
  ] : [
    ['Lift', `${r.lift_N.toFixed(2)} N`],
    ['Weight', `${(cfg.mass_kg*9.81).toFixed(2)} N`],
    ['Lift / weight', `${r.liftFrac.toFixed(2)}`],
    ['C<sub>L</sub>', `${r.CL.toFixed(3)}`],
    ['Induced drag', `${r.drag_N.toFixed(3)} N`],
    ['L / D<sub>i</sub>', `${r.LD.toFixed(1)}`],
    ['Side force', `${r.side_N.toFixed(2)} N`],
    ['Neutral point', `${(r.np*100).toFixed(1)} % MAC`],
    ['Static margin', `${(r.margin*100).toFixed(1)} % MAC`],
  ];
  /* Build the rows once, then count the values across on every later solve.
   *
   * This used to rewrite innerHTML on every update, which throws the elements
   * away and replaces each number with a different number. A value that jumps
   * tells you it changed; a value that counts across tells you which way and
   * by how much -- which, on a readout whose whole job is answering "did that
   * slider help?", is the entire point.
   */
  const want = rows.map(([k]) => k).join('|');
  if(el.dataset.keys !== want){
    el.innerHTML = rows.map(([k, v]) =>
      `<div class="tr"><dt>${k}</dt><dd>${v}</dd></div>`).join('');
    el.dataset.keys = want;
    revealRows(el.querySelectorAll('.tr'));
    return;
  }
  const dds = el.querySelectorAll('.tr dd');
  rows.forEach(([, v], i) => {
    const dd = dds[i];
    if(!dd) return;
    const m = String(v).match(/^(-?[\d.]+)(.*)$/);
    if(!m){ dd.textContent = v; return; }
    tweenNumber(dd, parseFloat(m[1]), {suffix: m[2]});
  });
}

export function drawSpanLoad(cv, strips){
  const g = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  g.clearRect(0, 0, W, H);
  if(!strips || !strips.length) return;
  const acc = new Map();
  for(const s of strips){
    const k = s.y.toFixed(4);
    acc.set(k, (acc.get(k) || 0) + s.dL);
  }
  const pts = [...acc.entries()].map(([y, d]) => [parseFloat(y), d])
                                .sort((a, b) => a[0] - b[0]);
  const ys = pts.map(p => p[0]);
  const y0 = Math.min(...ys), y1 = Math.max(...ys);
  const peak = Math.max(...pts.map(p => Math.abs(p[1]))) || 1;

  g.strokeStyle = '#2E3A40';
  g.beginPath(); g.moveTo(0, H-10); g.lineTo(W, H-10); g.stroke();

  g.beginPath();
  pts.forEach(([y, d], i) => {
    const x = (y - y0)/(y1 - y0 || 1) * (W - 8) + 4;
    const h = Math.abs(d)/peak * (H - 20);
    const yy = H - 10 - h;
    i ? g.lineTo(x, yy) : g.moveTo(x, yy);
  });
  g.strokeStyle = '#7FB2C9';
  g.lineWidth = 1.6;
  g.stroke();
  g.lineTo(W-4, H-10); g.lineTo(4, H-10); g.closePath();
  g.fillStyle = 'rgba(127,178,201,0.16)';
  g.fill();
}
