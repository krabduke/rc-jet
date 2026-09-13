/* Streamlines, drawn the way a CFD post-processor draws them.
 *
 * Steady streamlines: traced once through the solved field and then left
 * alone, so you can look at them. Coloured by local speed against a legend,
 * because a line that is only pretty tells you nothing -- the colour is where
 * the flow is accelerated over a shoulder and where it has stagnated against
 * a face.
 *
 * The field is the freestream plus the body's source panels plus the lifting
 * surfaces' horseshoe vortices, all of which are solved, not sketched.
 */

import * as THREE from 'three';
import { BodyField, traceLine, velocityColour } from './flowfield.js';

const TRACE_STEPS = 200;

export class CFDView {
  constructor(opts){
    this.cfg = opts.cfg;
    this.root = opts.root;
    this.bounds = opts.bounds;        // model bounds in scene (Y-up) metres
    this.bodyPanels = opts.bodyPanels;
    this.nLines = opts.nLines || 150;
    this.group = new THREE.Group();
    this.group.name = 'cfd';
    this.root.add(this.group);
    this.body = this.bodyPanels
      ? new BodyField(this.bodyPanels, !!this.cfg.ground) : null;
    this.vmax = 1;
    this.lines = null;
    this.arrows = null;
    this._buildSeeds();
  }

  /* Seed from an upstream rake that spans the model's frontal area with a
     margin, so lines wrap the body rather than all missing it. */
  /* Seed a rake big enough to contain the model at any attitude.
   *
   * The rake used to be sized on the body's own bounding box, so as soon as
   * the aircraft was pitched or yawed it rotated out of the stream and half
   * of it sat in air that had no streamlines in it at all. The domain is
   * sized on the model's *diagonal* now -- the largest extent it can present
   * at any angle -- with margin beyond that, so there is undisturbed
   * freestream visible around the model as well as the flow it disturbs.
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
    // the biggest half-extent the model can present once it is rotated
    const diag = 0.5*Math.hypot(L, W, Hh);
    const zc = 0.5*(z0 + z1);

    const spanY = diag * 1.9;
    const spanZ = diag * 1.7;
    const rows = Math.max(7, Math.round(Math.sqrt(this.nLines * 0.62)));
    const cols = Math.max(7, Math.round(this.nLines / rows));
    const seeds = [];
    const xs = x0 - L * 0.55;
    for(let i = 0; i < rows; i++){
      for(let j = 0; j < cols; j++){
        const fz = rows > 1 ? i/(rows - 1) : 0.5;
        const fy = cols > 1 ? j/(cols - 1) : 0.5;
        let z = zc + (fz - 0.5) * spanZ;
        if(this.cfg.ground){
          // there is nothing below a track: fill the space above it instead
          z = 0.015 + fz * spanZ * 1.05;
        }
        seeds.push([xs, (fy - 0.5) * spanY, z]);
      }
    }
    this.seeds = seeds;
    this.domain = {spanY, spanZ, xs, diag};
  }

  /* Couple the body to the lifting surfaces and trace.
   *
   * The body is solved in the freestream first, then the wings see the body's
   * velocity, then the body is re-solved including the wings. Two passes
   * settles it and leaves the validated lattice solve untouched.
   */
  run(vinf, latticeVel){
    if(!this.body) return;
    const N = this.body.N;
    // pass 1: body alone
    this.body.solve(vinf, null);
    // pass 2: body sees the wings
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

    const vel = (p, out) => {
      out[0] = vinf[0]; out[1] = vinf[1]; out[2] = vinf[2];
      this.body.add(p, out);
      if(latticeVel) latticeVel(p, out);
      return out;
    };

    const span = this.ext.x1 - this.ext.x0;
    const ds = span / 52;
    const xEnd = this.ext.x1 + span * 1.05;
    /* Colour scale.
     *
     * Not the observed maximum: a point-source panel is singular at its own
     * centroid, so a streamline that passes close to the skin picks up a
     * spike -- 672 m/s against a 69 m/s freestream, which normalises the
     * whole picture to dark blue and shows nothing. The scale is fixed to a
     * physical range instead, 0 to 1.5 times freestream, and the legend says
     * so. That is what a CFD post-processor does with a clipped range too.
     */
    const vfs = Math.hypot(vinf[0], vinf[1], vinf[2]);
    const traced = [];
    // A window, not a 0-based scale. Nearly all of the flow is at freestream
    // speed, so a scale starting at zero puts 90 % of the picture in one
    // colour and shows nothing. Running from 0.35 to 1.45 of freestream puts
    // undisturbed air in the middle of the map and leaves the whole blue end
    // for stagnation and the red end for acceleration -- which is where all
    // the information is.
    const vmin = vfs * 0.35, vmax = vfs * 1.45;
    let vpeak = 0;
    for(const s of this.seeds){
      const L = traceLine(vel, s, {
        maxSteps: TRACE_STEPS, ds, xEnd, body: this.body,
        bounds: [-this.domain.spanY*0.75, this.domain.spanY*0.75,
                 this.cfg.ground ? 0.0
                   : 0.5*(this.ext.z0+this.ext.z1) - this.domain.spanZ*0.75,
                 0.5*(this.ext.z0+this.ext.z1) + this.domain.spanZ*0.75],
      });
      if(L.spd.length > 6){
        traced.push(L);
        for(const m of L.spd) if(m > vpeak) vpeak = m;
      }
    }
    this.vmax = vmax;
    this.vmin = vmin;
    this._draw(traced);
    return {lines: traced.length, vmin, vmax, vpeak, vfs};
  }

  _draw(traced){
    let segs = 0;
    for(const L of traced) segs += (L.spd.length - 1);
    const pos = new Float32Array(segs * 6);
    const col = new Float32Array(segs * 6);
    const rgb = [0,0,0];
    let k = 0;
    for(const L of traced){
      const n = L.spd.length;
      for(let i = 0; i < n - 1; i++){
        // solver frame (x aft, y span, z up) -> glTF Y-up (x, z, -y)
        pos[k*6]   = L.pts[i*3];
        pos[k*6+1] = L.pts[i*3+2];
        pos[k*6+2] = -L.pts[i*3+1];
        pos[k*6+3] = L.pts[(i+1)*3];
        pos[k*6+4] = L.pts[(i+1)*3+2];
        pos[k*6+5] = -L.pts[(i+1)*3+1];
        const sc = 1/(this.vmax - this.vmin);
        velocityColour((L.spd[i] - this.vmin)*sc, rgb);
        col[k*6] = rgb[0]; col[k*6+1] = rgb[1]; col[k*6+2] = rgb[2];
        velocityColour((L.spd[i+1] - this.vmin)*sc, rgb);
        col[k*6+3] = rgb[0]; col[k*6+4] = rgb[1]; col[k*6+5] = rgb[2];
        k++;
      }
    }
    if(this.lines){
      this.group.remove(this.lines);
      this.lines.geometry.dispose();
      this.lines.material.dispose();
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.BufferAttribute(col, 3));
    const m = new THREE.LineBasicMaterial({
      vertexColors: true, transparent: true, opacity: 0.95, depthWrite: false,
    });
    this.lines = new THREE.LineSegments(g, m);
    this.lines.frustumCulled = false;
    this.lines.raycast = () => {};
    this.group.add(this.lines);
    this.traced = traced;
  }

  setVisible(on){ this.group.visible = on; }

  dispose(){
    if(this.lines){
      this.group.remove(this.lines);
      this.lines.geometry.dispose();
      this.lines.material.dispose();
      this.lines = null;
    }
  }
}

/* The legend. A colour bar with real numbers on it: without one the colours
   are decoration, with one they are data. */
export function drawLegend(canvas, vmin, vmax, unitLabel){
  const g = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  g.clearRect(0, 0, W, H);
  const barH = 12, top = 16, left = 4, right = W - 4;
  const rgb = [0,0,0];
  for(let x = left; x < right; x++){
    velocityColour((x - left)/(right - left), rgb);
    g.fillStyle = `rgb(${Math.round(rgb[0]*255)},${Math.round(rgb[1]*255)},${Math.round(rgb[2]*255)})`;
    g.fillRect(x, top, 1, barH);
  }
  g.strokeStyle = '#2C383E';
  g.strokeRect(left, top, right - left, barH);
  g.fillStyle = '#96A3AB';
  g.font = '10px ui-monospace, monospace';
  g.textBaseline = 'top';
  g.fillText(unitLabel, left, 2);
  g.textBaseline = 'alphabetic';
  for(let i = 0; i <= 4; i++){
    const x = left + (right - left) * i/4;
    const v = vmin + (vmax - vmin) * i/4;
    g.fillStyle = '#2C383E';
    g.fillRect(Math.min(x, right-1), top + barH, 1, 3);
    g.fillStyle = '#96A3AB';
    const t = v >= 100 ? v.toFixed(0) : v.toFixed(1);
    const tw = g.measureText(t).width;
    g.fillText(t, Math.max(left, Math.min(x - tw/2, right - tw)), top + barH + 13);
  }
}
