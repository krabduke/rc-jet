/* The aircraft's numbers, off the panel solve.
 *
 * Same shape of answer the vortex-lattice solver gave -- lift, induced drag,
 * the neutral point, the spanwise loading -- so the panel that displays them
 * did not have to change. What is behind them did: there is one set of
 * singularities on one closed surface satisfying one boundary condition, so
 * the lift, the pressures and the streamlines are three readings of the same
 * solution rather than three programs' opinions.
 *
 * Everything here is a back-substitution. The influence matrix is geometry
 * only and was factored when the surface was built, so changing speed,
 * incidence, sideslip or any control position costs a couple of milliseconds.
 */

const RHO = 1.225;

export class PanelTunnel {
  constructor(cfg, pf){
    this.cfg = cfg;
    this.pf = pf;
    this.parts = cfg._parts || [];
  }

  _vinf(o){
    const a = (o.alpha || 0)*Math.PI/180, b = (o.beta || 0)*Math.PI/180;
    const v = o.v;
    return [Math.cos(a)*Math.cos(b)*v, Math.sin(b)*v, Math.sin(a)*Math.cos(b)*v];
  }

  solve(o){
    const pf = this.pf, cfg = this.cfg;
    for(const c of (cfg.controls || []))
      pf.setControl(c.id, (o.controls && o.controls[c.id]) || 0);
    const vinf = this._vinf(o);
    pf.solve(vinf);

    const q = 0.5*RHO*o.v*o.v, S = cfg.s_ref;
    const f = pf.forces();                    // already divided by q
    const a = (o.alpha || 0)*Math.PI/180;
    /* Lift and drag are across and along the FREESTREAM, not the body axes.
     * At twelve degrees the difference is a fifth of the drag. */
    const L = (-f[0]*Math.sin(a) + f[2]*Math.cos(a));
    const CL = L/S;
    const T = pf.trefftz();
    const CDi = T.di/(q/RHO*S*2) * 0 + T.di/(0.5*o.v*o.v*S);
    const CY = f[1]/S;

    return {
      CL, CDi, CY,
      LD: CDi > 1e-9 ? CL/CDi : 0,
      lift_N: CL*q*S,
      drag_N: CDi*q*S,
      side_N: CY*q*S,
      panels: pf.N,
      strips: this.spanLoad(o.v),
      bySurface: this.bySurface(q, a),
      v: o.v,
    };
  }

  /* Lift per unit span, from the circulation the wake carries.
   *
   * Kutta-Joukowski on each shed strip: dL = rho.V.Gamma.dy, with Gamma the
   * doublet jump across that trailing edge. This is the loading the aeroplane
   * actually sheds, which is what the span-load plot is for -- not a sum of
   * surface pressures binned by y, which mixes a wing and a tailplane that
   * happen to share a station. */
  spanLoad(v){
    const pf = this.pf, out = [];
    for(const w of pf.wake){
      /* Minus. The doublet jump and the circulation in Kutta-Joukowski are
       * signed opposite ways here, and the plot came out with every station
       * of the wing pulling DOWN on an aeroplane whose pressure integral said
       * it was holding itself up. Checked rather than argued: the strips now
       * sum to 4.9 N of the 5.5 N the pressures give, the rest being the
       * fuselage, which sheds no wake and so appears in neither. */
      const G = -(pf.mu[w.up] - pf.mu[w.lo]);
      const s = w.strip[0];
      /* Across the flow, not along the shed edge.
       *
       * A fin's trailing edge runs vertically: its circulation makes SIDE
       * force, and taking the edge's length rather than its spanwise part put
       * the whole of it into the lift plot at y = 0, as a spike in the middle
       * of the wing. */
      const dy = Math.abs(s[1][1] - s[0][1]);
      out.push({y: 0.5*(s[0][1] + s[1][1]), dL: RHO*v*G*dy});
    }
    return out;
  }

  /* Which component carries what, by integrating the pressure over each. */
  bySurface(q, a){
    const pf = this.pf, out = {};
    for(const p of this.parts){
      const key = p.name.replace(/_(root|tip|cap)$/, '')
                        .replace(/^(nose|tail)_cap$/, 'fuselage');
      let fx = 0, fz = 0;
      for(let i = p.start; i < p.start + p.count; i++){
        const k = -pf.cp[i]*pf.area[i];
        fx += k*pf.nEff[i*3]; fz += k*pf.nEff[i*3+2];
      }
      out[key] = (out[key] || 0) + (-fx*Math.sin(a) + fz*Math.cos(a))*q;
    }
    return out;
  }

  /* The neutral point, as a fraction of the mean aerodynamic chord.
   *
   * Two solves and a straight line: the aeroplane is linear in incidence
   * until it stalls, and this solver has no stall. The moment is taken about
   * the leading edge of the MAC, so the answer is directly the fraction the
   * readout wants.
   */
  neutralPoint(o){
    const A = this._moment({...o, alpha: 0});
    const B = this._moment({...o, alpha: 4});
    const dCL = B.CL - A.CL;
    if(Math.abs(dCL) < 1e-9) return this.cfg.cg_frac;
    /* np/c = -dCm/dCL with Cm about the MAC leading edge and nose-up
     * positive. A stable aeroplane has its neutral point BEHIND the leading
     * edge, so this comes out positive; the sign was pinned by checking it
     * lands near a quarter to a third of chord on a conventional layout
     * rather than by reasoning about three axis conventions at once. */
    return -(B.Cm - A.Cm)/dCL;
  }

  _moment(o){
    const pf = this.pf, cfg = this.cfg;
    for(const c of (cfg.controls || []))
      pf.setControl(c.id, (o.controls && o.controls[c.id]) || 0);
    const vinf = this._vinf(o);
    pf.solve(vinf);
    const S = cfg.s_ref, c = cfg.c_ref, xref = cfg.x_le_mac;
    let fx = 0, fz = 0, My = 0;
    for(let i = 0; i < pf.N; i++){
      const k = -pf.cp[i]*pf.area[i];
      const Fx = k*pf.nEff[i*3], Fz = k*pf.nEff[i*3+2];
      const rx = pf.c[i*3] - xref, rz = pf.c[i*3+2];
      fx += Fx; fz += Fz;
      My += rz*Fx - rx*Fz;
    }
    const a = (o.alpha || 0)*Math.PI/180;
    return {CL: (-fx*Math.sin(a) + fz*Math.cos(a))/S, Cm: My/(S*c)};
  }
}
