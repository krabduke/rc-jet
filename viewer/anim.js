/* Motion, shared by all four viewers.
 *
 * anime.js was used nine ways in the F110 viewer -- the build sequence, the
 * camera, the flow opacity, the spool counters -- and in none of the three
 * viewers built after it. Every control in those three snapped: a layer
 * appeared, a number jumped, the camera teleported. Snapping is not "fast", it
 * is information thrown away: a value that jumps from 412 to 508 tells you the
 * number changed, while a value that counts across tells you which way and by
 * how much.
 *
 * Everything here respects prefers-reduced-motion by collapsing to the end
 * state, so the page is never worse for having motion available.
 */

import { animate, utils } from 'animejs';

const REDUCED = typeof matchMedia === 'function'
  && matchMedia('(prefers-reduced-motion: reduce)').matches;

/* Count a readout across to its new value instead of replacing it.
 *
 * `el` is any element whose textContent is a number with optional prefix and
 * suffix; the format is preserved. Numbers that jump hide their own direction,
 * which on a wind tunnel readout is most of what you want to know: whether the
 * change you just made helped.
 */
export function tweenNumber(el, to, opts){
  const {duration = 520, decimals = null, prefix = '', suffix = '',
         ease = 'outQuint'} = opts || {};
  const from = parseFloat(String(el.textContent).replace(/[^0-9.\-]/g, '')) || 0;
  const dp = decimals === null
    ? (String(to).includes('.') ? String(to).split('.')[1].length : 0)
    : decimals;
  const write = (v) => { el.textContent = prefix + v.toFixed(dp) + suffix; };
  if(REDUCED || !isFinite(from)){ write(to); return null; }
  const box = {v: from};
  return animate(box, {v: to, duration, ease,
                       onUpdate: () => write(box.v)});
}

/* Fade a three.js object in or out by its material opacity.
 *
 * The flow layers used to be switched with `visible = true/false`, which pops:
 * two hundred streamlines appearing between one frame and the next reads as a
 * glitch rather than as a layer arriving. Fading makes the change legible, and
 * makes it obvious which layer just changed when several are on.
 */
export function fadeObject(obj, on, opts){
  const {duration = 320, ease = 'outQuad', max = 1} = opts || {};
  if(!obj) return null;
  const mats = [];
  obj.traverse ? obj.traverse(o => { if(o.material) mats.push(o.material); })
               : (obj.material && mats.push(obj.material));
  if(!mats.length) return null;
  for(const m of mats){
    m.transparent = true;
    if(m.uniforms && m.uniforms.uOpacity){ /* shader ribbons */ }
  }
  if(on) obj.visible = true;
  if(REDUCED){
    obj.visible = on;
    for(const m of mats){
      if(m.uniforms && m.uniforms.uOpacity) m.uniforms.uOpacity.value = on ? max : 0;
      else m.opacity = on ? max : 0;
    }
    return null;
  }
  const box = {v: on ? 0 : max};
  return animate(box, {
    v: on ? max : 0, duration, ease,
    onUpdate(){
      for(const m of mats){
        if(m.uniforms && m.uniforms.uOpacity) m.uniforms.uOpacity.value = box.v;
        else m.opacity = box.v;
      }
    },
    onComplete(){ if(!on) obj.visible = false; },
  });
}

/* Move the camera to a viewpoint instead of cutting to it.
 *
 * A cut loses the relationship between the two views: you have to work out
 * where you ended up. A move of two thirds of a second tells you, for free,
 * that the nose you were looking at is the nose you are now looking past.
 */
export function flyCamera(camera, controls, to, opts){
  const {duration = 900, ease = 'inOutQuad', target = null} = opts || {};
  if(REDUCED){
    camera.position.set(to.x, to.y, to.z);
    if(target && controls) controls.target.set(target.x, target.y, target.z);
    if(controls) controls.update();
    return null;
  }
  const a = animate(camera.position, {x: to.x, y: to.y, z: to.z,
                                      duration, ease,
                                      onUpdate: () => controls && controls.update()});
  if(target && controls){
    animate(controls.target, {x: target.x, y: target.y, z: target.z,
                             duration, ease});
  }
  return a;
}

/* Bring a panel in from its edge. One orchestrated entrance, not a transition
 * on every element -- scattered effects read as noise. */
export function revealPanel(el, opts){
  const {duration = 420, from = 14, ease = 'outQuint'} = opts || {};
  if(!el) return null;
  if(REDUCED){ el.style.opacity = '1'; el.style.transform = 'none'; return null; }
  el.style.willChange = 'opacity, transform';
  utils.set(el, {opacity: 0, transform: `translateY(${from}px)`});
  return animate(el, {opacity: 1, translateY: 0, duration, ease,
                      onComplete: () => { el.style.willChange = ''; }});
}

/* Stagger a set of rows in, for a readout that has just been rebuilt. */
export function revealRows(nodes, opts){
  const {duration = 300, step = 26, ease = 'outQuad'} = opts || {};
  const list = [...nodes];
  if(!list.length) return null;
  if(REDUCED){ for(const n of list) n.style.opacity = '1'; return null; }
  utils.set(list, {opacity: 0});
  return animate(list, {opacity: 1, duration, ease, delay: (_, i) => i*step});
}

export const reducedMotion = REDUCED;
