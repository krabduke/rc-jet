/* Check that the viewer's JavaScript parses and its modules load.
 *
 * Nothing in any of these projects has ever checked the browser code at all.
 * Every gate compared a number to another number; the 16 to 34 kilobytes of
 * inline module script in each viewer's index.html were never parsed by
 * anything but a browser, at which point a stray brace is a blank screen and a
 * console message nobody sees.
 *
 * So: parse every <script type="module"> block in index.html, and import every
 * module in viewer/ against a three.js stub. A syntax error or a bad import
 * now fails the build.
 *
 *     node tools/validate_viewer.mjs
 */
import { readFileSync, writeFileSync, mkdtempSync, readdirSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import vm from 'node:vm';

const root = process.argv[2] || '.';
const viewer = join(root, 'viewer');

let fails = 0;
const ok = (m) => console.log('  ok   ' + m);
const bad = (m) => { console.log('  x    ' + m); fails++; };

/* ---- 1. the inline module scripts parse ---- */
const html = readFileSync(join(viewer, 'index.html'), 'utf8');
const blocks = [...html.matchAll(
  /<script\b[^>]*type=["']module["'][^>]*>([\s\S]*?)<\/script>/g)].map(m => m[1]);
if(!blocks.length) bad('index.html has no <script type="module">');
blocks.forEach((src, i) => {
  // strip the import/export statements: this is a syntax check, not a link
  const body = src
    .replace(/^\s*import\s[^;]*?;\s*$/gm, '')
    .replace(/^\s*import\s*\{[\s\S]*?\}\s*from[^;]*;\s*$/gm, '')
    .replace(/^\s*export\s+/gm, '');
  try {
    new vm.Script('(async () => {\n' + body + '\n})', {filename: `inline-${i}`});
    ok(`inline module ${i + 1} parses (${src.length.toLocaleString()} bytes)`);
  } catch (e) {
    bad(`inline module ${i + 1}: ${String(e.message).split('\n')[0]}`);
  }
});

/* ---- 2. every module in viewer/ loads against a three stub ---- */
const dir = mkdtempSync(join(tmpdir(), 'viewercheck-'));
writeFileSync(join(dir, 'package.json'), '{"type":"module"}');
writeFileSync(join(dir, 'three.js'), `
export class Group{constructor(){this.children=[]}add(){}remove(){}}
export class BufferGeometry{constructor(){this.a={}}setAttribute(k,v){this.a[k]=v}
  getAttribute(k){return this.a[k]}deleteAttribute(k){delete this.a[k]}
  setIndex(v){this.index=v}dispose(){}translate(){}}
export class BufferAttribute{constructor(a,b){this.array=a;this.itemSize=b;this.count=a?a.length/(b||1):0}
  getX(){return 0}getY(){return 0}getZ(){return 0}}
export class InstancedBufferAttribute extends BufferAttribute{}
export class ShaderMaterial{constructor(o){Object.assign(this,o)}dispose(){}}
export class MeshBasicMaterial{constructor(o){Object.assign(this,o)}dispose(){}}
export class MeshStandardMaterial{constructor(o){Object.assign(this,o)}dispose(){}}
export class LineBasicMaterial{constructor(o){Object.assign(this,o)}dispose(){}}
export class Mesh{constructor(g,m){this.geometry=g;this.material=m}updateWorldMatrix(){}traverse(){}}
export class LineSegments extends Mesh{}
export class InstancedMesh extends Mesh{constructor(g,m,c){super(g,m);this.count=c;
  this.instanceMatrix={needsUpdate:false}}setMatrixAt(){}}
export class ConeGeometry extends BufferGeometry{}
export class PlaneGeometry extends BufferGeometry{}
export class Matrix4{compose(){return this}copy(){return this}invert(){return this}}
export class Quaternion{setFromUnitVectors(){return this}}
export class Vector2{constructor(x,y){this.x=x;this.y=y}}
export class Vector3{constructor(x,y,z){this.x=x;this.y=y;this.z=z}
  set(){return this}normalize(){return this}applyMatrix4(){return this}clone(){return this}}
export class Box3{clone(){return this}translate(){return this}}
export class Color{constructor(){}}
export const DoubleSide=2, FrontSide=0, BackSide=1;
`);
writeFileSync(join(dir, 'animejs.js'), `
export function animate(t, o){ if(o && o.onUpdate) o.onUpdate(); 
  if(o && o.onComplete) o.onComplete(); return {}; }
export const utils = { set(){} };
`);
const mods = readdirSync(viewer).filter(f => f.endsWith('.js'));
for(const f of mods){
  const src = readFileSync(join(viewer, f), 'utf8')
    .replace(/from ['"]three['"]/g, "from './three.js'")
    .replace(/from ['"]animejs['"]/g, "from './animejs.js'");
  writeFileSync(join(dir, f), src);
}
for(const f of mods){
  try {
    await import('file://' + join(dir, f));
    ok(`${f} loads`);
  } catch (e) {
    bad(`${f}: ${String(e.message).split('\n')[0]}`);
  }
}

console.log('\n' + (fails
  ? `FAIL  ${fails} viewer problems`
  : 'PASS  viewer scripts parse and every module loads'));
process.exit(fails ? 1 : 0);
