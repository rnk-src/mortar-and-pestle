// Copies the scene, meshes, glue and a freshly built host wheel into public/ for dev and build.
// Single source of truth: the meshes come from sim/assets, the Python from host/.
import { execSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';

const web = path.dirname(path.dirname(new URL(import.meta.url).pathname));
const pub = path.join(web, 'public');
const meshes = path.join(web, '../sim/assets/so101/assets');

execSync('node scripts/build-scene.mjs', { cwd: web, stdio: 'inherit' });
fs.rmSync(pub, { recursive: true, force: true });
fs.mkdirSync(path.join(pub, 'scene/assets'), { recursive: true });
fs.mkdirSync(path.join(pub, 'py'), { recursive: true });

for (const f of ['scene.xml', 'so101.xml']) fs.copyFileSync(path.join(web, 'scene', f), path.join(pub, 'scene', f));
const stl = fs.readdirSync(meshes).filter((f) => f.endsWith('.stl'));
for (const f of stl) fs.copyFileSync(path.join(meshes, f), path.join(pub, 'scene/assets', f));
fs.writeFileSync(path.join(pub, 'scene/manifest.json'), JSON.stringify({ meshes: stl }));

fs.copyFileSync(path.join(web, 'py/browser.py'), path.join(pub, 'py/browser.py'));
const tmp = fs.mkdtempSync(path.join(web, '.wheel-'));
try {
  execSync(`uv build --wheel --out-dir ${tmp}`, { cwd: path.join(web, '../host'), stdio: 'pipe' });
  const wheel = fs.readdirSync(tmp).find((f) => f.endsWith('.whl'));
  fs.copyFileSync(path.join(tmp, wheel), path.join(pub, 'py/mortar_pestle.whl'));
} finally {
  fs.rmSync(tmp, { recursive: true, force: true });
}
console.log('prepared public/ with', stl.length, 'meshes');
