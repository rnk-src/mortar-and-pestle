import fs from 'node:fs';
import path from 'node:path';
import { expect, test } from 'vitest';
import load_mujoco from '@mujoco/mujoco';

const here = path.dirname(new URL(import.meta.url).pathname);
const assets = path.join(here, '../../sim/assets/so101/assets');

async function loadScene() {
  const mj = await load_mujoco();
  mj.FS.mkdir('/w');
  mj.FS.mkdir('/w/assets');
  for (const f of fs.readdirSync(assets)) {
    mj.FS.writeFile(`/w/assets/${f}`, fs.readFileSync(path.join(assets, f)));
  }
  for (const f of ['scene.xml', 'so101.xml']) {
    mj.FS.writeFile(`/w/${f}`, fs.readFileSync(path.join(here, '../scene', f), 'utf8'));
  }
  const model = mj.MjModel.from_xml_path('/w/scene.xml');
  return { mj, model, data: new mj.MjData(model) };
}

test('scene compiles and the Leader pestle rests on its mortar floor', async () => {
  const { mj, model, data } = await loadScene();
  const body = model.body('leader_pestle').id;
  for (let i = 0; i < 500; i++) mj.mj_step(model, data); // 1 s at 2 ms
  const z = data.xpos[body * 3 + 2];
  expect(Number.isFinite(z)).toBe(true);
  // floor top is 10 mm; the capsule centre rests one cap above it plus half the cylinder
  expect(z).toBeGreaterThan(0.01);
  expect(z).toBeLessThan(0.07);
  expect(model.body('follower_pestle').id).toBeGreaterThan(0);
});
