import fs from 'node:fs';
import path from 'node:path';
import load_mujoco from '@mujoco/mujoco';
import { World, type Files } from '../src/world';

const here = path.dirname(new URL(import.meta.url).pathname);
const assets = path.join(here, '../../sim/assets/so101/assets');

export async function loadWorld(): Promise<World> {
  const mj = await load_mujoco();
  const files: Files = {};
  for (const f of ['scene.xml', 'so101.xml']) {
    files[f] = fs.readFileSync(path.join(here, '../scene', f), 'utf8');
  }
  for (const f of fs.readdirSync(assets)) {
    files[`assets/${f}`] = new Uint8Array(fs.readFileSync(path.join(assets, f)));
  }
  return World.create(mj, files);
}
