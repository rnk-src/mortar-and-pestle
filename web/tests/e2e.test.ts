import { execSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { loadPyodide } from 'pyodide';
import { beforeAll, expect, test } from 'vitest';
import { Session } from '../src/session';
import { loadWorld } from './helpers';

const here = path.dirname(new URL(import.meta.url).pathname);
let wheel: Uint8Array;

beforeAll(() => {
  const out = fs.mkdtempSync(path.join(os.tmpdir(), 'mp-wheel-'));
  execSync(`uv build --wheel --out-dir ${out}`, { cwd: path.join(here, '../../host'), stdio: 'pipe' });
  const file = fs.readdirSync(out).find((f) => f.endsWith('.whl'))!;
  wheel = new Uint8Array(fs.readFileSync(path.join(out, file)));
}, 120_000);

test('scripted hand through the Python pipeline moves the Follower and presses', async () => {
  const pyodide = await loadPyodide({ indexURL: path.join(here, '../node_modules/pyodide') });
  const world = await loadWorld();
  const glue = fs.readFileSync(path.join(here, '../py/browser.py'), 'utf8');
  const session = Session.create({ pyodide, world, wheel, glue, mode: 'mirror' });

  let maxLeader = 0;
  let maxFollower = 0;
  let last = session.tick();
  let sawCircular = false; let circularRate = 0;
  for (let i = 0; i < 700; i++) {
    last = session.tick(); // 7 s of simulated time: 1 s idle, then the circular stroke
    maxLeader = Math.max(maxLeader, last.leader_force_n);
    maxFollower = Math.max(maxFollower, last.state.contact_force_n);
    // Safety Envelope: the commanded radius never exceeds 40 mm
    expect(Math.hypot(last.command.x_mm, last.command.y_mm)).toBeLessThanOrEqual(40.01);
    if (last.intent.pattern === 'circular') {
      sawCircular = true;
      circularRate = last.intent.rate_hz;
    }
  }
  expect(maxLeader).toBeGreaterThan(3);
  expect(maxFollower).toBeGreaterThan(3); // the Follower follows the Leader's force
  expect(Number.isFinite(last.state.x_mm)).toBe(true);
  expect(sawCircular).toBe(true);
  expect(circularRate).toBeGreaterThan(1.0); // the scripted stroke is 1.2 Hz
  expect(circularRate).toBeLessThan(1.4);
}, 180_000);
