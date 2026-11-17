import { expect, test } from 'vitest';
import { MujocoLeader, type HandTarget } from '../src/leader';
import { loadWorld } from './helpers';

test('Leader reads the hand: hover is no contact, a press reads force and position', async () => {
  const world = await loadWorld();
  let target: HandTarget = { x_mm: 10, y_mm: 5, down: false, force_n: 0 };
  let t = 0;
  const leader = new MujocoLeader(world, () => target, () => t);
  const tick = () => {
    const sample = leader.read();
    world.step();
    t += 10_000;
    return sample;
  };

  let sample = tick();
  for (let i = 0; i < 150; i++) sample = tick();
  expect(sample.contact).toBe(false);
  expect(sample.pressure_n).toBe(0);

  target = { x_mm: 10, y_mm: 5, down: true, force_n: 6 };
  const forces: number[] = [];
  for (let i = 0; i < 250; i++) {
    sample = tick();
    if (i >= 150) forces.push(sample.pressure_n);
  }
  const mean = forces.reduce((a, b) => a + b, 0) / forces.length;
  expect(mean).toBeGreaterThan(5);
  expect(mean).toBeLessThan(7);
  expect(Math.abs(sample.x_mm - 10)).toBeLessThan(3);
  expect(Math.abs(sample.y_mm - 5)).toBeLessThan(3);
  expect(sample.quat_wxyz[0]).toBeGreaterThan(0.99); // still upright
});
