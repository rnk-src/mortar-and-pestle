import { expect, test } from 'vitest';
import { MujocoFollower } from '../src/follower';
import { loadWorld } from './helpers';

test('Follower tracks a lifted target, then presses with the commanded force', async () => {
  const world = await loadWorld();
  const follower = new MujocoFollower(world);

  let state = follower.execute({ x_mm: 20, y_mm: 0, lift_mm: 8, pressure_n: 0 });
  for (let i = 0; i < 150; i++) state = follower.execute({ x_mm: 20, y_mm: 0, lift_mm: 8, pressure_n: 0 });
  expect(Math.abs(state.x_mm - 20)).toBeLessThan(4);
  expect(Math.abs(state.y_mm)).toBeLessThan(4);
  expect(state.contact_force_n).toBe(0);

  const forces: number[] = [];
  for (let i = 0; i < 300; i++) {
    state = follower.execute({ x_mm: 20, y_mm: 0, lift_mm: 0, pressure_n: 6 });
    if (i >= 200) forces.push(state.contact_force_n);
  }
  const mean = forces.reduce((a, b) => a + b, 0) / forces.length;
  expect(mean).toBeGreaterThan(3);
  expect(mean).toBeLessThan(9);
});

test('Leader pestle tip rests on the floor (tip height near zero)', async () => {
  const { LEADER } = await import('../src/world');
  const world = await loadWorld();
  for (let i = 0; i < 100; i++) world.step();
  const tip = world.tipMm(LEADER);
  expect(Math.abs(tip[2])).toBeLessThan(1.5);
});
