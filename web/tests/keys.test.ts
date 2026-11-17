import { expect, test } from 'vitest';
import { spacePress } from '../src/keys';

test('a tap on Space toggles the press', () => {
  expect(spacePress(false, { repeat: false })).toBe(true);
  expect(spacePress(true, { repeat: false })).toBe(false);
});

test('holding Space keeps the press steady through key repeats', () => {
  let down = spacePress(false, { repeat: false });
  for (let i = 0; i < 30; i++) {
    down = spacePress(down, { repeat: true });
    expect(down).toBe(true);
  }
});
