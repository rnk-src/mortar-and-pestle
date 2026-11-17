import { expect, test } from 'vitest';
import { formatNewtons } from '../src/format';

test('formats a force with one decimal and the unit', () => {
  expect(formatNewtons(3.14159)).toBe('3.1 N');
  expect(formatNewtons(0)).toBe('0.0 N');
  expect(formatNewtons(12.96)).toBe('13.0 N');
});

test('shows a whole number when asked, for the push setting', () => {
  expect(formatNewtons(6, 0)).toBe('6 N');
});

test('never shows a negative, tiny negative or non-finite force', () => {
  expect(formatNewtons(-0.04)).toBe('0.0 N');
  expect(formatNewtons(-2)).toBe('0.0 N');
  expect(formatNewtons(Number.NaN)).toBe('0.0 N');
  expect(formatNewtons(Number.POSITIVE_INFINITY)).toBe('0.0 N');
});
