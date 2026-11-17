/** A force for the HUD, e.g. "3.1 N". A negative or non-finite force shows as zero. */
export function formatNewtons(n: number, digits = 1): string {
  const safe = Number.isFinite(n) ? Math.max(0, n) : 0;
  return `${safe.toFixed(digits)} N`;
}
