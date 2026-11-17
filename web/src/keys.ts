/** Next press state after a Space keydown. A tap toggles the press; key repeats from a held key do not. */
export function spacePress(down: boolean, e: { repeat: boolean }): boolean {
  return e.repeat ? down : !down;
}
