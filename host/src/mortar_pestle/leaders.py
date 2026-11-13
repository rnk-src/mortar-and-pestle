"""Leader source adapters that need no hardware and no simulator."""

from collections.abc import Sequence

from mortar_pestle.synthetic import StrokeSpec, generate_strokes
from mortar_pestle.types import LeaderSample


class ReplayLeader:
    """Replays a Leader Sample stream, one entry per tick. Returns None after the end."""

    def __init__(self, samples: Sequence[LeaderSample | None]) -> None:
        self._samples = samples
        self._index = 0

    def read(self) -> LeaderSample | None:
        if self._index >= len(self._samples):
            return None
        sample = self._samples[self._index]
        self._index += 1
        return sample


class SyntheticLeader(ReplayLeader):
    """Yields ideal Leader Samples from stroke segments played back to back."""

    def __init__(self, specs: Sequence[StrokeSpec]) -> None:
        super().__init__(synthetic_samples(specs))


def synthetic_samples(specs: Sequence[StrokeSpec]) -> list[LeaderSample]:
    """Generate the stroke segments back to back on one timeline."""
    samples: list[LeaderSample] = []
    start_us = 0
    for spec in specs:
        segment = generate_strokes(spec, start_us)
        samples += segment
        start_us += round(len(segment) * 1_000_000 / spec.sample_hz)
    return samples
