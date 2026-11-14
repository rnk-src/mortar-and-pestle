"""Synthetic stroke generator: ideal Leader Samples with optional tremor and noise."""

import math
import random
from dataclasses import dataclass

from mortar_pestle.types import Direction, LeaderSample, Pattern

UPRIGHT_QUAT_WXYZ = (1.0, 0.0, 0.0, 0.0)
POUND_DOWN_FRACTION = 0.5  # part of each pounding cycle that the pestle is down
CONTACT_THRESHOLD_N = 0.2


@dataclass(frozen=True)
class StrokeSpec:
    pattern: Pattern
    duration_s: float
    rate_hz: float = 1.0
    radius_mm: float = 0.0
    direction: Direction = Direction.CW
    pressure_n: float = 0.0
    tremor_hz: float = 0.0
    tremor_mm: float = 0.0
    noise_mm: float = 0.0
    noise_n: float = 0.0
    seed: int = 0
    axis_deg: float = 0.0  # linear strokes: direction of the stroke axis in the floor plane
    points_mm: tuple[tuple[float, float], ...] = ((0.0, 0.0),)  # pounding: hit these in turn
    sample_hz: float = 100.0


def generate_strokes(spec: StrokeSpec, start_us: int = 0) -> list[LeaderSample]:
    """Return the Leader Samples of one stroke segment, starting at start_us."""
    rng = random.Random(spec.seed)
    period_us = round(1_000_000 / spec.sample_hz)
    count = round(spec.duration_s * spec.sample_hz)
    samples = []
    for index in range(count):
        t_s = index / spec.sample_hz
        x_mm, y_mm, pressure_n = _ideal(spec, t_s)
        contact = pressure_n > CONTACT_THRESHOLD_N
        if contact:
            tremor_rad = 2 * math.pi * spec.tremor_hz * t_s
            x_mm += spec.tremor_mm * math.sin(tremor_rad) + rng.gauss(0.0, spec.noise_mm)
            y_mm += spec.tremor_mm * math.cos(tremor_rad + 1.0) + rng.gauss(0.0, spec.noise_mm)
            pressure_n = max(0.0, pressure_n + rng.gauss(0.0, spec.noise_n))
        samples.append(
            LeaderSample(
                timestamp_us=start_us + index * period_us,
                quat_wxyz=UPRIGHT_QUAT_WXYZ,
                pressure_n=pressure_n,
                x_mm=x_mm,
                y_mm=y_mm,
                contact=contact,
            )
        )
    return samples


def _ideal(spec: StrokeSpec, t_s: float) -> tuple[float, float, float]:
    """Return the noise-free (x_mm, y_mm, pressure_n) at time t_s."""
    if spec.pattern is Pattern.CIRCULAR:
        sign = 1.0 if spec.direction is Direction.CCW else -1.0
        angle_rad = sign * 2 * math.pi * spec.rate_hz * t_s
        return (
            spec.radius_mm * math.cos(angle_rad),
            spec.radius_mm * math.sin(angle_rad),
            spec.pressure_n,
        )
    if spec.pattern is Pattern.LINEAR:
        # Back and forth through the centre, radius_mm to each side.
        along_mm = spec.radius_mm * math.sin(2 * math.pi * spec.rate_hz * t_s)
        axis_rad = math.radians(spec.axis_deg)
        return along_mm * math.cos(axis_rad), along_mm * math.sin(axis_rad), spec.pressure_n
    if spec.pattern is Pattern.POUNDING:
        hit, cycle = divmod(spec.rate_hz * t_s, 1.0)
        x_mm, y_mm = spec.points_mm[int(hit) % len(spec.points_mm)]
        if cycle < POUND_DOWN_FRACTION:
            # Each hit is a half-sine pressure pulse that peaks at pressure_n.
            pressure_n = spec.pressure_n * math.sin(math.pi * cycle / POUND_DOWN_FRACTION)
            return x_mm, y_mm, pressure_n
        return x_mm, y_mm, 0.0
    return 0.0, 0.0, 0.0
