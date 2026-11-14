"""Interpreter: Leader Sample stream -> Grind Intent -> Follower Command.

A Mode owns that whole mapping. Interpret Mode estimates what the user means
over a short stroke window, then commands a clean stroke with the same
parameters. Tremor does not reach the Follower, because the Follower never
copies the Leader's raw motion.
"""

import itertools
import math
from collections import deque
from dataclasses import dataclass
from typing import Protocol

from mortar_pestle.types import (
    IDLE_INTENT,
    Direction,
    FollowerCommand,
    GrindIntent,
    LeaderSample,
    Pattern,
)


class Mode(Protocol):
    """The seam between Leader Samples and Follower Commands."""

    def step(self, sample: LeaderSample) -> tuple[GrindIntent, FollowerCommand]:
        """Consume one Leader Sample. Return the current Grind Intent and Follower Command."""
        ...


@dataclass(frozen=True)
class InterpretConfig:
    window_s: float = 2.0  # stroke window that one Grind Intent covers
    min_window_s: float = 1.0  # data needed before the first non-idle Grind Intent
    samples_per_intent: int = 10  # 100 Hz Leader Samples -> 10 Hz Grind Intent
    min_contact_fraction: float = 0.8  # circular strokes keep the pestle down
    min_radius_mm: float = 3.0
    min_rate_hz: float = 0.2
    max_rate_hz: float = 4.0  # faster motion is tremor (4-12 Hz), not a stroke
    max_pound_spread_mm: float = 8.0
    rest_lift_mm: float = 15.0
    pound_lift_mm: float = 10.0
    pound_down_fraction: float = 0.5
    default_dt_s: float = 0.01


class InterpretMode:
    """Interpret Mode: estimate a Grind Intent, then command an ideal stroke."""

    def __init__(self, config: InterpretConfig | None = None) -> None:
        self._config = config or InterpretConfig()
        self._window: deque[LeaderSample] = deque()
        self._samples_seen = 0
        self._intent = IDLE_INTENT
        self._phase_cycles = 0.0  # stroke phase, in cycles
        self._last_us: int | None = None
        self._command = FollowerCommand(0.0, 0.0, self._config.rest_lift_mm, 0.0)

    def step(self, sample: LeaderSample) -> tuple[GrindIntent, FollowerCommand]:
        config = self._config
        dt_s = (
            config.default_dt_s
            if self._last_us is None
            else (sample.timestamp_us - self._last_us) / 1_000_000
        )
        self._last_us = sample.timestamp_us

        self._window.append(sample)
        oldest_us = sample.timestamp_us - round(config.window_s * 1_000_000)
        while self._window[0].timestamp_us < oldest_us:
            self._window.popleft()

        self._samples_seen += 1
        if self._samples_seen % config.samples_per_intent == 0:
            self._intent = estimate_intent(list(self._window), config)

        self._command = self._next_command(dt_s)
        return self._intent, self._command

    def _next_command(self, dt_s: float) -> FollowerCommand:
        config, intent = self._config, self._intent
        if intent.pattern is Pattern.CIRCULAR:
            sign = 1.0 if intent.direction is Direction.CCW else -1.0
            self._phase_cycles += sign * intent.rate_hz * dt_s
            angle_rad = 2 * math.pi * self._phase_cycles
            return FollowerCommand(
                intent.radius_mm * math.cos(angle_rad),
                intent.radius_mm * math.sin(angle_rad),
                0.0,
                intent.requested_pressure_n,
            )
        if intent.pattern is Pattern.POUNDING:
            self._phase_cycles += intent.rate_hz * dt_s
            if self._phase_cycles % 1.0 < config.pound_down_fraction:
                return FollowerCommand(0.0, 0.0, 0.0, intent.requested_pressure_n)
            return FollowerCommand(0.0, 0.0, config.pound_lift_mm, 0.0)
        # Idle: lift and hold above the last position.
        return FollowerCommand(self._command.x_mm, self._command.y_mm, config.rest_lift_mm, 0.0)


def estimate_intent(window: list[LeaderSample], config: InterpretConfig) -> GrindIntent:
    """Estimate the Grind Intent from the Leader Samples of one stroke window."""
    if len(window) < 2:
        return IDLE_INTENT
    span_s = (window[-1].timestamp_us - window[0].timestamp_us) / 1_000_000
    if span_s < config.min_window_s:
        return IDLE_INTENT

    down = [s for s in window if s.contact]
    if len(down) >= config.min_contact_fraction * len(window):
        return _estimate_circular(down, config)
    return _estimate_pounding(window, down, config)


def _estimate_circular(down: list[LeaderSample], config: InterpretConfig) -> GrindIntent:
    fit = _fit_circle([(s.x_mm, s.y_mm) for s in down])
    if fit is None:
        return IDLE_INTENT
    centre_x_mm, centre_y_mm, radius_mm = fit

    # Angular rate: slope of the unwrapped angle about the fitted centre.
    times_s = [(s.timestamp_us - down[0].timestamp_us) / 1_000_000 for s in down]
    angles_rad = _unwrap([math.atan2(s.y_mm - centre_y_mm, s.x_mm - centre_x_mm) for s in down])
    rate_rad_s = _slope(times_s, angles_rad)
    rate_hz = abs(rate_rad_s) / (2 * math.pi)

    if radius_mm < config.min_radius_mm or not config.min_rate_hz <= rate_hz <= config.max_rate_hz:
        return IDLE_INTENT
    return GrindIntent(
        pattern=Pattern.CIRCULAR,
        rate_hz=rate_hz,
        radius_mm=radius_mm,
        direction=Direction.CCW if rate_rad_s > 0 else Direction.CW,
        requested_pressure_n=sum(s.pressure_n for s in down) / len(down),
    )


def _estimate_pounding(
    window: list[LeaderSample], down: list[LeaderSample], config: InterpretConfig
) -> GrindIntent:
    # A hit starts where contact begins. Track each hit's start time and peak pressure.
    hit_starts_us: list[int] = []
    hit_peaks_n: list[float] = []
    for before, after in itertools.pairwise(window):
        if after.contact and not before.contact:
            hit_starts_us.append(after.timestamp_us)
            hit_peaks_n.append(after.pressure_n)
        elif after.contact and hit_peaks_n:
            hit_peaks_n[-1] = max(hit_peaks_n[-1], after.pressure_n)
    if len(hit_starts_us) < 2:
        return IDLE_INTENT
    if window[-1].contact:
        hit_peaks_n.pop()  # the last hit has not peaked yet

    centre_x_mm = sum(s.x_mm for s in down) / len(down)
    centre_y_mm = sum(s.y_mm for s in down) / len(down)
    spread_mm = max(math.hypot(s.x_mm - centre_x_mm, s.y_mm - centre_y_mm) for s in down)
    rate_hz = (len(hit_starts_us) - 1) / ((hit_starts_us[-1] - hit_starts_us[0]) / 1_000_000)
    if spread_mm > config.max_pound_spread_mm or rate_hz > config.max_rate_hz:
        return IDLE_INTENT
    return GrindIntent(
        pattern=Pattern.POUNDING,
        rate_hz=rate_hz,
        radius_mm=0.0,
        direction=Direction.CW,
        requested_pressure_n=sum(hit_peaks_n) / len(hit_peaks_n),
    )


def _fit_circle(points_mm: list[tuple[float, float]]) -> tuple[float, float, float] | None:
    """Least-squares circle fit (Kasa). Return (centre_x, centre_y, radius) or None.

    Solves x^2 + y^2 + D*x + E*y + F = 0 for D, E, F in the least-squares sense.
    """
    n = len(points_mm)
    sx = sum(x for x, _ in points_mm)
    sy = sum(y for _, y in points_mm)
    sxx = sum(x * x for x, _ in points_mm)
    syy = sum(y * y for _, y in points_mm)
    sxy = sum(x * y for x, y in points_mm)
    sxz = sum(x * (x * x + y * y) for x, y in points_mm)
    syz = sum(y * (x * x + y * y) for x, y in points_mm)
    sz = sxx + syy
    solution = _solve3(
        [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]],
        [-sxz, -syz, -sz],
    )
    if solution is None:
        return None
    d, e, f = solution
    centre_x, centre_y = -d / 2, -e / 2
    radius_squared = centre_x * centre_x + centre_y * centre_y - f
    if radius_squared <= 0:
        return None
    return centre_x, centre_y, math.sqrt(radius_squared)


def _solve3(a: list[list[float]], b: list[float]) -> tuple[float, float, float] | None:
    """Solve a 3x3 linear system by Cramer's rule. Return None if it is singular."""

    def det(m: list[list[float]]) -> float:
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    scale = max(abs(value) for row in a for value in row)
    d = det(a)
    if scale == 0 or abs(d) < 1e-9 * scale**3:
        return None
    columns = []
    for column in range(3):
        replaced = [row[:] for row in a]
        for row in range(3):
            replaced[row][column] = b[row]
        columns.append(det(replaced) / d)
    return columns[0], columns[1], columns[2]


def _unwrap(angles_rad: list[float]) -> list[float]:
    """Remove the 2*pi jumps from a sequence of angles."""
    unwrapped = [angles_rad[0]]
    for previous, current in itertools.pairwise(angles_rad):
        step = (current - previous + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + step)
    return unwrapped


def _slope(xs: list[float], ys: list[float]) -> float:
    """Least-squares slope of ys over xs."""
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((x - mean_x) ** 2 for x in xs)
    if denominator == 0:
        return 0.0
    return sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys, strict=True)) / denominator
