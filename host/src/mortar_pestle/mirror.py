"""Mirror Mode: the Follower follows the Leader pestle directly.

The Follower Command is the Leader's tip position and pressure, passed
through a low-pass filter that removes tremor. Any motion works: circles,
back-and-forth strokes, pounding at several points.

A Leader Sample carries a position only while the pestle touches the mortar.
So when contact starts at a new point, the Follower travels there lifted and
presses only after it arrives.
"""

import math
from dataclasses import dataclass

from mortar_pestle.interpreter import IntentTracker, InterpretConfig
from mortar_pestle.types import FollowerCommand, GrindIntent, LeaderSample


@dataclass(frozen=True)
class MirrorConfig:
    # Tremor is 4-12 Hz and grinding strokes are slower than about 2 Hz. Two cascaded
    # first-order low-pass stages at this cutoff pass 96 % of a 1 Hz stroke and 28 % of
    # an 8 Hz tremor, with about 60 ms of delay.
    cutoff_hz: float = 5.0
    hover_lift_mm: float = 8.0  # lift while the Leader pestle is up
    travel_lift_mm: float = 4.0  # lift while travelling to a new contact point
    travel_step_mm: float = 4.0  # travel per tick (400 mm/s at 100 Hz)
    jump_mm: float = 5.0  # a target this far from the last command is a new contact point
    default_dt_s: float = 0.01
    max_dt_s: float = 0.05


class _LowPass:
    """Two cascaded first-order low-pass stages."""

    def __init__(self) -> None:
        self._first = 0.0
        self._second = 0.0

    def reset(self, value: float) -> None:
        self._first = self._second = value

    def update(self, value: float, alpha: float) -> float:
        self._first += alpha * (value - self._first)
        self._second += alpha * (self._first - self._second)
        return self._second


class MirrorMode:
    """Mirror Mode. It also reports the estimated Grind Intent, for display and recording."""

    def __init__(
        self, config: MirrorConfig | None = None, intent_config: InterpretConfig | None = None
    ) -> None:
        self._config = config or MirrorConfig()
        self._tracker = IntentTracker(intent_config or InterpretConfig())
        self._filter_x, self._filter_y, self._filter_n = _LowPass(), _LowPass(), _LowPass()
        self._in_contact = False
        self._x_mm = 0.0  # last commanded position
        self._y_mm = 0.0
        self._last_us: int | None = None

    def step(self, sample: LeaderSample) -> tuple[GrindIntent, FollowerCommand]:
        config = self._config
        intent = self._tracker.update(sample)
        dt_s = (
            config.default_dt_s
            if self._last_us is None
            else min((sample.timestamp_us - self._last_us) / 1_000_000, config.max_dt_s)
        )
        self._last_us = sample.timestamp_us

        if not sample.contact:
            self._in_contact = False
            return intent, FollowerCommand(self._x_mm, self._y_mm, config.hover_lift_mm, 0.0)

        if not self._in_contact:
            # New contact: start the filters at the sample, so the old point does not drag.
            self._filter_x.reset(sample.x_mm)
            self._filter_y.reset(sample.y_mm)
            self._filter_n.reset(sample.pressure_n)
            self._in_contact = True
        alpha = 1.0 - math.exp(-2 * math.pi * config.cutoff_hz * dt_s)
        target_x_mm = self._filter_x.update(sample.x_mm, alpha)
        target_y_mm = self._filter_y.update(sample.y_mm, alpha)
        pressure_n = self._filter_n.update(sample.pressure_n, alpha)

        dx_mm, dy_mm = target_x_mm - self._x_mm, target_y_mm - self._y_mm
        distance_mm = math.hypot(dx_mm, dy_mm)
        if distance_mm > config.jump_mm:
            # Travel to the new contact point lifted, without pressure.
            scale = config.travel_step_mm / distance_mm
            self._x_mm += dx_mm * scale
            self._y_mm += dy_mm * scale
            return intent, FollowerCommand(self._x_mm, self._y_mm, config.travel_lift_mm, 0.0)

        self._x_mm, self._y_mm = target_x_mm, target_y_mm
        return intent, FollowerCommand(self._x_mm, self._y_mm, 0.0, pressure_n)
