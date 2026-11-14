"""Seam interfaces of the host pipeline, plus the injected clock."""

from typing import Protocol

from mortar_pestle.types import FollowerCommand, FollowerState, LeaderSample


class LeaderSource(Protocol):
    """Yields Leader Samples. Adapters: replay, synthetic, simulated; later serial."""

    def read(self) -> LeaderSample | None:
        """Return the Leader Sample of this tick, or None if no valid sample arrived."""
        ...


class Follower(Protocol):
    """Executes Follower Commands. Adapters: PyBullet; later MuJoCo, Gazebo, physical SO-101."""

    def execute(self, command: FollowerCommand) -> FollowerState:
        """Execute one Follower Command for one tick. Return the measured state."""
        ...


class Clock(Protocol):
    def now_us(self) -> int:
        """Return the current time in microseconds."""
        ...


class ManualClock:
    """A clock that moves only when told to. Simulations and tests use it."""

    def __init__(self, start_us: int = 0) -> None:
        self._now_us = start_us

    def now_us(self) -> int:
        return self._now_us

    def advance(self, dt_us: int) -> None:
        self._now_us += dt_us
