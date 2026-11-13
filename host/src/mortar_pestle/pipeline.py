"""Pipeline: one synchronous tick loop from Leader source to Follower.

Leader source -> Mode -> safety governor -> Follower -> recorder.
"""

from dataclasses import dataclass

from mortar_pestle.interpreter import Mode
from mortar_pestle.ports import Clock, Follower, LeaderSource, ManualClock
from mortar_pestle.recorder import Recorder, TickRecord
from mortar_pestle.safety import SafetyEnvelope, govern
from mortar_pestle.types import IDLE_INTENT, FollowerCommand, FollowerState, GrindIntent


@dataclass(frozen=True)
class TickResult:
    record: TickRecord
    state: FollowerState


class Pipeline:
    def __init__(
        self,
        leader: LeaderSource,
        mode: Mode,
        follower: Follower,
        envelope: SafetyEnvelope,
        clock: Clock,
        recorder: Recorder | None = None,
    ) -> None:
        self._leader = leader
        self._mode = mode
        self._follower = follower
        self._envelope = envelope
        self._clock = clock
        self._recorder = recorder
        self._last_valid_sample_us: int | None = None
        self._intent: GrindIntent = IDLE_INTENT
        # Start lifted, so the first commands lower the pestle at a safe speed.
        self._command = FollowerCommand(0.0, 0.0, envelope.max_lift_mm / 2, 0.0)
        self._safe_command = self._command

    def tick(self) -> TickResult:
        """Run one tick: read, interpret, govern, execute, record."""
        now_us = self._clock.now_us()
        sample = self._leader.read()
        if sample is not None:
            self._last_valid_sample_us = now_us
            self._intent, self._command = self._mode.step(sample)

        self._safe_command = govern(
            self._command,
            self._safe_command,
            now_us=now_us,
            last_valid_sample_us=self._last_valid_sample_us,
            envelope=self._envelope,
        )
        state = self._follower.execute(self._safe_command)

        record = TickRecord(now_us, sample, self._intent, self._safe_command)
        if self._recorder is not None:
            self._recorder.write(record)
        return TickResult(record, state)


def run_ticks(pipeline: Pipeline, clock: ManualClock, ticks: int, tick_us: int) -> list[TickResult]:
    """Run a fixed number of ticks on simulated time."""
    results = []
    for _ in range(ticks):
        results.append(pipeline.tick())
        clock.advance(tick_us)
    return results
