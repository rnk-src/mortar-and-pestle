import itertools
from pathlib import Path

from mortar_pestle.interpreter import InterpretMode
from mortar_pestle.leaders import ReplayLeader, SyntheticLeader, synthetic_samples
from mortar_pestle.pipeline import Pipeline, run_ticks
from mortar_pestle.ports import ManualClock
from mortar_pestle.recorder import Recorder, leader_samples, read_recording
from mortar_pestle.safety import SafetyEnvelope
from mortar_pestle.synthetic import StrokeSpec
from mortar_pestle.types import (
    Direction,
    FollowerCommand,
    FollowerState,
    GrindIntent,
    Pattern,
)

ENVELOPE = SafetyEnvelope()
TICK_US = ENVELOPE.tick_us
CIRCLE = StrokeSpec(
    pattern=Pattern.CIRCULAR,
    duration_s=4.0,
    rate_hz=1.5,
    radius_mm=30.0,
    direction=Direction.CW,
    pressure_n=8.0,
)


class FakeFollower:
    """Reaches every Follower Command at once and remembers it."""

    def __init__(self) -> None:
        self.commands: list[FollowerCommand] = []

    def execute(self, command: FollowerCommand) -> FollowerState:
        self.commands.append(command)
        return FollowerState(command.x_mm, command.y_mm, command.lift_mm, command.pressure_n)


def make_pipeline(
    leader: ReplayLeader, recorder: Recorder | None = None
) -> tuple[Pipeline, FakeFollower, ManualClock]:
    follower, clock = FakeFollower(), ManualClock()
    pipeline = Pipeline(leader, InterpretMode(), follower, ENVELOPE, clock, recorder)
    return pipeline, follower, clock


def test_circular_strokes_reach_the_follower_as_pressed_commands() -> None:
    pipeline, follower, clock = make_pipeline(SyntheticLeader([CIRCLE]))

    results = run_ticks(pipeline, clock, 400, TICK_US)

    assert results[-1].record.intent.pattern is Pattern.CIRCULAR
    assert follower.commands[-1].lift_mm == 0.0
    assert follower.commands[-1].pressure_n > 7.0


def test_follower_is_lowered_no_faster_than_the_safety_envelope_allows() -> None:
    pipeline, follower, clock = make_pipeline(SyntheticLeader([CIRCLE]))

    run_ticks(pipeline, clock, 400, TICK_US)

    max_step_mm = ENVELOPE.max_speed_mm_s * TICK_US / 1_000_000
    lifts = [command.lift_mm for command in follower.commands]
    assert all(abs(b - a) <= max_step_mm + 1e-9 for a, b in itertools.pairwise(lifts))


def test_stream_cut_releases_pressure_and_lifts_after_100_ms() -> None:
    samples = synthetic_samples([CIRCLE])
    pipeline, follower, clock = make_pipeline(ReplayLeader(samples[:300]))

    run_ticks(pipeline, clock, 320, TICK_US)

    # The last valid sample arrived at tick 299. Data is stale from tick 310 on.
    assert follower.commands[309].pressure_n > 7.0
    assert follower.commands[310].pressure_n == 0.0
    assert follower.commands[310].lift_mm == ENVELOPE.stale_lift_mm
    assert follower.commands[319] == follower.commands[310]


def test_recording_replays_to_identical_intents(tmp_path: Path) -> None:
    path = tmp_path / "session.jsonl"
    with Recorder(path) as recorder:
        pipeline, _, clock = make_pipeline(SyntheticLeader([CIRCLE]), recorder)
        run_ticks(pipeline, clock, 400, TICK_US)
    recorded = read_recording(path)

    def replay_intents() -> list[GrindIntent]:
        pipeline, _, clock = make_pipeline(ReplayLeader(leader_samples(recorded)))
        return [result.record.intent for result in run_ticks(pipeline, clock, 400, TICK_US)]

    first = replay_intents()
    assert first == replay_intents()
    assert first[-1].pattern is Pattern.CIRCULAR


def test_recorder_round_trips_records_with_missing_samples(tmp_path: Path) -> None:
    path = tmp_path / "session.jsonl"
    samples = synthetic_samples([CIRCLE])
    with Recorder(path) as recorder:
        pipeline, _, clock = make_pipeline(ReplayLeader([*samples[:5], None, None]), recorder)
        written = [result.record for result in run_ticks(pipeline, clock, 7, TICK_US)]

    records = read_recording(path)

    assert len(records) == 7
    assert [record.sample is None for record in records] == [False] * 5 + [True] * 2
    assert records[0].sample is not None and written[0].sample is not None
    assert records[0].sample.timestamp_us == written[0].sample.timestamp_us
    assert abs(records[3].sample.x_mm - written[3].sample.x_mm) < 1e-3  # type: ignore[union-attr]
    assert records[6].command == written[6].command
