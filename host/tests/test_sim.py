"""Headless PyBullet tests for the simulated Follower, the simulated Leader and the pipeline."""

import itertools
import math
from collections.abc import Iterator
from pathlib import Path

import pytest

from mortar_pestle.interpreter import InterpretMode
from mortar_pestle.leaders import ReplayLeader, synthetic_samples
from mortar_pestle.pipeline import Pipeline, TickResult, run_ticks
from mortar_pestle.ports import ManualClock
from mortar_pestle.recorder import leader_samples, read_recording
from mortar_pestle.safety import SafetyEnvelope
from mortar_pestle.sim.follower import PyBulletFollower
from mortar_pestle.sim.leader import HandTarget, SimLeader
from mortar_pestle.sim.scene import FOLLOWER_MORTAR_XY_M, LEADER_MORTAR_XY_M, scripted_hand
from mortar_pestle.sim.world import SimWorld, Station
from mortar_pestle.synthetic import StrokeSpec
from mortar_pestle.types import Direction, FollowerCommand, LeaderSample, Pattern

ENVELOPE = SafetyEnvelope()
TICK_US = ENVELOPE.tick_us
RECORDING = Path(__file__).resolve().parents[2] / "recordings" / "circular_1p5hz_30mm.jsonl"
CIRCLE = StrokeSpec(
    pattern=Pattern.CIRCULAR,
    duration_s=6.0,
    rate_hz=1.5,
    radius_mm=30.0,
    direction=Direction.CW,
    pressure_n=8.0,
)


@pytest.fixture
def world() -> Iterator[SimWorld]:
    world = SimWorld(gui=False)
    yield world
    world.close()


@pytest.fixture
def follower(world: SimWorld) -> Iterator[PyBulletFollower]:
    follower = PyBulletFollower(world, Station(world, FOLLOWER_MORTAR_XY_M, (0.8, 0.4, 0.2)))
    yield follower
    follower.close()


def replay(
    follower: PyBulletFollower, samples: list[LeaderSample | None], ticks: int
) -> list[TickResult]:
    clock = ManualClock()
    pipeline = Pipeline(ReplayLeader(samples), InterpretMode(), follower, ENVELOPE, clock)
    return run_ticks(pipeline, clock, ticks, TICK_US)


# --- Follower adapter -------------------------------------------------------------------


def test_follower_holds_the_pestle_above_the_mortar_when_lifted(
    follower: PyBulletFollower,
) -> None:
    for _ in range(100):
        state = follower.execute(FollowerCommand(0.0, 0.0, 15.0, 0.0))

    assert state.z_mm == pytest.approx(15.0, abs=1.5)
    assert math.hypot(state.x_mm, state.y_mm) < 1.5
    assert state.contact_force_n == 0.0


@pytest.mark.parametrize("pressure_n", [3.0, 8.0, 15.0])
def test_follower_presses_with_the_requested_pressure(
    follower: PyBulletFollower, pressure_n: float
) -> None:
    for lift_mm in (10.0, 5.0, 0.0):
        follower.execute(FollowerCommand(0.0, 0.0, lift_mm, 0.0))
    for _ in range(100):
        state = follower.execute(FollowerCommand(0.0, 0.0, 0.0, pressure_n))

    assert state.contact_force_n == pytest.approx(pressure_n, abs=0.5)
    assert state.z_mm == pytest.approx(0.0, abs=0.5)


def test_follower_reaches_targets_across_the_whole_workspace(
    follower: PyBulletFollower,
) -> None:
    for angle_deg in range(0, 360, 45):
        x_mm = 40.0 * math.cos(math.radians(angle_deg))
        y_mm = 40.0 * math.sin(math.radians(angle_deg))
        for _ in range(60):
            state = follower.execute(FollowerCommand(x_mm, y_mm, 10.0, 0.0))

        assert math.hypot(state.x_mm - x_mm, state.y_mm - y_mm) < 2.0, angle_deg
        assert state.z_mm == pytest.approx(10.0, abs=2.0), angle_deg


# --- Simulated Leader adapter -----------------------------------------------------------


def test_sim_leader_reports_the_force_and_position_of_a_pressing_hand(world: SimWorld) -> None:
    clock = ManualClock()
    target = HandTarget(20.0, -10.0, True, 6.0)
    leader = SimLeader(
        world, Station(world, LEADER_MORTAR_XY_M, (0.2, 0.5, 0.9)), lambda: target, clock
    )

    for _ in range(100):
        sample = leader.read()
        world.step()
        clock.advance(TICK_US)

    assert sample.contact
    assert sample.pressure_n == pytest.approx(6.0, abs=0.5)
    assert sample.x_mm == pytest.approx(20.0, abs=1.0)
    assert sample.y_mm == pytest.approx(-10.0, abs=1.0)
    assert sample.timestamp_us == 99 * TICK_US


def test_sim_leader_reports_no_contact_for_a_hovering_hand(world: SimWorld) -> None:
    clock = ManualClock()
    target = HandTarget(0.0, 0.0, False, 6.0)
    leader = SimLeader(
        world, Station(world, LEADER_MORTAR_XY_M, (0.2, 0.5, 0.9)), lambda: target, clock
    )

    for _ in range(50):
        sample = leader.read()
        world.step()

    assert not sample.contact
    assert sample.pressure_n == 0.0


# --- End to end -------------------------------------------------------------------------


def test_replaying_the_sample_recording_traces_the_circular_stroke(
    follower: PyBulletFollower,
) -> None:
    samples = leader_samples(read_recording(RECORDING))

    results = replay(follower, samples, len(samples))

    settled = results[-150:]  # the last 1.5 s: more than two full strokes
    radii = [math.hypot(r.state.x_mm, r.state.y_mm) for r in settled]
    forces = [r.state.contact_force_n for r in settled]
    assert results[-1].record.intent.pattern is Pattern.CIRCULAR
    assert min(radii) > 27.0
    assert max(radii) < 33.0
    assert min(forces) > 7.0
    assert max(forces) < 9.0
    # Clockwise seen from above: the cross product of successive tip positions is negative.
    crosses = [
        a.state.x_mm * b.state.y_mm - a.state.y_mm * b.state.x_mm
        for a, b in itertools.pairwise(settled)
    ]
    assert all(cross < 0 for cross in crosses)


def test_cutting_the_stream_releases_and_lifts_within_100_ms_of_sim_time(
    follower: PyBulletFollower,
) -> None:
    samples = synthetic_samples([CIRCLE])
    cut = 400

    results = replay(follower, list(samples[:cut]), cut + 40)

    assert results[cut - 1].state.contact_force_n > 7.0
    # Data is stale 100 ms after the last sample. Allow 50 ms for the arm to lift.
    lifted = results[cut + 15]
    assert lifted.record.command.pressure_n == 0.0
    assert lifted.state.contact_force_n == 0.0
    assert lifted.state.z_mm > 2.0
    held = results[-1]
    assert held.state.z_mm == pytest.approx(ENVELOPE.stale_lift_mm, abs=1.5)
    assert (
        math.hypot(
            held.state.x_mm - lifted.record.command.x_mm,
            held.state.y_mm - lifted.record.command.y_mm,
        )
        < 2.0
    )


def test_replaying_the_same_recording_twice_gives_identical_intents(world: SimWorld) -> None:
    samples = leader_samples(read_recording(RECORDING))

    def intents(xy_m: tuple[float, float]) -> list[object]:
        follower = PyBulletFollower(world, Station(world, xy_m, (0.8, 0.4, 0.2)))
        try:
            return [r.record.intent for r in replay(follower, samples, len(samples))]
        finally:
            follower.close()

    assert intents(FOLLOWER_MORTAR_XY_M) == intents(FOLLOWER_MORTAR_XY_M)


def test_sim_leader_drives_the_follower_to_the_same_stroke_and_force(world: SimWorld) -> None:
    clock = ManualClock()
    leader = SimLeader(
        world,
        Station(world, LEADER_MORTAR_XY_M, (0.2, 0.5, 0.9)),
        scripted_hand([CIRCLE]),
        clock,
    )
    follower = PyBulletFollower(world, Station(world, FOLLOWER_MORTAR_XY_M, (0.8, 0.4, 0.2)))
    pipeline = Pipeline(leader, InterpretMode(), follower, ENVELOPE, clock)

    results = run_ticks(pipeline, clock, 600, TICK_US)
    follower.close()

    intent = results[-1].record.intent
    assert intent.pattern is Pattern.CIRCULAR
    assert intent.direction is Direction.CW
    assert intent.rate_hz == pytest.approx(1.5, abs=0.1)
    assert intent.radius_mm == pytest.approx(30.0, abs=2.0)
    settled = results[-150:]
    leader_force = sum(r.record.sample.pressure_n for r in settled if r.record.sample) / 150
    follower_force = sum(r.state.contact_force_n for r in settled) / 150
    assert follower_force == pytest.approx(leader_force, abs=0.75)
    assert all(27.0 < math.hypot(r.state.x_mm, r.state.y_mm) < 33.0 for r in settled)
