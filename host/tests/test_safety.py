import math

import pytest

from mortar_pestle.safety import SafetyEnvelope, govern
from mortar_pestle.types import FollowerCommand

ENVELOPE = SafetyEnvelope(
    max_pressure_n=15.0,
    max_radius_mm=40.0,
    max_speed_mm_s=500.0,
    max_lift_mm=30.0,
    stale_after_us=100_000,
    stale_lift_mm=5.0,
    tick_us=10_000,
)
NOW = 1_000_000
FRESH = NOW - 10_000


def govern_fresh(command: FollowerCommand, previous: FollowerCommand) -> FollowerCommand:
    return govern(command, previous, now_us=NOW, last_valid_sample_us=FRESH, envelope=ENVELOPE)


def test_command_inside_the_envelope_passes_unchanged() -> None:
    previous = FollowerCommand(10.0, 0.0, 0.0, 5.0)
    command = FollowerCommand(11.0, 1.0, 0.0, 6.0)

    assert govern_fresh(command, previous) == command


def test_pressure_is_clamped_to_the_envelope() -> None:
    previous = FollowerCommand(0.0, 0.0, 0.0, 5.0)

    safe = govern_fresh(FollowerCommand(0.0, 0.0, 0.0, 99.0), previous)

    assert safe.pressure_n == 15.0


def test_negative_pressure_is_clamped_to_zero() -> None:
    previous = FollowerCommand(0.0, 0.0, 0.0, 5.0)

    safe = govern_fresh(FollowerCommand(0.0, 0.0, 0.0, -3.0), previous)

    assert safe.pressure_n == 0.0


def test_no_pressure_while_lifted() -> None:
    previous = FollowerCommand(0.0, 0.0, 10.0, 0.0)

    safe = govern_fresh(FollowerCommand(0.0, 0.0, 10.0, 8.0), previous)

    assert safe.pressure_n == 0.0


def test_target_outside_the_workspace_is_pulled_onto_its_edge() -> None:
    previous = FollowerCommand(39.0, 0.0, 0.0, 5.0)

    safe = govern_fresh(FollowerCommand(42.0, 0.0, 0.0, 5.0), previous)

    assert math.hypot(safe.x_mm, safe.y_mm) == pytest.approx(40.0)
    assert safe.y_mm == pytest.approx(0.0)


def test_lift_is_clamped_to_the_workspace() -> None:
    previous = FollowerCommand(0.0, 0.0, 29.0, 0.0)

    safe = govern_fresh(FollowerCommand(0.0, 0.0, 80.0, 0.0), previous)

    assert safe.lift_mm == 30.0

    assert govern_fresh(FollowerCommand(0.0, 0.0, -4.0, 0.0), previous).lift_mm >= 0.0


def test_horizontal_speed_is_limited_per_tick() -> None:
    previous = FollowerCommand(0.0, 0.0, 0.0, 5.0)

    safe = govern_fresh(FollowerCommand(30.0, 0.0, 0.0, 5.0), previous)

    # 500 mm/s over a 10 ms tick allows 5 mm.
    assert safe.x_mm == pytest.approx(5.0)
    assert safe.y_mm == pytest.approx(0.0)


def test_vertical_speed_is_limited_per_tick() -> None:
    previous = FollowerCommand(0.0, 0.0, 20.0, 0.0)

    safe = govern_fresh(FollowerCommand(0.0, 0.0, 0.0, 5.0), previous)

    assert safe.lift_mm == pytest.approx(15.0)
    assert safe.pressure_n == 0.0


def test_stale_leader_data_releases_pressure_lifts_and_holds_position() -> None:
    previous = FollowerCommand(20.0, 10.0, 0.0, 8.0)
    command = FollowerCommand(25.0, 12.0, 0.0, 8.0)

    safe = govern(
        command, previous, now_us=NOW, last_valid_sample_us=NOW - 100_001, envelope=ENVELOPE
    )

    assert safe == FollowerCommand(20.0, 10.0, 5.0, 0.0)


def test_data_exactly_at_the_stale_limit_is_still_valid() -> None:
    previous = FollowerCommand(20.0, 10.0, 0.0, 8.0)
    command = FollowerCommand(21.0, 10.0, 0.0, 8.0)

    safe = govern(
        command, previous, now_us=NOW, last_valid_sample_us=NOW - 100_000, envelope=ENVELOPE
    )

    assert safe == command


def test_stale_hold_never_lowers_an_already_lifted_pestle() -> None:
    previous = FollowerCommand(0.0, 0.0, 12.0, 0.0)

    safe = govern(
        FollowerCommand(5.0, 0.0, 0.0, 8.0),
        previous,
        now_us=NOW,
        last_valid_sample_us=NOW - 500_000,
        envelope=ENVELOPE,
    )

    assert safe == FollowerCommand(0.0, 0.0, 12.0, 0.0)


def test_no_sample_ever_received_counts_as_stale() -> None:
    previous = FollowerCommand(0.0, 0.0, 10.0, 0.0)

    safe = govern(
        FollowerCommand(5.0, 0.0, 0.0, 8.0),
        previous,
        now_us=NOW,
        last_valid_sample_us=None,
        envelope=ENVELOPE,
    )

    assert safe == previous
