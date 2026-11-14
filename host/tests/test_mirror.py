import math

import pytest

from mortar_pestle.mirror import MirrorMode
from mortar_pestle.synthetic import StrokeSpec, generate_strokes
from mortar_pestle.types import FollowerCommand, LeaderSample, Pattern

LAG_TICKS = 6  # the tremor filter delays the motion by about 60 ms


def run(samples: list[LeaderSample]) -> list[FollowerCommand]:
    mode = MirrorMode()
    return [mode.step(sample)[1] for sample in samples]


def linear(**overrides: object) -> StrokeSpec:
    base: dict[str, object] = {
        "pattern": Pattern.LINEAR,
        "duration_s": 3.0,
        "rate_hz": 1.0,
        "radius_mm": 30.0,
        "axis_deg": 90.0,
        "pressure_n": 8.0,
    }
    base.update(overrides)
    return StrokeSpec(**base)  # type: ignore[arg-type]


def test_back_and_forth_stroke_is_followed_within_a_short_lag() -> None:
    samples = generate_strokes(linear())

    commands = run(samples)

    errors_mm = [
        math.hypot(command.x_mm - sample.x_mm, command.y_mm - sample.y_mm)
        for command, sample in zip(commands[100:], samples[100 - LAG_TICKS :], strict=False)
    ]
    assert max(errors_mm) < 3.0
    assert max(c.y_mm for c in commands) > 27.0
    assert min(c.y_mm for c in commands) < -27.0


def test_following_starts_on_the_first_tick_of_contact() -> None:
    commands = run(generate_strokes(linear(duration_s=0.3)))

    assert commands[0].lift_mm == 0.0
    assert commands[5].pressure_n == pytest.approx(8.0, abs=0.5)


def test_pressure_is_mirrored() -> None:
    commands = run(generate_strokes(linear(pressure_n=11.0)))

    assert all(c.lift_mm == 0.0 for c in commands)
    assert all(c.pressure_n == pytest.approx(11.0, abs=0.2) for c in commands[10:])


def test_tremor_is_attenuated() -> None:
    still = StrokeSpec(
        pattern=Pattern.CIRCULAR,
        duration_s=3.0,
        rate_hz=1.0,
        radius_mm=0.0,
        pressure_n=8.0,
        tremor_hz=8.0,
        tremor_mm=3.0,
    )

    commands = run(generate_strokes(still))[100:]

    # 3 mm of 8 Hz tremor at the Leader moves the Follower by less than 1 mm.
    assert max(abs(c.x_mm) for c in commands) < 1.0
    assert max(abs(c.y_mm) for c in commands) < 1.0


def test_lifting_the_pestle_lifts_the_follower_at_once_and_holds_position() -> None:
    samples = generate_strokes(linear(duration_s=1.1))
    lifted = generate_strokes(
        StrokeSpec(pattern=Pattern.IDLE, duration_s=0.5), start_us=samples[-1].timestamp_us + 10_000
    )

    commands = run(samples + lifted)

    last_down, first_up = commands[len(samples) - 1], commands[len(samples)]
    assert first_up.pressure_n == 0.0
    assert first_up.lift_mm > 0.0
    assert (first_up.x_mm, first_up.y_mm) == (last_down.x_mm, last_down.y_mm)
    assert commands[-1] == first_up


def test_pounding_at_several_points_presses_at_each_point() -> None:
    points = ((-20.0, -15.0), (20.0, 10.0), (0.0, 25.0))
    spec = StrokeSpec(
        pattern=Pattern.POUNDING, duration_s=3.0, rate_hz=1.0, pressure_n=10.0, points_mm=points
    )

    commands = run(generate_strokes(spec))

    for hit, (x_mm, y_mm) in enumerate(points):
        window = commands[hit * 100 : hit * 100 + 50]  # the pestle is down for the first 0.5 s
        pressed = [c for c in window if c.lift_mm == 0.0 and c.pressure_n > 8.0]
        assert pressed, hit
        assert all(math.hypot(c.x_mm - x_mm, c.y_mm - y_mm) < 1.0 for c in pressed), hit


def test_follower_travels_to_a_new_point_lifted_and_without_pressure() -> None:
    points = ((-25.0, 0.0), (25.0, 0.0))
    spec = StrokeSpec(
        pattern=Pattern.POUNDING, duration_s=2.0, rate_hz=1.0, pressure_n=10.0, points_mm=points
    )

    commands = run(generate_strokes(spec))

    travelling = [c for c in commands[100:150] if abs(c.x_mm - 25.0) > 1.0]
    assert travelling
    assert all(c.lift_mm > 0.0 and c.pressure_n == 0.0 for c in travelling)
    # 50 mm of travel takes well under 200 ms.
    assert len(travelling) < 20


def test_same_samples_give_identical_commands() -> None:
    samples = generate_strokes(linear(tremor_hz=6.0, tremor_mm=2.0, noise_mm=0.3, seed=5))

    assert run(samples) == run(samples)
