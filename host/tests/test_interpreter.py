import itertools
import math

import pytest

from mortar_pestle.interpreter import InterpretMode, Mode
from mortar_pestle.synthetic import StrokeSpec, generate_strokes
from mortar_pestle.types import (
    Direction,
    FollowerCommand,
    GrindIntent,
    LeaderSample,
    Pattern,
)


def run(mode: Mode, samples: list[LeaderSample]) -> list[tuple[GrindIntent, FollowerCommand]]:
    return [mode.step(sample) for sample in samples]


def circular(**overrides: object) -> StrokeSpec:
    base: dict[str, object] = {
        "pattern": Pattern.CIRCULAR,
        "rate_hz": 1.5,
        "radius_mm": 30.0,
        "direction": Direction.CW,
        "pressure_n": 8.0,
        "duration_s": 4.0,
    }
    base.update(overrides)
    return StrokeSpec(**base)  # type: ignore[arg-type]


def test_clockwise_circular_stroke_yields_matching_intent() -> None:
    intent, _ = run(InterpretMode(), generate_strokes(circular()))[-1]

    assert intent.pattern is Pattern.CIRCULAR
    assert intent.direction is Direction.CW
    assert intent.rate_hz == pytest.approx(1.5, abs=0.05)
    assert intent.radius_mm == pytest.approx(30.0, abs=1.0)
    assert intent.requested_pressure_n == pytest.approx(8.0, abs=0.5)


def test_counterclockwise_stroke_yields_ccw_direction() -> None:
    samples = generate_strokes(circular(direction=Direction.CCW, rate_hz=0.8, radius_mm=20.0))

    intent, _ = run(InterpretMode(), samples)[-1]

    assert intent.direction is Direction.CCW
    assert intent.rate_hz == pytest.approx(0.8, abs=0.05)
    assert intent.radius_mm == pytest.approx(20.0, abs=1.0)


def test_tremor_and_noise_do_not_change_the_intent_beyond_tolerance() -> None:
    clean, _ = run(InterpretMode(), generate_strokes(circular()))[-1]
    shaky_samples = generate_strokes(
        circular(tremor_hz=8.0, tremor_mm=3.0, noise_mm=0.5, noise_n=0.3, seed=7)
    )

    shaky, _ = run(InterpretMode(), shaky_samples)[-1]

    assert shaky.pattern is Pattern.CIRCULAR
    assert shaky.direction is clean.direction
    assert shaky.rate_hz == pytest.approx(clean.rate_hz, abs=0.08)
    assert shaky.radius_mm == pytest.approx(clean.radius_mm, abs=1.5)
    assert shaky.requested_pressure_n == pytest.approx(clean.requested_pressure_n, abs=0.5)


def test_intent_is_idle_before_the_stroke_window_fills() -> None:
    samples = generate_strokes(circular(duration_s=0.5))

    intent, command = run(InterpretMode(), samples)[-1]

    assert intent.pattern is Pattern.IDLE
    assert command.pressure_n == 0.0
    assert command.lift_mm > 0.0


def test_lifting_the_pestle_gives_idle() -> None:
    samples = generate_strokes(circular(duration_s=3.0))
    samples += generate_strokes(
        StrokeSpec(pattern=Pattern.IDLE, duration_s=2.0), start_us=samples[-1].timestamp_us + 10_000
    )

    intent, command = run(InterpretMode(), samples)[-1]

    assert intent.pattern is Pattern.IDLE
    assert command.pressure_n == 0.0
    assert command.lift_mm > 0.0


def test_pounding_stroke_yields_pounding_intent() -> None:
    samples = generate_strokes(
        StrokeSpec(pattern=Pattern.POUNDING, rate_hz=2.0, pressure_n=12.0, duration_s=4.0)
    )

    intent, _ = run(InterpretMode(), samples)[-1]

    assert intent.pattern is Pattern.POUNDING
    assert intent.rate_hz == pytest.approx(2.0, abs=0.15)
    assert intent.requested_pressure_n == pytest.approx(12.0, abs=1.5)


def test_intent_updates_at_ten_hertz() -> None:
    results = run(InterpretMode(), generate_strokes(circular(duration_s=3.0)))

    changes = sum(1 for (a, _), (b, _) in itertools.pairwise(results) if a != b)

    # 300 samples at 100 Hz give at most 30 Intent updates.
    assert 1 <= changes <= 30


def test_circular_intent_commands_a_pressed_circle_of_the_same_radius_and_rate() -> None:
    results = run(InterpretMode(), generate_strokes(circular(duration_s=6.0)))
    commands = [command for _, command in results[-200:]]

    radii = [math.hypot(c.x_mm, c.y_mm) for c in commands]
    assert min(radii) == pytest.approx(30.0, abs=1.0)
    assert max(radii) == pytest.approx(30.0, abs=1.0)
    assert all(c.lift_mm == 0.0 for c in commands)
    assert all(c.pressure_n == pytest.approx(8.0, abs=0.5) for c in commands)

    # Clockwise seen from above means the angle decreases.
    angles = [math.atan2(c.y_mm, c.x_mm) for c in commands]
    step = (angles[1] - angles[0] + math.pi) % (2 * math.pi) - math.pi
    assert step == pytest.approx(-2 * math.pi * 1.5 / 100, rel=0.05)


def test_pounding_intent_commands_a_centred_up_and_down_motion() -> None:
    samples = generate_strokes(
        StrokeSpec(pattern=Pattern.POUNDING, rate_hz=2.0, pressure_n=12.0, duration_s=6.0)
    )
    commands = [command for _, command in run(InterpretMode(), samples)[-200:]]

    assert all(math.hypot(c.x_mm, c.y_mm) < 1.0 for c in commands)
    assert any(c.lift_mm > 5.0 for c in commands)
    assert any(c.lift_mm == 0.0 and c.pressure_n > 10.0 for c in commands)
    assert all(c.pressure_n == 0.0 for c in commands if c.lift_mm > 0.0)


def test_same_samples_give_identical_intents() -> None:
    samples = generate_strokes(circular(tremor_hz=6.0, tremor_mm=2.0, noise_mm=0.3, seed=3))

    assert run(InterpretMode(), samples) == run(InterpretMode(), samples)
