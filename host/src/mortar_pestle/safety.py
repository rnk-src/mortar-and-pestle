"""Safety governor: turn any Follower Command into a safe Follower Command.

`govern` is a pure function. It holds no state and reads no clock.
"""

import math
from dataclasses import dataclass

from mortar_pestle.types import FollowerCommand


@dataclass(frozen=True)
class SafetyEnvelope:
    """Limits that every Follower Command must respect."""

    max_pressure_n: float = 15.0
    max_radius_mm: float = 40.0
    max_speed_mm_s: float = 500.0
    max_lift_mm: float = 30.0
    stale_after_us: int = 100_000
    stale_lift_mm: float = 5.0
    tick_us: int = 10_000


def govern(
    command: FollowerCommand,
    previous: FollowerCommand,
    *,
    now_us: int,
    last_valid_sample_us: int | None,
    envelope: SafetyEnvelope,
) -> FollowerCommand:
    """Return a safe Follower Command.

    previous is the safe Follower Command of the last tick.
    last_valid_sample_us is the time of the last valid Leader Sample, or None if none arrived.
    """
    stale = last_valid_sample_us is None or now_us - last_valid_sample_us > envelope.stale_after_us
    if stale:
        # Release pressure, lift a few millimetres and hold position.
        return FollowerCommand(
            previous.x_mm, previous.y_mm, max(previous.lift_mm, envelope.stale_lift_mm), 0.0
        )

    # Workspace.
    x_mm, y_mm = command.x_mm, command.y_mm
    radius_mm = math.hypot(x_mm, y_mm)
    if radius_mm > envelope.max_radius_mm:
        scale = envelope.max_radius_mm / radius_mm
        x_mm, y_mm = x_mm * scale, y_mm * scale
    lift_mm = min(max(command.lift_mm, 0.0), envelope.max_lift_mm)

    # Speed: limit the travel per tick, in the floor plane and vertically.
    max_step_mm = envelope.max_speed_mm_s * envelope.tick_us / 1_000_000
    dx_mm, dy_mm = x_mm - previous.x_mm, y_mm - previous.y_mm
    travel_mm = math.hypot(dx_mm, dy_mm)
    if travel_mm > max_step_mm:
        scale = max_step_mm / travel_mm
        x_mm, y_mm = previous.x_mm + dx_mm * scale, previous.y_mm + dy_mm * scale
    lift_step_mm = min(max(lift_mm - previous.lift_mm, -max_step_mm), max_step_mm)
    lift_mm = previous.lift_mm + lift_step_mm

    # Force: press only while the pestle is down.
    pressure_n = min(max(command.pressure_n, 0.0), envelope.max_pressure_n)
    if lift_mm > 0.0:
        pressure_n = 0.0

    return FollowerCommand(x_mm, y_mm, lift_mm, pressure_n)
