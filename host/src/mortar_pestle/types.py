"""Domain value types shared by every host module.

Positions are in the mortar frame: origin at the centre of the mortar floor,
x and y in the floor plane, z up. Units are in the field names.
"""

from dataclasses import dataclass
from enum import StrEnum


class Pattern(StrEnum):
    IDLE = "idle"
    CIRCULAR = "circular"
    POUNDING = "pounding"
    LINEAR = "linear"  # back and forth along one axis; Interpret Mode does not estimate it yet


class Direction(StrEnum):
    """Stroke direction seen from above."""

    CW = "cw"
    CCW = "ccw"


@dataclass(frozen=True)
class LeaderSample:
    """One calibrated reading from the Leader."""

    timestamp_us: int
    quat_wxyz: tuple[float, float, float, float]
    pressure_n: float
    x_mm: float
    y_mm: float
    contact: bool


@dataclass(frozen=True)
class GrindIntent:
    """What the user means to do, estimated over a short stroke window."""

    pattern: Pattern
    rate_hz: float
    radius_mm: float
    direction: Direction
    requested_pressure_n: float


IDLE_INTENT = GrindIntent(Pattern.IDLE, 0.0, 0.0, Direction.CW, 0.0)


@dataclass(frozen=True)
class FollowerCommand:
    """The pestle-tip target that the Follower executes for one tick.

    lift_mm is the tip height above the mortar floor. The Follower presses
    with pressure_n only while lift_mm is zero.
    """

    x_mm: float
    y_mm: float
    lift_mm: float
    pressure_n: float


@dataclass(frozen=True)
class FollowerState:
    """What the Follower measured after it executed a Follower Command."""

    x_mm: float
    y_mm: float
    z_mm: float
    contact_force_n: float
