"""Simulated Leader adapter: a pestle in a sensed mortar, moved by a hand.

The hand is anything that says where it wants the pestle: the mouse, or a
scripted stroke. The pestle is a real rigid body. The Leader Sample comes
from its contact with the mortar, as a load-cell plate would measure it.
"""

import math
from collections.abc import Callable
from dataclasses import dataclass

import pybullet as p

from mortar_pestle.ports import Clock
from mortar_pestle.sim.world import (
    HOLD_FORCE_N,
    MORTAR_RADIUS_M,
    PESTLE_RADIUS_M,
    PRESS_DEPTH_M,
    SimWorld,
    Station,
    press_cap_n,
)
from mortar_pestle.types import LeaderSample

HOVER_MM = 15.0
MAX_RADIUS_MM = (MORTAR_RADIUS_M - PESTLE_RADIUS_M) * 1000


@dataclass(frozen=True)
class HandTarget:
    """Where the hand wants the pestle tip, and how hard it pushes down."""

    x_mm: float
    y_mm: float
    down: bool
    force_n: float


Hand = Callable[[], HandTarget]


class SimLeader:
    """Leader source adapter backed by a Station in the shared PyBullet world."""

    def __init__(self, world: SimWorld, station: Station, hand: Hand, clock: Clock) -> None:
        self._client = world.client
        self._station = station
        self._hand = hand
        self._clock = clock
        start = station.pestle_centre_for_tip(0.0, 0.0, HOVER_MM)
        self._hold: int = p.createConstraint(
            station.pestle_id,
            -1,
            -1,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, 0],
            list(start),
            physicsClientId=self._client,
        )

    def read(self) -> LeaderSample:
        """Apply the hand's current target, then return what the Leader sensors measure."""
        target = self._hand()
        radius_mm = math.hypot(target.x_mm, target.y_mm)
        scale = min(1.0, MAX_RADIUS_MM / radius_mm) if radius_mm > 0 else 1.0
        z_mm = -PRESS_DEPTH_M * 1000 if target.down else HOVER_MM
        p.changeConstraint(
            self._hold,
            list(
                self._station.pestle_centre_for_tip(target.x_mm * scale, target.y_mm * scale, z_mm)
            ),
            [0, 0, 0, 1],
            maxForce=press_cap_n(target.force_n) if target.down else HOLD_FORCE_N,
            physicsClientId=self._client,
        )

        contact = self._station.contact()
        return LeaderSample(
            timestamp_us=self._clock.now_us(),
            quat_wxyz=self._station.pestle_quat_wxyz(),
            pressure_n=contact.force_n,
            x_mm=contact.x_mm,
            y_mm=contact.y_mm,
            contact=contact.force_n > 0.0,
        )
