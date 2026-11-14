"""Pyodide glue: run the unchanged host pipeline against JS Leader and Follower adapters.

The browser has no PyBullet. The JS side owns the MuJoCo world and passes two objects in:
a Leader with read() and a Follower with execute(). This file wraps them as the two seam
ports, so Pipeline, the Modes and the safety governor run exactly as in the host.
"""

from typing import Any

from js import Object
from pyodide.ffi import to_js

from mortar_pestle.cli import DEMO_STROKES, ENVELOPE, MODES
from mortar_pestle.leaders import synthetic_samples
from mortar_pestle.pipeline import Pipeline
from mortar_pestle.ports import ManualClock
from mortar_pestle.types import FollowerCommand, FollowerState, LeaderSample


def _js(value: Any) -> Any:
    return to_js(value, dict_converter=Object.fromEntries)


class JsLeader:
    """LeaderSource port over a JS object with read()."""

    def __init__(self, js: Any) -> None:
        self._js = js

    def read(self) -> LeaderSample:
        s = self._js.read()
        q = s.quat_wxyz.to_py()
        return LeaderSample(
            timestamp_us=int(s.timestamp_us),
            quat_wxyz=(q[0], q[1], q[2], q[3]),
            pressure_n=float(s.pressure_n),
            x_mm=float(s.x_mm),
            y_mm=float(s.y_mm),
            contact=bool(s.contact),
        )


class JsFollower:
    """Follower port over a JS object with execute()."""

    def __init__(self, js: Any) -> None:
        self._js = js

    def execute(self, command: FollowerCommand) -> FollowerState:
        s = self._js.execute(
            _js(
                {
                    "x_mm": command.x_mm,
                    "y_mm": command.y_mm,
                    "lift_mm": command.lift_mm,
                    "pressure_n": command.pressure_n,
                }
            )
        )
        return FollowerState(float(s.x_mm), float(s.y_mm), float(s.z_mm), float(s.contact_force_n))


class Session:
    """One pipeline plus its clock. JS calls tick() once per 10 ms of simulated time."""

    def __init__(self, leader: Any, follower: Any, mode: str) -> None:
        self._leader = JsLeader(leader)
        self._follower = JsFollower(follower)
        self._clock = ManualClock()
        self.mode = mode
        self._pipeline = self._build(mode)

    def _build(self, mode: str) -> Pipeline:
        return Pipeline(self._leader, MODES[mode](), self._follower, ENVELOPE, self._clock)

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        self._pipeline = self._build(mode)

    def tick(self) -> Any:
        result = self._pipeline.tick()
        self._clock.advance(ENVELOPE.tick_us)
        record, state = result.record, result.state
        intent, command = record.intent, record.command
        return _js(
            {
                "intent": {
                    "pattern": str(intent.pattern),
                    "rate_hz": intent.rate_hz,
                    "radius_mm": intent.radius_mm,
                    "direction": str(intent.direction),
                    "requested_pressure_n": intent.requested_pressure_n,
                },
                "command": {
                    "x_mm": command.x_mm,
                    "y_mm": command.y_mm,
                    "lift_mm": command.lift_mm,
                    "pressure_n": command.pressure_n,
                },
                "state": {
                    "x_mm": state.x_mm,
                    "y_mm": state.y_mm,
                    "z_mm": state.z_mm,
                    "contact_force_n": state.contact_force_n,
                },
                "leader_force_n": record.sample.pressure_n if record.sample else 0.0,
            }
        )


def make_session(leader: Any, follower: Any, mode: str) -> Session:
    return Session(leader, follower, mode)


def demo_hand() -> Any:
    """The scripted hand: [x_mm, y_mm, down (0 or 1), force_n] per 10 ms tick."""
    samples = synthetic_samples(DEMO_STROKES)
    return _js([[s.x_mm, s.y_mm, 1 if s.contact else 0, s.pressure_n] for s in samples])
