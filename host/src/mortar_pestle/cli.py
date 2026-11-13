"""Command line entry point.

mortar-pestle sim              move the Leader pestle with the mouse, watch the Follower
mortar-pestle sim --auto       a scripted hand moves the Leader pestle
mortar-pestle replay FILE      replay a recorded session through the Follower
mortar-pestle synth FILE       record a synthetic session (headless)
"""

import argparse
import contextlib
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING

from mortar_pestle.interpreter import InterpretMode
from mortar_pestle.leaders import ReplayLeader, SyntheticLeader
from mortar_pestle.pipeline import Pipeline, TickResult
from mortar_pestle.ports import Follower, LeaderSource, ManualClock
from mortar_pestle.recorder import Recorder, leader_samples, read_recording
from mortar_pestle.safety import SafetyEnvelope
from mortar_pestle.synthetic import StrokeSpec
from mortar_pestle.types import Direction, Pattern

if TYPE_CHECKING:
    from mortar_pestle.sim.world import SimWorld

SAMPLE_STROKES = (
    StrokeSpec(
        pattern=Pattern.CIRCULAR,
        duration_s=5.0,
        rate_hz=1.5,
        radius_mm=30.0,
        direction=Direction.CW,
        pressure_n=8.0,
    ),
)
DEMO_STROKES = (
    StrokeSpec(pattern=Pattern.IDLE, duration_s=1.0),
    StrokeSpec(
        pattern=Pattern.CIRCULAR,
        duration_s=7.0,
        rate_hz=1.2,
        radius_mm=30.0,
        direction=Direction.CW,
        pressure_n=8.0,
        tremor_hz=7.0,
        tremor_mm=2.5,
    ),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=2.5),
    StrokeSpec(
        pattern=Pattern.CIRCULAR,
        duration_s=7.0,
        rate_hz=0.8,
        radius_mm=20.0,
        direction=Direction.CCW,
        pressure_n=12.0,
    ),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=2.5),
    StrokeSpec(pattern=Pattern.POUNDING, duration_s=6.0, rate_hz=1.5, pressure_n=10.0),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=2.5),
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="mortar-pestle", description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)

    sim = commands.add_parser("sim", help="simulated Leader next to the simulated Follower")
    sim.add_argument("--auto", action="store_true", help="a scripted hand moves the Leader pestle")
    sim.add_argument("--headless", action="store_true", help="no window (needs --auto)")
    sim.add_argument("--seconds", type=float, help="stop after this much simulated time")
    sim.add_argument("--record", type=Path, help="write the session to this JSON Lines file")
    sim.add_argument(
        "--mouse-scale",
        type=float,
        default=1.0,
        help="multiply mouse coordinates by this (try 2 or 0.5 if the pestle misses the cursor)",
    )

    replay = commands.add_parser("replay", help="replay a recorded session through the Follower")
    replay.add_argument("recording", type=Path)
    replay.add_argument("--gui", action="store_true", help="show the PyBullet window")

    synth = commands.add_parser("synth", help="record a synthetic session (headless)")
    synth.add_argument("recording", type=Path)

    args = parser.parse_args(argv)
    if args.command == "sim":
        if args.headless and not args.auto:
            parser.error("--headless needs --auto")
        return _run_sim(args.auto, not args.headless, args.seconds, args.record, args.mouse_scale)
    if args.command == "replay":
        samples = leader_samples(read_recording(args.recording))
        return _run_follower_only(ReplayLeader(samples), len(samples) + 100, args.gui, None)
    ticks = sum(round(spec.duration_s * spec.sample_hz) for spec in SAMPLE_STROKES)
    return _run_follower_only(SyntheticLeader(SAMPLE_STROKES), ticks, False, args.recording)


def _run_sim(
    auto: bool, gui: bool, seconds: float | None, record: Path | None, mouse_scale: float
) -> int:
    # PyBullet is imported here so that the other commands' help works without a display.
    from mortar_pestle.sim.app import Display, MouseHand, RealTimePacer
    from mortar_pestle.sim.follower import PyBulletFollower
    from mortar_pestle.sim.leader import Hand, SimLeader
    from mortar_pestle.sim.scene import (
        FOLLOWER_COLOUR,
        FOLLOWER_MORTAR_XY_M,
        LEADER_COLOUR,
        LEADER_MORTAR_XY_M,
        scripted_hand,
    )
    from mortar_pestle.sim.world import SimWorld, Station

    envelope = SafetyEnvelope()
    clock = ManualClock()
    world = SimWorld(gui=gui)
    leader_station = Station(world, LEADER_MORTAR_XY_M, LEADER_COLOUR)
    follower_station = Station(world, FOLLOWER_MORTAR_XY_M, FOLLOWER_COLOUR)
    mouse = None if auto else MouseHand(world, leader_station, mouse_scale)
    hand: Hand = scripted_hand(DEMO_STROKES, loop=True) if mouse is None else mouse
    leader = SimLeader(world, leader_station, hand, clock)
    follower = PyBulletFollower(world, follower_station)
    display = Display(world, leader_station, follower_station) if gui else None
    pacer = RealTimePacer(envelope.tick_us) if gui else None

    if gui:
        print(_SIM_HELP_AUTO if auto else _SIM_HELP_MOUSE)

    def after_tick(result: TickResult) -> None:
        if display is not None:
            display.update(result, None if mouse is None else mouse.force_n)
        if pacer is not None:
            pacer.wait()

    max_ticks = None if seconds is None else round(seconds * 1_000_000 / envelope.tick_us)
    try:
        _loop(leader, follower, envelope, clock, record, max_ticks, after_tick, world.client)
    finally:
        follower.close()
        _close(world)
    return 0


def _run_follower_only(leader: LeaderSource, ticks: int, gui: bool, record: Path | None) -> int:
    from mortar_pestle.sim.app import Display, RealTimePacer
    from mortar_pestle.sim.follower import PyBulletFollower
    from mortar_pestle.sim.scene import FOLLOWER_COLOUR, FOLLOWER_MORTAR_XY_M
    from mortar_pestle.sim.world import SimWorld, Station

    envelope = SafetyEnvelope()
    clock = ManualClock()
    world = SimWorld(gui=gui)
    station = Station(world, FOLLOWER_MORTAR_XY_M, FOLLOWER_COLOUR)
    follower = PyBulletFollower(world, station)
    display = Display(world, None, station) if gui else None
    pacer = RealTimePacer(envelope.tick_us) if gui else None

    def after_tick(result: TickResult) -> None:
        if display is not None:
            display.update(result)
        if pacer is not None:
            pacer.wait()

    try:
        _loop(leader, follower, envelope, clock, record, ticks, after_tick, world.client)
    finally:
        follower.close()
        _close(world)
    return 0


def _loop(
    leader: LeaderSource,
    follower: Follower,
    envelope: SafetyEnvelope,
    clock: ManualClock,
    record: Path | None,
    max_ticks: int | None,
    after_tick: Callable[[TickResult], None],
    client: int,
) -> None:
    import pybullet as p

    recorder = Recorder(record) if record is not None else None
    pipeline = Pipeline(leader, InterpretMode(), follower, envelope, clock, recorder)
    ticks = 0
    try:
        while (max_ticks is None or ticks < max_ticks) and p.isConnected(client):
            after_tick(pipeline.tick())
            clock.advance(envelope.tick_us)
            ticks += 1
    except KeyboardInterrupt:
        pass
    except p.error:
        pass  # the window was closed
    finally:
        if recorder is not None:
            recorder.close()


def _close(world: "SimWorld") -> None:
    import pybullet as p

    # The client is already gone if the user closed the window.
    with contextlib.suppress(p.error):
        world.close()


_SIM_HELP_MOUSE = """
Mortar & Pestle simulation
  Blue pestle  = Leader (you).  Orange pestle on the arm = Follower.
  Move the mouse over the blue mortar: the pestle follows the cursor.
  Hold the left mouse button (or tap SPACE to toggle) to press down.
  Grind in circles for about two seconds. The arm then copies the stroke.
  Up / down arrows: push harder / softer.
  Ctrl + drag rotates the camera. Close the window or press Ctrl+C here to quit.
"""
_SIM_HELP_AUTO = """
Mortar & Pestle simulation (scripted hand)
  Blue pestle  = Leader.  Orange pestle on the arm = Follower.
  The script grinds clockwise with tremor, then counterclockwise, then pounds.
  Close the window or press Ctrl+C here to quit.
"""


if __name__ == "__main__":
    raise SystemExit(main())
