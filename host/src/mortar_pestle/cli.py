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

from mortar_pestle.interpreter import InterpretMode, Mode
from mortar_pestle.leaders import ReplayLeader, SyntheticLeader
from mortar_pestle.mirror import MirrorMode
from mortar_pestle.pipeline import Pipeline
from mortar_pestle.ports import Follower, LeaderSource, ManualClock
from mortar_pestle.recorder import Recorder, leader_samples, read_recording
from mortar_pestle.safety import SafetyEnvelope
from mortar_pestle.synthetic import StrokeSpec
from mortar_pestle.types import Direction, Pattern

if TYPE_CHECKING:
    from mortar_pestle.sim.app import Display
    from mortar_pestle.sim.world import SimWorld

ENVELOPE = SafetyEnvelope()
STATUS_EVERY_TICKS = 10

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
        duration_s=5.0,
        rate_hz=1.2,
        radius_mm=30.0,
        direction=Direction.CW,
        pressure_n=8.0,
        tremor_hz=7.0,
        tremor_mm=2.5,
    ),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=1.0),
    StrokeSpec(
        pattern=Pattern.LINEAR,
        duration_s=5.0,
        rate_hz=1.0,
        radius_mm=30.0,
        axis_deg=0.0,
        pressure_n=12.0,
    ),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=1.0),
    StrokeSpec(
        pattern=Pattern.POUNDING,
        duration_s=6.0,
        rate_hz=1.0,
        pressure_n=10.0,
        points_mm=((-20.0, -15.0), (20.0, 10.0), (0.0, 25.0)),
    ),
    StrokeSpec(pattern=Pattern.IDLE, duration_s=1.0),
    StrokeSpec(
        pattern=Pattern.CIRCULAR,
        duration_s=5.0,
        rate_hz=0.8,
        radius_mm=20.0,
        direction=Direction.CCW,
        pressure_n=5.0,
    ),
)
MODES: dict[str, Callable[[], Mode]] = {"mirror": MirrorMode, "interpret": InterpretMode}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="mortar-pestle", description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)

    sim = commands.add_parser("sim", help="simulated Leader next to the simulated Follower")
    sim.add_argument("--auto", action="store_true", help="a scripted hand moves the Leader pestle")
    _add_mode_argument(sim)
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
    _add_mode_argument(replay)

    synth = commands.add_parser("synth", help="record a synthetic session (headless)")
    synth.add_argument("recording", type=Path)

    args = parser.parse_args(argv)
    if args.command == "sim":
        if args.headless and not args.auto:
            parser.error("--headless needs --auto")
        return _run_sim(
            args.mode, args.auto, not args.headless, args.seconds, args.record, args.mouse_scale
        )
    if args.command == "replay":
        samples = leader_samples(read_recording(args.recording))
        return _run_follower_only(
            args.mode, ReplayLeader(samples), len(samples) + 100, args.gui, None
        )
    ticks = sum(round(spec.duration_s * spec.sample_hz) for spec in SAMPLE_STROKES)
    # The sample recording keeps Interpret Mode, whose Grind Intents the replay tests compare.
    return _run_follower_only(
        "interpret", SyntheticLeader(SAMPLE_STROKES), ticks, False, args.recording
    )


def _add_mode_argument(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--mode",
        choices=sorted(MODES),
        default="mirror",
        help="mirror: follow the Leader directly. interpret: estimate the stroke, then repeat it",
    )


def _run_sim(
    mode_name: str,
    auto: bool,
    gui: bool,
    seconds: float | None,
    record: Path | None,
    mouse_scale: float,
) -> int:
    # PyBullet is imported here so that `--help` works without it.
    from mortar_pestle.sim.app import Display, MouseHand
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

    clock = ManualClock()
    world = SimWorld(gui=gui)
    leader_station = Station(world, LEADER_MORTAR_XY_M, LEADER_COLOUR)
    follower_station = Station(world, FOLLOWER_MORTAR_XY_M, FOLLOWER_COLOUR)
    mouse = None if auto else MouseHand(world, leader_station, mouse_scale)
    hand: Hand = scripted_hand(DEMO_STROKES, loop=True) if mouse is None else mouse
    leader = SimLeader(world, leader_station, hand, clock)
    follower = PyBulletFollower(world, follower_station)
    display = Display(world, leader_station, follower_station) if gui else None
    if gui:
        print(_SIM_HELP_AUTO if auto else _SIM_HELP_MOUSE)

    max_ticks = None if seconds is None else round(seconds * 1_000_000 / ENVELOPE.tick_us)
    try:
        _loop(
            mode_name,
            leader,
            follower,
            clock,
            record,
            max_ticks,
            display,
            (lambda: mouse.force_n) if mouse is not None else None,
            world,
        )
    finally:
        follower.close()
        _close(world)
    return 0


def _run_follower_only(
    mode_name: str, leader: LeaderSource, ticks: int, gui: bool, record: Path | None
) -> int:
    from mortar_pestle.sim.app import Display
    from mortar_pestle.sim.follower import PyBulletFollower
    from mortar_pestle.sim.scene import FOLLOWER_COLOUR, FOLLOWER_MORTAR_XY_M
    from mortar_pestle.sim.world import SimWorld, Station

    clock = ManualClock()
    world = SimWorld(gui=gui)
    station = Station(world, FOLLOWER_MORTAR_XY_M, FOLLOWER_COLOUR)
    follower = PyBulletFollower(world, station)
    display = Display(world, None, station) if gui else None
    try:
        _loop(mode_name, leader, follower, clock, record, ticks, display, None, world)
    finally:
        follower.close()
        _close(world)
    return 0


def _loop(
    mode_name: str,
    leader: LeaderSource,
    follower: Follower,
    clock: ManualClock,
    record: Path | None,
    max_ticks: int | None,
    display: "Display | None",
    hand_force_n: Callable[[], float] | None,
    world: "SimWorld",
) -> None:
    """Run the pipeline. With a display, pace it to real time and print a live readout."""
    import pybullet as p

    from mortar_pestle.sim.app import RealTimePacer, status_line

    recorder = Recorder(record) if record is not None else None
    pipeline = Pipeline(leader, MODES[mode_name](), follower, ENVELOPE, clock, recorder)
    pacer = RealTimePacer(ENVELOPE.tick_us) if display is not None else None
    ticks = 0
    try:
        while (max_ticks is None or ticks < max_ticks) and p.isConnected(world.client):
            result = pipeline.tick()
            clock.advance(ENVELOPE.tick_us)
            ticks += 1
            if display is not None and pacer is not None:
                push_n = hand_force_n() if hand_force_n is not None else None
                display.update(result, push_n)
                if ticks % STATUS_EVERY_TICKS == 0:
                    speed = pacer.speed()
                    line = f"{status_line(result, mode_name, push_n)} | speed {speed:.2f}x"
                    print(f"\r{line:<118}", end="", flush=True)
                pacer.wait()
    except KeyboardInterrupt:
        pass
    except p.error:
        pass  # the window was closed
    finally:
        if display is not None:
            print()
        if recorder is not None:
            recorder.close()


def _close(world: "SimWorld") -> None:
    import pybullet as p

    # The client is already gone if the user closed the window.
    with contextlib.suppress(p.error):
        world.close()


_SIM_HELP_MOUSE = """
Mortar & Pestle simulation
  Blue pestle = Leader (you).  Orange pestle on the arm = Follower.
  Move the mouse over the blue mortar: the pestle follows the cursor.
  Hold the left mouse button (or tap SPACE to toggle) to press down.
  The arm follows your pestle: circles, back and forth, pounding at any point.
  Up / down arrows: push harder / softer (the white mark on the blue gauge).
  The two gauges between the mortars show Leader force (blue) and Follower force (orange).
  Ctrl + drag rotates the camera. Close the window or press Ctrl+C here to quit.
"""
_SIM_HELP_AUTO = """
Mortar & Pestle simulation (scripted hand)
  Blue pestle = Leader.  Orange pestle on the arm = Follower.
  The script grinds in circles with tremor, strokes back and forth,
  pounds at three points, then grinds the other way.
  The two gauges between the mortars show Leader force (blue) and Follower force (orange).
  Close the window or press Ctrl+C here to quit.
"""


if __name__ == "__main__":
    raise SystemExit(main())
