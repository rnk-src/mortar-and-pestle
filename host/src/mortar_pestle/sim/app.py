"""Interactive PyBullet GUI: mouse hand, on-screen readouts and real-time pacing.

This module is glue around the PyBullet window. The ray maths is pure and tested.
"""

import math
import time
from collections.abc import Sequence
from typing import Any

import pybullet as p

from mortar_pestle.pipeline import TickResult
from mortar_pestle.sim.leader import MAX_RADIUS_MM, HandTarget
from mortar_pestle.sim.world import MORTAR_WALL_HEIGHT_M, SimWorld, Station, Vec3
from mortar_pestle.types import Pattern

MIN_FORCE_N = 1.0
MAX_FORCE_N = 15.0
FORCE_STEP_N = 1.0
START_FORCE_N = 6.0
HUD_EVERY_TICKS = 10
TRAIL_EVERY_TICKS = 2
TRAIL_LIFETIME_S = 2.5
SPACE_KEY = ord(" ")
LEFT_BUTTON = 0
MOUSE_MOVE_EVENT = 1
MOUSE_BUTTON_EVENT = 2


def mouse_ray(camera: Sequence[Any], mouse_x: float, mouse_y: float) -> tuple[Vec3, Vec3]:
    """Return the world ray (origin, direction) under a mouse position.

    camera is the tuple that pybullet.getDebugVisualizerCamera returns.
    """
    width, height = camera[0], camera[1]
    forward, horizontal, vertical = camera[5], camera[6], camera[7]
    distance, target = camera[10], camera[11]
    origin = tuple(target[i] - distance * forward[i] for i in range(3))
    far = 10_000.0
    centre = tuple(origin[i] + far * forward[i] for i in range(3))
    point = tuple(
        centre[i]
        - 0.5 * horizontal[i]
        + 0.5 * vertical[i]
        + mouse_x / width * horizontal[i]
        - mouse_y / height * vertical[i]
        for i in range(3)
    )
    direction = tuple(point[i] - origin[i] for i in range(3))
    return (origin[0], origin[1], origin[2]), (direction[0], direction[1], direction[2])


def intersect_plane_z(origin: Vec3, direction: Vec3, plane_z: float) -> tuple[float, float] | None:
    """Return the (x, y) where a ray meets a horizontal plane, or None if it never does."""
    if abs(direction[2]) < 1e-12:
        return None
    t = (plane_z - origin[2]) / direction[2]
    if t <= 0:
        return None
    return origin[0] + t * direction[0], origin[1] + t * direction[1]


class MouseHand:
    """The user's hand: the mouse moves the Leader pestle, a button or key presses it down.

    - Move the mouse over the Leader mortar: the pestle follows the cursor.
    - Hold the left mouse button, or tap SPACE to toggle: press down.
    - Up and down arrows: push harder or softer.
    """

    def __init__(self, world: SimWorld, station: Station, mouse_scale: float = 1.0) -> None:
        self._client = world.client
        self._station = station
        self._mouse_scale = mouse_scale
        self._x_mm = 0.0
        self._y_mm = 0.0
        self._button_down = False
        self._toggled_down = False
        self.force_n = START_FORCE_N

    def __call__(self) -> HandTarget:
        self._read_mouse()
        self._read_keyboard()
        return HandTarget(
            self._x_mm, self._y_mm, self._button_down or self._toggled_down, self.force_n
        )

    def _read_mouse(self) -> None:
        position = None
        for event in p.getMouseEvents(physicsClientId=self._client):
            event_type, x, y, button, state = event
            if event_type == MOUSE_MOVE_EVENT:
                position = (x * self._mouse_scale, y * self._mouse_scale)
            elif event_type == MOUSE_BUTTON_EVENT and button == LEFT_BUTTON:
                if state & p.KEY_WAS_TRIGGERED:
                    self._button_down = True
                if state & p.KEY_WAS_RELEASED:
                    self._button_down = False
        if position is None:
            return
        camera = p.getDebugVisualizerCamera(physicsClientId=self._client)
        origin, direction = mouse_ray(camera, *position)
        hit = intersect_plane_z(origin, direction, self._station.origin_m[2])
        if hit is None:
            return
        x_mm = (hit[0] - self._station.origin_m[0]) * 1000
        y_mm = (hit[1] - self._station.origin_m[1]) * 1000
        # Ignore the cursor while it is far from the Leader mortar.
        if math.hypot(x_mm, y_mm) <= 2 * MAX_RADIUS_MM:
            self._x_mm, self._y_mm = x_mm, y_mm

    def _read_keyboard(self) -> None:
        keys = p.getKeyboardEvents(physicsClientId=self._client)
        if keys.get(SPACE_KEY, 0) & p.KEY_WAS_TRIGGERED:
            self._toggled_down = not self._toggled_down
        if keys.get(p.B3G_UP_ARROW, 0) & p.KEY_WAS_TRIGGERED:
            self.force_n = min(self.force_n + FORCE_STEP_N, MAX_FORCE_N)
        if keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_WAS_TRIGGERED:
            self.force_n = max(self.force_n - FORCE_STEP_N, MIN_FORCE_N)


class Display:
    """Camera, labels, force readouts and tip trails in the PyBullet window."""

    def __init__(self, world: SimWorld, leader: Station | None, follower: Station) -> None:
        self._client = world.client
        self._leader = leader
        self._follower = follower
        self._ticks = 0
        self._text_ids: dict[str, int] = {}
        self._last_tips: dict[str, Vec3] = {}
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self._client)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0, physicsClientId=self._client)
        centre_y = follower.origin_m[1] if leader is None else -0.10
        p.resetDebugVisualizerCamera(
            cameraDistance=0.55,
            cameraYaw=90,
            cameraPitch=-50,
            cameraTargetPosition=[0.17, centre_y, 0.03],
            physicsClientId=self._client,
        )

    def update(self, result: TickResult, hand_force_n: float | None = None) -> None:
        self._ticks += 1
        if self._ticks % TRAIL_EVERY_TICKS == 0:
            if self._leader is not None:
                self._trail("leader", self._leader, [0.2, 0.5, 0.9])
            self._trail("follower", self._follower, [0.9, 0.45, 0.2])
        if self._ticks % HUD_EVERY_TICKS != 0:
            return

        intent, sample = result.record.intent, result.record.sample
        if self._leader is not None:
            leader_force_n = sample.pressure_n if sample is not None else 0.0
            setting = "" if hand_force_n is None else f"  (push {hand_force_n:.0f} N)"
            self._text(
                "leader", f"LEADER  {leader_force_n:4.1f} N{setting}", self._leader, [0.1, 0.3, 0.7]
            )
        self._text(
            "follower",
            f"FOLLOWER  {result.state.contact_force_n:4.1f} N",
            self._follower,
            [0.7, 0.3, 0.1],
        )
        if intent.pattern is Pattern.IDLE:
            line = "INTENT  idle (waiting for a stroke)"
        else:
            line = (
                f"INTENT  {intent.pattern.value}  {intent.rate_hz:.2f} Hz  "
                f"r={intent.radius_mm:.0f} mm  {intent.direction.value}  "
                f"{intent.requested_pressure_n:.1f} N"
            )
        self._text("intent", line, self._follower, [0.1, 0.1, 0.1], height_m=0.11)

    def _text(
        self, key: str, text: str, station: Station, colour: list[float], height_m: float = 0.07
    ) -> None:
        position = [station.origin_m[0], station.origin_m[1] - 0.05, station.origin_m[2] + height_m]
        arguments: dict[str, Any] = {
            "textColorRGB": colour,
            "textSize": 1.4,
            "physicsClientId": self._client,
        }
        if key in self._text_ids:
            arguments["replaceItemUniqueId"] = self._text_ids[key]
        self._text_ids[key] = p.addUserDebugText(text, position, **arguments)

    def _trail(self, key: str, station: Station, colour: list[float]) -> None:
        tip = station.to_world(*station.tip_mm())
        last = self._last_tips.get(key)
        self._last_tips[key] = tip
        if last is not None and tip[2] < station.origin_m[2] + MORTAR_WALL_HEIGHT_M:
            p.addUserDebugLine(
                list(last),
                list(tip),
                colour,
                lineWidth=2,
                lifeTime=TRAIL_LIFETIME_S,
                physicsClientId=self._client,
            )


class RealTimePacer:
    """Sleeps so that one tick takes one tick of wall time."""

    def __init__(self, tick_us: int) -> None:
        self._tick_s = tick_us / 1_000_000
        self._next_s = time.monotonic()

    def wait(self) -> None:
        self._next_s += self._tick_s
        delay_s = self._next_s - time.monotonic()
        if delay_s > 0:
            time.sleep(delay_s)
        else:
            self._next_s = time.monotonic()  # running late: do not try to catch up
