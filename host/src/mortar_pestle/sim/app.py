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
from mortar_pestle.sim.world import SimWorld, Station, Vec3
from mortar_pestle.types import Pattern

MIN_FORCE_N = 1.0
MAX_FORCE_N = 15.0
FORCE_STEP_N = 1.0
START_FORCE_N = 6.0
TRAIL_EVERY_TICKS = 2
TRAIL_DOTS = 60
GAUGE_M_PER_N = 0.006  # 15 N is 90 mm of gauge height
LABEL_FORWARD_M = 0.085  # labels sit in front of the mortars, towards the camera
LEADER_RGB = (0.2, 0.5, 0.9)
FOLLOWER_RGB = (0.85, 0.45, 0.2)
CAMERA_DISTANCE_M = 0.40
CAMERA_YAW_DEG = 65.0
CAMERA_PITCH_DEG = -35.0
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
        for event in p.getMouseEvents(physicsClientId=self._client) or ():
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
        keys = p.getKeyboardEvents(physicsClientId=self._client) or {}
        if keys.get(SPACE_KEY, 0) & p.KEY_WAS_TRIGGERED:
            self._toggled_down = not self._toggled_down
        if keys.get(p.B3G_UP_ARROW, 0) & p.KEY_WAS_TRIGGERED:
            self.force_n = min(self.force_n + FORCE_STEP_N, MAX_FORCE_N)
        if keys.get(p.B3G_DOWN_ARROW, 0) & p.KEY_WAS_TRIGGERED:
            self.force_n = max(self.force_n - FORCE_STEP_N, MIN_FORCE_N)


class Display:
    """Camera, labels, force gauges and tip trails in the PyBullet window.

    Adding or changing debug text or lines blocks PyBullet for about 55 ms per call.
    So text is drawn once at start-up. Everything that changes per tick is a small
    body that gets moved, which is cheap.
    """

    def __init__(self, world: SimWorld, leader: Station | None, follower: Station) -> None:
        self._client = world.client
        self._leader = leader
        self._follower = follower
        self._ticks = 0
        if world.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self._client)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0, physicsClientId=self._client)
            p.configureDebugVisualizer(
                p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0, physicsClientId=self._client
            )
            p.resetDebugVisualizerCamera(
                cameraDistance=CAMERA_DISTANCE_M,
                cameraYaw=CAMERA_YAW_DEG,
                cameraPitch=CAMERA_PITCH_DEG,
                cameraTargetPosition=list(camera_target(leader is not None)),
                physicsClientId=self._client,
            )

        # Both force gauges stand side by side between the mortars, so their heights compare.
        gauge_x = follower.origin_m[0]
        middle_y = (
            follower.origin_m[1] - 0.10 if leader is not None else follower.origin_m[1] - 0.08
        )
        self._follower_gauge = _Gauge(self._client, (gauge_x, middle_y + 0.014), FOLLOWER_RGB)
        self._follower_trail = _Trail(self._client, follower, FOLLOWER_RGB)
        self._leader_gauge: _Gauge | None = None
        self._leader_trail: _Trail | None = None
        self._push_marker: int | None = None
        if leader is not None:
            self._leader_gauge = _Gauge(self._client, (gauge_x, middle_y - 0.014), LEADER_RGB)
            self._leader_trail = _Trail(self._client, leader, LEADER_RGB)
            self._push_marker = _marker(self._client, [0.007, 0.013, 0.001], [0.95, 0.95, 0.95])
            self._label("LEADER", leader, LEADER_RGB)
        self._label("FOLLOWER", follower, FOLLOWER_RGB)
        world.show()

    def update(self, result: TickResult, hand_force_n: float | None = None) -> None:
        self._ticks += 1
        sample = result.record.sample
        self._follower_gauge.show(result.state.contact_force_n)
        if self._leader_gauge is not None:
            self._leader_gauge.show(sample.pressure_n if sample is not None else 0.0)
            if self._push_marker is not None and hand_force_n is not None:
                self._leader_gauge.place(self._push_marker, hand_force_n)
        if self._ticks % TRAIL_EVERY_TICKS == 0:
            self._follower_trail.add(result.state.contact_force_n > 0.0)
            if self._leader_trail is not None:
                self._leader_trail.add(sample is not None and sample.contact)

    def _label(self, text: str, station: Station, colour: Sequence[float]) -> None:
        # Text grows to the right of its anchor on screen. Each label starts at the left
        # edge of its own mortar, on the floor in front of it, clear of the arm and gauges.
        p.addUserDebugText(
            text,
            [station.origin_m[0] + LABEL_FORWARD_M, station.origin_m[1] - 0.05, 0.0],
            textColorRGB=list(colour),
            textSize=1.3,
            physicsClientId=self._client,
        )


def camera_target(with_leader: bool) -> Vec3:
    """Return the point the camera looks at."""
    return (0.19, -0.10 if with_leader else 0.0, 0.03)


def _marker(client: int, half_extents_m: list[float], colour: Sequence[float]) -> int:
    visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=half_extents_m, rgbaColor=[*colour, 1], physicsClientId=client
    )
    marker_id: int = p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=visual, basePosition=[0, 0, -1], physicsClientId=client
    )
    return marker_id


class _Gauge:
    """A post with a block that rises with force. Tick marks every 5 N."""

    def __init__(self, client: int, xy_m: tuple[float, float], colour: Sequence[float]) -> None:
        self._client = client
        self._xy_m = xy_m
        height_m = GAUGE_M_PER_N * MAX_FORCE_N
        post = _marker(client, [0.002, 0.002, height_m / 2], [0.3, 0.3, 0.3])
        p.resetBasePositionAndOrientation(
            post, [xy_m[0], xy_m[1], height_m / 2], [0, 0, 0, 1], physicsClientId=client
        )
        for force_n in range(0, int(MAX_FORCE_N) + 1, 5):
            self.place(_marker(client, [0.004, 0.008, 0.0007], [0.3, 0.3, 0.3]), force_n)
        self._block = _marker(client, [0.006, 0.011, 0.003], colour)

    def show(self, force_n: float) -> None:
        self.place(self._block, force_n)

    def place(self, marker_id: int, force_n: float) -> None:
        height_m = GAUGE_M_PER_N * min(max(force_n, 0.0), MAX_FORCE_N)
        p.resetBasePositionAndOrientation(
            marker_id,
            [self._xy_m[0], self._xy_m[1], 0.002 + height_m],
            [0, 0, 0, 1],
            physicsClientId=self._client,
        )


class _Trail:
    """Dots on the mortar floor where the pestle tip pressed recently."""

    def __init__(self, client: int, station: Station, colour: Sequence[float]) -> None:
        self._client = client
        self._station = station
        visual = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.0015, rgbaColor=[*colour, 1], physicsClientId=client
        )
        self._dots: list[int] = [
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=visual,
                basePosition=[0, 0, -1],
                physicsClientId=client,
            )
            for _ in range(TRAIL_DOTS)
        ]
        self._next = 0

    def add(self, pressing: bool) -> None:
        if not pressing:
            return
        x_mm, y_mm, _ = self._station.tip_mm()
        p.resetBasePositionAndOrientation(
            self._dots[self._next],
            list(self._station.to_world(x_mm, y_mm, 1.0)),
            [0, 0, 0, 1],
            physicsClientId=self._client,
        )
        self._next = (self._next + 1) % TRAIL_DOTS


def status_line(result: TickResult, mode_name: str, hand_force_n: float | None) -> str:
    """Return the one-line live readout for the terminal."""
    intent, sample = result.record.intent, result.record.sample
    parts = []
    if sample is not None:
        push = "" if hand_force_n is None else f" (push {hand_force_n:.0f} N)"
        parts.append(f"Leader {sample.pressure_n:4.1f} N{push}")
    parts.append(f"Follower {result.state.contact_force_n:4.1f} N")
    parts.append(mode_name)
    if intent.pattern is Pattern.IDLE:
        parts.append("stroke: -")
    else:
        parts.append(
            f"stroke: {intent.pattern.value} {intent.rate_hz:.1f} Hz r={intent.radius_mm:.0f} mm "
            f"{intent.direction.value} {intent.requested_pressure_n:.1f} N"
        )
    return " | ".join(parts)


class RealTimePacer:
    """Sleeps so that one tick takes one tick of wall time. Reports the achieved speed."""

    def __init__(self, tick_us: int) -> None:
        self._tick_s = tick_us / 1_000_000
        self._next_s = time.monotonic()
        self._window_start_s = self._next_s
        self._window_ticks = 0
        self._speed = 1.0

    def wait(self) -> None:
        self._next_s += self._tick_s
        delay_s = self._next_s - time.monotonic()
        if delay_s > 0:
            time.sleep(delay_s)
        else:
            self._next_s = time.monotonic()  # running late: do not try to catch up
        self._window_ticks += 1
        if self._window_ticks == 100:
            now_s = time.monotonic()
            self._speed = 100 * self._tick_s / (now_s - self._window_start_s)
            self._window_start_s, self._window_ticks = now_s, 0

    def speed(self) -> float:
        """Return simulated time per wall time over the last 100 ticks. 1.0 is real time."""
        return self._speed
