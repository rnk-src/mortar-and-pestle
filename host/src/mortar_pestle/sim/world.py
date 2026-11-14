"""PyBullet world shared by the simulated Leader and the simulated Follower.

A Station is one mortar with one pestle. The pestle is a free rigid body.
Whatever holds it (the user's hand or the robot) does so through a
force-capped constraint, so the cap sets how hard the pestle can press.
"""

import math
import os
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import pybullet as p
import pybullet_data

Vec3 = tuple[float, float, float]

MORTAR_RADIUS_M = 0.05
MORTAR_FLOOR_HEIGHT_M = 0.01
MORTAR_WALL_HEIGHT_M = 0.025
FRICTION = 0.2
CONTACT_THRESHOLD_N = 0.2
PRESS_DEPTH_M = 0.003  # how far below the floor a pressing hold aims
HOLD_FORCE_N = 50.0  # hold force while the pestle is not pressing
MIN_PRESS_FORCE_N = 0.3


QUERY_ATTEMPTS = 5


def query(call: Callable[..., Any], *args: Any, **kwargs: Any) -> Any:
    """Run a PyBullet query. Retry it if it fails.

    In GUI mode a query now and then raises pybullet.error or returns None, although
    the client is still connected. The next attempt normally succeeds.
    Return None if no attempt answers. Raise if the last attempt raises.
    """
    for attempt in range(QUERY_ATTEMPTS):
        try:
            result = call(*args, **kwargs)
        except p.error:
            if attempt == QUERY_ATTEMPTS - 1:
                raise
            continue
        if result is not None:
            return result
    return None


def assets_dir() -> Path:
    """Return the robot model directory. MORTAR_PESTLE_ASSETS overrides the repo default."""
    override = os.environ.get("MORTAR_PESTLE_ASSETS")
    if override:
        return Path(override)
    return Path(__file__).resolve().parents[4] / "sim" / "assets"


@dataclass(frozen=True)
class PestleSpec:
    """A pestle: a rod with round ends."""

    radius_m: float
    length_m: float
    mass_kg: float
    colour: Vec3

    @property
    def weight_n(self) -> float:
        return self.mass_kg * 9.81


def press_cap_n(pressure_n: float, weight_n: float) -> float:
    """Return the hold-force cap that makes a pestle of weight_n press with pressure_n.

    The pestle's own weight also presses, so the hold supplies the rest.
    """
    return max(pressure_n - weight_n, MIN_PRESS_FORCE_N)


@dataclass(frozen=True)
class Contact:
    force_n: float
    x_mm: float  # centre of pressure in the mortar frame
    y_mm: float


class SimWorld:
    """Owns the PyBullet client and steps it one pipeline tick at a time."""

    def __init__(self, gui: bool = False, tick_us: int = 10_000, substeps: int = 5) -> None:
        self.client: int = p.connect(p.GUI if gui else p.DIRECT)
        self.gui = gui
        self._substeps = substeps
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        p.setTimeStep(tick_us / 1_000_000 / substeps, physicsClientId=self.client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.loadURDF("plane.urdf", physicsClientId=self.client)
        if gui:
            # Building the scene with rendering on costs about 50 ms per body.
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0, physicsClientId=self.client)

    def show(self) -> None:
        """Turn rendering on. Call it once the scene is built."""
        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1, physicsClientId=self.client)

    def step(self, before_substep: Callable[[], None] | None = None) -> None:
        """Advance the physics by one pipeline tick.

        before_substep runs before every physics substep. External forces last for one
        substep only, so a caller that applies one must do it there.
        """
        for _ in range(self._substeps):
            if before_substep is not None:
                before_substep()
            p.stepSimulation(physicsClientId=self.client)

    def close(self) -> None:
        p.disconnect(physicsClientId=self.client)


class Station:
    """One mortar and one pestle. The mortar frame origin is the centre of the floor top."""

    def __init__(
        self, world: SimWorld, centre_xy_m: tuple[float, float], pestle: PestleSpec
    ) -> None:
        self._client = world.client
        self.origin_m: Vec3 = (centre_xy_m[0], centre_xy_m[1], MORTAR_FLOOR_HEIGHT_M)
        self.mortar_id = self._create_mortar()
        self.pestle = pestle
        self.pestle_id = self._create_pestle()

    def to_world(self, x_mm: float, y_mm: float, z_mm: float) -> Vec3:
        """Convert a mortar-frame position to world metres."""
        return (
            self.origin_m[0] + x_mm / 1000,
            self.origin_m[1] + y_mm / 1000,
            self.origin_m[2] + z_mm / 1000,
        )

    def pestle_centre_for_tip(self, x_mm: float, y_mm: float, z_mm: float) -> Vec3:
        """Return the world position of an upright pestle's centre for a given tip position."""
        tip = self.to_world(x_mm, y_mm, z_mm)
        return (tip[0], tip[1], tip[2] + self.pestle.length_m / 2)

    def tip_mm(self) -> Vec3:
        """Return the pestle tip (its lowest point) in the mortar frame."""
        position, orientation = query(
            p.getBasePositionAndOrientation, self.pestle_id, physicsClientId=self._client
        )
        matrix = p.getMatrixFromQuaternion(orientation)
        axis = (matrix[2], matrix[5], matrix[8])  # pestle long axis in the world
        reach = self.pestle.length_m / 2 - self.pestle.radius_m  # centre of the tip sphere
        return (
            (position[0] - axis[0] * reach - self.origin_m[0]) * 1000,
            (position[1] - axis[1] * reach - self.origin_m[1]) * 1000,
            (position[2] - axis[2] * reach - self.pestle.radius_m - self.origin_m[2]) * 1000,
        )

    def pestle_quat_wxyz(self) -> tuple[float, float, float, float]:
        _, (x, y, z, w) = query(
            p.getBasePositionAndOrientation, self.pestle_id, physicsClientId=self._client
        )
        return (w, x, y, z)

    def contact(self) -> Contact:
        """Return the pestle-on-mortar force and its centre of pressure.

        This is what a load-cell plate under the mortar measures.
        """
        points = (
            query(
                p.getContactPoints,
                bodyA=self.pestle_id,
                bodyB=self.mortar_id,
                physicsClientId=self._client,
            )
            or ()
        )
        force_n = float(sum(point[9] for point in points))
        if force_n < CONTACT_THRESHOLD_N:
            tip = self.tip_mm()
            return Contact(0.0, tip[0], tip[1])
        x_m = sum(point[6][0] * point[9] for point in points) / force_n
        y_m = sum(point[6][1] * point[9] for point in points) / force_n
        return Contact(force_n, (x_m - self.origin_m[0]) * 1000, (y_m - self.origin_m[1]) * 1000)

    def _create_mortar(self) -> int:
        floor_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[MORTAR_RADIUS_M, MORTAR_RADIUS_M, MORTAR_FLOOR_HEIGHT_M / 2],
            physicsClientId=self._client,
        )
        floor_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=MORTAR_RADIUS_M,
            length=MORTAR_FLOOR_HEIGHT_M,
            rgbaColor=[0.75, 0.75, 0.72, 1],
            physicsClientId=self._client,
        )
        mortar_id: int = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=floor_collision,
            baseVisualShapeIndex=floor_visual,
            basePosition=[self.origin_m[0], self.origin_m[1], MORTAR_FLOOR_HEIGHT_M / 2],
            physicsClientId=self._client,
        )
        p.changeDynamics(mortar_id, -1, lateralFriction=FRICTION, physicsClientId=self._client)
        self._create_wall()
        return mortar_id

    def _create_wall(self) -> None:
        """Add a ring of wall segments. It is visual only."""
        segments = 24
        half_width = math.pi * MORTAR_RADIUS_M / segments
        wall_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.002, half_width, MORTAR_WALL_HEIGHT_M / 2],
            rgbaColor=[0.6, 0.6, 0.58, 1],
            physicsClientId=self._client,
        )
        for index in range(segments):
            angle = 2 * math.pi * index / segments
            p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=wall_visual,
                basePosition=[
                    self.origin_m[0] + MORTAR_RADIUS_M * math.cos(angle),
                    self.origin_m[1] + MORTAR_RADIUS_M * math.sin(angle),
                    self.origin_m[2] + MORTAR_WALL_HEIGHT_M / 2,
                ],
                baseOrientation=p.getQuaternionFromEuler([0, 0, angle]),
                physicsClientId=self._client,
            )

    def _create_pestle(self) -> int:
        pestle = self.pestle
        cylinder_height = pestle.length_m - 2 * pestle.radius_m
        collision = p.createCollisionShape(
            p.GEOM_CAPSULE,
            radius=pestle.radius_m,
            height=cylinder_height,
            physicsClientId=self._client,
        )
        visual = p.createVisualShape(
            p.GEOM_CAPSULE,
            radius=pestle.radius_m,
            length=cylinder_height,
            rgbaColor=[*pestle.colour, 1],
            physicsClientId=self._client,
        )
        pestle_id: int = p.createMultiBody(
            baseMass=pestle.mass_kg,
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=list(self.pestle_centre_for_tip(0.0, 0.0, 15.0)),
            physicsClientId=self._client,
        )
        p.changeDynamics(pestle_id, -1, lateralFriction=FRICTION, physicsClientId=self._client)
        return pestle_id
