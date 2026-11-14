"""PyBullet Follower adapter: a simulated SO-101 that grinds with a pestle.

The arm is position-controlled in joint space with capped joint torque.
The pestle is coupled to the tool frame by a force-capped constraint. The
cap is the requested pressure, so the pestle presses as hard as commanded
and no harder.
"""

import pybullet as p

from mortar_pestle.sim.kinematics import ARM_JOINTS, REST_JOINTS_RAD, TOOL_LINK, ArmKinematics
from mortar_pestle.sim.world import (
    HOLD_FORCE_N,
    PESTLE_LENGTH_M,
    PESTLE_RADIUS_M,
    PRESS_DEPTH_M,
    SimWorld,
    Station,
    assets_dir,
    press_cap_n,
)
from mortar_pestle.types import FollowerCommand, FollowerState

GRIPPER_JOINT = 6
GRIPPER_CLOSED_RAD = 0.3
JOINT_TORQUE_CAP_NM = 10.0  # the SO-101 servo effort limit from the URDF
TIP_OFFSET_M = 0.04  # pestle tip below the tool frame origin
START_LIFT_MM = 15.0


class PyBulletFollower:
    """Follower adapter. The arm base sits at the world origin."""

    def __init__(self, world: SimWorld, station: Station) -> None:
        self._world = world
        self._client = world.client
        self._station = station
        urdf_path = assets_dir() / "so101" / "so101_new_calib.urdf"
        # Track the centre of the pestle's round tip. Its height above the floor does not
        # depend on how far the pestle tilts.
        self._kinematics = ArmKinematics(urdf_path, TIP_OFFSET_M - PESTLE_RADIUS_M)
        self._robot: int = p.loadURDF(
            str(urdf_path), useFixedBase=True, physicsClientId=self._client
        )

        self._joints_rad = self._kinematics.solve(
            self._tip_centre_m(0.0, 0.0, START_LIFT_MM), list(REST_JOINTS_RAD)
        )
        for joint, angle in zip(ARM_JOINTS, self._joints_rad, strict=True):
            p.resetJointState(self._robot, joint, angle, physicsClientId=self._client)
        p.resetJointState(
            self._robot, GRIPPER_JOINT, GRIPPER_CLOSED_RAD, physicsClientId=self._client
        )

        # The arm passes through its own pestle and mortar. Only pestle-on-mortar contact counts.
        for link in range(-1, p.getNumJoints(self._robot, physicsClientId=self._client)):
            for body in (station.pestle_id, station.mortar_id):
                p.setCollisionFilterPair(
                    self._robot, body, link, -1, enableCollision=0, physicsClientId=self._client
                )

        p.resetBasePositionAndOrientation(
            station.pestle_id,
            list(station.pestle_centre_for_tip(0.0, 0.0, START_LIFT_MM)),
            [0, 0, 0, 1],
            physicsClientId=self._client,
        )
        # Tool frame +z points along the pestle towards its tip. The pestle's own +z points
        # from tip to top, so the pestle is mounted flipped (half a turn about x).
        self._coupling: int = p.createConstraint(
            self._robot,
            TOOL_LINK,
            station.pestle_id,
            -1,
            p.JOINT_FIXED,
            [0, 0, 0],
            [0, 0, TIP_OFFSET_M - PESTLE_LENGTH_M / 2],
            [0, 0, 0],
            parentFrameOrientation=[1, 0, 0, 0],
            physicsClientId=self._client,
        )

    def close(self) -> None:
        self._kinematics.close()

    def _tip_centre_m(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        """Return the world position of the round tip's centre for a tip at the given point."""
        x_m, y_m, z_m = self._station.to_world(x_mm, y_mm, z_mm)
        return (x_m, y_m, z_m + PESTLE_RADIUS_M)

    def execute(self, command: FollowerCommand) -> FollowerState:
        pressing = command.lift_mm <= 0.0 and command.pressure_n > 0.0
        z_mm = -PRESS_DEPTH_M * 1000 if pressing else max(command.lift_mm, 0.5)
        self._joints_rad = self._kinematics.solve(
            self._tip_centre_m(command.x_mm, command.y_mm, z_mm), self._joints_rad
        )
        p.setJointMotorControlArray(
            self._robot,
            list(ARM_JOINTS),
            p.POSITION_CONTROL,
            targetPositions=self._joints_rad,
            forces=[JOINT_TORQUE_CAP_NM] * len(ARM_JOINTS),
            physicsClientId=self._client,
        )
        p.setJointMotorControl2(
            self._robot,
            GRIPPER_JOINT,
            p.POSITION_CONTROL,
            targetPosition=GRIPPER_CLOSED_RAD,
            force=JOINT_TORQUE_CAP_NM,
            physicsClientId=self._client,
        )
        p.changeConstraint(
            self._coupling,
            maxForce=press_cap_n(command.pressure_n) if pressing else HOLD_FORCE_N,
            physicsClientId=self._client,
        )

        self._world.step()

        x_mm, y_mm, tip_z_mm = self._station.tip_mm()
        return FollowerState(x_mm, y_mm, tip_z_mm, self._station.contact().force_n)
