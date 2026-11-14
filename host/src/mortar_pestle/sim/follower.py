"""PyBullet Follower adapter: a simulated SO-101 that grinds with a stone pestle.

The arm is position-controlled in joint space with capped joint torque.

The gripper holds the pestle like a press actuator. A slider constraint keeps
the pestle rigid sideways and lets it slide a few millimetres along its own
axis. The adapter sets the force along that axis:
  - pressing: the requested pressure minus the pestle's weight;
  - not pressing: a stiff spring that holds the pestle in place and carries its weight.
The same force acts on the gripper in the opposite direction, so the arm feels the load.
"""

import pybullet as p

from mortar_pestle.sim.kinematics import ARM_JOINTS, REST_JOINTS_RAD, TOOL_LINK, ArmKinematics
from mortar_pestle.sim.world import (
    HOLD_FORCE_N,
    PRESS_DEPTH_M,
    SimWorld,
    Station,
    assets_dir,
    query,
)
from mortar_pestle.types import FollowerCommand, FollowerState

GRIPPER_JOINT = 6
GRIPPER_LINK = 4  # gripper body with the fixed jaw
GRIPPER_GRASP_RAD = -0.03  # jaw angle that closes the moving jaw onto the pestle shaft
JOINT_TORQUE_CAP_NM = 10.0  # the SO-101 servo effort limit from the URDF
TIP_OFFSET_M = 0.04  # pestle tip beyond the tool frame origin, along the jaws
GRASP_OFFSET_M = -0.005  # pestle axis offset from the tool axis, so both jaw tips touch it
START_LIFT_MM = 15.0
GRIP_FORCE_N = 300.0  # sideways holding force of the grasp
SLIDE_LIMIT_M = 0.004  # free travel of the pestle along its axis, each way
SLIDE_STIFFNESS_N_PER_M = 4000.0
SLIDE_DAMPING_NS_PER_M = 80.0  # critical damping for a 0.4 kg pestle


class PyBulletFollower:
    """Follower adapter. The arm base sits at the world origin."""

    def __init__(self, world: SimWorld, station: Station) -> None:
        self._world = world
        self._client = world.client
        self._station = station
        urdf_path = assets_dir() / "so101" / "so101_new_calib.urdf"
        # Track the centre of the pestle's round tip. Its height above the floor does not
        # depend on how far the pestle tilts.
        pestle = station.pestle
        self._mount_m = (GRASP_OFFSET_M, 0.0, TIP_OFFSET_M - pestle.length_m / 2)
        self._press_n: float | None = None  # requested pressure while pressing, else None
        self._kinematics = ArmKinematics(
            urdf_path, (GRASP_OFFSET_M, 0.0, TIP_OFFSET_M - pestle.radius_m)
        )
        self._robot: int = p.loadURDF(
            str(urdf_path), useFixedBase=True, physicsClientId=self._client
        )

        self._joints_rad = self._kinematics.solve(
            self._tip_centre_m(0.0, 0.0, START_LIFT_MM), list(REST_JOINTS_RAD)
        )
        for joint, angle in zip(ARM_JOINTS, self._joints_rad, strict=True):
            p.resetJointState(self._robot, joint, angle, physicsClientId=self._client)
        p.resetJointState(
            self._robot, GRIPPER_JOINT, GRIPPER_GRASP_RAD, physicsClientId=self._client
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
            p.JOINT_PRISMATIC,
            [0, 0, 1],
            list(self._mount_m),
            [0, 0, 0],
            parentFrameOrientation=[1, 0, 0, 0],
            physicsClientId=self._client,
        )
        p.changeConstraint(self._coupling, maxForce=GRIP_FORCE_N, physicsClientId=self._client)

    def close(self) -> None:
        self._kinematics.close()

    def _drive_slide(self) -> None:
        """Apply this substep's force along the pestle axis, to the pestle and to the gripper."""
        link = query(
            p.getLinkState,
            self._robot,
            TOOL_LINK,
            computeLinkVelocity=1,
            computeForwardKinematics=True,
            physicsClientId=self._client,
        )
        origin, matrix = link[4], p.getMatrixFromQuaternion(link[5])
        axis = (matrix[2], matrix[5], matrix[8])  # tool +z: along the pestle towards its tip
        mount = [
            origin[i] + sum(matrix[3 * i + j] * self._mount_m[j] for j in range(3))
            for i in range(3)
        ]
        pestle_id = self._station.pestle_id
        position, _ = query(
            p.getBasePositionAndOrientation, pestle_id, physicsClientId=self._client
        )
        velocity, _ = query(p.getBaseVelocity, pestle_id, physicsClientId=self._client)
        # Slide of the pestle from its mount, and its rate, both positive towards the tip.
        slide_m = sum((position[i] - mount[i]) * axis[i] for i in range(3))
        rate_m_s = sum((velocity[i] - link[6][i]) * axis[i] for i in range(3))

        weight_n = self._station.pestle.weight_n
        spring_n = -SLIDE_STIFFNESS_N_PER_M * slide_m - SLIDE_DAMPING_NS_PER_M * rate_m_s
        if self._press_n is None:
            # Hold the pestle at its mount and carry its weight.
            force_n = min(max(spring_n, -HOLD_FORCE_N), HOLD_FORCE_N) - weight_n
        else:
            # Gravity already presses with the pestle's weight. Supply the rest.
            force_n = self._press_n - weight_n
            if abs(slide_m) > SLIDE_LIMIT_M:  # end stop
                limit_m = SLIDE_LIMIT_M if slide_m > 0 else -SLIDE_LIMIT_M
                force_n += (
                    -SLIDE_STIFFNESS_N_PER_M * (slide_m - limit_m)
                    - SLIDE_DAMPING_NS_PER_M * rate_m_s
                )

        force = [force_n * component for component in axis]
        reaction = [-component for component in force]
        p.applyExternalForce(
            pestle_id, -1, force, position, p.WORLD_FRAME, physicsClientId=self._client
        )
        p.applyExternalForce(
            self._robot,
            GRIPPER_LINK,
            reaction,
            position,
            p.WORLD_FRAME,
            physicsClientId=self._client,
        )

    def _tip_centre_m(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        """Return the world position of the round tip's centre for a tip at the given point."""
        x_m, y_m, z_m = self._station.to_world(x_mm, y_mm, z_mm)
        return (x_m, y_m, z_m + self._station.pestle.radius_m)

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
            targetPosition=GRIPPER_GRASP_RAD,
            force=JOINT_TORQUE_CAP_NM,
            physicsClientId=self._client,
        )
        self._press_n = command.pressure_n if pressing else None

        self._world.step(self._drive_slide)

        x_mm, y_mm, tip_z_mm = self._station.tip_mm()
        return FollowerState(x_mm, y_mm, tip_z_mm, self._station.contact().force_n)
