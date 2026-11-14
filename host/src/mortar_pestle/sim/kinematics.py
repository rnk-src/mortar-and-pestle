"""Inverse kinematics for the SO-101: pestle tip position, with the pestle as upright as it can be.

The tip position is the main task. Upright is a weak preference, because the
wrist joint limit does not allow an upright pestle everywhere in the mortar.

This is a damped least-squares solver over joint space. It does not use
PyBullet's built-in IK, which converged to a bad local minimum for this arm.
Each solve starts from the previous joint angles, so the result is
deterministic and continuous.

The solver keeps its own headless copy of the arm for forward kinematics.
It never touches the simulated arm.
"""

import math
from pathlib import Path

import pybullet as p

Vec3 = tuple[float, float, float]

ARM_JOINTS = (0, 1, 2, 3, 4)  # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
TOOL_LINK = 5  # gripper_frame_link: origin at the jaw tip, +z along the approach direction
REST_JOINTS_RAD = (0.0, 0.0, 0.0, 1.57, 0.0)  # tool pointing straight down in front of the base

_AXIS_WEIGHT_M = 0.003  # metres of tip error that one unit of axis error is worth (weak)
_DAMPING = 1e-6
_FD_STEP_RAD = 1e-5
_MAX_STEP_RAD = 0.2
_TOLERANCE_M = 5e-5
_MAX_ITERATIONS = 20
_DOWN: Vec3 = (0.0, 0.0, -1.0)


class ArmKinematics:
    def __init__(self, urdf_path: Path, tip_offset_m: float) -> None:
        """tip_offset_m is the distance from the tool frame origin to the tracked tip point."""
        self._client: int = p.connect(p.DIRECT)
        self._robot: int = p.loadURDF(
            str(urdf_path), useFixedBase=True, physicsClientId=self._client
        )
        self._tip_offset_m = tip_offset_m
        self._limits = [
            p.getJointInfo(self._robot, joint, physicsClientId=self._client)[8:10]
            for joint in ARM_JOINTS
        ]

    def close(self) -> None:
        p.disconnect(physicsClientId=self._client)

    def tip(self, joints_rad: list[float]) -> Vec3:
        """Return the pestle tip position in the robot base frame."""
        task = self._task(joints_rad)
        return (task[0], task[1], task[2])

    def solve(self, tip_target_m: Vec3, seed_rad: list[float]) -> list[float]:
        """Return joint angles that put the tip at tip_target_m, with the pestle near upright."""
        target = [*tip_target_m, *(_AXIS_WEIGHT_M * component for component in _DOWN)]
        joints = list(seed_rad)
        for _ in range(_MAX_ITERATIONS):
            task = self._task(joints)
            error = [goal - value for goal, value in zip(target, task, strict=True)]
            if math.sqrt(sum(e * e for e in error)) < _TOLERANCE_M:
                break
            jacobian = self._jacobian(joints, task)
            step = _damped_least_squares(jacobian, error)
            largest = max(abs(value) for value in step)
            scale = min(1.0, _MAX_STEP_RAD / largest) if largest > 0 else 1.0
            joints = [
                min(max(angle + scale * delta, low), high)
                for angle, delta, (low, high) in zip(joints, step, self._limits, strict=True)
            ]
        return joints

    def _task(self, joints_rad: list[float]) -> list[float]:
        """Return [tip x, y, z, weighted tool axis x, y, z] for the given joint angles."""
        for joint, angle in zip(ARM_JOINTS, joints_rad, strict=True):
            p.resetJointState(self._robot, joint, angle, physicsClientId=self._client)
        state = p.getLinkState(
            self._robot, TOOL_LINK, computeForwardKinematics=True, physicsClientId=self._client
        )
        origin, matrix = state[4], p.getMatrixFromQuaternion(state[5])
        axis = (matrix[2], matrix[5], matrix[8])
        return [
            origin[0] + self._tip_offset_m * axis[0],
            origin[1] + self._tip_offset_m * axis[1],
            origin[2] + self._tip_offset_m * axis[2],
            _AXIS_WEIGHT_M * axis[0],
            _AXIS_WEIGHT_M * axis[1],
            _AXIS_WEIGHT_M * axis[2],
        ]

    def _jacobian(self, joints_rad: list[float], task: list[float]) -> list[list[float]]:
        """Finite-difference Jacobian of the task vector: rows are task entries, columns joints."""
        columns = []
        for index in range(len(joints_rad)):
            nudged = list(joints_rad)
            nudged[index] += _FD_STEP_RAD
            moved = self._task(nudged)
            columns.append([(m - t) / _FD_STEP_RAD for m, t in zip(moved, task, strict=True)])
        return [[column[row] for column in columns] for row in range(len(task))]


def _damped_least_squares(jacobian: list[list[float]], error: list[float]) -> list[float]:
    """Solve (J^T J + damping * I) step = J^T error."""
    size = len(jacobian[0])
    normal = [
        [
            sum(row[i] * row[j] for row in jacobian) + (_DAMPING if i == j else 0.0)
            for j in range(size)
        ]
        for i in range(size)
    ]
    rhs = [sum(row[i] * e for row, e in zip(jacobian, error, strict=True)) for i in range(size)]
    return _solve(normal, rhs)


def _solve(a: list[list[float]], b: list[float]) -> list[float]:
    """Solve a linear system by Gaussian elimination with partial pivoting."""
    size = len(b)
    m = [[*row, value] for row, value in zip(a, b, strict=True)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(m[row][column]))
        m[column], m[pivot] = m[pivot], m[column]
        for row in range(column + 1, size):
            factor = m[row][column] / m[column][column]
            for k in range(column, size + 1):
                m[row][k] -= factor * m[column][k]
    solution = [0.0] * size
    for row in reversed(range(size)):
        rest = sum(m[row][k] * solution[k] for k in range(row + 1, size))
        solution[row] = (m[row][size] - rest) / m[row][row]
    return solution
