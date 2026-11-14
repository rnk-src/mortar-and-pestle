// Follower adapter. Port of sim/follower.py and sim/kinematics.py.
// The arm base is at the world origin. Joint targets come from a damped least-squares
// solver on a scratch copy of the model, as in the Python.

/* eslint-disable @typescript-eslint/no-explicit-any */
import { FOLLOWER, GRAVITY, World, type Vec3 } from './world';

const ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'] as const;
const REST_RAD = [0.0, 0.0, 0.0, 1.57, 0.0];
const GRIPPER_GRASP_RAD = -0.03;
const JOINT_TORQUE_CAP_NM = 10.0;
const TIP_OFFSET_M = 0.04;
const START_LIFT_MM = 15.0;
const PRESS_DEPTH_M = 0.003;
const HOLD_FORCE_N = 50.0;
const SLIDE_LIMIT_M = 0.004;
const SLIDE_STIFFNESS = 4000.0;
const SLIDE_DAMPING = 80.0;

const AXIS_WEIGHT_M = 0.003;
const DAMPING = 1e-6;
const FD_STEP_RAD = 1e-5;
const MAX_STEP_RAD = 0.2;
const TOLERANCE_M = 5e-5;
const MAX_ITERATIONS = 20;

export interface FollowerCommand {
  x_mm: number;
  y_mm: number;
  lift_mm: number;
  pressure_n: number;
}
export interface FollowerState {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  contact_force_n: number;
}

export class MujocoFollower {
  private readonly world: World;
  private readonly scratch: any;
  private readonly tipOffset: Vec3;
  private readonly limits: [number, number][];
  private readonly qpos: number[];
  private readonly slide: any;
  private joints: number[];
  private pressN: number | null = null;

  constructor(world: World) {
    this.world = world;
    const { mj, model } = world;
    this.scratch = new mj.MjData(model);
    // Track the centre of the pestle's round tip, in the tool frame.
    this.tipOffset = [0, 0, TIP_OFFSET_M - FOLLOWER.radiusM];
    this.qpos = ARM.map((n) => model.jnt(n).qposadr as number);
    this.limits = ARM.map((n) => {
      const r = model.jnt(n).range;
      return [r[0], r[1]] as [number, number];
    });
    this.slide = world.data.jnt('pestle_slide');
    for (let i = 0; i < model.nu; i++) {
      model.actuator_forcerange[2 * i] = -JOINT_TORQUE_CAP_NM;
      model.actuator_forcerange[2 * i + 1] = JOINT_TORQUE_CAP_NM;
    }
    this.joints = this.solve(this.tipCentre(0, 0, START_LIFT_MM), REST_RAD);
    ARM.forEach((_, i) => (world.data.qpos[this.qpos[i]] = this.joints[i]));
    world.data.qpos[model.jnt('gripper').qposadr] = GRIPPER_GRASP_RAD;
    this.setControls();
    mj.mj_forward(model, world.data);
  }

  private setControls(): void {
    const { data, model } = this.world;
    ARM.forEach((n, i) => (data.ctrl[model.actuator(n).id] = this.joints[i]));
    data.ctrl[model.actuator('gripper').id] = GRIPPER_GRASP_RAD;
  }

  private tipCentre(xMm: number, yMm: number, zMm: number): Vec3 {
    return [
      FOLLOWER.originM[0] + xMm / 1000,
      FOLLOWER.originM[1] + yMm / 1000,
      0.01 + zMm / 1000 + FOLLOWER.radiusM,
    ];
  }

  /** [tip x, y, z, weighted tool axis x, y, z] for the given joint angles. */
  private task(joints: number[]): number[] {
    const { mj, model } = this.world;
    joints.forEach((a, i) => (this.scratch.qpos[this.qpos[i]] = a));
    mj.mj_kinematics(model, this.scratch);
    const site = this.scratch.site('gripperframe');
    const o = site.xpos as ArrayLike<number>;
    const m = site.xmat as ArrayLike<number>;
    const [a, b, c] = this.tipOffset;
    return [
      o[0] + m[0] * a + m[1] * b + m[2] * c,
      o[1] + m[3] * a + m[4] * b + m[5] * c,
      o[2] + m[6] * a + m[7] * b + m[8] * c,
      AXIS_WEIGHT_M * m[2],
      AXIS_WEIGHT_M * m[5],
      AXIS_WEIGHT_M * m[8],
    ];
  }

  private jacobian(joints: number[], task: number[]): number[][] {
    const cols = joints.map((_, i) => {
      const nudged = [...joints];
      nudged[i] += FD_STEP_RAD;
      const moved = this.task(nudged);
      return moved.map((v, r) => (v - task[r]) / FD_STEP_RAD);
    });
    return task.map((_, r) => cols.map((col) => col[r]));
  }

  /** Joint angles that put the tip at target, with the pestle near upright. */
  solve(target: Vec3, seed: number[]): number[] {
    const goal = [...target, 0, 0, -AXIS_WEIGHT_M];
    let joints = [...seed];
    for (let it = 0; it < MAX_ITERATIONS; it++) {
      const task = this.task(joints);
      const error = goal.map((g, i) => g - task[i]);
      if (Math.hypot(...error) < TOLERANCE_M) break;
      const step = dampedLeastSquares(this.jacobian(joints, task), error);
      const largest = Math.max(...step.map(Math.abs));
      const scale = largest > 0 ? Math.min(1, MAX_STEP_RAD / largest) : 1;
      joints = joints.map((a, i) =>
        Math.min(Math.max(a + scale * step[i], this.limits[i][0]), this.limits[i][1]),
      );
    }
    return joints;
  }

  /** Force along the pestle axis, each substep. Joint constraint gives the arm its reaction. */
  private driveSlide = (): void => {
    const slideM = this.slide.qpos[0] as number;
    const rate = this.slide.qvel[0] as number;
    const weight = FOLLOWER.massKg * GRAVITY;
    const spring = -SLIDE_STIFFNESS * slideM - SLIDE_DAMPING * rate;
    let force: number;
    if (this.pressN === null) {
      force = Math.min(Math.max(spring, -HOLD_FORCE_N), HOLD_FORCE_N) - weight;
    } else {
      force = this.pressN - weight;
      if (Math.abs(slideM) > SLIDE_LIMIT_M) {
        const limit = slideM > 0 ? SLIDE_LIMIT_M : -SLIDE_LIMIT_M;
        force += -SLIDE_STIFFNESS * (slideM - limit) - SLIDE_DAMPING * rate;
      }
    }
    this.slide.qfrc_applied[0] = force;
  };

  execute(command: FollowerCommand): FollowerState {
    const pressing = command.lift_mm <= 0 && command.pressure_n > 0;
    const zMm = pressing ? -PRESS_DEPTH_M * 1000 : Math.max(command.lift_mm, 0.5);
    this.joints = this.solve(this.tipCentre(command.x_mm, command.y_mm, zMm), this.joints);
    this.setControls();
    this.pressN = pressing ? command.pressure_n : null;
    this.world.step(this.driveSlide);
    const tip = this.world.tipMm(FOLLOWER);
    return {
      x_mm: tip[0],
      y_mm: tip[1],
      z_mm: tip[2],
      contact_force_n: this.world.contact(FOLLOWER).forceN,
    };
  }
}

function dampedLeastSquares(jac: number[][], error: number[]): number[] {
  const n = jac[0].length;
  const normal = Array.from({ length: n }, (_, i) =>
    Array.from(
      { length: n },
      (_, j) => jac.reduce((s, row) => s + row[i] * row[j], 0) + (i === j ? DAMPING : 0),
    ),
  );
  const rhs = Array.from({ length: n }, (_, i) =>
    jac.reduce((s, row, r) => s + row[i] * error[r], 0),
  );
  return solveLinear(normal, rhs);
}

function solveLinear(a: number[][], b: number[]): number[] {
  const n = b.length;
  const m = a.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let pivot = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[pivot][c])) pivot = r;
    [m[c], m[pivot]] = [m[pivot], m[c]];
    for (let r = c + 1; r < n; r++) {
      const f = m[r][c] / m[c][c];
      for (let k = c; k <= n; k++) m[r][k] -= f * m[c][k];
    }
  }
  const x = new Array<number>(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let rest = 0;
    for (let k = r + 1; k < n; k++) rest += m[r][k] * x[k];
    x[r] = (m[r][n] - rest) / m[r][r];
  }
  return x;
}
