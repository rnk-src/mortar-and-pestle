// Simulated Leader adapter. Port of sim/leader.py.
// A hand moves a light dummy pestle in a sensed mortar. The Leader Sample comes from the
// pestle's contact with the mortar, as a load-cell plate would measure it.
//
// PyBullet holds the pestle with a force-capped constraint. Here a per-substep hook does the
// same job: a critically damped PD on x and y, a tilt PD to keep it upright, and a capped
// vertical force. While pressing, the capped force pushes straight down, which is what a
// saturated constraint does. The pestle's own weight adds to the press.

/* eslint-disable @typescript-eslint/no-explicit-any */
import { GRAVITY, LEADER, MORTAR_FLOOR_M, World } from './world';

export const HOVER_MM = 15.0;
export const MORTAR_RADIUS_M = 0.05;
export const MAX_RADIUS_MM = (MORTAR_RADIUS_M - 0.008) * 1000;
const PRESS_DEPTH_M = 0.003;
const HOLD_FORCE_N = 50.0;
const MIN_PRESS_FORCE_N = 0.3;
const PESTLE_LENGTH_M = 0.1;
const DESCENT_SPEED_M_S = 0.3; // how fast a hand lowers the pestle
const PRESS_ZONE_M = 0.0015; // below this tip height the capped press takes over

// Closed-loop bandwidth of the hold. Low enough for stable explicit forces at a 2 ms substep.
const OMEGA = 2 * Math.PI * 15;
const MASS = LEADER.massKg;
const KP = MASS * OMEGA * OMEGA;
const KD = 2 * MASS * OMEGA;
const TILT_INERTIA = MASS * (PESTLE_LENGTH_M ** 2 / 12 + LEADER.radiusM ** 2 / 4);
const KP_TILT = TILT_INERTIA * OMEGA * OMEGA;
const KD_TILT = 2 * TILT_INERTIA * OMEGA;

export interface HandTarget {
  x_mm: number;
  y_mm: number;
  down: boolean;
  force_n: number;
}
export type Hand = () => HandTarget;

export interface LeaderSample {
  timestamp_us: number;
  quat_wxyz: [number, number, number, number];
  pressure_n: number;
  x_mm: number;
  y_mm: number;
  contact: boolean;
}

/** Hold-force cap that makes a pestle of weight press with pressure. */
export function pressCapN(pressureN: number, weightN: number): number {
  return Math.max(pressureN - weightN, MIN_PRESS_FORCE_N);
}

export class MujocoLeader {
  private readonly world: World;
  private readonly hand: Hand;
  private readonly nowUs: () => number;
  private readonly bodyId: number;
  private readonly free: any;
  private target: [number, number, number] = [0, 0, 0];
  private down = false;
  private capN = HOLD_FORCE_N;

  constructor(world: World, hand: Hand, nowUs: () => number) {
    this.world = world;
    this.hand = hand;
    this.nowUs = nowUs;
    this.bodyId = world.model.body('leader_pestle').id;
    this.free = world.data.jnt('leader_pestle_free');
    this.target = this.centreFor(0, 0, HOVER_MM);
    world.hooks.push(this.hold);
  }

  private centreFor(xMm: number, yMm: number, zMm: number): [number, number, number] {
    return [
      LEADER.originM[0] + xMm / 1000,
      LEADER.originM[1] + yMm / 1000,
      MORTAR_FLOOR_M + zMm / 1000 + PESTLE_LENGTH_M / 2,
    ];
  }

  private hold = (): void => {
    const { data } = this.world;
    const pos = data.body('leader_pestle').xpos as ArrayLike<number>;
    const q = data.body('leader_pestle').xquat as ArrayLike<number>; // w x y z
    const m = data.body('leader_pestle').xmat as ArrayLike<number>;
    const v = this.free.qvel as ArrayLike<number>; // linear: world frame; angular: body frame
    const w = [
      m[0] * v[3] + m[1] * v[4] + m[2] * v[5],
      m[3] * v[3] + m[4] * v[4] + m[5] * v[5],
      m[6] * v[3] + m[7] * v[4] + m[8] * v[5],
    ];
    const weight = MASS * GRAVITY;

    let fx = KP * (this.target[0] - pos[0]) - KD * v[0];
    let fy = KP * (this.target[1] - pos[1]) - KD * v[1];
    let fz: number;
    const tipHeightM = pos[2] - PESTLE_LENGTH_M / 2 - MORTAR_FLOOR_M;
    if (this.down && tipHeightM <= PRESS_ZONE_M) {
      fz = -this.capN; // saturated hold: pushes down with exactly the cap
    } else if (this.down) {
      // Lower at a limited speed, never harder than the cap.
      fz = Math.max(weight - KD * (DESCENT_SPEED_M_S + v[2]), -this.capN);
      fz = Math.min(fz, HOLD_FORCE_N);
    } else {
      fz = weight + KP * (this.target[2] - pos[2]) - KD * v[2];
      fz = Math.min(Math.max(fz, -HOLD_FORCE_N), HOLD_FORCE_N);
    }
    const lateral = Math.hypot(fx, fy);
    if (lateral > HOLD_FORCE_N) {
      fx *= HOLD_FORCE_N / lateral;
      fy *= HOLD_FORCE_N / lateral;
    }

    // Small-angle tilt error: 2 * sign(w) * (x, y) of the quaternion, about world x and y.
    const s = q[0] < 0 ? -1 : 1;
    const tx = -KP_TILT * 2 * s * q[1] - KD_TILT * w[0];
    const ty = -KP_TILT * 2 * s * q[2] - KD_TILT * w[1];

    const b = 6 * this.bodyId;
    data.xfrc_applied[b] = fx;
    data.xfrc_applied[b + 1] = fy;
    data.xfrc_applied[b + 2] = fz;
    data.xfrc_applied[b + 3] = tx;
    data.xfrc_applied[b + 4] = ty;
    data.xfrc_applied[b + 5] = 0;
  };

  /** Apply the hand's current target, then return what the Leader sensors measure. */
  read(): LeaderSample {
    const t = this.hand();
    const radius = Math.hypot(t.x_mm, t.y_mm);
    const scale = radius > 0 ? Math.min(1, MAX_RADIUS_MM / radius) : 1;
    const zMm = t.down ? -PRESS_DEPTH_M * 1000 : HOVER_MM;
    this.target = this.centreFor(t.x_mm * scale, t.y_mm * scale, zMm);
    this.down = t.down;
    this.capN = pressCapN(t.force_n, MASS * GRAVITY);

    const c = this.world.contact(LEADER);
    const q = this.world.data.body('leader_pestle').xquat as ArrayLike<number>;
    return {
      timestamp_us: this.nowUs(),
      quat_wxyz: [q[0], q[1], q[2], q[3]],
      pressure_n: c.forceN,
      x_mm: c.xMm,
      y_mm: c.yMm,
      contact: c.forceN > 0,
    };
  }
}
