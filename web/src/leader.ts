// Simulated Leader adapter. Port of sim/leader.py.
// A hand moves a light dummy pestle in a sensed mortar. The Leader Sample comes from the
// pestle's contact with the mortar, as a load-cell plate would measure it.
//
// PyBullet holds the pestle with a force-capped constraint. Here the pestle has three slide
// joints, so it stays upright, and a per-substep hook does the rest: a critically damped PD
// on x and y, and a capped vertical force. While pressing, the capped force pushes straight
// down, which is what a saturated constraint does. The pestle's own weight adds to the press.

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
  private readonly slides: any[];
  private target: [number, number, number] = [0, 0, 0];
  private down = false;
  private capN = HOLD_FORCE_N;

  constructor(world: World, hand: Hand, nowUs: () => number) {
    this.world = world;
    this.hand = hand;
    this.nowUs = nowUs;
    this.slides = ['leader_x', 'leader_y', 'leader_z'].map((n) => world.data.jnt(n));
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
    const pos = this.world.data.body('leader_pestle').xpos as ArrayLike<number>;
    const v = this.slides.map((j) => j.qvel[0] as number);
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
    this.slides[0].qfrc_applied[0] = fx;
    this.slides[1].qfrc_applied[0] = fy;
    this.slides[2].qfrc_applied[0] = fz;
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
    return {
      timestamp_us: this.nowUs(),
      quat_wxyz: [1, 0, 0, 0], // held upright
      pressure_n: c.forceN,
      x_mm: c.xMm,
      y_mm: c.yMm,
      contact: c.forceN > 0,
    };
  }
}
