// Wires the Python pipeline (Pyodide) to the MuJoCo Leader and Follower.
/* eslint-disable @typescript-eslint/no-explicit-any */
import { MujocoFollower, type FollowerCommand, type FollowerState } from './follower';
import { MujocoLeader, type HandTarget } from './leader';
import type { World } from './world';

export type Mode = 'mirror' | 'interpret';
export const TICK_US = 10_000;

export interface TickView {
  intent: {
    pattern: string;
    rate_hz: number;
    radius_mm: number;
    direction: string;
    requested_pressure_n: number;
  };
  command: FollowerCommand;
  state: FollowerState;
  leader_force_n: number;
}

/** The hand: a scripted stroke list, or a pointer the user drives. */
export class HandControl {
  auto = true;
  pointer: HandTarget = { x_mm: 0, y_mm: 0, down: false, force_n: 6 };
  private script: number[][] = [];
  private index = 0;

  setScript(rows: number[][]): void {
    this.script = rows;
    this.index = 0;
  }

  next = (): HandTarget => {
    if (!this.auto || this.script.length === 0) return this.pointer;
    const [x, y, down, force] = this.script[this.index];
    this.index = (this.index + 1) % this.script.length;
    return { x_mm: x, y_mm: y, down: down === 1, force_n: force };
  };
}

export class Session {
  readonly hand = new HandControl();
  readonly leader: MujocoLeader;
  readonly follower: MujocoFollower;
  private ticks = 0;
  private readonly py: any;

  private constructor(readonly world: World, pyodide: any, mode: Mode) {
    this.leader = new MujocoLeader(world, this.hand.next, () => this.ticks * TICK_US);
    this.follower = new MujocoFollower(world);
    const make = pyodide.globals.get('make_session');
    this.py = make(this.leader, this.follower, mode);
    make.destroy();
    const demo = pyodide.globals.get('demo_hand');
    const rows = demo();
    this.hand.setScript(rows as number[][]);
    demo.destroy();
  }

  /** wheel: the built mortar_pestle wheel. glue: the text of py/browser.py. */
  static create(opts: {
    pyodide: any;
    world: World;
    wheel: Uint8Array;
    glue: string;
    mode?: Mode;
  }): Session {
    opts.pyodide.unpackArchive(opts.wheel, 'wheel');
    opts.pyodide.runPython(opts.glue);
    return new Session(opts.world, opts.pyodide, opts.mode ?? 'mirror');
  }

  get mode(): Mode {
    return this.py.mode as Mode;
  }

  setMode(mode: Mode): void {
    this.py.set_mode(mode);
  }

  /** Run one 10 ms tick of the whole pipeline. */
  tick(): TickView {
    const view = this.py.tick() as TickView;
    this.ticks += 1;
    return view;
  }
}
