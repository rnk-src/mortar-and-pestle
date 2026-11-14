// Shared MuJoCo world. Port of host/src/mortar_pestle/sim/world.py.
// Units: metres in the world, millimetres in the mortar frame.

/* eslint-disable @typescript-eslint/no-explicit-any */
export type Vec3 = [number, number, number];

export const TICK_S = 0.01;
export const SUBSTEPS = 5;
export const MORTAR_FLOOR_M = 0.01;
export const CONTACT_THRESHOLD_N = 0.2;
export const GRAVITY = 9.81;

export interface Station {
  name: 'leader' | 'follower';
  originM: [number, number];
  pestleGeom: string;
  floorGeom: string;
  radiusM: number;
  halfCylinderM: number;
  massKg: number;
  /** +1 if the tip is at +z of the capsule geom frame, -1 if at -z. */
  tipSign: 1 | -1;
}

// MuJoCo fromto geoms: the geom z axis points from the 'to' end back to the 'from' end.
const LEADER_SIGN = 1; // fromto runs bottom to top, so geom z points down, to the tip
const FOLLOWER_SIGN = -1; // fromto runs top to tip, so geom z points up, away from the tip

export const LEADER: Station = {
  name: 'leader',
  originM: [0.19, -0.2],
  pestleGeom: 'leader_pestle_geom',
  floorGeom: 'leader_floor',
  radiusM: 0.008,
  halfCylinderM: 0.042,
  massKg: 0.03,
  tipSign: LEADER_SIGN,
};
export const FOLLOWER: Station = {
  name: 'follower',
  originM: [0.19, 0.0],
  pestleGeom: 'follower_pestle_geom',
  floorGeom: 'follower_floor',
  radiusM: 0.01,
  halfCylinderM: 0.04,
  massKg: 0.4,
  tipSign: FOLLOWER_SIGN,
};

export interface Contact {
  forceN: number;
  xMm: number;
  yMm: number;
}

export type Files = Record<string, Uint8Array | string>;

export class World {
  readonly mj: any;
  readonly model: any;
  readonly data: any;

  constructor(mj: any, model: any, data: any) {
    this.mj = mj;
    this.model = model;
    this.data = data;
  }

  /** files: "scene.xml", "so101.xml" and "assets/<mesh>.stl". */
  static create(mj: any, files: Files): World {
    const dirs = new Set<string>(['/w']);
    for (const name of Object.keys(files)) {
      const parts = name.split('/').slice(0, -1);
      let dir = '/w';
      for (const part of parts) {
        dir += `/${part}`;
        dirs.add(dir);
      }
    }
    for (const dir of [...dirs].sort()) {
      try {
        mj.FS.mkdir(dir);
      } catch {
        /* already there */
      }
    }
    for (const [name, content] of Object.entries(files)) {
      mj.FS.writeFile(`/w/${name}`, content);
    }
    const model = mj.MjModel.from_xml_path('/w/scene.xml');
    const data = new mj.MjData(model);
    return new World(mj, model, data);
  }

  /** Advance one tick (SUBSTEPS physics steps). beforeSubstep applies forces each substep. */
  step(beforeSubstep?: () => void): void {
    for (let i = 0; i < SUBSTEPS; i++) {
      beforeSubstep?.();
      this.mj.mj_step(this.model, this.data);
    }
  }

  /** Pestle tip (lowest point of its round end) in the mortar frame, mm. */
  tipMm(s: Station): Vec3 {
    const geom = this.data.geom(s.pestleGeom);
    const c = geom.xpos as ArrayLike<number>;
    const m = geom.xmat as ArrayLike<number>;
    const axis = [m[2], m[5], m[8]];
    const k = s.tipSign * s.halfCylinderM;
    return [
      (c[0] + axis[0] * k - s.originM[0]) * 1000,
      (c[1] + axis[1] * k - s.originM[1]) * 1000,
      (c[2] + axis[2] * k - s.radiusM - MORTAR_FLOOR_M) * 1000,
    ];
  }

  /** Pestle-on-mortar force and centre of pressure, as a load-cell plate would measure. */
  contact(s: Station): Contact {
    const pg = this.model.geom(s.pestleGeom).id;
    const fg = this.model.geom(s.floorGeom).id;
    let total = 0;
    let sx = 0;
    let sy = 0;
    const n = this.data.ncon as number;
    for (let i = 0; i < n; i++) {
      const c = this.data.contact.get(i);
      if (!c) continue;
      const pair = (c.geom1 === pg && c.geom2 === fg) || (c.geom1 === fg && c.geom2 === pg);
      if (!pair) continue;
      // Elliptic cones: the first constraint row of a contact is its normal force.
      // (mj_contactForce cannot fill a JS array through embind, so read efc_force.)
      if (c.efc_address < 0) continue;
      const f = Math.abs(this.data.efc_force[c.efc_address] as number);
      total += f;
      sx += c.pos[0] * f;
      sy += c.pos[1] * f;
    }
    if (total < CONTACT_THRESHOLD_N) {
      const tip = this.tipMm(s);
      return { forceN: 0, xMm: tip[0], yMm: tip[1] };
    }
    return {
      forceN: total,
      xMm: (sx / total - s.originM[0]) * 1000,
      yMm: (sy / total - s.originM[1]) * 1000,
    };
  }
}
