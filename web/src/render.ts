// Three.js view of the MuJoCo world. Meshes come from the compiled model, so the picture
// always matches the physics. Z is up, as in MuJoCo.
/* eslint-disable @typescript-eslint/no-explicit-any */
import * as THREE from 'three';
import { FOLLOWER, LEADER, MORTAR_FLOOR_M, type World } from './world';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const GEOM_CAPSULE = 3;
const GEOM_CYLINDER = 5;
const GEOM_BOX = 6;
const GEOM_MESH = 7;
const TRAIL_POINTS = 70;

export const LEADER_RGB = 0x4d80e6;
export const FOLLOWER_RGB = 0xdb7333;

interface Item {
  mesh: THREE.Mesh;
  geom: number;
}

export class View {
  readonly renderer: THREE.WebGLRenderer;
  readonly camera: THREE.PerspectiveCamera;
  private readonly scene = new THREE.Scene();
  private readonly items: Item[] = [];
  private readonly trails: { line: THREE.Line; pts: THREE.Vector3[]; leader: boolean }[] = [];
  private readonly world: World;
  private readonly plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -MORTAR_FLOOR_M);
  private readonly raycaster = new THREE.Raycaster();

  constructor(canvas: HTMLCanvasElement, world: World) {
    this.world = world;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.scene.background = new THREE.Color(0x10141c);
    this.camera = new THREE.PerspectiveCamera(38, 1, 0.01, 5);
    this.placeCamera();

    this.scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x303845, 1.7));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(0.3, -0.5, 1.0);
    this.scene.add(sun);

    const ground = new THREE.Mesh(
      new THREE.CircleGeometry(0.9, 64),
      new THREE.MeshStandardMaterial({ color: 0x1a2030, roughness: 1 }),
    );
    ground.position.z = -0.002;
    this.scene.add(ground);

    this.buildGeoms();
    for (const leader of [true, false]) {
      const geometry = new THREE.BufferGeometry();
      const pts = Array.from({ length: TRAIL_POINTS }, () => new THREE.Vector3(0, 0, -1));
      geometry.setFromPoints(pts);
      const line = new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({ color: leader ? LEADER_RGB : FOLLOWER_RGB }),
      );
      line.frustumCulled = false;
      this.scene.add(line);
      this.trails.push({ line, pts, leader });
    }
  }

  private placeCamera(): void {
    // From the front, above: the arm reaches towards the camera, the Leader mortar is on the left.
    const target = new THREE.Vector3(0.13, -0.1, 0.04);
    this.camera.position.set(0.62, -0.52, 0.42);
    this.camera.lookAt(target);
  }

  private buildGeoms(): void {
    const { model } = this.world;
    for (let g = 0; g < model.ngeom; g++) {
      const group = model.geom_group[g];
      if (group === 3) continue; // collision proxies
      const type = model.geom_type[g];
      let geometry: THREE.BufferGeometry | null = null;
      const size = [model.geom_size[3 * g], model.geom_size[3 * g + 1], model.geom_size[3 * g + 2]];
      if (type === GEOM_MESH) {
        geometry = this.meshGeometry(model.geom_dataid[g]);
      } else if (type === GEOM_BOX) {
        geometry = new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
      } else if (type === GEOM_CYLINDER) {
        geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 48);
        geometry.rotateX(Math.PI / 2);
      } else if (type === GEOM_CAPSULE) {
        geometry = new THREE.CapsuleGeometry(size[0], 2 * size[1], 8, 16);
        geometry.rotateX(Math.PI / 2); // Three's capsule runs along y; MuJoCo's along z
      }
      if (!geometry) continue;
      const matid = model.geom_matid[g];
      const rgba = matid >= 0 ? model.mat_rgba.subarray(4 * matid, 4 * matid + 4) : model.geom_rgba.subarray(4 * g, 4 * g + 4);
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
        roughness: 0.65,
        metalness: 0.05,
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.matrixAutoUpdate = false;
      this.scene.add(mesh);
      this.items.push({ mesh, geom: g });
    }
  }

  private meshGeometry(meshId: number): THREE.BufferGeometry {
    const { model } = this.world;
    const vAdr = model.mesh_vertadr[meshId];
    const vNum = model.mesh_vertnum[meshId];
    const fAdr = model.mesh_faceadr[meshId];
    const fNum = model.mesh_facenum[meshId];
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(model.mesh_vert.subarray(3 * vAdr, 3 * (vAdr + vNum))), 3),
    );
    geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(model.mesh_face.subarray(3 * fAdr, 3 * (fAdr + fNum))), 1));
    geometry.computeVertexNormals();
    return geometry;
  }

  resize(width: number, height: number): void {
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / Math.max(1, height);
    // Keep both mortars in view on tall, narrow screens.
    this.camera.fov = this.camera.aspect < 1 ? 62 : 38;
    this.camera.updateProjectionMatrix();
  }

  /** Mortar-frame position (mm) of the leader plane under a pointer, or null. */
  pointerToLeaderMm(ndcX: number, ndcY: number): [number, number] | null {
    this.raycaster.setFromCamera(new THREE.Vector2(ndcX, ndcY), this.camera);
    const hit = new THREE.Vector3();
    if (!this.raycaster.ray.intersectPlane(this.plane, hit)) return null;
    return [(hit.x - LEADER.originM[0]) * 1000, (hit.y - LEADER.originM[1]) * 1000];
  }

  private pushTrail(trail: (typeof this.trails)[number], x: number, y: number, z: number): void {
    const { pts, line } = trail;
    pts.push(pts.shift()!.set(x, y, z));
    line.geometry.setFromPoints(pts);
  }

  private tickCount = 0;

  /** Sync meshes to the physics state and draw. */
  draw(): void {
    const { data } = this.world;
    const m = new THREE.Matrix4();
    for (const { mesh, geom } of this.items) {
      const p = data.geom_xpos;
      const r = data.geom_xmat;
      const i = 3 * geom;
      const j = 9 * geom;
      m.set(r[j], r[j + 1], r[j + 2], p[i], r[j + 3], r[j + 4], r[j + 5], p[i + 1], r[j + 6], r[j + 7], r[j + 8], p[i + 2], 0, 0, 0, 1);
      mesh.matrix.copy(m);
      mesh.matrixWorldNeedsUpdate = true;
    }
    if (this.tickCount++ % 2 === 0) {
      for (const t of this.trails) {
        const s = t.leader ? LEADER : FOLLOWER;
        const tip = this.world.tipMm(s);
        const inContact = tip[2] < 1.5;
        if (inContact) {
          this.pushTrail(t, s.originM[0] + tip[0] / 1000, s.originM[1] + tip[1] / 1000, MORTAR_FLOOR_M + 0.0008);
        }
      }
    }
    this.renderer.render(this.scene, this.camera);
  }
}
