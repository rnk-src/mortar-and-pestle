// Vertex-clustering simplification of a binary STL. The arm meshes are display-only, so a
// coarse grid is enough. Vertices that share a grid cell merge into their average.
import fs from 'node:fs';

/** Returns a smaller binary STL as a Buffer. cell is the grid size in mesh units (metres). */
export function simplifyStl(input, cell) {
  const triCount = input.readUInt32LE(80);
  const cellOf = new Map(); // "ix,iy,iz" -> cluster index
  const sums = []; // [x, y, z, count] per cluster
  const tri = new Int32Array(triCount * 3);
  for (let t = 0; t < triCount; t++) {
    const base = 84 + t * 50 + 12; // skip the 12-byte normal
    for (let k = 0; k < 3; k++) {
      const x = input.readFloatLE(base + k * 12);
      const y = input.readFloatLE(base + k * 12 + 4);
      const z = input.readFloatLE(base + k * 12 + 8);
      const key = `${Math.floor(x / cell)},${Math.floor(y / cell)},${Math.floor(z / cell)}`;
      let id = cellOf.get(key);
      if (id === undefined) {
        id = sums.length;
        cellOf.set(key, id);
        sums.push([0, 0, 0, 0]);
      }
      const s = sums[id];
      s[0] += x; s[1] += y; s[2] += z; s[3] += 1;
      tri[t * 3 + k] = id;
    }
  }
  const pos = sums.map(([x, y, z, n]) => [x / n, y / n, z / n]);
  const kept = [];
  const seen = new Set();
  for (let t = 0; t < triCount; t++) {
    const a = tri[t * 3], b = tri[t * 3 + 1], c = tri[t * 3 + 2];
    if (a === b || b === c || a === c) continue;
    const key = [a, b, c].sort((p, q) => p - q).join(',');
    if (seen.has(key)) continue; // a duplicate face
    seen.add(key);
    kept.push([a, b, c]);
  }
  const out = Buffer.alloc(84 + kept.length * 50);
  input.copy(out, 0, 0, 80);
  out.writeUInt32LE(kept.length, 80);
  kept.forEach(([a, b, c], i) => {
    const o = 84 + i * 50;
    const [p, q, r] = [pos[a], pos[b], pos[c]];
    const u = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
    const v = [r[0] - p[0], r[1] - p[1], r[2] - p[2]];
    const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const len = Math.hypot(...n) || 1;
    [n[0] / len, n[1] / len, n[2] / len].forEach((val, j) => out.writeFloatLE(val, o + j * 4));
    [p, q, r].forEach((pt, k) => pt.forEach((val, j) => out.writeFloatLE(val, o + 12 + k * 12 + j * 4)));
  });
  return out;
}

if (process.argv[1] === new URL(import.meta.url).pathname) {
  const [src, dst, cell] = process.argv.slice(2);
  const out = simplifyStl(fs.readFileSync(src), Number(cell));
  fs.writeFileSync(dst, out);
}
