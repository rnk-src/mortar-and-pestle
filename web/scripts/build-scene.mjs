// Builds scene.xml: both mortars, the Leader pestle and the ring walls, around so101.xml.
// Numbers come from host/src/mortar_pestle/sim/world.py and scene.py.
import fs from 'node:fs';

const R = 0.05, FLOOR = 0.01, WALL = 0.025, SEGMENTS = 24;
const stations = { follower: [0.19, 0.0], leader: [0.19, -0.2] };

function mortar(name, [x, y]) {
  let out = `<body name="${name}_mortar" pos="${x} ${y} 0">
    <geom name="${name}_floor" type="box" size="${R} ${R} ${FLOOR / 2}" pos="0 0 ${FLOOR / 2}" rgba="0.75 0.75 0.72 1" friction="0.2 0.005 0.0001" contype="1" conaffinity="1"/>\n`;
  for (let i = 0; i < SEGMENTS; i++) {
    const a = (2 * Math.PI * i) / SEGMENTS;
    const hw = (Math.PI * R) / SEGMENTS;
    out += `    <geom type="box" size="0.002 ${hw.toFixed(5)} ${WALL / 2}" pos="${(R * Math.cos(a)).toFixed(5)} ${(R * Math.sin(a)).toFixed(5)} ${FLOOR + WALL / 2}" euler="0 0 ${((a * 180) / Math.PI).toFixed(3)}" rgba="0.6 0.6 0.58 1" contype="0" conaffinity="0"/>\n`;
  }
  return out + '  </body>\n';
}

const [lx, ly] = stations.leader;
const scene = `<mujoco model="mortar_pestle">
  <include file="so101.xml"/>
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <worldbody>
    ${mortar('follower', stations.follower)}
    ${mortar('leader', stations.leader)}
    <body name="leader_pestle" pos="${lx} ${ly} ${FLOOR + 0.015 + 0.05}">
      <freejoint name="leader_pestle_free"/>
      <geom name="leader_pestle_geom" type="capsule" fromto="0 0 -0.042 0 0 0.042" size="0.008" mass="0.03" rgba="0.2 0.5 0.9 1" friction="0.2 0.005 0.0001" contype="1" conaffinity="1"/>
    </body>
  </worldbody>
</mujoco>
`;
fs.writeFileSync(new URL('../scene/scene.xml', import.meta.url), scene);
console.log('wrote scene/scene.xml');
