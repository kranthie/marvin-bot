// Shared rendering for the site's Marvin Bot viewers: turns a part of window.MARVIN_MODEL into
// three.js geometry with smooth shading, and sets up physically based lighting.
//
// The code-CAD meshes are exported as plain triangles with no normals. Flat shading shows every
// facet; shading every vertex smooth rounds off the real edges. Like a CAD viewer or USD's
// crease-angle normals in Isaac Sim, the vertex normals are smoothed only across edges flatter than
// CREASE, so cylinders and fillets look round and corners stay sharp.
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { toCreasedNormals } from 'three/addons/utils/BufferGeometryUtils.js';

const CREASE = (35 * Math.PI) / 180;

export function decodeBin(M) {
  const raw = atob(M.bin), buf = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) buf[i] = raw.charCodeAt(i);
  return buf.buffer;
}

export function partGeometry(p, AB) {
  if (p.kind === 'box') return new THREE.BoxGeometry(...p.size);
  if (p.kind !== 'mesh') { const g = new THREE.CylinderGeometry(p.r, p.r, p.len, 48); g.rotateX(Math.PI / 2); return g; }
  const nv = p.v[1], q = new Uint16Array(AB, p.v[0], nv * 3), pos = new Float32Array(nv * 3);
  for (let i = 0; i < nv * 3; i++) pos[i] = p.lo[i % 3] + (q[i] / 65535) * p.span[i % 3];
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setIndex(new THREE.BufferAttribute(p.i[2] === 4 ? new Uint32Array(AB, p.i[0], p.i[1]) : new Uint16Array(AB, p.i[0], p.i[1]), 1));
  return toCreasedNormals(g, CREASE);
}

// Finish colours: what the built robot looks like, the same palette as tools/render_robot.py (which
// renders docs/media/robot.webp): machined aluminium, graphite printed parts, dark motors, light
// covers and a glossy dark visor. The URDF's own colours instead code each motor model by colour
// (codeColour), which the 3-D page uses for its X-ray and "by layer" views.
const FINISH = {
  alu: [0.78, 0.80, 0.83], graphite: [0.16, 0.17, 0.19], motor: [0.30, 0.31, 0.34], cover: [0.92, 0.93, 0.94],
  gloss: [0.05, 0.05, 0.06], dark: [0.09, 0.09, 0.10], bought: [0.25, 0.25, 0.27], tpu: [0.10, 0.10, 0.11], box: [0.22, 0.23, 0.26],
};
function finishOf(p) {
  if (p.kind === 'cyl' || p.cat === 'actuator') return 'motor';
  if (p.kind === 'box') return 'box';
  const n = p.name || '';
  if (n === 'head_visor') return 'gloss';
  if (n === 'head_shell' || n === 'neck_cover' || n === 'head_camera_frame') return 'cover';
  if (n === 'pelvis_cover' || n.includes('glute')) return 'dark';
  if (p.process === 'cover') return 'cover';
  if (p.process === 'cnc' || p.process === 'service') return 'alu';
  if (p.material === 'tpu') return 'tpu';
  if (p.process === 'bought') return 'bought';
  return 'graphite';
}
const srgb = (c) => new THREE.Color().setRGB(c[0], c[1], c[2], THREE.SRGBColorSpace);

// the URDF colour (sRGB values, as a colour picker gives them): each motor model has its own
export const codeColour = (p) => srgb(p.rgba);

export function partMaterial(p) {
  const f = finishOf(p);
  const metal = f === 'alu' || f === 'motor';
  return new THREE.MeshStandardMaterial({
    color: srgb(FINISH[f]),
    metalness: metal ? 0.65 : 0.0,
    roughness: f === 'gloss' ? 0.15 : metal ? 0.35 : f === 'cover' ? 0.45 : 0.6,
    envMapIntensity: metal || f === 'gloss' ? 0.9 : 0.3,
    transparent: true,
  });
}

// Neutral tone mapping (keeps base colours true, unlike filmic curves that grey them) and an image-based "studio" environment for reflections, plus a key light
// that casts soft shadows. Returns the key light so a viewer can aim it.
export function setupLighting(renderer, scene) {
  renderer.toneMapping = THREE.NeutralToneMapping;
  renderer.toneMappingExposure = 0.9;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  pmrem.dispose();
  scene.add(new THREE.HemisphereLight(0xdfe8f5, 0x1a1f26, 0.25));
  const key = new THREE.DirectionalLight(0xffffff, 1.8);
  key.position.set(2, -1.5, 3.2);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.radius = 4;
  key.shadow.bias = -0.0004;
  key.shadow.normalBias = 0.01;
  Object.assign(key.shadow.camera, { left: -1, right: 1, top: 1, bottom: -1, near: 0.5, far: 8 });
  scene.add(key);
  const rim = new THREE.DirectionalLight(0x9fc4ff, 0.6);
  rim.position.set(-2, 2, 2);
  scene.add(rim);
  return key;
}

// A floor that only shows the robot's shadow, for transparent backgrounds.
export function shadowFloor(size = 4) {
  const f = new THREE.Mesh(new THREE.PlaneGeometry(size, size), new THREE.ShadowMaterial({ opacity: 0.35 }));
  f.receiveShadow = true;
  f.position.z = -0.0005;
  return f;
}
