import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { decodeBin, partGeometry as buildGeometry, partMaterial, setupLighting } from '../../scripts/marvin-render.js';

const $ = id => document.getElementById(id);
window.__m3dStarted = true;   // the 3-D library loaded; from here on, errors are reported by this script
const M = window.MARVIN_MODEL;
if (!M) { $('vmsg').textContent = 'The model data (guide/model/marvin_model.js) did not load.'; throw new Error('no model'); }
const D2R = Math.PI / 180, R2D = 180 / Math.PI, G = 9.81;

// ---------- categories, modules, colours
const CATS = {
  cnc:{label:'Machined aluminium', col:'#9fb4c7'}, service:{label:'Aluminium cut by a service', col:'#5fa8d3'},
  print:{label:'Printed structure', col:'#e0a458'}, cover:{label:'Body shells (add-on)', col:'#efe9dc'},
  actuator:{label:'Motors and servos', col:'#d9544f'}, bought:{label:'Bearings and push rods', col:'#8a8f98'},
  electronics:{label:'Power and CAN electronics', col:'#4cc38a'}, sensors:{label:'Sensors (IMU, depth camera)', col:'#b48cff'},
  compute:{label:'Jetson AGX Orin', col:'#ffd166'}, battery:{label:'Battery packs', col:'#ff6b6b'},
  hands:{label:'Hands (AmazingHand)', col:'#c0d6e4'}, padding:{label:'Fall padding', col:'#6b7280'}};
const MODS = {pelvis:{label:'Pelvis', col:'#ffb454'}, leg:{label:'Legs', col:'#4fb3ff'}, torso:{label:'Torso core', col:'#4cc38a'},
  battery:{label:'Battery', col:'#ff6b6b'}, arm:{label:'Arms', col:'#b48cff'}, head:{label:'Head', col:'#e3d26f'}, cover:{label:'Shells', col:'#efe9dc'}};
const AXCOL = {roll:0xff6b6b, pitch:0x4cc38a, yaw:0x4fb3ff};

// ---------- decode the packed meshes
const AB = decodeBin(M);

// ---------- three.js scene (Z up, metres, like the URDF)
const host = $('viewer');
let renderer;
try { renderer = new THREE.WebGLRenderer({antialias:true}); }
catch (e) {
  $('vmsg').innerHTML = `<div><b>Your browser has 3-D graphics (WebGL) turned off, so the model cannot be drawn.</b><br><br>
    To check, open <code>chrome://gpu</code> (Chrome or Chromium) and look for “WebGL: Hardware accelerated”. To turn it on:
    in <code>chrome://settings/system</code> switch on “Use graphics acceleration when available” and relaunch; if it is still off,
    your graphics card may be on the browser's block list, which <code>chrome://flags/#ignore-gpu-blocklist</code> overrides (at your own judgement).
    Or open this page in another browser, such as Firefox.<br><br>The text below this picture still works without 3-D.</div>`;
  throw e;
}
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
host.appendChild(renderer.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color('#0c1015');
const camera = new THREE.PerspectiveCamera(32, 1, 0.02, 50); camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.72); controls.enableDamping = true;
setupLighting(renderer, scene);
const grid = new THREE.GridHelper(6, 24, 0x2a3340, 0x1b222b); grid.rotation.x = Math.PI / 2; scene.add(grid);
const floor = new THREE.Mesh(new THREE.CircleGeometry(3, 48), new THREE.MeshStandardMaterial({color:0x10151b, roughness:1})); floor.position.z = -0.001; floor.receiveShadow = true; scene.add(floor);
function resize() { const w = host.clientWidth, h = host.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
new ResizeObserver(resize).observe(host);
function view(dir) {
  const t = controls.target, d = 3.1;
  const v = {front:[1,0,0.12], side:[0,1,0.12], back:[-1,0,0.12], iso:[0.8,-0.75,0.35]}[dir];
  camera.position.set(t.x + v[0]*d, t.y + v[1]*d, t.z + v[2]*d); controls.update();
}
view('iso');

// ---------- build the kinematic tree
const linkG = {}; const base = new THREE.Group(); scene.add(base);
const linkNames = new Set([M.root, ...M.joints.map(j => j.child)]);
for (const n of linkNames) { linkG[n] = new THREE.Group(); linkG[n].name = n; }
base.add(linkG[M.root]);
const J = {}; const children = {};
for (const j of M.joints) {
  const frame = new THREE.Group(); frame.position.set(...j.xyz); frame.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
  const rot = new THREE.Group(); frame.add(rot); rot.add(linkG[j.child]); linkG[j.parent].add(frame);
  const type = /roll/.test(j.name) ? 'roll' : /yaw/.test(j.name) ? 'yaw' : 'pitch';
  J[j.name] = {...j, frame, rot, q:0, axisV:new THREE.Vector3(...j.axis).normalize(), type, home:new THREE.Vector3(...j.xyz), lift:new THREE.Vector3()};
  (children[j.parent] ||= []).push(j.child);
}
const JN = M.joints.map(j => j.name);
function setQ(name, q) { const j = J[name]; if (!j) return; j.q = Math.min(j.upper, Math.max(j.lower, q)); j.rot.quaternion.setFromAxisAngle(j.axisV, j.q); }
// links beyond each joint (its subtree)
const subtree = {};
function collect(l, out) { out.push(l); for (const c of children[l] || []) collect(c, out); return out; }
for (const n of JN) subtree[n] = collect(J[n].child, []);

// ---------- parts
const meshes = []; const matOf = new Map();
const partGeometry = p => buildGeometry(p, AB);
for (const p of M.parts) {
  const mat = partMaterial(p);
  const m = new THREE.Mesh(partGeometry(p), mat); m.castShadow = m.receiveShadow = true;
  m.position.set(...p.xyz); m.rotation.set(p.rpy[0], p.rpy[1], p.rpy[2], 'ZYX');
  m.userData.p = p; m.userData.real = mat.color.clone(); m.userData.home = m.position.clone();
  linkG[p.link].add(m); meshes.push(m);
}
// motors per joint (a RobStride motor is drawn as a body and a driver cap; RS00 and servos as one piece)
const motorCount = {};
for (const p of M.parts) if (p.cat === 'actuator') motorCount[p.joint] = (motorCount[p.joint] || 0) + ((p.model === 'RS00' || p.model === 'STS3250') ? 1 : 0.5);
function catMass(cat) {
  if (cat === 'actuator') return Object.entries(motorCount).reduce((s, [jn, n]) => s + n * M.actuators[M.motor_of[jn]].mass, 0);
  return M.parts.filter(p => p.cat === cat).reduce((s, p) => s + (p.mass || 0), 0);
}

// joint axis arrows
const axisArrows = [];
for (const n of JN) { const j = J[n]; const a = new THREE.ArrowHelper(j.axisV, new THREE.Vector3(), 0.12, AXCOL[j.type], 0.03, 0.018); a.visible = false; j.rot.add(a); axisArrows.push(a); }

// CoM marker and support outline
const comBall = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), new THREE.MeshBasicMaterial({color:0xffb454, depthTest:false}));
comBall.renderOrder = 10; scene.add(comBall);
const comLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineDashedMaterial({color:0xffb454, dashSize:0.02, gapSize:0.015, depthTest:false}));
comLine.renderOrder = 10; scene.add(comLine);
const supportLine = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:0x4cc38a})); scene.add(supportLine);
const stair = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.9, 0.18), new THREE.MeshStandardMaterial({color:0x3a4656, roughness:0.9})); stair.visible = false; scene.add(stair);

// ---------- state
const DEF = {tab:'layers', colour:'real', xray:0, explode:0, axes:false, com:true, focus:[], show:Object.fromEntries(Object.keys(CATS).map(c => [c, c !== 'cover'])),
             step:0, skill:'stand', speed:1, playing:true, motorcol:'both', couple:true};
const state = structuredClone(DEF);
let selected = null;

// ---------- poses
const STANCE = M.stance;   // degrees, robot.yaml convention (same sign in the URDF)
function applyDeg(map) { for (const [k, v] of Object.entries(map)) setQ(k, v * D2R); }
function stanceMap() {
  const m = {};
  for (const s of ['left', 'right']) {
    m[`${s}_hip_pitch`] = STANCE.hip_pitch; m[`${s}_knee`] = STANCE.knee; m[`${s}_ankle_pitch`] = STANCE.ankle_pitch;
    m[`${s}_shoulder_roll`] = STANCE.shoulder_roll; m[`${s}_elbow`] = STANCE.elbow;
  }
  return m;
}
function zeroAll() { for (const n of JN) setQ(n, 0); }
function placePelvis(x, y, z, r = 0, p = 0, yw = 0) { base.position.set(x, y, z); base.rotation.set(r, p, yw, 'ZYX'); }

// reference foot frames: stance pose, pelvis at its stance height
zeroAll(); applyDeg(stanceMap()); placePelvis(0, 0, M.pelvis_height); scene.updateMatrixWorld(true);
const FOOT0 = {};
for (const s of ['left', 'right']) { const T = linkG[`${s}_foot_link`].matrixWorld; FOOT0[s] = {p:new THREE.Vector3().setFromMatrixPosition(T), q:new THREE.Quaternion().setFromRotationMatrix(T)}; }
const SOLE = new THREE.Vector3(0.03, 0, -0.06), TOE = new THREE.Vector3(0.15, 0, -0.06), toeOnly = {};           // sole centre in the foot frame (the foot's contact box)
const FOOT_RECT = [[-0.09, -0.055], [0.15, -0.055], [0.15, 0.055], [-0.09, 0.055]];   // 0.24 x 0.11 m about x = 0.03
// lift the whole robot so the soles touch z = 0 at stance
const soleZ0 = new THREE.Vector3().copy(SOLE).applyMatrix4(linkG.left_foot_link.matrixWorld).z;
const PELVIS_Z = M.pelvis_height - soleZ0;
for (const s of ['left', 'right']) FOOT0[s].p.z -= soleZ0;

// ---------- leg inverse kinematics (damped least squares on the real chain)
const LEG = s => [`${s}_hip_roll`, `${s}_hip_pitch`, `${s}_hip_yaw`, `${s}_knee`, `${s}_ankle_pitch`, `${s}_ankle_roll`];
const _v = new THREE.Vector3(), _a = new THREE.Vector3(), _p = new THREE.Vector3(), _q = new THREE.Quaternion(), _qi = new THREE.Quaternion();
function solve6(A, b) {   // Gaussian elimination, 6x6
  const n = 6, m = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c; for (let r = c + 1; r < n; r++) if (Math.abs(m[r][c]) > Math.abs(m[piv][c])) piv = r;
    [m[c], m[piv]] = [m[piv], m[c]]; const d = m[c][c] || 1e-12;
    for (let r = 0; r < n; r++) if (r !== c) { const f = m[r][c] / d; for (let k = c; k <= n; k++) m[r][k] -= f * m[c][k]; }
  }
  return m.map((r, i) => r[n] / (r[i] || 1e-12));
}
function ikLeg(s, tp, tq, iters = 14) {
  const names = LEG(s), foot = linkG[`${s}_foot_link`]; let err = 0;
  for (let it = 0; it < iters; it++) {
    base.updateMatrixWorld(true);
    const fp = new THREE.Vector3().setFromMatrixPosition(foot.matrixWorld), fq = new THREE.Quaternion().setFromRotationMatrix(foot.matrixWorld);
    const ep = tp.clone().sub(fp);
    _qi.copy(fq).invert(); _q.copy(tq).multiply(_qi); if (_q.w < 0) { _q.x = -_q.x; _q.y = -_q.y; _q.z = -_q.z; _q.w = -_q.w; }
    const eo = new THREE.Vector3(_q.x, _q.y, _q.z).multiplyScalar(2);
    const e = [ep.x, ep.y, ep.z, eo.x * 0.3, eo.y * 0.3, eo.z * 0.3];
    err = ep.length();
    if (err < 2e-4 && eo.length() < 2e-3) break;
    const Jc = names.map(n => {
      const j = J[n]; _p.setFromMatrixPosition(j.rot.matrixWorld);
      _a.copy(j.axisV).transformDirection(j.rot.matrixWorld);
      const lin = new THREE.Vector3().crossVectors(_a, _v.copy(fp).sub(_p));
      return [lin.x, lin.y, lin.z, _a.x * 0.3, _a.y * 0.3, _a.z * 0.3];
    });
    const lam2 = 0.0025, A = [];
    for (let r = 0; r < 6; r++) { A.push([]); for (let c = 0; c < 6; c++) { let s2 = 0; for (let k = 0; k < 6; k++) s2 += Jc[k][r] * Jc[k][c]; A[r].push(s2 + (r === c ? lam2 : 0)); } }
    const y = solve6(A, e);
    names.forEach((n, k) => { let dq = 0; for (let r = 0; r < 6; r++) dq += Jc[k][r] * y[r]; setQ(n, J[n].q + Math.max(-0.3, Math.min(0.3, dq))); });
  }
  return err;
}

// ---------- skills: each returns pelvis pose, foot targets (relative to stance), contacts and joint angles
const ease = x => x <= 0 ? 0 : x >= 1 ? 1 : x * x * (3 - 2 * x);
const seg = (t, a, b) => ease((t - a) / (b - a));
function keyed(t, keys) {   // keys: [[t, value]...], smooth between
  if (t <= keys[0][0]) return keys[0][1];
  for (let i = 1; i < keys.length; i++) if (t <= keys[i][0]) {
    const [t0, v0] = keys[i-1], [t1, v1] = keys[i], s = ease((t - t0) / (t1 - t0));
    return Array.isArray(v0) ? v0.map((x, k) => x + (v1[k] - x) * s) : v0 + (v1 - v0) * s;
  }
  return keys[keys.length - 1][1];
}
const arms = (pitch, roll, yaw, elbow, side) => ({[`${side}_shoulder_pitch`]:pitch, [`${side}_shoulder_roll`]:roll, [`${side}_shoulder_yaw`]:yaw, [`${side}_elbow`]:elbow});
const restArms = () => ({...arms(0, STANCE.shoulder_roll, 0, STANCE.elbow, 'left'), ...arms(0, STANCE.shoulder_roll, 0, STANCE.elbow, 'right')});
const STAIR = {x:0.30, h:0.18};
const SKILLS = {
  stand:{label:'Stand', T:4, about:'The default stance the walking policies start from: knees bent 15°, feet under the hips.', fn:t => ({
    pel:[0, 0, PELVIS_Z + 0.004 * Math.sin(2 * Math.PI * t / 4), 0, 0, 0], q:{...restArms(), neck_pitch:4 * Math.sin(2 * Math.PI * t / 4)}})},
  squat:{label:'Squat', T:5, about:'A deep squat with the arms forward for balance: hips back, knees and ankles bend together.', fn:t => {
    const s = (1 - Math.cos(2 * Math.PI * t / 5)) / 2;
    return {pel:[-0.10 * s, 0, PELVIS_Z - 0.27 * s, 0, 22 * s * D2R, 0], q:{...arms(75 * s, 8, 0, 15, 'left'), ...arms(75 * s, 8, 0, 15, 'right'), neck_pitch:-15 * s}};
  }},
  walk:{label:'Slow walk', T:1.4, about:'A slow, careful walk on a treadmill (0.21 m/s): the hips sway over each foot so the centre of mass stays above the foot that is down. That keeps the loads meaningful for this still-picture estimate; the trained policy walks faster and dynamically, letting the centre of mass fall between steps (see 02).', walk:true, fn:t => {
    const T = 1.4, L = 0.18, H = 0.05, ph = (t / T) % 1, v = L / (0.6 * T);
    const foot = o => { const f = (ph + o) % 1; if (f < 0.6) return {dx:L / 2 - L * f / 0.6, dz:0, c:true};
      const s = (f - 0.6) / 0.4; return {dx:-L / 2 + L * ease(s), dz:H * Math.sin(Math.PI * s), c:false}; };
    const l = foot(0), r = foot(0.5);
    const sway = 0.085 * Math.sin(2 * Math.PI * (ph - 0.05));
    return {pel:[0, sway, PELVIS_Z - 0.02 + 0.006 * Math.cos(4 * Math.PI * ph), 0, 3 * D2R, 0], feet:{left:[l.dx, 0, l.dz, l.c], right:[r.dx, 0, r.dz, r.c]}, v,
      q:{...arms(-14 * Math.sin(2 * Math.PI * ph), 8, 0, 25, 'left'), ...arms(14 * Math.sin(2 * Math.PI * ph), 8, 0, 25, 'right'), waist_yaw:5 * Math.sin(2 * Math.PI * ph)}};
  }},
  stair:{label:'Step up 18 cm', T:8, about:'Stepping onto an 18 cm stair, the height the project designs for. The worst moment is lifting the whole body on the bent leading leg.', stair:true, fn:t => {
    const L0 = FOOT0.left.p, R0 = FOOT0.right.p, rx = STAIR.x + 0.06 - R0.x;
    const pel = keyed(t, [[0, [0, 0, 0, 0]], [1.0, [0.0, 0.07, -0.02, 0]], [2.2, [0.02, 0.07, -0.03, 6]], [3.2, [0.15, -0.05, -0.02, 14]],
                          [4.2, [rx - 0.08, -0.085, 0.06, 20]], [5.4, [rx, -0.09, STAIR.h - 0.01, 10]], [6.6, [rx, 0.0, STAIR.h, 2]], [8, [rx, 0, STAIR.h, 0]]]);
    const rf = keyed(t, [[1.2, [0, 0, 0]], [1.7, [0.10, 0, STAIR.h + 0.07]], [2.4, [rx, 0, STAIR.h]]]);
    const lf = keyed(t, [[4.2, [0, 0, 0.0]], [5.0, [0.12, 0, 0.12]], [5.8, [rx * 0.7, 0, STAIR.h + 0.08]], [6.6, [rx, 0, STAIR.h]]]);
    const heel = t < 4.2 ? 38 * seg(t, 2.8, 4.0) : 38 * (1 - seg(t, 4.2, 5.0));
    const rc = t < 1.2 || t > 2.4, lc = t < 4.2 || t > 6.6;
    return {pel:[pel[0], pel[1], PELVIS_Z + pel[2], 0, pel[3] * D2R, 0], feet:{left:[lf[0], 0, lf[2], lc, heel], right:[rf[0], 0, rf[2], rc]},
      q:{...arms(10 * Math.sin(t), 10, 0, 25, 'left'), ...arms(-10 * Math.sin(t), 10, 0, 25, 'right'), neck_pitch:15}};
  }},
  wave:{label:'Wave', T:6, about:'Right arm raised out to the side, the forearm swinging at the elbow.', fn:t => {
    const up = seg(t, 0, 1) * (1 - seg(t, 5, 6)), w = 30 * Math.sin(2 * Math.PI * 1.4 * t) * up;
    return {pel:[0, 0, PELVIS_Z, 0, 0, 0], q:{...restArms(), ...arms(20 * up, 8 + 95 * up, -20 * up, 20 + 70 * up + w, 'right'), right_wrist_yaw:-40 * up, neck_yaw:-20 * up, waist_yaw:-8 * up}};
  }},
  clap:{label:'Clap', T:4, about:'Both arms forward with the shoulders rolled inwards below 0°, which the coupled limit allows only with the arm forward.', fn:t => {
    const up = seg(t, 0, 0.8) * (1 - seg(t, 3.3, 4)), c = (1 - Math.cos(2 * Math.PI * 2.5 * t)) / 2 * up;
    const pitch = 85 * up, roll = -18 * c * up + 6 * (1 - up);
    return {pel:[0, 0, PELVIS_Z, 0, 0, 0], q:{...arms(pitch, roll, 25 * up, 25 + 20 * up, 'left'), ...arms(pitch, roll, 25 * up, 25 + 20 * up, 'right'), neck_pitch:15 * up}};
  }},
  pick:{label:'Reach low', T:6, about:'Squat and reach both hands forward and down towards the floor, as for picking up a box.', fn:t => {
    const s = seg(t, 0, 2) * (1 - seg(t, 4, 6));
    return {pel:[-0.11 * s, 0, PELVIS_Z - 0.25 * s, 0, 32 * s * D2R, 0], q:{...arms(55 * s, 10, 0, 15, 'left'), ...arms(55 * s, 10, 0, 15, 'right'), right_wrist_pitch:-30 * s, left_wrist_pitch:-30 * s, neck_pitch:20 * s}};
  }},
  look:{label:'Look around', T:8, about:'Turning the waist and neck, as when the depth camera scans a room.', fn:t => {
    const a = Math.sin(2 * Math.PI * t / 8);
    return {pel:[0, 0, PELVIS_Z, 0, 0, 0], q:{...restArms(), waist_yaw:55 * a, neck_yaw:45 * Math.sin(2 * Math.PI * t / 8 + 0.6), neck_pitch:12 * Math.sin(4 * Math.PI * t / 8)}};
  }},
};

// ---------- statics
const inert = Object.entries(M.inertia).filter(([, v]) => v).map(([n, v]) => ({n, m:v.mass, c:new THREE.Vector3(...v.com)}));
const MASS = inert.reduce((s, i) => s + i.m, 0);
const ANKLE_CAP = {pitch:{rated:32, peak:96}, roll:{rated:25, peak:74}};
function rating(n) {
  if (/ankle_pitch/.test(n)) return {...ANKLE_CAP.pitch, model:'2 × RS03 linkage'};
  if (/ankle_roll/.test(n)) return {...ANKLE_CAP.roll, model:'2 × RS03 linkage'};
  const mdl = M.motor_of[n], a = M.actuators[mdl]; return {rated:a.rated_torque, peak:a.peak_torque, model:mdl, speed:a.no_load_speed_rpm * 2 * Math.PI / 60};
}
function convexHull(pts) {
  pts = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]); const cr = (o, a, b) => (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0]);
  const lo = [], up = []; for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length-2], lo[lo.length-1], p) <= 0) lo.pop(); lo.push(p); }
  for (const p of pts.slice().reverse()) { while (up.length >= 2 && cr(up[up.length-2], up[up.length-1], p) <= 0) up.pop(); up.push(p); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}
const inside = (hull, x, y) => hull.length > 2 && hull.every((p, i) => { const q = hull[(i + 1) % hull.length]; return (q[0]-p[0])*(y-p[1]) - (q[1]-p[1])*(x-p[0]) >= -1e-9; });
function statics(contacts) {
  scene.updateMatrixWorld(true);
  const W = inert.map(i => ({n:i.n, m:i.m, c:i.c.clone().applyMatrix4(linkG[i.n].matrixWorld)}));
  const C = W.reduce((s, w) => s.addScaledVector(w.c, w.m), new THREE.Vector3()).multiplyScalar(1 / MASS);
  const soles = contacts.map(s => ({s, link:`${s}_foot_link`, p:SOLE.clone().applyMatrix4(linkG[`${s}_foot_link`].matrixWorld)}));
  let hull = [];
  for (const s of contacts) for (const [x0, y] of FOOT_RECT) { const x = toeOnly[s] ? Math.max(x0, 0.13) : x0; const p = new THREE.Vector3(x, y, -0.06).applyMatrix4(linkG[`${s}_foot_link`].matrixWorld); hull.push([p.x, p.y]); }
  hull = hull.length ? convexHull(hull) : [];
  // floor push: total weight, centred under the CoM (the centre of pressure), each foot's share by the lever rule,
  // and each foot's push kept inside that foot's outline
  const clampToFoot = (s, x, y, z) => { const T = linkG[`${s}_foot_link`].matrixWorld, inv = T.clone().invert();
    const l = new THREE.Vector3(x, y, z).applyMatrix4(inv); l.x = Math.max(toeOnly[s] ? 0.13 : -0.09, Math.min(0.15, l.x)); l.y = Math.max(-0.055, Math.min(0.055, l.y)); l.z = -0.06;
    return l.applyMatrix4(T); };
  if (soles.length === 1) { soles[0].F = MASS * G; soles[0].p = clampToFoot(soles[0].s, C.x, C.y, soles[0].p.z); }
  if (soles.length === 2) { const A = soles[0].p, B = soles[1].p, ab = new THREE.Vector2(B.x - A.x, B.y - A.y);
    const t = Math.max(0, Math.min(1, ((C.x - A.x) * ab.x + (C.y - A.y) * ab.y) / Math.max(ab.lengthSq(), 1e-9)));
    const dx = C.x - (A.x + ab.x * t), dy = C.y - (A.y + ab.y * t);
    soles[0].F = (1 - t) * MASS * G; soles[1].F = t * MASS * G;
    soles[0].p = clampToFoot(soles[0].s, A.x + dx, A.y + dy, A.z); soles[1].p = clampToFoot(soles[1].s, B.x + dx, B.y + dy, B.z); }
  const tau = {};
  for (const n of JN) {
    const j = J[n]; const pj = new THREE.Vector3().setFromMatrixPosition(j.rot.matrixWorld), ax = j.axisV.clone().transformDirection(j.rot.matrixWorld);
    const sum = new THREE.Vector3(), set = subtree[n];
    for (const w of W) if (set.includes(w.n)) sum.add(new THREE.Vector3().crossVectors(w.c.clone().sub(pj), new THREE.Vector3(0, 0, -w.m * G)));
    for (const f of soles) if (set.includes(f.link)) sum.add(new THREE.Vector3().crossVectors(f.p.clone().sub(pj), new THREE.Vector3(0, 0, f.F)));
    tau[n] = Math.abs(sum.dot(ax));
  }
  return {C, hull, tau, stable:contacts.length ? inside(hull, C.x, C.y) : null};
}

// ---------- look: visibility, colours, X-ray, load colours
function moduleVisible(p) { if (state.tab !== 'assembly') return true; return ASM[state.step].show.includes(p.module); }
function loadColour(f) { return f < 0.7 ? new THREE.Color('#4cc38a') : f < 1 ? new THREE.Color('#ffb454') : new THREE.Color('#ff6b6b'); }
let lastStat = null;
function applyLook() {
  const skillLoads = state.tab === 'skills' && state.motorcol !== 'off' && lastStat;
  for (const m of meshes) {
    const p = m.userData.p, mat = m.material;
    m.visible = state.tab === 'assembly' ? moduleVisible(p) : state.show[p.cat];
    if (state.tab === 'skills' && p.cat === 'cover') m.visible = state.show.cover;
    let col = m.userData.real.clone();
    if (state.colour === 'cat') col = new THREE.Color(p.cat === 'actuator' ? m.userData.real : CATS[p.cat].col);
    if (state.colour === 'module') col = new THREE.Color(MODS[p.module].col);
    let op = 1, emis = 0x000000;
    const focus = state.focus.length ? state.focus.includes(p.cat) : false;
    if (state.xray > 0 && state.focus.length && !focus) op = Math.max(0.04, 1 - state.xray * 0.97);
    if (focus && state.xray > 0) emis = 0x3a2a08;
    if (state.tab === 'assembly') { const st = ASM[state.step]; if (st.hi.includes(p.module)) emis = 0x40300a; }
    if (skillLoads) {
      if (p.cat === 'actuator') {
        const mv = Math.min(1, moveSpeed(p) / 1.5);   // full glow from 1.5 rad/s (about 90°/s) upwards
        op = 1; emis = 0x000000;
        if (state.motorcol === 'move') col = new THREE.Color('#2c333d').lerp(new THREE.Color('#4fb3ff'), mv);
        else {
          col = loadColour(loadFrac(p));
          if (state.motorcol === 'both') { if (mv < 0.03) col.multiplyScalar(0.3); else { col.multiplyScalar(0.3 + 0.7 * Math.min(1, mv * 3)); mat.emissive.copy(col).multiplyScalar(0.8 * mv); } }
        }
        mat.color.copy(col); mat.opacity = 1; mat.depthWrite = true; if (state.motorcol !== 'both' || mv < 0.03) mat.emissive.setHex(0);
        if (selected === m) mat.emissive.setHex(0x1d4f7a);
        continue;
      }
      else op = Math.min(op, 0.22);
    }
    if (selected === m) emis = 0x1d4f7a;
    mat.color.copy(col); mat.opacity = op; mat.depthWrite = op > 0.9; mat.emissive.setHex(emis);
  }
  for (const a of axisArrows) a.visible = state.axes;
  comBall.visible = comLine.visible = supportLine.visible = state.com;
}
function moveSpeed(p) {   // joint speed in rad/s (for the ankle motors, the faster of ankle pitch and roll)
  const side = (p.joint || '').split('_')[0];
  if (/ankle_pitch/.test(p.joint)) return Math.max(J[`${side}_ankle_pitch`].speed || 0, J[`${side}_ankle_roll`].speed || 0);
  return J[p.joint].speed || 0;
}
function loadFrac(p) {
  const t = lastStat.tau, side = (p.joint || '').split('_')[0];
  if (/ankle_pitch/.test(p.joint)) return Math.max(t[`${side}_ankle_pitch`] / ANKLE_CAP.pitch.rated, t[`${side}_ankle_roll`] / ANKLE_CAP.roll.rated);
  return (t[p.joint] || 0) / rating(p.joint).rated;
}

// ---------- assembly steps
const ASM = [
  {t:'Pelvis', d:'Back plate with the two hip roll RS04s, top plate and gussets, the waist RS03 and the pelvis power board. Bench-test the three motors on a short CAN cable.', show:['pelvis'], hi:['pelvis'], anim:{}},
  {t:'Legs', d:'Each leg is built from the hip roll bracket down to the foot: five motors, the ankle linkage set with the foot flat at zero, then tested on a single-leg jig before it is bolted to its hip roll motor’s output flange.', show:['pelvis', 'leg'], hi:['leg'], anim:{legs:true}},
  {t:'Torso core', d:'Frame, shoulder pitch RS03s, neck yaw servo, electronics bay, the Orin and the battery cradle with the e-stop chain. Powered up without limbs. The battery slides in from the back.', show:['pelvis', 'leg', 'torso', 'battery'], hi:['torso', 'battery'], anim:{torsoLift:true, battery:true}},
  {t:'Join torso to pelvis', d:'The torso core bolts to the waist motor’s output flange; the trunk harness (power plus five CAN pairs) is plugged in through its service loop.', show:['pelvis', 'leg', 'torso', 'battery'], hi:['torso'], anim:{torsoJoin:true}},
  {t:'Arms and head', d:'Each arm bolts to its shoulder pitch motor’s output flange with one power and one signal connector; the head goes onto the neck yaw servo with the camera’s USB-C cable.', show:['pelvis', 'leg', 'torso', 'battery', 'arm', 'head'], hi:['arm', 'head'], anim:{arms:true}},
  {t:'Calibrate, then shells', d:'Hung from the gantry, every joint’s zero is calibrated. Then, if wanted, the 24 clip-on shells go on.', show:['pelvis', 'leg', 'torso', 'battery', 'arm', 'head', 'cover'], hi:['cover'], anim:{covers:true}},
];
let asmT0 = 0;
function asmOffsets(now) {
  const s = ASM[state.step], k = 1 - ease((now - asmT0) / 1400), a = s.anim;
  for (const n of JN) J[n].lift.set(0, 0, 0);
  for (const sd of ['left', 'right']) {
    if (a.legs) J[`${sd}_hip_roll`].lift.set(0, 0, -0.25 * k);
    if (a.arms) J[`${sd}_shoulder_pitch`].lift.set(0, (sd === 'left' ? 1 : -1) * 0.25 * k, 0);
  }
  if (a.arms) J.neck_yaw.lift.set(0, 0, 0.2 * k);
  if (a.torsoLift) J.waist_yaw.lift.set(0, 0, 0.25);
  if (a.torsoJoin) J.waist_yaw.lift.set(0, 0, 0.25 * k);
  for (const m of meshes) {
    m.position.copy(m.userData.home);
    if (a.battery && m.userData.p.module === 'battery') m.position.x -= 0.25 * k;
    if (a.covers && m.userData.p.cat === 'cover') { const d = m.userData.home.clone(); m.position.addScaledVector(d.lengthSq() > 1e-6 ? d.normalize() : d, 0.12 * k); }
  }
}
function placeJoints() { for (const n of JN) { const j = J[n]; j.frame.position.copy(j.home).multiplyScalar(1 + state.explode * 0.9).add(j.lift); } }

// ---------- UI: tabs
document.querySelectorAll('[data-tab]').forEach(b => b.addEventListener('click', () => setTab(b.dataset.tab)));
function setTab(t) {
  state.tab = t; document.querySelectorAll('[data-tab]').forEach(b => b.classList.toggle('on', b.dataset.tab === t));
  document.querySelectorAll('.tabpane').forEach(p => p.classList.toggle('on', p.id === 'tab-' + t));
  for (const n of JN) J[n].lift.set(0, 0, 0); for (const m of meshes) m.position.copy(m.userData.home);
  stair.visible = false; grid.position.x = 0;
  if (t === 'layers' || t === 'joints') { if (t === 'layers') { zeroAll(); applyDeg(stanceMap()); } placePelvis(0, 0, PELVIS_Z); syncSliders(); }
  if (t === 'assembly') { zeroAll(); applyDeg(stanceMap()); placePelvis(0, 0, PELVIS_Z); setStep(state.step); }
  if (t === 'skills') { setSkill(state.skill); }
  if (t !== 'skills') $('loadwrap').innerHTML = '';
  if (t === 'joints') updateStatics([]);
  else if (t !== 'skills') updateStatics(['left', 'right']);
  updateAll();
}

// layers list
function renderCats() {
  $('cats').innerHTML = Object.entries(CATS).map(([c, v]) => {
    const n = M.parts.filter(p => p.cat === c).length, kg = catMass(c);
    return `<div class="catrow ${state.focus.includes(c) ? 'focus' : ''}" data-c="${c}"><input type="checkbox" ${state.show[c] ? 'checked' : ''} aria-label="Show ${v.label}"><i style="background:${v.col}"></i><span>${v.label}</span><span class="m">${kg ? kg.toFixed(2) + ' kg' : n}</span></div>`;
  }).join('');
  $('cats').querySelectorAll('.catrow').forEach(r => {
    const c = r.dataset.c;
    r.querySelector('input').addEventListener('click', e => { e.stopPropagation(); state.show[c] = e.target.checked; updateAll(); });
    r.addEventListener('click', () => { const i = state.focus.indexOf(c); i < 0 ? state.focus.push(c) : state.focus.splice(i, 1); if (state.focus.length && state.xray === 0) setVal('xray', 0.85); renderCats(); updateAll(); });
  });
}
const VIEWS = {
  bare:{show:{cover:false}, focus:[], xray:0, colour:'real'},
  shells:{show:{cover:true}, focus:[], xray:0, colour:'real'},
  xact:{show:{cover:false}, focus:['actuator'], xray:0.85, colour:'real'},
  xelec:{show:{cover:false}, focus:['electronics', 'sensors', 'compute', 'battery'], xray:0.85, colour:'cat'},
  made:{show:{cover:false}, focus:[], xray:0, colour:'cat'},
  buy:{show:{cnc:false, service:false, print:false, cover:false, padding:false}, focus:[], xray:0, colour:'cat'},
};
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => {
  const v = VIEWS[b.dataset.view];
  state.show = Object.fromEntries(Object.keys(CATS).map(c => [c, c !== 'cover'])); Object.assign(state.show, v.show);
  state.focus = [...v.focus]; setVal('xray', v.xray); state.colour = v.colour; $('colour').value = v.colour; state.view = b.dataset.view;
  renderCats(); updateAll();
}));
function setVal(id, v) { state[id] = v; $(id).value = v; const o = $('o_' + id); if (o) o.textContent = fmt[id] ? fmt[id](v) : v; }
const fmt = {xray:v => Math.round(v * 100) + '%', explode:v => Math.round(v * 100) + '%', speed:v => v.toFixed(2) + '×'};
for (const id of ['xray', 'explode', 'speed']) { $(id).addEventListener('input', e => { setVal(id, parseFloat(e.target.value)); updateAll(); }); setVal(id, state[id]); }
$('colour').addEventListener('change', e => { state.colour = e.target.value; updateAll(); });
$('axes').addEventListener('change', e => { state.axes = e.target.checked; updateAll(); });
$('com').addEventListener('change', e => { state.com = e.target.checked; updateAll(); });
$('motorcol').addEventListener('change', e => { state.motorcol = e.target.value; updateAll(); });
['Front', 'Side', 'Back', 'Iso'].forEach(v => $('v' + v).addEventListener('click', () => view(v.toLowerCase())));

// assembly list
$('steps').innerHTML = ASM.map((s, i) => `<li data-i="${i}"><b>${i + 1}.</b> ${s.t}<div class="hint" style="color:var(--muted);font-size:12px">${s.d}</div></li>`).join('');
$('steps').querySelectorAll('li').forEach(li => li.addEventListener('click', () => setStep(+li.dataset.i)));
function setStep(i) { state.step = Math.max(0, Math.min(ASM.length - 1, i)); asmT0 = performance.now(); $('steps').querySelectorAll('li').forEach((li, k) => li.classList.toggle('on', k === state.step)); updateAll(); }
$('asmPrev').addEventListener('click', () => setStep(state.step - 1)); $('asmNext').addEventListener('click', () => setStep(state.step + 1)); $('asmReplay').addEventListener('click', () => setStep(state.step));

// skills list
$('skills').innerHTML = Object.entries(SKILLS).map(([k, s]) => `<button data-skill="${k}">${s.label}</button>`).join('');
$('skills').querySelectorAll('button').forEach(b => b.addEventListener('click', () => setSkill(b.dataset.skill)));
let simT = 0, peaks = {}, prevQ = {}, worstFlag = '';
function setSkill(k) {
  state.skill = k; simT = 0; peaks = {}; prevQ = {}; worstFlag = '';
  $('skills').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.skill === k));
  zeroAll(); applyDeg(stanceMap()); placePelvis(0, 0, PELVIS_Z); scene.updateMatrixWorld(true);
  stair.visible = !!SKILLS[k].stair; if (stair.visible) stair.position.set(STAIR.x + 0.35, 0, STAIR.h / 2);
  updateAll();
}
$('playbtn').addEventListener('click', () => { state.playing = !state.playing; $('playbtn').textContent = state.playing ? '❚❚' : '▶'; });
$('time').addEventListener('input', e => { state.playing = false; $('playbtn').textContent = '▶'; simT = parseFloat(e.target.value) * SKILLS[state.skill].T; });

function runSkill(dt) {
  const S = SKILLS[state.skill]; simT = (simT + dt * state.speed) % S.T;
  const P = S.fn(simT);
  placePelvis(P.pel[0], P.pel[1], P.pel[2], P.pel[3], P.pel[4], P.pel[5]);
  for (const n of JN) if (!/hip|knee|ankle/.test(n)) setQ(n, 0);
  applyDeg(stanceMap()); for (const s of ['left', 'right']) for (const n of LEG(s)) setQ(n, prevQ[n] ?? J[n].q);
  for (const n of ['left_shoulder_roll', 'right_shoulder_roll']) {}
  applyDeg(P.q || {});
  // shoulder coupled limit, as the controller would enforce it
  for (const s of ['left', 'right']) { const lo = coupledMin(J[`${s}_shoulder_pitch`].q * R2D); if (J[`${s}_shoulder_roll`].q * R2D < lo) setQ(`${s}_shoulder_roll`, lo * D2R); }
  const contacts = []; let maxErr = 0; for (const k in toeOnly) delete toeOnly[k];
  for (const s of ['left', 'right']) {
    const f = P.feet ? P.feet[s] : [0, 0, 0, true], heel = (f[4] || 0) * D2R;
    let tp = FOOT0[s].p.clone().add(new THREE.Vector3(f[0], f[1], f[2])), tq = FOOT0[s].q;
    if (heel) {   // heel lifted: the foot pivots about its toe edge (toes on the ground, pushing off)
      const toe = TOE.clone().applyQuaternion(FOOT0[s].q).add(tp), rq = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), heel);
      tq = rq.clone().multiply(FOOT0[s].q); tp = toe.clone().add(TOE.clone().negate().applyQuaternion(tq));
    }
    maxErr = Math.max(maxErr, ikLeg(s, tp, tq, dt === 0 ? 80 : 14));
    if (f[3]) { contacts.push(s); toeOnly[s] = heel > 0.02; }
  }
  if (S.walk) grid.position.x = -((simT * P.v) % 0.25);
  const st = updateStatics(contacts);
  const dtq = Math.max(dt * state.speed, 1e-3);
  for (const n of JN) { const r = rating(n), sp = prevQ[n] === undefined || dt === 0 ? (J[n].speed || 0) : Math.abs(J[n].q - prevQ[n]) / dtq;
    const pk = peaks[n] ||= {tau:0, sp:0}; pk.tau = Math.max(pk.tau, st.tau[n]); pk.sp = Math.max(pk.sp, sp); J[n].speed = sp; prevQ[n] = J[n].q; }
  worstFlag = maxErr > 0.01 ? `A foot target was out of reach by ${(maxErr * 100).toFixed(1)} cm (a joint hit its limit).` : '';
  $('time').value = simT / S.T; $('o_time').textContent = simT.toFixed(1) + ' s';
}
function coupledMin(pitchDeg) {
  const c = (M.coupled[0] || {}).min || []; if (!c.length || pitchDeg <= c[0][0] || pitchDeg >= c[c.length - 1][0]) return J.left_shoulder_roll.lower * R2D < 0 ? 0 : 0;
  for (let i = 1; i < c.length; i++) if (pitchDeg <= c[i][0]) { const [x0, y0] = c[i-1], [x1, y1] = c[i]; return y0 + (y1 - y0) * (pitchDeg - x0) / (x1 - x0); }
  return 0;
}

// ---------- joint sliders
const GROUPS = [['Legs (left)', LEG('left')], ['Legs (right)', LEG('right')], ['Waist and neck', ['waist_yaw', 'neck_yaw', 'neck_pitch']],
  ['Left arm', ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw', 'left_elbow', 'left_wrist_yaw', 'left_wrist_pitch']],
  ['Right arm', ['right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw', 'right_elbow', 'right_wrist_yaw', 'right_wrist_pitch']]];
$('jsliders').innerHTML = GROUPS.map(([g, ns]) => `<div class="jgrp">${g}</div>` + ns.map(n => {
  const j = J[n], short = n.replace(/^(left|right)_/, '').replace(/_/g, ' ');
  return `<div class="jrow" id="jr_${n}"><span title="${n}">${short}</span><input type="range" id="j_${n}" min="${Math.round(j.lower * R2D)}" max="${Math.round(j.upper * R2D)}" step="1" aria-label="${n}"><output id="jo_${n}"></output></div>`;
}).join('')).join('');
for (const n of JN) $('j_' + n).addEventListener('input', e => { setQ(n, parseFloat(e.target.value) * D2R); enforceCoupled(n); syncSliders(); updateStatics([]); updateAll(); });
function enforceCoupled(changed) {
  for (const s of ['left', 'right']) {
    const lo = coupledMin(J[`${s}_shoulder_pitch`].q * R2D), roll = J[`${s}_shoulder_roll`].q * R2D;
    $('jr_' + s + '_shoulder_roll').classList.toggle('bad', roll < lo - 0.5);
    if (state.couple && roll < lo) setQ(`${s}_shoulder_roll`, lo * D2R);
  }
}
function syncSliders() { for (const n of JN) { const d = J[n].q * R2D; $('j_' + n).value = Math.round(d); $('jo_' + n).textContent = Math.round(d) + '°'; } enforceCoupled(); }
$('couple').addEventListener('change', e => { state.couple = e.target.checked; enforceCoupled(); syncSliders(); updateStatics([]); updateAll(); });
const JPRE = {
  jStance:() => { zeroAll(); applyDeg(stanceMap()); }, jZero:() => zeroAll(),
  jArms:() => { zeroAll(); applyDeg(stanceMap()); applyDeg({left_shoulder_pitch:90, right_shoulder_pitch:90, left_shoulder_roll:-15, right_shoulder_roll:-15, left_elbow:10, right_elbow:10}); },
  jClap:() => { zeroAll(); applyDeg(stanceMap()); applyDeg({left_shoulder_pitch:90, right_shoulder_pitch:90, left_shoulder_roll:-20, right_shoulder_roll:-20, left_shoulder_yaw:30, right_shoulder_yaw:30, left_elbow:40, right_elbow:40}); }};
for (const [id, f] of Object.entries(JPRE)) $(id).addEventListener('click', () => { f(); state.jpreset = id; placePelvis(0, 0, PELVIS_Z); syncSliders(); updateStatics([]); updateAll(); });

// ---------- statics readouts
function updateStatics(contacts) {
  const st = statics(contacts); lastStat = st;
  comBall.position.copy(st.C); comLine.geometry.setFromPoints([st.C, new THREE.Vector3(st.C.x, st.C.y, 0.002)]); comLine.computeLineDistances();
  supportLine.geometry.setFromPoints(st.hull.map(([x, y]) => new THREE.Vector3(x, y, 0.003 + (state.tab === 'skills' && SKILLS[state.skill].stair && x > STAIR.x ? STAIR.h : 0))));
  supportLine.visible = state.com && st.hull.length > 2;
  return st;
}
function loadRows() {
  if (!lastStat) return [];
  return JN.map(n => { const r = rating(n), t = lastStat.tau[n]; return {n, t, r, f:t / r.rated, pk:(peaks[n] || {}).tau || t, sp:J[n].speed || 0}; }).sort((a, b) => b.f - a.f);
}
function renderLoads() {
  const rows = loadRows();
  if (state.tab === 'skills' || state.tab === 'joints') {
    $('loadwrap').innerHTML = `<table class="loadtab"><tr><th>Joint</th><th>Motor</th><th class="num">Angle</th><th class="num">Torque now</th><th>vs rated (line = rated)</th><th class="num">Peak in loop</th><th class="num">Speed</th></tr>` +
      rows.slice(0, state.allRows ? 27 : 10).map(o => {
        const w = Math.min(100, o.f * 50), c = o.f < 0.7 ? 'var(--good)' : o.f < 1 ? 'var(--warn)' : 'var(--bad)';
        const spPct = o.r.speed ? ` (${Math.round(o.sp / o.r.speed * 100)}%)` : '';
        return `<tr><td>${o.n.replace(/_/g, ' ')}</td><td>${o.r.model}</td><td class="num">${(J[o.n].q * R2D).toFixed(0)}°</td><td class="num">${o.t.toFixed(1)} N·m</td><td><div class="bar"><i style="width:${w}%;background:${c}"></i><s></s></div></td><td class="num">${state.tab === 'skills' ? o.pk.toFixed(1) + ' N·m' : '–'}</td><td class="num">${state.tab === 'skills' ? o.sp.toFixed(1) + ' rad/s' + spPct : '–'}</td></tr>`;
      }).join('') + `</table><button id="allrows">${state.allRows ? 'Show the top 10' : 'Show all 27 joints'}</button> <span style="font-size:12px;color:var(--muted)">Bar: torque as a share of the motor's continuous rating; the white line is 100%, the bar's end 200%. Speed in brackets: share of the motor's no-load speed.</span>`;
    $('allrows').addEventListener('click', () => { state.allRows = !state.allRows; renderLoads(); });
  }
}
function setRO(id, v, cls) { const e = $(id); e.querySelector('.v').textContent = v; e.className = 'ro' + (cls ? ' ' + cls : ''); }
function readouts() {
  const shown = M.parts.filter(p => state.show[p.cat] && moduleVisible(p));
  setRO('r_mass', `${shown.length} parts`);
  if (lastStat) {
    setRO('r_com', `${lastStat.C.z.toFixed(3)} m`);
    if (state.tab === 'joints') setRO('r_support', 'on gantry');
    else setRO('r_support', lastStat.stable === null ? '–' : lastStat.stable ? 'CoM over feet' : 'would tip', lastStat.stable ? 'ok' : 'bad');
    const w = loadRows()[0];
    if (w && (state.tab === 'skills' || state.tab === 'joints')) setRO('r_worst', `${w.n.replace(/_/g, ' ')} ${Math.round(w.f * 100)}%`, w.f < 0.7 ? 'ok' : w.f < 1 ? 'warn' : 'bad');
    else setRO('r_worst', 'see Skills');
  }
  $('legend').innerHTML = state.tab === 'skills' && state.motorcol !== 'off' ? (state.motorcol === 'move' ? '<span><i style="background:#2c333d"></i>motor holding still</span><span><i style="background:#4fb3ff"></i>motor turning (brighter = faster)</span>' : '<span><i style="background:#4cc38a"></i>motor below 70% of rated</span><span><i style="background:#ffb454"></i>70–100%</span><span><i style="background:#ff6b6b"></i>above rated</span>' + (state.motorcol === 'both' ? '<span>dark = holding still, glowing = turning</span>' : '')) + '<span><i style="background:#ffb454;border-radius:50%"></i>centre of mass</span><span><i style="background:#4cc38a"></i>outline of the feet on the ground</span>'
    : (state.colour === 'module' ? Object.values(MODS) : state.colour === 'cat' ? Object.values(CATS) : []).map(v => `<span><i style="background:${v.col}"></i>${v.label}</span>`).join('');
  const sf = $('skillflag');
  if (state.tab === 'skills') {
    const S = SKILLS[state.skill];
    sf.innerHTML = `<div class="flag">${S.about}</div>` + (worstFlag ? `<div class="flag bad">${worstFlag}</div>` : '') +
      (lastStat && lastStat.stable === false ? `<div class="flag bad">Right now the centre of mass is outside the feet: standing still here, the robot would tip. A moving robot can pass through such moments (<a href="../02-balance/">02</a>).</div>` : '');
  }
}

// ---------- picking
const ray = new THREE.Raycaster(); let downAt = null;
renderer.domElement.addEventListener('pointerdown', e => downAt = [e.clientX, e.clientY]);
renderer.domElement.addEventListener('pointerup', e => {
  if (!downAt || Math.hypot(e.clientX - downAt[0], e.clientY - downAt[1]) > 5) return;
  const r = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1), camera);
  const hit = ray.intersectObjects(meshes.filter(m => m.visible && m.material.opacity > 0.3), false)[0];
  selected = hit ? hit.object : null; showInfo(); updateAll();
});
function showInfo() {
  const box = $('vinfo'); if (!selected) { box.hidden = true; return; }
  const p = selected.userData.p; let html = `<span class="x" id="vx">✕</span><b>${p.name.replace(/_/g, ' ')}</b>${CATS[p.cat].label} · module: ${MODS[p.module].label}<br>`;
  if (p.cat === 'actuator') {
    const a = M.actuators[p.model], jn = p.joint;
    html += /ankle/.test(jn) ? `One of the two RS03s that drive ${jn.split('_')[0]} ankle pitch and roll through push rods (<a href="../06-ankle-linkage/">06</a>).<br>` : `Drives <b style="display:inline">${jn.replace(/_/g, ' ')}</b>.<br>`;
    html += `${p.model}: ${a.peak_torque} N·m peak, ${a.rated_torque} N·m continuous, ${a.no_load_speed_rpm} rpm, ${a.mass} kg. <a href="../04-actuators/">More on 04</a>`;
  } else {
    if (p.material && p.material !== 'bought') html += `Material: ${({al6061:'6061 aluminium', al7075:'7075 aluminium', pa_cf:'carbon-fibre nylon (PA-CF)', tpu:'TPU (rubbery print)'})[p.material] || p.material}<br>`;
    if (p.mass) html += `Mass: ${(p.mass * 1000).toFixed(0)} g<br>`;
    html += `Rides on: ${p.link.replace(/_/g, ' ')}`;
    if (p.kind === 'box') html += '<br>Drawn as an outline box until the real part is chosen.';
  }
  box.innerHTML = html; box.hidden = false; $('vx').addEventListener('click', () => { selected = null; showInfo(); updateAll(); });
}

// ---------- decision note
function updateNote() {
  const parts = [];
  if (state.tab === 'layers') {
    const off = Object.keys(CATS).filter(c => !state.show[c] && c !== 'cover');
    if (state.show.cover) parts.push('with the body shells on');
    if (state.focus.length && state.xray > 0) parts.push(`X-raying ${state.focus.map(c => CATS[c].label.toLowerCase()).join(', ')}`);
    if (off.length) parts.push(`with ${off.map(c => CATS[c].label.toLowerCase()).join(', ')} hidden`);
    if (selected) parts.push(`looking at the part "${selected.userData.p.name}" (${CATS[selected.userData.p.cat].label.toLowerCase()})`);
  }
  if (state.tab === 'assembly') parts.push(`at assembly step ${state.step + 1} of 6, "${ASM[state.step].t}"`);
  if ((state.tab === 'skills' || state.tab === 'joints') && lastStat) {
    const rows = JN.map(n => ({n, pk:state.tab === 'skills' ? (peaks[n] || {}).tau || 0 : lastStat.tau[n], r:rating(n)})).map(o => ({...o, f:o.pk / o.r.rated})).sort((a, b) => b.f - a.f).slice(0, 3);
    const what = state.tab === 'skills' ? `playing the scripted "${SKILLS[state.skill].label}" skill` : 'posing the robot by hand on the gantry';
    parts.push(`${what}; the highest quasi-static loads ${state.tab === 'skills' ? 'over the loop' : 'in this pose'} were ` + rows.map(o => `${o.n.replace(/_/g, ' ')} ${o.pk.toFixed(1)} N·m (${Math.round(o.f * 100)}% of its ${o.r.model}'s continuous rating)`).join(', '));
  }
  $('note').textContent = parts.length
    ? `In the Marvin Bot 3-D playground (the 1.44 m, 41 kg open humanoid, model generated from its code-CAD) I was ${parts.join('; ')}. These loads are quasi-static estimates (gravity only, no accelerations or impacts). Questions I want to explore next: which of these joints needs more margin, and what would moving mass or changing a motor do to it?`
    : 'Change something in the playground to build a note.';
}
$('copy').addEventListener('click', () => navigator.clipboard.writeText($('note').textContent).then(() => { $('copy').textContent = 'Copied!'; setTimeout(() => $('copy').textContent = 'Copy', 1200); }));

// ---------- main loop
let lastUI = 0, lastT = performance.now();
function updateAll() { placeJoints(); applyLook(); readouts(); renderLoads(); updateNote(); }
function frame(now) {
  const dt = Math.min(0.05, (now - lastT) / 1000); lastT = now;
  if (state.tab === 'assembly') { asmOffsets(now); placeJoints(); }
  if (state.tab === 'skills' && state.playing) runSkill(dt);
  else if (state.tab === 'skills' && !lastStat) runSkill(0);
  if (state.tab === 'skills' && (now - lastUI > 120 || !state.playing)) { lastUI = now; applyLook(); readouts(); renderLoads(); updateNote(); if (!state.playing) runSkill(0); }
  controls.update(); renderer.render(scene, camera); requestAnimationFrame(frame);
}
renderCats(); syncSliders(); resize(); updateStatics(['left', 'right']); updateAll();
// deep links: #view=xact, #tab=assembly&step=3, #tab=skills&skill=stair&t=4.2&pause=1
function fromHash() {
  const h = new URLSearchParams(location.hash.slice(1)); if (![...h.keys()].length) return;
  if (h.get('tab')) setTab(h.get('tab'));
  if (h.get('view')) document.querySelector(`[data-view="${h.get('view')}"]`)?.click();
  if (h.get('step')) setStep(parseInt(h.get('step'), 10) - 1);
  if (h.get('skill') && SKILLS[h.get('skill')]) { setTab('skills'); setSkill(h.get('skill')); }
  if (h.get('t')) { simT = parseFloat(h.get('t')); runSkill(0); }
  if (h.get('pause')) { state.playing = false; $('playbtn').textContent = '▶'; }
  if (h.get('cam')) view(h.get('cam'));
}
fromHash(); addEventListener('hashchange', fromHash);
$('vmsg').remove();
window.__marvin = {state, J, statics, SKILLS, setSkill, setTab, runSkill, PELVIS_Z, MASS, catMass, meshes, peaks:() => peaks};
requestAnimationFrame(frame);
