// A light Marvin Bot viewer for the home page: the generated model at its stance, orbit controls,
// slow auto-rotation, a few looks (finished, bare, X-ray motors, X-ray electronics) and an idle
// animation. Needs window.MARVIN_MODEL, written by site/scripts/build_model.py.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { decodeBin, partGeometry, partMaterial, setupLighting, shadowFloor } from './marvin-render.js';

export function mountMarvin(host, opts = {}) {
  const M = window.MARVIN_MODEL;
  if (!M) { host.insertAdjacentHTML('beforeend', '<div class="vmsg">The 3-D model data has not been built (see site/README.md).</div>'); return null; }
  const o = {look:'shells', rotate:true, animate:'idle', bg:null, camera:'iso', target:0.72, dist:3.0, ...opts};
  const AB = decodeBin(M);
  let renderer;
  try { renderer = new THREE.WebGLRenderer({antialias:true, alpha:!o.bg}); }
  catch (e) { host.insertAdjacentHTML('beforeend', '<div class="vmsg">3-D graphics (WebGL) are turned off in this browser, so the model cannot be shown. The guide\'s 3-D page explains how to turn them on.</div>'); return null; }
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); host.appendChild(renderer.domElement);
  renderer.domElement.style.cssText = 'width:100%;height:100%;display:block';
  const scene = new THREE.Scene(); if (o.bg) scene.background = new THREE.Color(o.bg);
  const camera = new THREE.PerspectiveCamera(30, 1, 0.02, 50); camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, renderer.domElement); controls.target.set(0, 0, o.target); controls.enableDamping = true;
  controls.autoRotate = o.rotate; controls.autoRotateSpeed = 0.8; controls.enableZoom = o.zoom !== false;
  setupLighting(renderer, scene); scene.add(shadowFloor());
  if (o.grid !== false) { const g = new THREE.GridHelper(6, 24, 0x2a3340, 0x1b222b); g.rotation.x = Math.PI / 2; g.material.transparent = true; g.material.opacity = 0.6; scene.add(g); }

  const linkG = {}, J = {}, base = new THREE.Group(); scene.add(base);
  for (const n of new Set([M.root, ...M.joints.map(j => j.child)])) linkG[n] = new THREE.Group();
  base.add(linkG[M.root]);
  for (const j of M.joints) {
    const f = new THREE.Group(); f.position.set(...j.xyz); f.rotation.set(j.rpy[0], j.rpy[1], j.rpy[2], 'ZYX');
    const r = new THREE.Group(); f.add(r); r.add(linkG[j.child]); linkG[j.parent].add(f);
    J[j.name] = {...j, rot:r, axis:new THREE.Vector3(...j.axis).normalize()};
  }
  const setDeg = (n, d) => { const j = J[n]; if (!j) return; const q = Math.min(j.upper, Math.max(j.lower, d * Math.PI / 180)); j.rot.quaternion.setFromAxisAngle(j.axis, q); };
  const meshes = [];
  for (const p of M.parts) {
    const g = partGeometry(p, AB), mat = partMaterial(p);
    const m = new THREE.Mesh(g, mat); m.castShadow = m.receiveShadow = true; m.position.set(...p.xyz); m.rotation.set(p.rpy[0], p.rpy[1], p.rpy[2], 'ZYX');
    m.userData = {p, real:mat.color.clone()}; linkG[p.link].add(m); meshes.push(m);
  }
  const ST = M.stance, stance = {};
  for (const s of ['left', 'right']) Object.assign(stance, {[`${s}_hip_pitch`]:ST.hip_pitch, [`${s}_knee`]:ST.knee, [`${s}_ankle_pitch`]:ST.ankle_pitch, [`${s}_shoulder_roll`]:ST.shoulder_roll, [`${s}_elbow`]:ST.elbow});
  const pose = d => { for (const n in J) setDeg(n, (d[n] ?? stance[n] ?? 0)); };
  pose({}); base.position.set(0, 0, M.pelvis_height); scene.updateMatrixWorld(true);
  const sole = new THREE.Vector3(0.03, 0, -0.06).applyMatrix4(linkG.left_foot_link.matrixWorld).z; base.position.z -= sole;

  const XR = {motors:['actuator'], electronics:['electronics', 'sensors', 'compute', 'battery']};
  function setLook(look) {
    o.look = look;
    for (const m of meshes) {
      const p = m.userData.p, mat = m.material;
      m.visible = p.cat !== 'cover' || look === 'shells'; m.castShadow = !(XR[look] && !XR[look].includes(p.cat));
      const focus = XR[look] && XR[look].includes(p.cat), fade = XR[look] && !focus;
      mat.color.copy(m.userData.real); mat.opacity = fade ? 0.08 : 1; mat.depthWrite = !fade;
      mat.emissive.setHex(focus ? 0x2a1d05 : 0);
    }
  }
  setLook(o.look);
  const ANIM = {
    idle:t => ({neck_yaw:20 * Math.sin(t * 0.5), neck_pitch:5 * Math.sin(t * 0.8), waist_yaw:8 * Math.sin(t * 0.35)}),
    wave:t => { const u = Math.min(1, Math.max(0, Math.sin(t * 0.4) * 2)); return {right_shoulder_roll:8 + 95 * u, right_shoulder_pitch:20 * u, right_shoulder_yaw:-20 * u, right_elbow:20 + 70 * u + 28 * Math.sin(t * 8) * u, neck_yaw:-15 * u}; },
    none:() => ({}),
  };
  const views = {front:[1, 0, 0.1], iso:[0.85, -0.7, 0.3], side:[0, 1, 0.1]};
  const setCam = v => { const d = views[v] || views.iso, k = o.dist; camera.position.set(d[0] * k, d[1] * k, o.target + d[2] * k); controls.update(); };
  setCam(o.camera);
  const resize = () => { const w = host.clientWidth, h = host.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h;
    if (o.offsetX && w > 800) camera.setViewOffset(w, h, -w * o.offsetX, 0, w, h); else camera.clearViewOffset(); camera.updateProjectionMatrix(); };
  new ResizeObserver(resize).observe(host); resize();
  const t0 = performance.now();
  (function frame(now) { pose(ANIM[o.animate]((now - t0) / 1000)); controls.update(); renderer.render(scene, camera); requestAnimationFrame(frame); })(t0);
  renderer.domElement.addEventListener('pointerdown', () => { controls.autoRotate = false; });
  return {setLook, setAnim:a => { o.animate = a; }, setCam};
}
