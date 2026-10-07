// motionAmigo browser demo: three.js front end for the WebAssembly planner.
import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';
import init, { Demo, simdBackend, pandaReady } from './pkg/motionamigo_wasm.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const $ = (id) => document.getElementById(id);
const COLORS = {
  table: 0x9c7a54, chair: 0x6b7480, laptop: 0x2d3540, mug: 0x3d8fd1, bowl: 0xd8b25a,
  book: 0xc0504d, shelf_panel: 0xb08a5f, shelf_board: 0xc39a6b, box: 0x8f6fbf, bottle: 0x4caf8a,
  cage_plate: 0x7d8792, cage_wall: 0x7d8792, cage_bar: 0x9aa4ae, block: 0xe07b39,
};
const GRASPABLE = new Set(['mug', 'bowl', 'book', 'laptop', 'box', 'bottle', 'block']);
const ROBOT_COLOR = 0xeef1f4;
const COLLISION_COLOR = 0xef5b5b;
const SPEED = 1.4; // joint-space radians per second during playback
const CAMERAS = {
  tabletop: [[0.25, -2.3, 1.65], [0.5, 0.05, 0.38]],
  shelf: [[-1.05, -1.7, 1.65], [0.5, 0.05, 0.55]],
  cage: [[-0.6, -1.6, 1.45], [0.5, 0.05, 0.45]],
};

// ---------------------------------------------------------------- three.js setup

const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
$('viewport').appendChild(renderer.domElement);

const scene3d = new THREE.Scene();
scene3d.background = new THREE.Color(0x0f1419);
scene3d.fog = new THREE.Fog(0x0f1419, 6, 14);

const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 50);
camera.position.set(0.55, -2.15, 1.4);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.45, 0.05, 0.42);
controls.enableDamping = true;
controls.update();

scene3d.add(new THREE.HemisphereLight(0xdfe8f2, 0x2a2118, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(2.0, -1.5, 3.0);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -2, right: 2, top: 2, bottom: -2, near: 0.5, far: 8 });
scene3d.add(sun);

const floor = new THREE.Mesh(
  new THREE.CircleGeometry(4, 64),
  new THREE.MeshStandardMaterial({ color: 0x1a2129, roughness: 0.95 }),
);
floor.receiveShadow = true;
scene3d.add(floor);
const grid = new THREE.GridHelper(6, 30, 0x2c3743, 0x222b35);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.001;
scene3d.add(grid);

function resize() {
  const w = window.innerWidth;
  const h = window.innerHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

// ---------------------------------------------------------------- robot drawn as capsules

const Y = new THREE.Vector3(0, 1, 0);

function capsule(radius, length, material) {
  const mesh = new THREE.Mesh(new THREE.CapsuleGeometry(radius, Math.max(length, 1e-4), 6, 16), material);
  mesh.castShadow = true;
  return mesh;
}

class RobotView {
  constructor(demo, { opacity = 1, color = ROBOT_COLOR } = {}) {
    const transparent = opacity < 1;
    this.material = new THREE.MeshStandardMaterial({
      color, roughness: 0.45, metalness: 0.1, transparent, opacity, depthWrite: !transparent,
    });
    this.jointMaterial = new THREE.MeshStandardMaterial({
      color: transparent ? color : 0x3a434e, roughness: 0.6, transparent, opacity, depthWrite: !transparent,
    });
    this.group = new THREE.Group();
    // Rigid segments between joint origins (indices into the frame list) and their radii.
    this.segments = [[0, 1, 0.075], [1, 3, 0.064], [3, 4, 0.058], [4, 5, 0.055], [5, 7, 0.048], [7, 'flange', 0.042]];
    const pts = this.points(demo.frames(pandaReady()));
    this.links = this.segments.map(([a, b, r]) => {
      const mesh = capsule(r, pts[a].distanceTo(pts[b]), this.material);
      this.group.add(mesh);
      return mesh;
    });
    this.joints = [1, 3, 4, 5, 7].map(() => {
      const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.068, 20, 14), this.jointMaterial);
      mesh.castShadow = true;
      this.group.add(mesh);
      return mesh;
    });
    this.hand = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.2, 0.065), this.material);
    this.hand.castShadow = true;
    this.fingers = [capsule(0.011, 0.04, this.jointMaterial), capsule(0.011, 0.04, this.jointMaterial)];
    this.group.add(this.hand, ...this.fingers);
    if (transparent) this.group.traverse((o) => { o.castShadow = false; });
  }

  points(flat) {
    const m = [];
    for (let i = 0; i < flat.length / 16; i++) m.push(new THREE.Matrix4().fromArray(flat, 16 * i));
    const pts = m.map((x) => new THREE.Vector3().setFromMatrixPosition(x));
    // m[0..7]: frames 0 to 7, m[8]: tool center point.
    const tcp = m[8];
    const z = new THREE.Vector3().setFromMatrixColumn(tcp, 2);
    pts.flange = pts[8].clone().addScaledVector(z, -0.1034);
    pts.tcp = pts[8];
    pts.tcpMatrix = tcp;
    return pts;
  }

  update(flat) {
    const pts = this.points(flat);
    this.segments.forEach(([a, b], i) => {
      const dir = new THREE.Vector3().subVectors(pts[b], pts[a]);
      const mesh = this.links[i];
      mesh.position.copy(pts[a]).addScaledVector(dir, 0.5);
      if (dir.lengthSq() > 1e-10) mesh.quaternion.setFromUnitVectors(Y, dir.normalize());
    });
    [1, 3, 4, 5, 7].forEach((k, i) => this.joints[i].position.copy(pts[k]));
    const rot = new THREE.Quaternion().setFromRotationMatrix(pts.tcpMatrix);
    const ax = (k) => new THREE.Vector3().setFromMatrixColumn(pts.tcpMatrix, k);
    this.hand.quaternion.copy(rot);
    this.hand.position.copy(pts.flange).addScaledVector(ax(2), 0.035);
    this.fingers.forEach((f, i) => {
      f.quaternion.copy(rot).multiply(new THREE.Quaternion().setFromUnitVectors(Y, new THREE.Vector3(0, 0, 1)));
      f.position.copy(pts.flange).addScaledVector(ax(2), 0.085).addScaledVector(ax(1), i ? -0.035 : 0.035);
    });
    return pts.tcp;
  }

  setColor(hex) {
    this.material.color.setHex(hex);
  }
}

// ---------------------------------------------------------------- application state

const state = {
  demo: null,
  sceneName: 'tabletop',
  presets: {},
  q: null,
  goal: null,
  objects: new Map(),
  animating: false,
  seed: 0,
  randomCount: 0,
  lastResult: null,
};

let robot;
let ghost;
let sphereMesh;
let trace;

function setStatus(text, cls = '') {
  const el = $('status');
  el.textContent = text;
  el.className = cls;
}

function fmtMs(ms) {
  if (!Number.isFinite(ms)) return 'n/a';
  // Browsers coarsen performance.now() to about 0.1 ms, so very fast plans can measure as zero.
  if (ms === 0) return '< 0.1 ms';
  return ms < 1 ? `${(ms * 1000).toFixed(0)} µs` : `${ms.toFixed(2)} ms`;
}

function buildObjects(sceneJson) {
  for (const mesh of state.objects.values()) scene3d.remove(mesh);
  state.objects.clear();
  for (const o of sceneJson.objects) {
    const color = COLORS[o.label] ?? 0x8899aa;
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(...o.size),
      new THREE.MeshStandardMaterial({ color, roughness: 0.7 }),
    );
    mesh.position.set(...o.center);
    mesh.rotation.z = o.yaw;
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData = { id: o.id, label: o.label };
    scene3d.add(mesh);
    state.objects.set(o.id, mesh);
  }
}

function updateSpheres() {
  sphereMesh.visible = $('show-spheres').checked;
  if (!sphereMesh.visible) return;
  const s = state.demo.spheres(state.q);
  const m = new THREE.Matrix4();
  for (let i = 0; i < s.length / 4; i++) {
    const r = s[4 * i + 3];
    m.makeScale(r, r, r).setPosition(s[4 * i], s[4 * i + 1], s[4 * i + 2]);
    sphereMesh.setMatrixAt(i, m);
  }
  sphereMesh.instanceMatrix.needsUpdate = true;
}

function showConfig(q) {
  state.q = q;
  robot.update(state.demo.frames(q));
  robot.setColor(state.demo.configValid(q) ? ROBOT_COLOR : COLLISION_COLOR);
  updateSpheres();
}

function showGoal() {
  ghost.group.visible = $('show-goal').checked && !!state.goal;
  if (state.goal) ghost.update(state.demo.frames(state.goal));
}

function fillGoals(sceneJson) {
  const select = $('goal');
  select.innerHTML = '';
  const configs = document.createElement('optgroup');
  configs.label = 'joint configurations';
  for (const p of state.presets[state.sceneName] ?? []) {
    const opt = document.createElement('option');
    opt.value = JSON.stringify(p.q);
    opt.textContent = p.name;
    configs.appendChild(opt);
  }
  select.appendChild(configs);
  // Pre-grasp goals are computed with inverse kinematics when selected.
  const grasps = document.createElement('optgroup');
  grasps.label = 'pre-grasp above (IK)';
  for (const o of sceneJson.objects.filter((x) => GRASPABLE.has(x.label))) {
    const opt = document.createElement('option');
    opt.value = `pregrasp:${o.id}`;
    opt.textContent = `above ${o.id}`;
    grasps.appendChild(opt);
  }
  if (grasps.children.length) select.appendChild(grasps);
  selectGoal();
}

function selectGoal() {
  const v = $('goal').value;
  if (v.startsWith('pregrasp:')) {
    const id = v.slice('pregrasp:'.length);
    const t0 = performance.now();
    const q = state.demo.pregraspGoal(id, state.q);
    const ms = performance.now() - t0;
    if (q.length === 0) {
      state.goal = null;
      setStatus(`No collision-free pre-grasp pose above ${id}.`, 'bad');
    } else {
      state.goal = q;
      setStatus(`IK for the pre-grasp pose above ${id} took ${fmtMs(ms)}. Press Plan.`);
    }
  } else {
    state.goal = v ? new Float64Array(JSON.parse(v)) : null;
  }
  showGoal();
}

async function loadScene(name) {
  state.sceneName = name;
  $('scene').value = name;
  const json = await (await fetch(`scenes/${name}.json`)).text();
  state.demo.setScene(json);
  const sceneJson = JSON.parse(json);
  buildObjects(sceneJson);
  const [pos, target] = CAMERAS[name] ?? CAMERAS.tabletop;
  camera.position.set(...pos);
  controls.target.set(...target);
  controls.update();
  trace.visible = false;
  showConfig(new Float64Array(pandaReady()));
  fillGoals(sceneJson);
  setStatus(`${name}: pick a goal and press Plan.`);
}

function randomGoal() {
  state.randomCount += 1;
  const q = state.demo.randomGoal(BigInt(1000 + state.randomCount), 0.45, 20000);
  if (q.length === 0) {
    setStatus('No random goal found, try again.', 'bad');
    return;
  }
  const select = $('goal');
  const opt = document.createElement('option');
  opt.value = JSON.stringify(Array.from(q));
  opt.textContent = `random ${state.randomCount}`;
  select.querySelector('optgroup').appendChild(opt);
  select.value = opt.value;
  selectGoal();
}

// Dense configurations along the path, parameterized by joint-space arc length.
function densify(path, dof) {
  const wps = [];
  for (let i = 0; i < path.length; i += dof) wps.push(path.slice(i, i + dof));
  const out = [wps[0]];
  for (let i = 1; i < wps.length; i++) {
    const a = wps[i - 1];
    const b = wps[i];
    const d = Math.hypot(...a.map((v, k) => b[k] - v));
    const n = Math.max(1, Math.ceil(d / 0.02));
    for (let s = 1; s <= n; s++) out.push(a.map((v, k) => v + ((b[k] - v) * s) / n));
  }
  return out;
}

function drawTrace(dense) {
  const pts = dense.map((q) => {
    const f = state.demo.frames(Float64Array.from(q));
    return new THREE.Vector3(f[8 * 16 + 12], f[8 * 16 + 13], f[8 * 16 + 14]);
  });
  trace.geometry.dispose();
  trace.geometry = new THREE.BufferGeometry().setFromPoints(pts);
  trace.visible = true;
}

function setBusy(busy) {
  state.animating = busy;
  for (const id of ['plan', 'reset', 'scene', 'goal', 'random']) $(id).disabled = busy;
}

function plan() {
  if (state.animating || !state.goal) return null;
  const seed = state.seed++;
  $('seed').textContent = seed;
  const out = state.demo.plan(state.q, state.goal, BigInt(seed));
  state.lastResult = out;
  if (!out.solved) {
    setStatus(out.message, 'bad');
    out.free();
    return null;
  }
  $('t-plan').textContent = fmtMs(out.planningMs);
  $('t-simp').textContent = fmtMs(out.simplifyMs);
  $('waypoints').textContent = out.waypoints;
  $('length').textContent = `${out.length.toFixed(2)} rad`;
  $('iterations').textContent = out.iterations;
  setStatus(`Found a path in ${fmtMs(out.planningMs + out.simplifyMs)}.`, 'ok');
  const dense = densify(Array.from(out.path), state.demo.dof);
  out.free();
  drawTrace(dense);
  animate(dense);
  return dense.length;
}

function animate(dense) {
  setBusy(true);
  const start = performance.now();
  const step = 0.02;
  const total = (dense.length - 1) * step;
  const tick = () => {
    const s = Math.min(total, ((performance.now() - start) / 1000) * SPEED);
    const i = Math.min(dense.length - 1, Math.floor(s / step));
    showConfig(Float64Array.from(dense[i]));
    if (s < total) requestAnimationFrame(tick);
    else setBusy(false);
  };
  requestAnimationFrame(tick);
}

// ---------------------------------------------------------------- dragging objects

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let drag = null;

function updateRay(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.set(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(pointer, camera);
}

function pick(event) {
  updateRay(event);
  return raycaster.intersectObjects([...state.objects.values()], false)[0];
}

renderer.domElement.addEventListener('pointerdown', (event) => {
  if (state.animating || event.button !== 0) return;
  const hit = pick(event);
  if (!hit) return;
  const mesh = hit.object;
  const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), -hit.point.z);
  drag = { mesh, plane, offset: mesh.position.clone().sub(hit.point) };
  controls.enabled = false;
  renderer.domElement.setPointerCapture(event.pointerId);
});

renderer.domElement.addEventListener('pointermove', (event) => {
  const tip = $('tooltip');
  if (drag) {
    updateRay(event);
    const p = raycaster.ray.intersectPlane(drag.plane, new THREE.Vector3());
    if (p) {
      drag.mesh.position.x = p.x + drag.offset.x;
      drag.mesh.position.y = p.y + drag.offset.y;
      const m = drag.mesh;
      state.demo.moveObject(m.userData.id, m.position.x, m.position.y, m.position.z, m.rotation.z);
      showConfig(state.q);
    }
    tip.hidden = true;
    return;
  }
  const hit = pick(event);
  if (hit) {
    tip.hidden = false;
    tip.textContent = hit.object.userData.id;
    tip.style.left = `${event.clientX + 12}px`;
    tip.style.top = `${event.clientY + 12}px`;
  } else {
    tip.hidden = true;
  }
});

renderer.domElement.addEventListener('pointerup', (event) => {
  if (!drag) return;
  drag = null;
  controls.enabled = true;
  renderer.domElement.releasePointerCapture(event.pointerId);
  trace.visible = false;
  if (state.demo.configValid(state.q)) setStatus('Object moved. Plan again.');
  else setStatus('The robot is now in collision, move the object away.', 'bad');
});

// ---------------------------------------------------------------- start

function render() {
  controls.update();
  renderer.render(scene3d, camera);
  requestAnimationFrame(render);
}

async function main() {
  await init();
  state.presets = await (await fetch('presets.json')).json();
  const json = await (await fetch(`scenes/${state.sceneName}.json`)).text();
  state.demo = new Demo(json);
  robot = new RobotView(state.demo);
  ghost = new RobotView(state.demo, { opacity: 0.22, color: 0x4cc38a });
  scene3d.add(robot.group, ghost.group);
  sphereMesh = new THREE.InstancedMesh(
    new THREE.SphereGeometry(1, 16, 12),
    new THREE.MeshStandardMaterial({ color: 0xf28c28, transparent: true, opacity: 0.45, depthWrite: false }),
    state.demo.spheres(pandaReady()).length / 4,
  );
  scene3d.add(sphereMesh);
  trace = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xf28c28 }));
  scene3d.add(trace);
  $('backend').textContent = `Rust compiled to WebAssembly, SIMD backend: ${simdBackend()}`;

  $('scene').addEventListener('change', (e) => loadScene(e.target.value));
  $('goal').addEventListener('change', selectGoal);
  $('random').addEventListener('click', randomGoal);
  $('plan').addEventListener('click', plan);
  $('reset').addEventListener('click', () => { trace.visible = false; showConfig(new Float64Array(pandaReady())); });
  $('show-spheres').addEventListener('change', updateSpheres);
  $('show-goal').addEventListener('change', showGoal);

  await loadScene(state.sceneName);
  // Warm up the WebAssembly JIT so that the first measured plan is representative.
  const presets = state.presets[state.sceneName];
  if (presets?.length > 1) {
    for (let i = 0; i < 3; i++) {
      state.demo.plan(Float64Array.from(presets[0].q), Float64Array.from(presets[1].q), BigInt(i)).free();
    }
  }
  render();

  // Small API for automated screenshots (web/tools/record.mjs).
  window.motionAmigoDemo = {
    loadScene,
    selectGoalByName(name) {
      const opt = [...$('goal').options].find((o) => o.textContent === name);
      if (!opt) throw new Error(`no goal ${name}`);
      $('goal').value = opt.value;
      selectGoal();
    },
    plan,
    isAnimating: () => state.animating,
    // Plans without animating; returns the number of dense samples for showSample().
    planStatic() {
      const seed = state.seed++;
      $('seed').textContent = seed;
      const out = state.demo.plan(state.q, state.goal, BigInt(seed));
      if (!out.solved) { const m = out.message; out.free(); throw new Error(m); }
      $('t-plan').textContent = fmtMs(out.planningMs);
      $('t-simp').textContent = fmtMs(out.simplifyMs);
      $('waypoints').textContent = out.waypoints;
      $('length').textContent = `${out.length.toFixed(2)} rad`;
      $('iterations').textContent = out.iterations;
      setStatus(`Found a path in ${fmtMs(out.planningMs + out.simplifyMs)}.`, 'ok');
      state.dense = densify(Array.from(out.path), state.demo.dof);
      out.free();
      drawTrace(state.dense);
      return state.dense.length;
    },
    showSample(i) { showConfig(Float64Array.from(state.dense[Math.min(i, state.dense.length - 1)])); },
    renderNow() { controls.update(); renderer.render(scene3d, camera); },
    setCamera(pos, target) {
      camera.position.set(...pos);
      controls.target.set(...target);
      controls.update();
    },
    // Screen position of a scene object's center (for automated drag tests).
    screenPosition(id) {
      const v = state.objects.get(id).position.clone().project(camera);
      const rect = renderer.domElement.getBoundingClientRect();
      return [rect.left + ((v.x + 1) / 2) * rect.width, rect.top + ((1 - v.y) / 2) * rect.height];
    },
    objectPosition: (id) => state.objects.get(id).position.toArray(),
    sceneJson: () => state.demo.sceneJson(),
    toggleSpheres(on) { $('show-spheres').checked = on; updateSpheres(); },
  };
  window.dispatchEvent(new Event('motionamigo-ready'));
}

main().catch((err) => {
  console.error(err);
  setStatus(`Failed to start: ${err.message}`, 'bad');
});
