// Rig do Microduck, portado de pollen-robotics/microduck-simulator
// (app/src/game/duck.js, Apache-2.0). Lê kinematics.json (gerado do MJCF)
// e monta uma árvore de Object3D com um Group por corpo; as juntas viram
// rotações locais. Cortes em relação ao original: sem mandíbula animada,
// sem contorno, sem variante de rodinhas.

import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { mergeVertices, toCreasedNormals } from "three/addons/utils/BufferGeometryUtils.js";

export const MODEL_DIR = "/pato/robot";
const GLB_URL = `${MODEL_DIR}/microduck.glb`;
const CREASE = Math.PI / 5;

export async function loadKinematics(arquivo = "kinematics.json") {
  const r = await fetch(`${MODEL_DIR}/${arquivo}`);
  if (!r.ok) throw new Error(`kinematics ${r.status}`);
  const k = await r.json();
  k.mesh_dir = `${MODEL_DIR}/meshes`;
  return k;
}

let glbGeomsPromise = null;
export function loadGlbGeometries() {
  if (!glbGeomsPromise) {
    glbGeomsPromise = new GLTFLoader().loadAsync(GLB_URL).then((gltf) => {
      const map = new Map();
      gltf.scene.traverse((o) => {
        if (!o.isMesh || !o.geometry) return;
        const name = o.userData.meshFile || o.name || o.geometry.name;
        if (!name || map.has(name)) return;
        const welded = o.geometry;
        welded.deleteAttribute("normal");
        // toCreasedNormals usa uma grade de 0,01: escala pra mm e volta.
        const scaled = welded.clone();
        scaled.scale(1000, 1000, 1000);
        const display = toCreasedNormals(scaled, CREASE);
        display.scale(1e-3, 1e-3, 1e-3);
        const entry = { display, welded };
        map.set(name, entry);
        if (o.name && o.name !== name) map.set(o.name, entry);
      });
      return map;
    });
  }
  return glbGeomsPromise;
}

// O compilador do MuJoCo WASM quer STL binário no VFS: reconstrói a partir
// do GLB já carregado, sem baixar os STLs de novo.
export function geometryToBinaryStl(geometry) {
  const pos = geometry.attributes.position;
  const idx = geometry.index;
  const triCount = (idx ? idx.count : pos.count) / 3;
  const buf = new ArrayBuffer(84 + triCount * 50);
  const view = new DataView(buf);
  view.setUint32(80, triCount, true);
  let off = 84;
  const vx = (i) => {
    const j = idx ? idx.getX(i) : i;
    return [pos.getX(j), pos.getY(j), pos.getZ(j)];
  };
  for (let t = 0; t < triCount; t++) {
    const a = vx(t * 3), b = vx(t * 3 + 1), c = vx(t * 3 + 2);
    const nx = (b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]);
    const ny = (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]);
    const nz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    const len = Math.hypot(nx, ny, nz) || 1;
    const vals = [nx / len, ny / len, nz / len, ...a, ...b, ...c];
    for (let v = 0; v < 12; v++) view.setFloat32(off + v * 4, vals[v], true);
    view.setUint16(off + 48, 0, true);
    off += 50;
  }
  return buf;
}

export async function buildRig(k, materialForMesh) {
  const placer = new THREE.Group();
  // root aplica a troca MJCF +Z pra cima -> three.js +Y pra cima.
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2;
  placer.add(root);

  const bodies = new Map();
  const joints = new Map();
  const geomByName = await loadGlbGeometries();
  const extra = new Map();
  const loadMesh = (name) => {
    const entry = geomByName.get(name);
    if (entry) return Promise.resolve(entry);
    if (!extra.has(name)) {
      extra.set(name, new STLLoader().loadAsync(`${k.mesh_dir}/${name}`).then((raw) => {
        raw.deleteAttribute("normal");
        const welded = mergeVertices(raw, 1e-4);
        welded.scale(1000, 1000, 1000);
        const display = toCreasedNormals(welded, CREASE);
        display.scale(1e-3, 1e-3, 1e-3);
        welded.scale(1e-3, 1e-3, 1e-3);
        return { display, welded };
      }));
    }
    return extra.get(name);
  };

  for (const b of k.bodies) {
    const g = new THREE.Group();
    g.name = b.name;
    g.position.set(b.pos[0], b.pos[1], b.pos[2]);
    g.quaternion.set(b.quat[1], b.quat[2], b.quat[3], b.quat[0]);
    bodies.set(b.name, g);
  }
  for (const b of k.bodies) {
    const g = bodies.get(b.name);
    if (b.parent && bodies.has(b.parent)) bodies.get(b.parent).add(g);
    else root.add(g);
  }
  for (const b of k.bodies) {
    if (!b.joint || (b.joint.type && b.joint.type !== "hinge")) continue;
    const g = bodies.get(b.name);
    joints.set(b.joint.name, {
      body: g,
      axis: new THREE.Vector3(...b.joint.axis).normalize(),
      baseQuat: g.quaternion.clone(),
      range: b.joint.range ?? null,
    });
  }

  const matCache = new Map();
  const matFor = (spec) => {
    const key = `${spec.color.join(",")}|${spec.roughness}|${spec.metalness}`;
    let m = matCache.get(key);
    if (!m) {
      m = new THREE.MeshStandardMaterial({
        color: new THREE.Color(...spec.color),
        roughness: spec.roughness ?? 0.5,
        metalness: spec.metalness ?? 0,
      });
      matCache.set(key, m);
    }
    return m;
  };

  const pending = [];
  const seen = new Set();
  for (const b of k.bodies) {
    const g = bodies.get(b.name);
    for (const geom of b.geoms) {
      if (geom.type && geom.type !== "mesh") continue;
      if (!geom.mesh) continue;
      const dup = `${b.name}|${geom.mesh}|${geom.pos}|${geom.quat}`;
      if (seen.has(dup)) continue;
      seen.add(dup);
      pending.push(loadMesh(geom.mesh).then(({ display }) => {
        const m = new THREE.Mesh(display, matFor(materialForMesh(geom.mesh)));
        m.castShadow = true;
        m.receiveShadow = true;
        if (geom.pos) m.position.set(...geom.pos);
        if (geom.quat) m.quaternion.set(geom.quat[1], geom.quat[2], geom.quat[3], geom.quat[0]);
        g.add(m);
      }));
    }
  }
  await Promise.all(pending);
  return { placer, root, bodies, joints };
}

const _q = new THREE.Quaternion();
export function setJoint(rig, name, angle) {
  const j = rig.joints.get(name);
  if (!j) return;
  let a = angle;
  if (j.range) a = Math.min(j.range[1], Math.max(j.range[0], a));
  j.body.quaternion.copy(j.baseQuat).multiply(_q.setFromAxisAngle(j.axis, a));
}

// Cores do Microduck "classic" (variants.js do simulador oficial).
const AMBER_YELLOW = { color: [1.0, 0.413, 0.007], roughness: 0.4, metalness: 0 };
const BRIGHT_ORANGE = { color: [1.0, 0.144, 0.008], roughness: 0.45, metalness: 0 };
const CLEAN_YELLOW = { color: [1.0, 0.608, 0.021], roughness: 0.4, metalness: 0 };
const CREAM = { color: [0.888, 0.86, 0.798], roughness: 0.35, metalness: 0 };
const DARK = { color: [0.012, 0.012, 0.014], roughness: 0.55, metalness: 0.3 };
const GRAY = { color: [0.256, 0.256, 0.279], roughness: 0.5, metalness: 0.35 };
const LENS = { color: [0.01, 0.012, 0.02], roughness: 0.05, metalness: 0 };
const WARM_GRAY = { color: [0.328, 0.312, 0.283], roughness: 0.35, metalness: 0 };

const MESH_MATERIALS = {
  "top_head_shell.stl": CREAM,
  "bottom_head_shell.stl": BRIGHT_ORANGE,
  "face_part.stl": WARM_GRAY,
  "noenoeil.stl": AMBER_YELLOW,
  "lens.stl": LENS,
  "m12_lens_holder.stl": DARK,
  "soft_mouth_top.stl": AMBER_YELLOW,
  "jaw.stl": BRIGHT_ORANGE,
  "jaw_soft.stl": AMBER_YELLOW,
  "trunk_base.stl": CREAM,
  "left_shell.stl": CREAM,
  "right_shell.stl": CREAM,
  "upper_leg_left.stl": CREAM,
  "upper_leg_right.stl": CREAM,
  "hip_l.stl": GRAY,
  "foot_left.stl": BRIGHT_ORANGE,
  "foot_right.stl": BRIGHT_ORANGE,
  "ankle_left.stl": BRIGHT_ORANGE,
  "ankle_right.stl": BRIGHT_ORANGE,
  "sole_left.stl": CLEAN_YELLOW,
  "sole_right.stl": CLEAN_YELLOW,
  "xl330.stl": DARK,
  "leg.stl": GRAY,
  "seeed_bearing__configuration_default.stl": DARK,
  "yaw2roll.stl": DARK,
  "bearing_roll.stl": DARK,
  "neck.stl": GRAY,
  "np_f970.stl": DARK,
  "pcb__raspberry_pi_zero_2_w.stl": DARK,
  "elec_rpi_robot_hat_pcb.stl": DARK,
  "banana_pcb_locker.stl": DARK,
  "speaker.stl": DARK,
  // Variante com patins: lâmina e suporte no lugar do sapato, aros no tom
  // da sola, pneus escuros.
  "ankle_l_v1.stl": BRIGHT_ORANGE,
  "ankle_r_v1.stl": BRIGHT_ORANGE,
  "roller_blade.stl": BRIGHT_ORANGE,
  "rim.stl": CLEAN_YELLOW,
  "tire.stl": DARK,
};
export const classicMaterial = (mesh) => MESH_MATERIALS[mesh] ?? GRAY;
