// Núcleo do protótipo "Faça o pato marcar um gol".
//
// Física e controle portados de pollen-robotics/microduck-simulator
// (app/src/game/game.js, Apache-2.0): MuJoCo WASM roda o MJCF em que as
// políticas foram treinadas, onnxruntime-web roda as políticas ONNX a 50 Hz
// (timestep 0,005 s, decimação 4). Observação de 61 dimensões:
//   [ang_vel(3), gravidade projetada(3), juntas(14), vel. juntas(14),
//    última ação(14), comando(13)]
//
// O que é nosso: a arena com gol, os três modos (controle, solto, visão),
// a câmera da cabeça renderizada num canvas próprio, o filtro de visão
// (pixels -> bordas Sobel -> máscara de cor) e o controlador que anda até
// a bola usando só o que a câmera vê.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import {
  loadKinematics, loadGlbGeometries, geometryToBinaryStl, buildRig, setJoint,
  classicMaterial, MODEL_DIR,
} from "./duck.js";
import { ParticleFilter, rayBoxes } from "./localize.js";
import { Grade, aEstrela } from "./planejar.js";
import { criarLaserChao } from "./laser-chao.js";
import { criarGravador } from "./malha-aberta.js";
import { Marcas, TINTA, SOL } from "./marcas.js";
import { chinelo as criarChinelo } from "../cena/campo.js";
import { criarCena, configurarRenderer, Percepcao } from "../cena/cena.js";
import {
  geomsDaSala, colisaoXml, mapaDoLaser, GOL, INICIO, BOLA_RAIO, CAMPO_MJ, PISO,
} from "./sala-fisica.js";

// ── Constantes do simulador oficial (constants.js) ─────────────────────
const JOINT_NAMES = [
  "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
  "neck_pitch", "head_pitch", "head_yaw", "head_roll",
  "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
];
const DEFAULT_POSE = new Float32Array([
  0, -0.08726646259971647, -0.457924, -0.004940, 0.452984,
  0.3490658503988659, 0.3490658503988659, 0, 0,
  0, 0.08726646259971647, 0.457924, 0.004940, -0.452984,
]);
const NUM_JOINTS = 14;
const OBS_SIZE = 61;
const CMD_SIZE = 13;
const TIMESTEP = 0.005;
const DECIMATION = 4;
const CTRL_DT = TIMESTEP * DECIMATION;
const VEL_FWD = 0.25, VEL_BACK = -0.2, VEL_ANG = 1.0;
// Giro no teclado acima do limite do simulador oficial (1,0 rad/s): a
// política gira parada a só ~0,1 rad/s com 1,0. Medido no navegador:
// comando 2,5 parado dá ~1,5 rad/s e 1,8 andando, sem quedas. Fora da faixa
// de treino, mas estável nos testes.
const TURN_INPLACE = 2.5, TURN_WALKING = 1.8;
const KICK_STEPS = 25; // chute cego de 0,5 s
// Depois do chute o pato fica 1 s parado se reequilibrando (o oficial usa
// 0,4 s; com comandos logo em seguida ele caía, principalmente após o
// chute direito).
const POST_KICK_LOCK_STEPS = 50;

const POLICIES = {
  walk: "/pato/policies/BEST_alpha_walking.onnx",
  kickL: "/pato/policies/ball_kick_left.onnx",
  kickR: "/pato/policies/ball_kick_right.onnx",
  stand: "/pato/policies/BEST_alpha_stand.onnx",
};

// ── Sala, campo e gol ────────────────────────────────────────────────
// A cena é a sala de casa de lib/cena (thread de design); a física dela
// (paredes, móveis, chinelos do gol) está em sala-fisica.js. Coordenadas
// MJCF: x igual ao da sala, y = -z da sala, z pra cima.
const BALL_RADIUS = BOLA_RAIO; // bola de tênis, 3,35 cm
const SPAWN = [INICIO.pato.x, INICIO.pato.y];
const SPAWN_YAW = INICIO.pato.yaw;
// Bola um pouco além do alcance do pé: o primeiro chute pega no ar.
const BALL_START = [INICIO.bola.x, INICIO.bola.y];

// Obstáculo do modo localizar ("um livro" no chão entre a bola e o gol):
// baixo demais pro laser, fora do mapa. Fica estacionado longe nos outros
// modos.
const BOOK = { pos: [GOL.linhaX - 0.32, GOL.centroY], half: [0.08, 0.12, 0.018] };
const PARK = [50, 50];
// Decidir: três objetos que o visitante arrasta (meias medidas em MJCF).
const OBSTACULOS = [
  { nome: "livro", half: [0.08, 0.11, 0.02] },
  { nome: "chinelo", half: [0.13, 0.045, 0.02] },
  { nome: "caixa", half: [0.14, 0.09, 0.06] },
];

// Laser (LiDAR 2D) saindo do sensor "tof" da cabeça do pato.
const LIDAR_RAYS = 90;
const LIDAR_MAX = 3;
const LIDAR_EVERY = 5; // passos de controle entre varreduras (10 Hz)


// Visão da cabeça: resolução baixa de propósito (é o que a gente mostra).
export const VISION_W = 240, VISION_H = 160;

function yawOf(qpos) {
  return Math.atan2(
    2 * (qpos[3] * qpos[6] + qpos[4] * qpos[5]),
    1 - 2 * (qpos[5] * qpos[5] + qpos[6] * qpos[6]),
  );
}
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const wrapA = (a) => Math.atan2(Math.sin(a), Math.cos(a));
function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
// Fora do campo pintado por mais que `m` metros?
const foraDoCampo = (x, y, m = 0) =>
  x < CAMPO_MJ.xmin - m || x > CAMPO_MJ.xmax + m || y < CAMPO_MJ.ymin - m || y > CAMPO_MJ.ymax + m;

// ── Detecção de bola na CPU: mesmo critério da máscara do shader ───────
// A câmera da cabeça renderiza a sala em HDR linear (meio float) e passa
// pelo mesmo tom da Percepcao (ACES com exposição 1,1, sem sRGB), pra que o
// pato veja a sala como o visitante vê. A máscara olha HSV nesse espaço.
const EXPOSICAO = 1.1;
const aces = (x) => {
  x *= EXPOSICAO;
  return Math.min(1, Math.max(0, (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14)));
};
function isBallPixel(r, g, b) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  const v = max, d = max - min;
  if (v < 0.3) return false;
  const s = max > 0 ? d / max : 0;
  // Saturação quase 1 é o aro âmbar do olho do pato (material puro); a
  // bola de tênis fica entre 0,76 e 0,92.
  if (s < 0.6 || s > 0.97) return false;
  let h;
  if (max === r) h = ((g - b) / d) % 6;
  else if (max === g) h = (b - r) / d + 2;
  else h = (r - g) / d + 4;
  h *= 60;
  if (h < 0) h += 360;
  // Bola de tênis: amarelo-esverdeado (matiz 55–64 medido na sala, com o
  // sol da tarde). O piso fica em ~33, o chinelo em ~210, a planta em ~110.
  return h > 54 && h < 78;
}

const VISION_FRAG = /* glsl */ `
  uniform sampler2D tImg;
  uniform vec2 texel;
  uniform float layer;
  uniform vec4 box; // x0, y0, x1, y1 em uv
  uniform float hasBox;
  varying vec2 vUv;
  vec3 aces(vec3 x) {
    x *= ${EXPOSICAO.toFixed(2)};
    return clamp((x * (2.51 * x + .03)) / (x * (2.43 * x + .59) + .14), 0., 1.);
  }
  vec3 px(vec2 o) { return aces(texture2D(tImg, vUv + o * texel).rgb); }
  float lum(vec3 c) { return dot(c, vec3(0.299, 0.587, 0.114)); }
  float isBall(vec3 c) {
    float mx = max(c.r, max(c.g, c.b));
    float mn = min(c.r, min(c.g, c.b));
    float d = mx - mn;
    if (mx < 0.3) return 0.0;
    float sat = d / max(mx, 1e-5);
    if (sat < 0.6 || sat > 0.97) return 0.0;
    float h;
    if (mx == c.r) h = mod((c.g - c.b) / d, 6.0);
    else if (mx == c.g) h = (c.b - c.r) / d + 2.0;
    else h = (c.r - c.g) / d + 4.0;
    h *= 60.0;
    if (h < 0.0) h += 360.0;
    return (h > 54.0 && h < 78.0) ? 1.0 : 0.0;
  }
  // Visual da thread de design: pixels (grade 48x32 amostrada), bordas
  // (tinta sobre papel) e bola (anel laranja em volta do que a máscara achou).
  vec3 pxq(vec2 uv) { return aces(texture2D(tImg, uv).rgb); }
  void main() {
    vec3 c = px(vec2(0.0));
    vec2 grade = vec2(48.0, 32.0);
    vec3 pix = pxq((floor(vUv * grade) + 0.5) / grade);
    // Sobel 3x3 sobre a luminância
    float tl = lum(px(vec2(-1.0, 1.0))), t = lum(px(vec2(0.0, 1.0))), tr = lum(px(vec2(1.0, 1.0)));
    float l = lum(px(vec2(-1.0, 0.0))), r = lum(px(vec2(1.0, 0.0)));
    float bl = lum(px(vec2(-1.0, -1.0))), b = lum(px(vec2(0.0, -1.0))), br = lum(px(vec2(1.0, -1.0)));
    float gx = (tr + 2.0 * r + br) - (tl + 2.0 * l + bl);
    float gy = (tl + 2.0 * t + tr) - (bl + 2.0 * b + br);
    float e = smoothstep(0.12, 0.45, length(vec2(gx, gy)));
    vec3 papel = vec3(0.969, 0.965, 0.949), tinta = vec3(0.137, 0.133, 0.122), sol = vec3(0.85, 0.41, 0.17);
    vec3 bordas = mix(papel, tinta, e);
    vec3 col = layer < 0.5 ? pix : layer < 1.5 ? bordas : c;
    if (hasBox > 0.5 && layer > 1.5) {
      // Anel em volta da bola achada (box.xy = centro, box.z = raio, em uv
      // com o eixo x corrigido pela proporção da imagem).
      vec2 d = (vUv - box.xy) * vec2(texel.y / texel.x, 1.0);
      float rr = length(d);
      float w = 1.5 * texel.y;
      float anel = smoothstep(box.z - w, box.z, rr) * (1.0 - smoothstep(box.z + w, box.z + 2.0 * w, rr));
      col = mix(col, sol, anel);
    }
    gl_FragColor = vec4(col, 1.0);
  }
`;

// Corpos: pernas (o pato de sempre) ou patins (mesmo pato com rodas passivas
// e a política de patinar do simulador oficial). Limites de velocidade do
// patins como no runtime: 0,6 / -0,5 m/s e giro de no máximo 0,3 rad/s.
const VARIANTES = {
  pernas: { xml: "robot_allcollisions.xml", kin: "kinematics.json" },
  patins: { xml: "robot_allcollisions_rollers.xml", kin: "kinematics_rollers.json", politica: "/pato/policies/BEST_roller.onnx" },
};

export async function createPato({ canvas, visionCanvas, onProgress, onGoal, variante = "pernas" }) {
  const V = VARIANTES[variante];
  const patins = variante === "patins";
  let disposed = false;
  const cleanups = [];
  const progress = (msg) => !disposed && onProgress?.(msg);

  // ── Runtimes ─────────────────────────────────────────────────────────
  progress("carregando física e redes");
  const [{ default: loadMujoco }, ort] = await Promise.all([
    import("@mujoco/mujoco"),
    import("onnxruntime-web/wasm"),
  ]);
  ort.env.wasm.wasmPaths = { wasm: "/pato/wasm/ort-wasm-simd-threaded.wasm" };
  ort.env.wasm.numThreads = 1;

  async function buildPhysicsXml() {
    const src = await (await fetch(`${MODEL_DIR}/${V.xml}`)).text();
    const doc = new DOMParser().parseFromString(src, "text/xml");
    for (const g of [...doc.querySelectorAll('geom[class="visual"]')]) g.remove();
    const used = new Set([...doc.querySelectorAll("geom[mesh]")].map((g) => g.getAttribute("mesh")));
    for (const m of [...doc.querySelectorAll("asset > mesh")]) {
      const name = m.getAttribute("name") ?? m.getAttribute("file").replace(/\.stl$/i, "");
      if (!used.has(name)) m.remove();
    }
    const root = doc.documentElement;
    const world = doc.querySelector("worldbody");
    const el = (tag, attrs) => {
      const e = doc.createElement(tag);
      for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
      return e;
    };
    root.appendChild(el("option", { timestep: String(TIMESTEP) }));
    world.appendChild(el("geom", { name: "floor", type: "plane", size: "0 0 0.05", pos: "0 0 0" }));
    // Sala: caixas de colisão do design (paredes, rack, chinelos, sofá, puff,
    // vaso) + objetos baixos e limites nossos.
    for (const g of colisaoXml()) world.appendChild(doc.importNode(g, true));
    for (const g of geomsDaSala()) {
      const attrs = {
        name: g.name, type: g.type, pos: g.pos.join(" "), size: g.size.join(" "), group: String(g.group),
      };
      if (g.yaw) attrs.euler = `0 0 ${g.yaw}`;
      world.appendChild(el("geom", attrs));
    }
    // Obstáculos do decidir: geoms estáticos (grupo 2, baixos pro laser
    // horizontal), estacionados longe nos outros modos.
    OBSTACULOS.forEach((o, i) => world.appendChild(el("geom", {
      name: `obst_${o.nome}`, type: "box", pos: `${PARK[0] + i} ${PARK[1]} ${o.half[2]}`,
      size: o.half.join(" "), group: "2",
    })));
    // Livro: geom estático, estacionado longe até o modo localizar.
    world.appendChild(el("geom", {
      name: "book", type: "box", pos: `${PARK[0]} ${PARK[1]} ${BOOK.half[2]}`,
      size: BOOK.half.join(" "), group: "2",
    }));
    // Bola leve (sensação de bola de praia), igual ao simulador oficial.
    const ball = el("body", { name: "ball", pos: `${BALL_START[0]} ${BALL_START[1]} ${BALL_RADIUS}` });
    ball.appendChild(el("freejoint", { name: "ball_freejoint" }));
    ball.appendChild(el("geom", {
      // Bola de tênis: 57 g (a de praia do simulador oficial tinha 30 g).
      name: "ball_geom", type: "sphere", size: String(BALL_RADIUS),
      mass: "0.057", friction: "0.4 0.01 0.003", solref: "0.03 0.4", condim: "6",
      // grupo 1: o laser (que só olha o grupo 0) não enxerga a bola.
      group: "1",
    }));
    world.appendChild(ball);
    // Keyframe STAND: tronco no spawn, 14 juntas no DEFAULT_POSE, bola.
    const poseByName = new Map(JOINT_NAMES.map((n, i) => [n, DEFAULT_POSE[i]]));
    const qj = [...doc.querySelectorAll("body > joint")]
      .map((j) => poseByName.get(j.getAttribute("name")) ?? 0).join(" ");
    const kf = doc.createElement("keyframe");
    kf.appendChild(el("key", {
      name: "STAND",
      qpos: `${SPAWN[0]} ${SPAWN[1]} 0.12 ${Math.cos(SPAWN_YAW / 2)} 0 0 ${Math.sin(SPAWN_YAW / 2)} ${qj} ${BALL_START[0]} ${BALL_START[1]} ${BALL_RADIUS} 1 0 0 0`,
      ctrl: Array.from(DEFAULT_POSE).join(" "),
    }));
    root.appendChild(kf);
    const meshFiles = [...doc.querySelectorAll("asset > mesh")].map((m) => m.getAttribute("file"));
    return { xml: new XMLSerializer().serializeToString(doc), meshFiles };
  }

  const [mujoco, { xml, meshFiles }, kin] = await Promise.all([
    loadMujoco({ locateFile: (p) => (p.endsWith(".wasm") ? "/pato/wasm/mujoco.wasm" : p) }),
    buildPhysicsXml(),
    loadKinematics(V.kin),
  ]);
  if (disposed) return null;

  progress("montando o pato");
  const vfs = new mujoco.MjVFS();
  const geoms = await loadGlbGeometries();
  await Promise.all(meshFiles.map(async (f) => {
    const entry = geoms.get(f);
    const buf = entry
      ? geometryToBinaryStl(entry.welded)
      : await (await fetch(`${MODEL_DIR}/meshes/${f}`)).arrayBuffer();
    vfs.addBuffer(`assets/${f}`, new Uint8Array(buf));
  }));

  const rigPromise = buildRig(kin, classicMaterial);
  progress("carregando as políticas");
  const opts = { executionProviders: ["wasm"] };
  const sessions = {};
  [sessions.walk, sessions.kickL, sessions.kickR, sessions.stand] = await Promise.all([
    ort.InferenceSession.create(patins ? V.politica : POLICIES.walk, opts),
    ort.InferenceSession.create(POLICIES.kickL, opts),
    ort.InferenceSession.create(POLICIES.kickR, opts),
    ort.InferenceSession.create(POLICIES.stand, opts),
  ]);
  if (disposed) return null;

  const model = mujoco.MjModel.from_xml_string(xml, vfs);
  const data = new mujoco.MjData(model);
  const qposAdr = JOINT_NAMES.map((n) => model.jnt(n).qposadr);
  const dofAdr = JOINT_NAMES.map((n) => model.jnt(n).dofadr);
  const gyroAdr = model.sensor("imu_ang_vel").adr;
  const trunkId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "trunk_base");
  const ballBodyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "ball");
  const standKeyId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY.value, "STAND");
  const headCamId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA.value, "head_camera");
  const ankleIds = {
    left: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "ankle_left"),
    right: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY.value, "ankle_right"),
  };
  const ballQ = model.jnt("ball_freejoint").qposadr;
  const ballV = model.jnt("ball_freejoint").dofadr;
  const tofId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE.value, "tof");
  const bookId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "book");
  const floorId = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, "floor");
  const obstIds = OBSTACULOS.map((o) => mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM.value, `obst_${o.nome}`));
  const obst = OBSTACULOS.map((o, i) => ({ ...o, x: PARK[0] + i, y: PARK[1], yaw: 0, mesh: null }));

  // Mapa conhecido pro filtro de partículas: os mesmos geoms da física que
  // ficam na altura do laser (paredes, rack, sofá, puff, vaso).
  const MAP_BOXES = mapaDoLaser();
  const pf = new ParticleFilter({
    boxes: MAP_BOXES,
    bounds: PISO,
    n: 1500,
    maxRange: LIDAR_MAX,
  });

  // ── Estado de controle ───────────────────────────────────────────────
  const lastAction = new Float32Array(NUM_JOINTS);
  const obs = new Float32Array(OBS_SIZE);
  const cmd = new Float32Array(CMD_SIZE);
  const twist = new Float32Array(3); // comando vindo do modo atual
  const headTarget = new Float32Array(4);
  const headSmooth = new Float32Array(4);
  let policy = "walk"; // walk | kickL | kickR
  let kickRun = null;
  let postKickLock = 0;
  let recovery = null; // { state: "fallen" | "recovering", steps, upright }
  let fallDebounce = 0;
  let fallenSince = null;
  // controle | solto | visao | gancho | odometria | laser | jogar
  let mode = "controle";
  let goalCooldown = 0;
  let sceneResetAt = null;
  const GOAL_RESET_S = 4;
  let goals = 0;
  let ballStuckFor = 0;
  let ballRespawnAt = null;

  const _q = new THREE.Quaternion();
  const _g = new THREE.Vector3();
  function projGravZ() {
    const xq = data.body(trunkId).xquat;
    _q.set(xq[1], xq[2], xq[3], xq[0]).conjugate();
    return _g.set(0, 0, -1).applyQuaternion(_q).z;
  }

  function placeBall(x, y) {
    const qpos = data.qpos, qvel = data.qvel;
    qpos[ballQ] = x; qpos[ballQ + 1] = y; qpos[ballQ + 2] = BALL_RADIUS + 0.005;
    qpos[ballQ + 3] = 1; qpos[ballQ + 4] = 0; qpos[ballQ + 5] = 0; qpos[ballQ + 6] = 0;
    for (let i = 0; i < 6; i++) qvel[ballV + i] = 0;
    mujoco.mj_forward(model, data);
  }

  function randomBallSpot() {
    // No jogar, a bola volta pra trás do livro.
    if (mode === "jogar") return [BOOK.pos[0] - 0.35 - Math.random() * 0.25, GOL.centroY - 0.15 + Math.random() * 0.3];
    // No campo, longe do pato e fora do gol.
    const qpos = data.qpos, c = CAMPO_MJ;
    for (let i = 0; i < 20; i++) {
      const x = c.xmin + 0.15 + Math.random() * (c.xmax - c.xmin - 0.45);
      const y = c.ymin + 0.12 + Math.random() * (c.ymax - c.ymin - 0.24);
      if (Math.hypot(x - qpos[0], y - qpos[1]) > 0.3) return [x, y];
    }
    return [BALL_START[0], BALL_START[1]];
  }

  function resetSim() {
    endGrab();
    kickRun = null; postKickLock = 0; recovery = null; windup = null;
    sceneResetAt = null; ballRespawnAt = null; ballStuckFor = 0;
    fallDebounce = 0; fallenSince = null; policy = "walk";
    headTarget.fill(0); headSmooth.fill(0);
    mujoco.mj_resetDataKeyframe(model, data, standKeyId);
    mujoco.mj_forward(model, data);
    lastAction.fill(0);
  }

  function triggerKick(foot) {
    if (policy !== "walk" || recovery || postKickLock > 0) return false;
    policy = foot === "left" ? "kickL" : "kickR";
    const v = data.qvel;
    kickRun = { steps: 0, boosted: false, prevV: Math.hypot(v[ballV], v[ballV + 1]), inReach: ballInReach() };
    return true;
  }

  // Ajuda no chute (não existe no simulador oficial, é nossa): parado, o
  // pé da política só alcança ~11 cm e precisa pegar a bola de lado, então
  // a maioria dos chutes "de perto" errava ou mal empurrava. Duas regras:
  //  - contato real (salto de velocidade da bola durante o chute): garante
  //    uma velocidade mínima na direção em que ela já saiu;
  //  - sem contato, mas a bola estava ao alcance quando o chute começou:
  //    no passo em que o pé está mais à frente (8 de 25, medido), a bola
  //    recebe o mesmo impulso, como se o pé tivesse pegado.
  const KICK_MIN_SPEED = 1.4;
  const KICK_PEAK_STEP = 8;
  function boostOnContact() {
    const v = data.qvel;
    const sp = Math.hypot(v[ballV], v[ballV + 1]);
    const jumped = sp - kickRun.prevV > 0.15;
    kickRun.prevV = sp;
    if (kickRun.boosted || grab) return;
    const assist = !jumped && kickRun.inReach && kickRun.steps === KICK_PEAK_STEP && ballInReach();
    if (!jumped && !assist) return;
    kickRun.boosted = true;
    if (sp >= KICK_MIN_SPEED) return;
    // Direção: metade a saída real da bola (ou do tronco pra bola), metade
    // a frente do pato.
    const q = data.qpos;
    const yaw = yawOf(q);
    let ox = v[ballV], oy = v[ballV + 1];
    if (assist || sp < 1e-3) { ox = q[ballQ] - q[0]; oy = q[ballQ + 1] - q[1]; }
    const on = Math.hypot(ox, oy) || 1;
    let dx = ox / on + Math.cos(yaw), dy = oy / on + Math.sin(yaw);
    const n = Math.hypot(dx, dy) || 1;
    v[ballV] = (dx / n) * KICK_MIN_SPEED;
    v[ballV + 1] = (dy / n) * KICK_MIN_SPEED;
  }

  // Bola no referencial do pato: x pra frente, y pra esquerda.
  function ballLocal() {
    const q = data.qpos;
    const yaw = yawOf(q);
    const dx = q[ballQ] - q[0], dy = q[ballQ + 1] - q[1];
    return [Math.cos(yaw) * dx + Math.sin(yaw) * dy, -Math.sin(yaw) * dx + Math.cos(yaw) * dy];
  }
  // Ao alcance do pé (medido parado: o chute só pega até ~12 cm à frente).
  function ballInReach() {
    const [x, y] = ballLocal();
    return x > 0.03 && x < 0.16 && Math.abs(y) < 0.12;
  }

  // Chutar andando derruba o pato (parece um pulo, e ele se levanta). No
  // controle e no modo solto isso fica: antes de "aprender", ele cai. No
  // modo visão o chute passa por aqui: o pato zera o comando, espera ficar
  // parado e firme, e só então dispara a política de chute.
  // windup: { since, still, foot }
  let windup = null;
  function requestKick(foot) {
    if (windup || policy !== "walk" || recovery || postKickLock > 0) return false;
    windup = { since: simTime, still: 0, foot };
    return true;
  }
  function driveWindup() {
    if (!windup) return;
    const v = data.qvel;
    const still = Math.hypot(v[0], v[1]) < 0.05 && Math.abs(v[5]) < 0.4;
    windup.still = still ? windup.still + CTRL_DT : 0;
    const waited = simTime - windup.since;
    if ((windup.still >= 0.25 && waited >= 0.4) || waited > 1.2) {
      const foot = windup.foot;
      windup = null;
      triggerKick(foot);
    }
  }

  function buildObs() {
    const qpos = data.qpos, qvel = data.qvel, sens = data.sensordata;
    let i = 0;
    for (let a = 0; a < 3; a++) obs[i++] = sens[gyroAdr + a];
    const xq = data.body(trunkId).xquat;
    _q.set(xq[1], xq[2], xq[3], xq[0]).conjugate();
    _g.set(0, 0, -1).applyQuaternion(_q);
    obs[i++] = _g.x; obs[i++] = _g.y; obs[i++] = _g.z;
    for (let j = 0; j < NUM_JOINTS; j++) obs[i++] = qpos[qposAdr[j]] - DEFAULT_POSE[j];
    for (let j = 0; j < NUM_JOINTS; j++) obs[i++] = qvel[dofAdr[j]];
    for (let j = 0; j < NUM_JOINTS; j++) obs[i++] = lastAction[j];
    cmd.fill(0);
    const locked = policy !== "walk" || postKickLock > 0 || recovery || windup;
    if (!locked) { cmd[0] = twist[0]; cmd[1] = 0; cmd[2] = twist[2]; }
    // Cabeça: EMA a 50 Hz rumo ao alvo, como no runtime do robô.
    for (let h = 0; h < 4; h++) headSmooth[h] += 0.2 * (headTarget[h] - headSmooth[h]);
    if (!recovery) for (let h = 0; h < 4; h++) cmd[3 + h] = headSmooth[h];
    for (let c = 0; c < CMD_SIZE; c++) obs[i++] = cmd[c];
    return obs;
  }

  const activeSession = () => {
    if (recovery?.state === "recovering") return sessions.stand;
    // Aprender com o treino de verdade: cada geração é um checkpoint.
    if (mode === "aprender" && policy === "walk" && aprender.sessoes) return aprender.sessoes[aprender.geracao];
    return sessions[policy];
  };

  // ── Arrastar a bola (força mola-amortecedor, como o viewer do MuJoCo) ─
  const GRAB_K = 100, GRAB_D = Math.sqrt(GRAB_K), GRAB_MAX = 200;
  let grab = null; // { target: [x,y,z] }
  function applyGrab() {
    if (!grab) return;
    const q = data.qpos, v = data.qvel;
    let fx = GRAB_K * (grab.target[0] - q[ballQ]) - GRAB_D * v[ballV];
    let fy = GRAB_K * (grab.target[1] - q[ballQ + 1]) - GRAB_D * v[ballV + 1];
    let fz = GRAB_K * (grab.target[2] - q[ballQ + 2]) - GRAB_D * v[ballV + 2];
    const n = Math.hypot(fx, fy, fz);
    if (n > GRAB_MAX) { const s = GRAB_MAX / n; fx *= s; fy *= s; fz *= s; }
    const x = data.xfrc_applied, a = ballBodyId * 6;
    x[a] = 0.03 * fx; x[a + 1] = 0.03 * fy; x[a + 2] = 0.03 * fz + 0.03 * 9.81;
  }
  function endGrab() {
    if (!grab) return;
    const x = data.xfrc_applied, a = ballBodyId * 6;
    x[a] = 0; x[a + 1] = 0; x[a + 2] = 0;
    grab = null;
  }

  // ── Comportamentos por modo ──────────────────────────────────────────
  const keys = new Set();
  // Joystick de toque (celular): frente/trás e giro em -1..1.
  const toque = { x: 0, y: 0 };
  function keyboardTwist() {
    let vx = 0, wz = 0;
    if (keys.has("fwd")) vx += patins ? 0.6 : VEL_FWD;
    if (keys.has("back")) vx += patins ? -0.5 : VEL_BACK;
    if (Math.abs(toque.y) > 0.25) vx = toque.y > 0 ? (patins ? 0.6 : VEL_FWD) * toque.y : (patins ? 0.5 : -VEL_BACK) * toque.y;
    const turn = patins ? 0.3 : vx === 0 ? TURN_INPLACE : TURN_WALKING;
    if (keys.has("left")) wz += turn;
    if (keys.has("right")) wz -= turn;
    if (Math.abs(toque.x) > 0.25) wz = -turn * toque.x;
    twist[0] = vx; twist[2] = wz;
  }

  // Solto: comandos aleatórios e chutes no vazio.
  const wander = { until: 0, vx: 0, wz: 0, nextKick: 0 };
  function wanderTwist(t) {
    if (t > wander.until) {
      wander.until = t + 1 + Math.random() * 2;
      // Comandos fortes o bastante pra sair da zona morta da política.
      wander.vx = Math.random() < 0.3 ? 0 : 0.2 + Math.random() * 0.05;
      wander.wz = (Math.random() < 0.5 ? -1 : 1) * (wander.vx === 0 ? TURN_INPLACE : 0.3 + Math.random() * 1.2);
    }
    // Perto da parede, vira (o pato não vê nada, mas a gente não quer
    // que ele passe o tempo todo empacado num canto).
    const q = data.qpos;
    if (foraDoCampo(q[0], q[1], 0.1)) wander.wz = Math.sign(wander.wz || 1) * VEL_ANG;
    twist[0] = wander.vx; twist[2] = wander.wz;
    if (t > wander.nextKick) {
      wander.nextKick = t + 2.5 + Math.random() * 3;
      triggerKick(Math.random() < 0.5 ? "left" : "right");
    }
  }

  // Visão: controlador simples sobre a detecção da câmera da cabeça.
  // Sem rede neural: máscara de cor -> centróide e caixa -> girar e andar.
  const det = { seen: false, cx: 0.5, cy: 0.5, x0: 0, y0: 0, x1: 0, y1: 0, n: 0, at: -1e9 };
  const lastSeen = { cx: 0.5, top: 0, at: -1e9 };
  const seek = { dir: 1, kickedAt: -1e9 };
  // A política de andar tem uma "zona morta": parada, comandos fracos não
  // tiram o pato do lugar. O controlador usa comandos fortes (girar sempre
  // andando um pouco) e dá um empurrão quando percebe que empacou.
  const SEEK_VX = 0.15;
  let hold = false; // depuração: congela o controlador
  const stall = { since: null, boostUntil: 0 };
  function visionTwist(t) {
    twist[0] = 0; twist[2] = 0;
    if (hold || windup || policy !== "walk" || postKickLock > 0 || t - seek.kickedAt < 1.2) {
      stall.since = null;
      return;
    }
    const fresh = det.seen && t - det.at < 0.25;
    if (fresh) {
      const ex = det.cx - 0.5; // + = bola à direita na imagem
      seek.dir = ex > 0 ? -1 : 1;
      // Profundidade pela altura da caixa na imagem: com a cabeça inclinada,
      // quanto mais baixo o topo da bola, mais perto ela está (calibrado com
      // a bola de tênis: topo em 0,86 ~ 17 cm do tronco).
      if (det.y0 > 0.86 && Math.abs(ex) < 0.14) {
        // Para, se firma e chuta (o pé sai do lado da bola na imagem).
        if (requestKick(ex < 0 ? "left" : "right")) seek.kickedAt = t;
        return;
      }
      if (Math.abs(ex) > 0.15) {
        // Bola de lado: gira no lugar até ela ficar no meio da imagem.
        twist[0] = 0; twist[2] = TURN_INPLACE * seek.dir;
      } else {
        twist[0] = det.y0 > 0.5 ? 0.2 : VEL_FWD;
        twist[2] = clamp(-2.6 * ex, -VEL_ANG, VEL_ANG);
      }
    } else if (t - lastSeen.at < 0.6 && lastSeen.top > 0.84 && Math.abs(lastSeen.cx - 0.5) < 0.2) {
      // Sumiu por baixo da imagem: está nos pés. Chuta.
      if (requestKick(lastSeen.cx < 0.5 ? "left" : "right")) seek.kickedAt = t;
      return;
    } else {
      // Procura: gira no lugar pro lado em que a bola foi vista por último.
      // Saiu do campo (num canto, contra um móvel)? Volta pro meio dele.
      const q = data.qpos;
      if (foraDoCampo(q[0], q[1], 0.05)) {
        const cx = (CAMPO_MJ.xmin + CAMPO_MJ.xmax) / 2, cy = (CAMPO_MJ.ymin + CAMPO_MJ.ymax) / 2;
        const e = wrapA(Math.atan2(cy - q[1], cx - q[0]) - yawOf(q));
        if (Math.abs(e) > 0.5) { twist[0] = 0; twist[2] = Math.sign(e) * TURN_INPLACE; }
        else { twist[0] = VEL_FWD; twist[2] = clamp(2 * e, -VEL_ANG, VEL_ANG); }
      } else {
        twist[0] = 0;
        twist[2] = TURN_INPLACE * seek.dir;
      }
    }
    stallCheck(t);
  }

  // Empacou? Mesmo comando, pato parado por meio segundo: empurrão (sobe o
  // comando de avanço por meio segundo; não é força).
  function stallCheck(t) {
    const v = data.qvel;
    const still = Math.hypot(v[0], v[1]) < 0.03 && Math.abs(v[5]) < 0.15;
    if (still && policy === "walk" && !recovery) stall.since ??= t;
    else stall.since = null;
    if (stall.since !== null && t - stall.since > 0.5) { stall.boostUntil = t + 0.5; stall.since = null; }
    if (t < stall.boostUntil) {
      // Empacou girando: giro mais forte. Empacou andando: avanço cheio.
      if (twist[0] === 0 && twist[2] !== 0) twist[2] = Math.sign(twist[2]) * 3;
      else twist[0] = VEL_FWD;
    }
  }

  // ── Localizar ────────────────────────────────────────────────────────
  const usesVision = () => mode === "visao" || mode === "gancho" || mode === "jogar" || mode === "decidir";
  const usesLaser = () => mode === "laser" || mode === "jogar" || mode === "decidir";
  const showsGhost = () => mode === "odometria" || usesLaser();

  // Odometria: integra só a velocidade comandada que a política recebeu.
  // Nunca olha a pose real; o desvio é a diferença entre comando e o que a
  // política de fato executa.
  const odo = { x: 0, y: 0, t: 0 };
  function syncOdoToTruth() {
    const q = data.qpos;
    odo.x = q[0]; odo.y = q[1]; odo.t = yawOf(q);
  }

  // Laser: raios horizontais saindo do sensor "tof" da cabeça, via mj_ray
  // (geoms dos grupos 0 e 4: chão, paredes e móveis; o pato é grupo 3, a
  // bola 1, os objetos baixos 2). Os impactos viram o mapa desenhado no chão.
  let est = null; // estimativa do filtro { x, y, t, spread }
  let pfStats = null;
  const odoAcc = { dx: 0, dth: 0 };
  let lidarCount = 0;
  const scan = {
    ox: 0, oy: 0, oz: 0, n: 0,
    hx: new Float32Array(LIDAR_RAYS), hy: new Float32Array(LIDAR_RAYS), hit: new Uint8Array(LIDAR_RAYS),
  };
  // Mapa acumulado numa grade de 1,5 cm: cada célula tocada pelo laser
  // entra uma vez (vira a base da grade de ocupação).
  const MAP_MAX = 12000;
  const MAP_CELL = 0.015;
  const mapPts = new Float32Array(MAP_MAX * 2);
  const mapCells = new Set();
  let mapN = 0;
  // Grupos 0 (chão) e 4 (paredes e móveis do design).
  const rayGroups = new Uint8Array([1, 0, 0, 0, 1, 0]);
  const rayGeom = new mujoco.IntBuffer(1);
  const rayNormal = new mujoco.DoubleBuffer(3);
  const rayPnt = new Float64Array(3), rayVec = new Float64Array(3);
  function lidarScan() {
    const p = data.site_xpos, m = data.site_xmat, a = tofId * 3, b = tofId * 9;
    const ox = p[a], oy = p[a + 1], oz = p[a + 2];
    // Eixo óptico do sensor (+Z do quadro, como na câmera) projetado no chão.
    const yawS = Math.atan2(m[b + 5], m[b + 2]);
    const q = data.qpos, ty = yawOf(q);
    const c = Math.cos(ty), sn = Math.sin(ty);
    // Onde o laser está no corpo do pato: o robô sabe pela própria
    // cinemática (ângulos das juntas), não é trapaça.
    const sensor = {
      ox: c * (ox - q[0]) + sn * (oy - q[1]),
      oy: -sn * (ox - q[0]) + c * (oy - q[1]),
      oth: wrapA(yawS - ty),
    };
    const beams = [];
    rayPnt[0] = ox; rayPnt[1] = oy; rayPnt[2] = oz;
    for (let k = 0; k < LIDAR_RAYS; k++) {
      const rel = (k / LIDAR_RAYS) * 2 * Math.PI;
      const ang = yawS + rel;
      rayVec[0] = Math.cos(ang); rayVec[1] = Math.sin(ang); rayVec[2] = 0;
      const d = mujoco.mj_ray(model, data, rayPnt, rayVec, rayGroups, true, -1, rayGeom, rayNormal);
      const hit = d >= 0 && d < LIDAR_MAX;
      const r = hit ? d : LIDAR_MAX;
      scan.hit[k] = hit ? 1 : 0;
      scan.hx[k] = ox + rayVec[0] * r;
      scan.hy[k] = oy + rayVec[1] * r;
      if (hit && mapN < MAP_MAX && mode !== "decidir") {
        const key = Math.round(scan.hx[k] / MAP_CELL) * 100000 + Math.round(scan.hy[k] / MAP_CELL);
        if (!mapCells.has(key)) {
          mapCells.add(key);
          // Desenhado 2 cm aquém do impacto, pra não sumir sob a parede.
          mapPts[mapN * 2] = scan.hx[k] - rayVec[0] * 0.02;
          mapPts[mapN * 2 + 1] = scan.hy[k] - rayVec[1] * 0.02;
          mapN++;
        }
      }
      if (k % 2 === 0) beams.push({ a: rel, r });
      if (mode === "decidir" && hit) grade.raio(scan.hx[k], scan.hy[k], true);
    }
    scan.ox = ox; scan.oy = oy; scan.oz = oz; scan.n++;
    return { beams, sensor };
  }
  function stepLocalization(sentVx, sentWz) {
    odo.t = wrapA(odo.t + sentWz * CTRL_DT);
    odo.x += Math.cos(odo.t) * sentVx * CTRL_DT;
    odo.y += Math.sin(odo.t) * sentVx * CTRL_DT;
    if (!usesLaser()) return;
    // Movimento do filtro: avanço comandado (com ruído largo, porque a
    // política cumpre só parte dele) e giro pelo giroscópio da IMU.
    odoAcc.dx += sentVx * CTRL_DT;
    odoAcc.dth += data.sensordata[gyroAdr + 2] * CTRL_DT;
    if (++lidarCount % LIDAR_EVERY !== 0) return;
    // Decidir: o laser inclinado olha o chão a 5 Hz e marca o que é baixo.
    if (mode === "decidir" && lidarCount % (LIDAR_EVERY * 2) === 0) {
      laserChao((hx, hy, bateu) => grade.raio(hx, hy, bateu));
    }
    const { beams, sensor } = lidarScan();
    pf.predict(odoAcc.dx, odoAcc.dth);
    odoAcc.dx = 0; odoAcc.dth = 0;
    pfStats = pf.update(beams, sensor);
    est = pf.estimate();
  }

  // Sequestro: o visitante carrega o pato pelo ar e larga em outro lugar.
  let carry = null; // { target: [x, y] }
  function holdCarry() {
    if (!carry) return;
    const q = data.qpos, v = data.qvel;
    q[0] = carry.target[0]; q[1] = carry.target[1]; q[2] = 0.2;
    for (let i = 0; i < 6; i++) v[i] = 0;
  }

  // Jogar: com a pose estimada, ele mira no gol mesmo sem vê-lo.
  const HFOV = 2 * Math.atan(Math.tan((72 * Math.PI) / 360) * (VISION_W / VISION_H));
  const ballEst = { x: 0, y: 0, at: -1e9 };
  // topo da bola na imagem -> distância ao tronco (m), medido.
  const DIST_TABELA = [[0.26, 0.6], [0.39, 0.45], [0.47, 0.35], [0.6, 0.25], [0.75, 0.2], [0.86, 0.17], [0.97, 0.13]];
  const interp = (t, v) => {
    for (let i = 1; i < t.length; i++) {
      if (v <= t[i][0]) { const [a0, b0] = t[i - 1], [a1, b1] = t[i]; return b0 + ((v - a0) / (a1 - a0)) * (b1 - b0); }
    }
    return t[t.length - 1][1];
  };
  function updateBallEstimate(t) {
    if (!est || !det.seen || t - det.at > 0.25) return;
    // Cabeça em movimento (entre baixa e alta): espera assentar.
    const baixa = Math.abs(headSmooth[1] - HEAD_DOWN) < 0.05;
    if (!baixa && Math.abs(headSmooth[1] - HEAD_UP) > 0.05) return;
    // Distância pelo topo da bola na imagem (tabela medida na sala, cabeça
    // baixa); de cabeça alta ou longe da tabela, pela largura aparente. Rumo
    // pela posição horizontal na imagem.
    let dh;
    if (baixa && det.y0 >= DIST_TABELA[0][0]) dh = interp(DIST_TABELA, det.y0);
    else {
      const w = (det.x1 - det.x0) * HFOV;
      if (w <= 0) return;
      const d3 = BALL_RADIUS / Math.tan(w / 2);
      dh = 0.85 * Math.sqrt(Math.max(0, d3 * d3 - 0.2 * 0.2)) + 0.07;
    }
    const a = est.t - (det.cx - 0.5) * HFOV;
    const bx = est.x + Math.cos(a) * dh, by = est.y + Math.sin(a) * dh;
    const k = t - ballEst.at > 1 ? 1 : 0.3;
    ballEst.x += k * (bx - ballEst.x);
    ballEst.y += k * (by - ballEst.y);
    ballEst.at = t;
  }
  const nav = { stage: "ir", since: 0 };
  function goTo(tx, ty) {
    const e = wrapA(Math.atan2(ty - est.y, tx - est.x) - est.t);
    if (Math.abs(e) > 0.4) { twist[0] = SEEK_VX; twist[2] = Math.sign(e) * VEL_ANG * 1.5; }
    else { twist[0] = VEL_FWD; twist[2] = clamp(2.2 * e, -VEL_ANG, VEL_ANG); }
  }
  function locTwist(t) {
    twist[0] = 0; twist[2] = 0;
    if (hold || windup || policy !== "walk" || postKickLock > 0 || recovery || t - seek.kickedAt < 1.2) {
      stall.since = null;
      return;
    }
    updateBallEstimate(t);
    // Ainda sem saber onde está: age como no modo visão.
    if (!est || est.spread > 0.2) { visionTwist(t); return; }
    if (t - ballEst.at > 5) {
      twist[0] = SEEK_VX; twist[2] = VEL_ANG * seek.dir;
      stallCheck(t);
      return;
    }
    const gx = GOL.linhaX + 0.05;
    let ux = gx - ballEst.x, uy = GOL.centroY - ballEst.y;
    const un = Math.hypot(ux, uy) || 1;
    ux /= un; uy /= un;
    const goalDir = Math.atan2(uy, ux);
    if (nav.stage === "ir") {
      // Ponto atrás da bola, na linha bola-gol.
      const ax = ballEst.x - 0.28 * ux, ay = ballEst.y - 0.28 * uy;
      if (Math.hypot(ax - est.x, ay - est.y) > 0.12) {
        // A bola no caminho? Contorna por um ponto ao lado dela.
        const px = ax - est.x, py = ay - est.y, L = Math.hypot(px, py);
        const bx = ballEst.x - est.x, by = ballEst.y - est.y;
        const along = (bx * px + by * py) / L;
        const across = Math.abs(bx * py - by * px) / L;
        if (along > 0 && along < L && across < 0.2) {
          let nx = -uy, ny = ux;
          if ((est.x - ballEst.x) * nx + (est.y - ballEst.y) * ny < 0) { nx = -nx; ny = -ny; }
          goTo(ballEst.x + nx * 0.3, ballEst.y + ny * 0.3);
        } else goTo(ax, ay);
        stallCheck(t);
        return;
      }
      nav.stage = "alinhar";
    }
    if (nav.stage === "alinhar") {
      const e = wrapA(goalDir - est.t);
      if (Math.abs(e) > 0.2) { twist[0] = 0; twist[2] = Math.sign(e) * TURN_INPLACE; return; }
      nav.stage = "atacar";
      nav.since = t;
    }
    // Atacar: reto rumo ao gol; a visão decide a hora do chute. Não desvia
    // de nada que esteja no caminho.
    const fresh = det.seen && t - det.at < 0.25;
    if (fresh && det.y0 > 0.86 && Math.abs(det.cx - 0.5) < 0.14) {
      if (requestKick(det.cx < 0.5 ? "left" : "right")) { seek.kickedAt = t; nav.stage = "ir"; }
      return;
    }
    const e = wrapA(goalDir - est.t);
    twist[0] = 0.2;
    twist[2] = fresh ? clamp(-2.6 * (det.cx - 0.5), -VEL_ANG, VEL_ANG) : clamp(2 * e, -VEL_ANG, VEL_ANG);
    if (t - nav.since > 6) nav.stage = "ir";
    stallCheck(t);
  }

  // ── Decidir: grade de ocupação + A* ────────────────────────────────────
  // A grade é feita pelo que os lasers tocam (o horizontal vê paredes e
  // móveis; o inclinado vê o que é baixo, como livro e chinelo). O pato
  // planeja dela até um ponto atrás da bola e segue a rota.
  const grade = new Grade(PISO);
  const laserChao = criarLaserChao({ mujoco, model, data, siteId: tofId, floorId });
  const RAIO_PATO = 0.08;
  const plano = { caminho: null, alvo: null, em: -1e9, expandidos: 0, falhou: false };
  function planejar(t) {
    if (!est || ballEst.at < -1e8) { plano.caminho = null; return; }
    plano.em = t;
    const bloq = grade.inflar(RAIO_PATO);
    // A bola também é obstáculo até o ponto de chute (não empurrar antes).
    const kb = grade.idx(ballEst.x, ballEst.y);
    if (kb >= 0) {
      const r = Math.ceil(0.1 / 0.04), bi = kb % grade.nx, bj = (kb / grade.nx) | 0;
      for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
        const ii = bi + di, jj = bj + dj;
        if (ii >= 0 && jj >= 0 && ii < grade.nx && jj < grade.ny && di * di + dj * dj <= r * r) bloq[jj * grade.nx + ii] = 1;
      }
    }
    const gx = GOL.linhaX + 0.05;
    const base = Math.atan2(GOL.centroY - ballEst.y, gx - ballEst.x);
    // Ponto de chute atrás da bola; se estiver bloqueado, tenta ângulos e
    // distâncias vizinhos.
    for (const da of [0, 0.25, -0.25, 0.5, -0.5]) {
      for (const d of [0.28, 0.36, 0.45]) {
        const a = base + da, ax = ballEst.x - Math.cos(a) * d, ay = ballEst.y - Math.sin(a) * d;
        const ka = grade.idx(ax, ay);
        if (ka < 0 || bloq[ka]) continue;
        const r = aEstrela(grade, bloq, [est.x, est.y], [ax, ay]);
        if (r && r.caminho) { aceitarPlano(r, [ax, ay, a]); return; }
        if (r) plano.ordem = r.ordem;
      }
    }
    plano.caminho = null; plano.falhou = true;
  }
  // Plano novo de verdade (rota ficou mais de 5 cm mais longa ou o ponto de
  // chute andou): reinicia a animação das casas testadas + rota.
  const compr = (c) => { let L = 0; for (let i = 1; i < c.length; i++) L += Math.hypot(c[i][0] - c[i - 1][0], c[i][1] - c[i - 1][1]); return L; };
  function aceitarPlano(r, alvo) {
    const L = compr(r.caminho);
    // Encurtar porque o pato andou não é plano novo; ficar mais longo (um
    // desvio) ou mudar de destino é.
    const mudou = (!plano.caminho || L - (plano.comprimento ?? 0) > 0.08 ||
      !plano.alvo || Math.hypot(alvo[0] - plano.alvo[0], alvo[1] - plano.alvo[1]) > 0.12) &&
      simTime - (plano.animIni ?? -1e9) > 3;
    plano.caminho = r.caminho; plano.alvo = alvo; plano.expandidos = r.expandidos; plano.falhou = false;
    plano.comprimento = L;
    if (mudou) { plano.ordem = r.ordem; plano.animIni = simTime; }
  }
  // Segue a rota: mira num ponto 18 cm à frente no caminho.
  function seguirRota() {
    const c = plano.caminho;
    let melhor = 0, dMin = Infinity;
    for (let i = 0; i < c.length; i++) {
      const d = Math.hypot(c[i][0] - est.x, c[i][1] - est.y);
      if (d < dMin) { dMin = d; melhor = i; }
    }
    let alvo = c[c.length - 1];
    for (let i = melhor; i < c.length; i++) {
      if (Math.hypot(c[i][0] - est.x, c[i][1] - est.y) > 0.18) { alvo = c[i]; break; }
    }
    plano.olhando = alvo;
    const e = wrapA(Math.atan2(alvo[1] - est.y, alvo[0] - est.x) - est.t);
    if (Math.abs(e) > 0.45) { twist[0] = 0; twist[2] = Math.sign(e) * TURN_INPLACE; }
    else { twist[0] = VEL_FWD; twist[2] = clamp(2.2 * e, -VEL_ANG, VEL_ANG); }
  }
  function decidirTwist(t) {
    twist[0] = 0; twist[2] = 0;
    if (hold || windup || policy !== "walk" || postKickLock > 0 || recovery || t - seek.kickedAt < 1.2) {
      stall.since = null;
      return;
    }
    updateBallEstimate(t);
    if (!est || est.spread > 0.2) { stallCheck(t); return; } // parado até se localizar
    if (t - ballEst.at > 6) {
      // Sem saber da bola: gira no lugar procurando (não sai andando antes
      // de ter um plano).
      twist[0] = 0; twist[2] = TURN_INPLACE * seek.dir;
      const fresh = det.seen && t - det.at < 0.3;
      if (fresh) seek.dir = det.cx > 0.5 ? -1 : 1;
      stallCheck(t);
      return;
    }
    if (t - plano.em > 0.4) planejar(t);
    if (nav.stage === "ir") {
      if (!plano.caminho) { stallCheck(t); return; }
      const [ax, ay] = plano.alvo;
      if (Math.hypot(ax - est.x, ay - est.y) > 0.08) { seguirRota(); stallCheck(t); return; }
      nav.stage = "alinhar";
    }
    const goalDir = plano.alvo ? plano.alvo[2] : 0;
    if (nav.stage === "alinhar") {
      const e = wrapA(goalDir - est.t);
      if (Math.abs(e) > 0.2) { twist[0] = 0; twist[2] = Math.sign(e) * TURN_INPLACE; stallCheck(t); return; }
      nav.stage = "atacar";
      nav.since = t;
    }
    const fresh = det.seen && t - det.at < 0.25;
    if (fresh && det.y0 > 0.86 && Math.abs(det.cx - 0.5) < 0.14) {
      if (requestKick(det.cx < 0.5 ? "left" : "right")) { seek.kickedAt = t; nav.stage = "ir"; plano.em = -1e9; }
      return;
    }
    const e = wrapA(goalDir - est.t);
    twist[0] = 0.2;
    twist[2] = fresh ? clamp(-2.6 * (det.cx - 0.5), -VEL_ANG, VEL_ANG) : clamp(2 * e, -VEL_ANG, VEL_ANG);
    if (t - nav.since > 5) nav.stage = "ir";
    stallCheck(t);
  }

  // ── Mover: empurrão + malha aberta ─────────────────────────────────────
  // O empurrão é uma força externa no tronco (xfrc_applied), só enquanto o
  // visitante arrasta: mola até o ponto do chão sob o cursor, limitada.
  const gravador = criarGravador(NUM_JOINTS);
  const mover = { equilibrio: true, ciclo: null, idx: 0, quedas: { com: 0, sem: 0 }, alvo: null };
  const EMPURRAO_K = 12, EMPURRAO_MAX = 3.0; // N/m, N
  function applyEmpurrao() {
    const x = data.xfrc_applied, a = trunkId * 6;
    if (!mover.alvo || mode !== "mover") { x[a] = 0; x[a + 1] = 0; return; }
    const q = data.qpos;
    let fx = EMPURRAO_K * (mover.alvo[0] - q[0]), fy = EMPURRAO_K * (mover.alvo[1] - q[1]);
    const n = Math.hypot(fx, fy);
    if (n > EMPURRAO_MAX) { fx *= EMPURRAO_MAX / n; fy *= EMPURRAO_MAX / n; }
    x[a] = fx; x[a + 1] = fy;
  }
  function contarQueda() {
    if (mode === "mover") mover.quedas[mover.equilibrio ? "com" : "sem"]++;
    if (mode === "aprender") {
      aprender.quedas[aprender.geracao] = (aprender.quedas[aprender.geracao] ?? 0) + 1;
      // Fantasma: a pose no momento da queda.
      aprender.queda.unshift({ q: Float64Array.from(data.qpos), ini: [...aprender.ini] });
      aprender.queda.length = Math.min(aprender.queda.length, 4);
    }
  }
  function setEquilibrio(on) {
    if (on === mover.equilibrio) return true;
    if (!on) {
      const ciclo = gravador.ciclo();
      if (!ciclo) return false; // ainda gravando
      mover.ciclo = ciclo; mover.idx = 0;
    }
    mover.equilibrio = on;
    return true;
  }

  // ── Aprender: gerações simuladas com ruído decrescente ────────────────
  // Aproximação: não é treino. A política final é a mesma em todas as
  // gerações; o que muda é o ruído somado às ações (geração 0 = muito
  // ruído, que se debate e cai).
  let GERACOES = 6;
  const GERACAO_S = 10;
  // Cada geração: o pato sai do mesmo ponto rumo à bola; caiu ou chegou, a
  // tentativa recomeça (como um episódio de treino).
  const aprender = {
    geracao: 0, inicio: 0, sigma: 0, quedas: [], fim: false, ruido: new Float32Array(NUM_JOINTS),
    queda: [], alcance: [], ini: [0, 0], alvo: [0, 0],
  };
  const sigmaDa = (g) => 1.0 * Math.pow(0.5, g);
  // Gerações reais: checkpoints do treino da marcha exportados pra ONNX
  // (public/pato/policies/geracoes/geracoes.json). Sem eles, a aproximação
  // com ruído.
  async function carregarGeracoes() {
    if (aprender.sessoes || aprender.semReal) return;
    try {
      const r = await fetch("/pato/policies/geracoes/geracoes.json");
      if (!r.ok) throw new Error(String(r.status));
      const lista = await r.json();
      const sess = await Promise.all(lista.geracoes.map((g) =>
        ort.InferenceSession.create(`/pato/policies/geracoes/${g.arquivo}`, opts)));
      aprender.sessoes = sess;
      aprender.iteracoes = lista.geracoes.map((g) => g.iteracao);
      aprender.origem = lista.origem ?? "";
    } catch {
      aprender.semReal = true;
    }
  }
  function iniciarAprender() {
    if (aprender.sessoes) GERACOES = aprender.sessoes.length;
    aprender.geracao = 0; aprender.inicio = simTime; aprender.sigma = aprender.sessoes ? 0 : sigmaDa(0);
    aprender.quedas = new Array(GERACOES).fill(0); aprender.fim = false;
    aprender.alcance = new Array(GERACOES).fill(0); aprender.queda = [];
  }
  function passoAprender() {
    // Quão longe chegou rumo à bola nesta geração.
    const q = data.qpos, [ix, iy] = aprender.ini, [bx, by] = aprender.alvo;
    const L = Math.hypot(bx - ix, by - iy) || 1;
    const prog = ((q[0] - ix) * (bx - ix) + (q[1] - iy) * (by - iy)) / L;
    const g = aprender.geracao;
    if (!aprender.fim) aprender.alcance[g] = Math.max(aprender.alcance[g] ?? 0, Math.min(prog, L));
    if (Math.hypot(q[0] - bx, q[1] - by) < 0.15) resetScene(); // chegou: nova tentativa
    if (aprender.fim) return;
    if (simTime - aprender.inicio >= GERACAO_S) {
      aprender.geracao++;
      aprender.inicio = simTime;
      if (aprender.geracao >= GERACOES) { aprender.fim = true; aprender.geracao = GERACOES - 1; aprender.sigma = 0; return; }
      aprender.sigma = aprender.sessoes ? 0 : sigmaDa(aprender.geracao);
      lastAction.fill(0); // política nova, histórico de ação novo
    }
  }

  function setObstaculo(i, x, y, yaw) {
    const o = obst[i], g = obstIds[i];
    o.x = x; o.y = y; o.yaw = yaw;
    model.geom_pos[g * 3] = x; model.geom_pos[g * 3 + 1] = y; model.geom_pos[g * 3 + 2] = o.half[2];
    model.geom_quat[g * 4] = Math.cos(yaw / 2); model.geom_quat[g * 4 + 1] = 0;
    model.geom_quat[g * 4 + 2] = 0; model.geom_quat[g * 4 + 3] = Math.sin(yaw / 2);
  }

  // Disposição da cena por modo: livro e pose inicial.
  function layoutForMode() {
    const book = mode === "jogar";
    if (mode !== "decidir") obst.forEach((o, i) => setObstaculo(i, PARK[0] + i, PARK[1], 0));
    model.geom_pos[bookId * 3] = book ? BOOK.pos[0] : PARK[0];
    model.geom_pos[bookId * 3 + 1] = book ? BOOK.pos[1] : PARK[1];
    const q = data.qpos;
    if (mode === "gancho") {
      // De costas pro gol: com só a visão, ele chuta pro lado errado.
      q[0] = SPAWN[0] + 0.1; q[1] = GOL.centroY;
      q[3] = 0; q[4] = 0; q[5] = 0; q[6] = 1;
      placeBall(SPAWN[0] - 0.22, GOL.centroY + 0.05);
    } else if (mode === "mover") {
      q[0] = (CAMPO_MJ.xmin + CAMPO_MJ.xmax) / 2; q[1] = (CAMPO_MJ.ymin + CAMPO_MJ.ymax) / 2;
      q[3] = 1; q[4] = 0; q[5] = 0; q[6] = 0;
      placeBall(PARK[0], PARK[1]); // sem bola nesse modo
    } else if (mode === "aprender") {
      const ix = CAMPO_MJ.xmin + 0.25, iy = GOL.centroY;
      aprender.ini = [ix, iy]; aprender.alvo = [ix + 1.0, iy];
      q[0] = ix; q[1] = iy; q[3] = 1; q[4] = 0; q[5] = 0; q[6] = 0;
      placeBall(ix + 1.0, iy);
    } else if (mode === "decidir") {
      // Pato de um lado, bola do outro, e três objetos no caminho.
      const cy = GOL.centroY;
      q[0] = -0.4; q[1] = cy; q[3] = 1; q[4] = 0; q[5] = 0; q[6] = 0;
      placeBall(0.42, cy);
      setObstaculo(0, -0.08, cy, Math.PI / 2);
      setObstaculo(1, 0.1, cy + 0.34, 0.4);
      setObstaculo(2, 0.1, cy - 0.34, 0);
    } else if (mode === "jogar") {
      // A bola atrás do livro: o caminho até o gol passa por cima dele.
      const bx = BOOK.pos[0] - 0.4, by = GOL.centroY + 0.06;
      const far = Math.hypot(q[0] - bx, q[1] - by) > 0.3;
      placeBall(far ? bx : bx - 0.1, far ? by : by - 0.25);
    }
    mujoco.mj_forward(model, data);
  }
  function resetScene() {
    resetSim();
    layoutForMode();
    if (mode === "odometria") syncOdoToTruth();
  }

  // ── Laço de controle a 50 Hz ─────────────────────────────────────────
  let simTime = 0;
  let HEAD_DOWN = 0.45;
  const HEAD_UP = 0.12;
  let headLow = false;
  let twistOverride = null; // depuração
  async function controlStep() {
    simTime += CTRL_DT;
    if (twistOverride) { twist[0] = twistOverride[0]; twist[2] = twistOverride[1]; }
    else if (mode === "controle") keyboardTwist();
    else if (mode === "solto") wanderTwist(simTime);
    else if (mode === "odometria" || mode === "laser" || mode === "patins") keyboardTwist();
    else if (mode === "jogar") locTwist(simTime);
    else if (mode === "decidir") decidirTwist(simTime);
    else if (mode === "mover") {
      // Anda devagar em círculo, sempre com o mesmo comando.
      twist[0] = 0.2; twist[2] = 0.5;
    } else if (mode === "aprender") {
      // Rumo à bola, corrigindo o rumo.
      const q = data.qpos, [bx, by] = aprender.alvo;
      const e = wrapA(Math.atan2(by - q[1], bx - q[0]) - yawOf(q));
      twist[0] = 0.22; twist[2] = clamp(2 * e, -VEL_ANG, VEL_ANG);
    }
    else visionTwist(simTime);
    // Na visão, a cabeça olha um pouco pra baixo pra enxergar o chão à frente.
    // Durante o preparo e o chute a cabeça volta ao neutro: com ela
    // inclinada o chute direito derrubava o pato (2 de 2 nos testes).
    const kicking = windup || policy !== "walk" || postKickLock > 0;
    // Cabeça baixa pra bola perto (é onde a tabela de distância foi medida);
    // procurando, ou com a bola longe no alto da imagem, a cabeça sobe um
    // pouco pra enxergar o resto do chão da sala.
    const fresh = det.seen && simTime - det.at < 0.3;
    // Sem a bola: alterna olhar perto e longe a cada 1,5 s.
    if (!fresh) headLow = Math.floor(simTime / 1.5) % 2 === 0;
    else if (det.y0 > 0.36) headLow = true;
    else if (det.y0 < 0.12) headLow = false;
    headTarget[1] = usesVision() && !kicking ? (headLow ? HEAD_DOWN : HEAD_UP) : 0;

    const malhaAberta = mode === "mover" && !mover.equilibrio && mover.ciclo && !recovery && policy === "walk";
    if (malhaAberta) {
      // Sem equilíbrio: toca o ciclo de passo gravado, sem olhar sensor.
      buildObs(); // só pra manter o comando/cabeça em dia; a saída é ignorada
      const a = mover.ciclo[mover.idx++ % mover.ciclo.length];
      lastAction.set(a);
      const ctrl = data.ctrl;
      for (let j = 0; j < NUM_JOINTS; j++) ctrl[j] = DEFAULT_POSE[j] + a[j];
    } else if (recovery?.state !== "fallen") {
      const session = activeSession();
      const out = await session.run({
        [session.inputNames[0]]: new ort.Tensor("float32", buildObs(), [1, OBS_SIZE]),
      });
      if (disposed) return;
      let act = out[session.outputNames[0]].data;
      // Aprender: ruído nas ações da política de andar (não na de levantar).
      if (mode === "aprender" && policy === "walk" && !recovery && aprender.sigma > 0) {
        // Ruído correlacionado no tempo (~0,15 s): ruído branco a 50 Hz é
        // filtrado pelos motores e quase não derruba o pato.
        const c = 0.87, k = Math.sqrt(1 - c * c);
        for (let j = 0; j < NUM_JOINTS; j++) aprender.ruido[j] = c * aprender.ruido[j] + k * gauss();
        act = Float32Array.from(act, (v, j) => v + aprender.sigma * aprender.ruido[j]);
      }
      // Mover: grava a marcha da política enquanto ela anda com equilíbrio.
      if (mode === "mover" && mover.equilibrio && policy === "walk" && !recovery) gravador.gravar(act);
      let finite = true;
      for (let j = 0; j < NUM_JOINTS; j++) if (!Number.isFinite(act[j])) { finite = false; break; }
      if (finite) {
        lastAction.set(act);
        const ctrl = data.ctrl;
        for (let j = 0; j < NUM_JOINTS; j++) ctrl[j] = DEFAULT_POSE[j] + act[j];
      }
    }
    for (let s = 0; s < DECIMATION; s++) {
      applyGrab();
      applyEmpurrao();
      holdCarry();
      mujoco.mj_step(model, data);
    }
    holdCarry();
    // Comando que a política de fato recebeu neste passo (zero quando travado).
    const sentLocked = recovery || policy !== "walk" || postKickLock > 0 || windup;
    stepLocalization(sentLocked ? 0 : cmd[0], sentLocked ? 0 : cmd[2]);

    // Queda: máquina de recuperação com a política de levantar.
    const z = data.qpos[2], gz = projGravZ();
    if (!Number.isFinite(z) || !Number.isFinite(gz)) {
      resetScene();
    } else if (recovery) {
      recovery.steps++;
      if (recovery.state === "fallen") {
        if (recovery.steps >= 15) { recovery = { state: "recovering", steps: 0, upright: 0 }; lastAction.fill(0); }
      } else {
        recovery.upright = gz < -0.85 ? recovery.upright + 1 : 0;
        if (recovery.upright >= 50) { recovery = null; policy = "walk"; lastAction.fill(0); }
        else if (recovery.steps >= 300) resetScene();
      }
    } else if (carry) {
      fallDebounce = 0; fallenSince = null;
    } else if (gz > -0.5 || z < 0.02) {
      // De patins não há política de levantar (o runtime também não usa):
      // caiu, volta pro início depois de 1 s.
      if (mode === "aprender" && !carry) {
        // Aprender: caiu, termina a tentativa e recomeça do ponto de saída.
        if (++fallDebounce >= 10) { fallDebounce = 0; contarQueda(); resetScene(); }
      } else if (policy === "walk" && postKickLock === 0 && !patins) {
        if (++fallDebounce >= 10) { fallDebounce = 0; recovery = { state: "fallen", steps: 0 }; contarQueda(); }
      } else {
        if (fallenSince === null) contarQueda();
        fallenSince ??= simTime;
        if (simTime - fallenSince > 1) resetScene();
      }
    } else {
      fallDebounce = 0; fallenSince = null;
    }

    if (postKickLock > 0 && policy === "walk") postKickLock--;
    driveWindup();
    if (kickRun) boostOnContact();
    if (kickRun && ++kickRun.steps >= KICK_STEPS) {
      kickRun = null; policy = "walk"; postKickLock = POST_KICK_LOCK_STEPS;
    }

    // Gol e bola fugida.
    const q = data.qpos;
    const bx = q[ballQ], by = q[ballQ + 1];
    if (goalCooldown > 0) goalCooldown -= CTRL_DT;
    else if (bx > GOL.linhaX + BALL_RADIUS * 0.6 && Math.abs(by - GOL.centroY) < GOL.meiaLargura && !grab) {
      // Uns 4 s depois do gol a cena reseta: pato e bola voltam ao início.
      goalCooldown = GOAL_RESET_S + 1;
      sceneResetAt = simTime + GOAL_RESET_S;
      ballRespawnAt = null;
      goals++;
      onGoal?.(mode);
    }
    const semBola = mode === "mover" || mode === "aprender";
    if (!semBola && (bx < PISO.xmin - 0.3 || bx > PISO.xmax + 0.6 || by < PISO.ymin - 0.3 || by > PISO.ymax + 0.6 || !Number.isFinite(bx))) {
      ballRespawnAt = simTime;
    }
    // Bola parada fora do campo por 6 s (fora do modo controle): o pato
    // sozinho não sabe tirá-la de baixo do sofá, então ela volta pro campo.
    const atWall = foraDoCampo(bx, by, 0.05);
    const slow = Math.hypot(data.qvel[ballV], data.qvel[ballV + 1]) < 0.05;
    if (mode !== "controle" && !semBola && atWall && slow && !grab) {
      ballStuckFor += CTRL_DT;
      if (ballStuckFor > 6 && ballRespawnAt === null) ballRespawnAt = simTime;
    } else ballStuckFor = 0;
    if (sceneResetAt !== null && simTime >= sceneResetAt) resetScene();
    if (mode === "aprender") passoAprender();
    if (ballRespawnAt !== null && simTime >= ballRespawnAt) {
      ballRespawnAt = null;
      endGrab();
      placeBall(...randomBallSpot());
    }
  }

  resetSim();
  let running = true;
  (async function loop() {
    let next = performance.now();
    while (running && !disposed) {
      await controlStep();
      next += CTRL_DT * 1000;
      const wait = next - performance.now();
      if (wait > 0) await new Promise((r) => setTimeout(r, wait));
      else next = performance.now();
    }
  })();

  // ── Cena ─────────────────────────────────────────────────────────────
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  // A sala aprovada: luz, sombras (PCF) e tom vêm de lib/cena. O
  // tone mapping é feito no passe final da Percepcao, então o renderer fica
  // sem tone mapping próprio.
  configurarRenderer(renderer);
  const cena = criarCena();
  const scene = new THREE.Scene();
  scene.background = cena.fundo;
  scene.add(cena.sala, cena.campo, cena.luz);

  const camera = new THREE.PerspectiveCamera(cena.camera.fov, 1, 0.02, 30);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.enablePan = false;
  controls.minDistance = 0.5;
  controls.maxDistance = 4;
  controls.maxPolarAngle = Math.PI * 0.46;
  camera.position.copy(cena.camera.posicao);
  controls.target.copy(cena.camera.alvo);

  // "O que o pato percebe": nítido perto dele, pontilhado e papel longe.
  // No celular/máquina fraca a Percepcao entra sozinha na versão leve.
  const percepcao = new Percepcao({ qualidade: cena.qualidade });

  // Tudo que vive em coordenadas MJCF fica sob este grupo (z pra cima).
  const mj = new THREE.Group();
  mj.rotation.x = -Math.PI / 2;
  scene.add(mj);

  const addBox = (pos, size, mat, parent = mj) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2), mat);
    m.position.set(...pos);
    m.castShadow = true;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };

  const ballMesh = new THREE.Mesh(
    new THREE.SphereGeometry(BALL_RADIUS, 48, 32),
    new THREE.MeshStandardMaterial({ map: cena.texturaBola, roughness: 1 }),
  );
  ballMesh.castShadow = true;
  ballMesh.receiveShadow = true;
  mj.add(ballMesh);

  const rig = await rigPromise;
  if (disposed) return null;
  scene.add(rig.placer);
  const trunk = rig.bodies.get("trunk_base");

  // ── Localizar: fantasma, mapa no chão, partículas, raios, livro ───────
  // Pato fantasma: a pose que o pato acha que tem (odometria ou filtro).
  const ghost = await buildRig(kin, classicMaterial);
  if (disposed) return null;
  const ghostMat = new THREE.MeshStandardMaterial({
    color: "#7a8a9a", transparent: true, opacity: 0.32, depthWrite: false, roughness: 0.6,
  });
  ghost.placer.traverse((o) => { if (o.isMesh) { o.material = ghostMat; o.castShadow = false; o.receiveShadow = false; } });
  ghost.placer.visible = false;
  scene.add(ghost.placer);
  const ghostTrunk = ghost.bodies.get("trunk_base");

  // ── Marcas no chão (regra da thread de design) ───────────────────────
  // Tinta pro que o pato sabe (mapa, palpites, grade); laranja só pro que
  // acontece agora ou é o objetivo (feixe do laser, rota escolhida, centro
  // de massa, anel na bola). Marcas no chão somem atrás dos móveis; centro
  // de massa e polígono dos pés vão em raio X.
  const marcas = new Marcas({ max: 12000 });
  const raioX = new Marcas({ max: 1500, raioX: true });
  scene.add(marcas.mesh, raioX.mesh);
  // Desenho pro visitante, não existe pro pato: camada 1 (a câmera da cabeça
  // não renderiza).
  ghost.placer.traverse((c) => c.layers.set(1));
  camera.layers.enable(1);

  // Obstáculos do decidir: livro, chinelo (o mesmo desenho dos chinelos do
  // gol, noutra cor) e uma caixa de sapato.
  {
    const livro = addBox([PARK[0], PARK[1], 0], OBSTACULOS[0].half, new THREE.MeshStandardMaterial({ color: "#8c2f39", roughness: 0.8 }));
    const chin = criarChinelo("#a8633a");
    scene.add(chin);
    const caixa = addBox([PARK[0], PARK[1], 0], OBSTACULOS[2].half, new THREE.MeshStandardMaterial({ color: "#d8cbb4", roughness: 0.9 }));
    obst[0].mesh = livro; obst[1].mesh = chin; obst[2].mesh = caixa;
    obst[1].solto = true; // o chinelo vive no referencial da sala, não no do MJCF
  }
  // Livro (o obstáculo).
  const bookMesh = addBox([PARK[0], PARK[1], BOOK.half[2]], BOOK.half,
    new THREE.MeshStandardMaterial({ color: "#2f4a5c", roughness: 0.8 }));

  // Aprender: fantasmas brancos das últimas 4 quedas reais.
  const fantasmas = [];
  for (let i = 0; i < 4; i++) {
    const r = await buildRig(kin, classicMaterial);
    if (disposed) return null;
    const m = new THREE.MeshStandardMaterial({
      color: "#ffffff", transparent: true, opacity: 0.32 * (1 - i / 4.5), depthWrite: false, roughness: 0.7,
    });
    r.placer.traverse((o) => { if (o.isMesh) { o.material = m; o.castShadow = false; o.receiveShadow = false; } o.layers.set(1); });
    r.placer.visible = false;
    scene.add(r.placer);
    fantasmas.push(r);
  }

  const _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion(), _zAxis = new THREE.Vector3(0, 0, 1);
  const _v1 = new THREE.Vector3(), _v2 = new THREE.Vector3(), _v3 = new THREE.Vector3(), _mb = new THREE.Matrix4();
  // Placar que a interface mostra (atualizado ao desenhar).
  const placar = { texto: "" };
  // MJCF (x, y) -> sala (x, z).
  const S = (x, y) => [x, -y];

  function syncLocalization() {
    const q = data.qpos;
    for (const o of obst) {
      if (o.solto) { o.mesh.position.set(o.x, 0, -o.y); o.mesh.rotation.y = o.yaw; }
      else { o.mesh.position.set(o.x, o.y, o.half[2]); o.mesh.rotation.z = o.yaw; }
      o.mesh.visible = mode === "decidir";
    }
    bookMesh.visible = mode === "jogar";
    bookMesh.position.set(model.geom_pos[bookId * 3], model.geom_pos[bookId * 3 + 1], BOOK.half[2]);
    const laser = usesLaser();
    const g = mode === "odometria" ? odo : laser ? est : null;
    ghost.placer.visible = showsGhost() && !!g;
    if (ghost.placer.visible) {
      // Mesma postura do pato real, na pose estimada.
      _qa.set(q[4], q[5], q[6], q[3]);
      _qb.setFromAxisAngle(_zAxis, g.t - yawOf(q));
      ghostTrunk.quaternion.copy(_qb.multiply(_qa));
      ghostTrunk.position.set(g.x, g.y, q[2]);
      for (let j = 0; j < NUM_JOINTS; j++) setJoint(ghost, JOINT_NAMES[j], q[qposAdr[j]]);
    }
    for (let i = 0; i < fantasmas.length; i++) {
      const f = aprender.queda[i];
      fantasmas[i].placer.visible = mode === "aprender" && !!f;
      if (!f) continue;
      const tr = fantasmas[i].bodies.get("trunk_base");
      tr.position.set(f.q[0], f.q[1], f.q[2]);
      tr.quaternion.set(f.q[4], f.q[5], f.q[6], f.q[3]);
      for (let j = 0; j < NUM_JOINTS; j++) setJoint(fantasmas[i], JOINT_NAMES[j], f.q[qposAdr[j]]);
    }
    desenhar();
  }

  function desenhar() {
    marcas.inicio(); raioX.inicio();
    placar.texto = "";
    const t = simTime;
    if (mode === "visao") campoDeVisao();
    if (mode === "laser") desenharLaser(t);
    if (mode === "decidir") desenharDecidir(t);
    if (mode === "mover") desenharMover();
    if (mode === "aprender") desenharAprender();
    marcas.fim(); raioX.fim();
  }

  // Visão: o campo de visão da câmera da cabeça, tracejado no chão.
  function campoDeVisao() {
    const p = data.cam_xpos, X = data.cam_xmat, a = headCamId * 3, b = headCamId * 9;
    const yaw = Math.atan2(X[b + 5], X[b + 2]);
    const [ox, oz] = S(p[a], p[a + 1]);
    for (const s of [-0.62, 0.62]) {
      const ang = yaw + s;
      const [ex, ez] = S(p[a] + Math.cos(ang) * 1.1, p[a + 1] + Math.sin(ang) * 1.1);
      marcas.tracejado(ox, oz, ex, ez, 0.006, TINTA, 0.5);
    }
  }

  // Localizar: impactos do laser (o mapa), o feixe girando, os palpites.
  function desenharLaser(t) {
    for (let i = Math.max(0, mapN - 2200); i < mapN; i++) {
      const [x, z] = S(mapPts[i * 2], mapPts[i * 2 + 1]);
      marcas.ponto(x, z, 0.018, TINTA, 0.85, 0.003);
    }
    // O feixe: as 90 direções da última varredura, desenhadas girando com
    // rastro de 24 raios (é a mesma varredura, mostrada em sequência).
    if (scan.n > 0) {
      const base = Math.floor(t * 240) % LIDAR_RAYS;
      const [ox, oz] = S(scan.ox, scan.oy);
      for (let i = 0; i < 24; i++) {
        const k = (base - i + LIDAR_RAYS) % LIDAR_RAYS;
        let hx = scan.hx[k] - scan.ox, hy = scan.hy[k] - scan.oy;
        const L = Math.hypot(hx, hy);
        if (L > 1.6) { hx *= 1.6 / L; hy *= 1.6 / L; }
        const [ex, ez] = S(scan.ox + hx, scan.oy + hy);
        marcas.seg(ox, oz, ex, ez, 0.004, SOL, 0.8 * (1 - i / 24), 0.006);
      }
    }
    // Palpites (partículas): ponto + traço de 5 cm na direção.
    const celulas = new Set();
    for (let i = 0; i < pf.n; i += 2) {
      const [x, z] = S(pf.x[i], pf.y[i]);
      const [ex, ez] = S(pf.x[i] + Math.cos(pf.t[i]) * 0.05, pf.y[i] + Math.sin(pf.t[i]) * 0.05);
      marcas.ponto(x, z, 0.012, TINTA, 0.6, 0.005);
      marcas.seg(x, z, ex, ez, 0.003, TINTA, 0.6, 0.005);
    }
    for (let i = 0; i < pf.n; i++) celulas.add(Math.round(pf.x[i] / 0.05) * 1000 + Math.round(pf.y[i] / 0.05));
    const achou = est && est.spread < 0.05;
    if (achou) { const [x, z] = S(est.x, est.y); marcas.anel(x, z, 0.13, SOL, 0.95, 0.006); }
    placar.texto = achou ? "posição encontrada" : `${celulas.size} palpites ainda possíveis`;
  }

  // Decidir: a grade do A*, as casas testadas em ordem e a rota escolhida.
  function desenharDecidir(t) {
    const c = 0.04;
    for (let k = 0; k < grade.ocup.length; k++) {
      const [mx, my] = grade.centro(k);
      const [x, z] = S(mx, my);
      if (grade.ocup[k] >= 0.5) marcas.chao(x, z, c * 0.62, c * 0.62, 0, TINTA, 0.2, 0.003);
      else if (grade.visto[k]) marcas.ponto(x, z, 0.006, TINTA, 0.3, 0.003);
    }
    const tA = t - (plano.animIni ?? -1e9);
    const ordem = plano.ordem ?? [];
    if (tA < 2.9) {
      const n = Math.floor(Math.min(1, tA / 1.6) * ordem.length);
      const alfa = 0.55 * (tA < 2.4 ? 1 : Math.max(0, 1 - (tA - 2.4) / 0.5));
      for (let i = 0; i < n; i++) {
        const [mx, my] = grade.centro(ordem[i]);
        const [x, z] = S(mx, my);
        marcas.ponto(x, z, 0.016, TINTA, alfa, 0.004);
      }
    }
    if (plano.caminho && !plano.falhou) {
      const cam = plano.caminho;
      const frac = Math.max(0, Math.min(1, (tA - 1.6) / 0.8));
      let resta = frac * (plano.comprimento ?? 0);
      for (let i = 1; i < cam.length && resta > 0; i++) {
        const [x1, z1] = S(cam[i - 1][0], cam[i - 1][1]);
        let [x2, z2] = S(cam[i][0], cam[i][1]);
        const L = Math.hypot(x2 - x1, z2 - z1);
        if (L > resta) { x2 = x1 + ((x2 - x1) * resta) / L; z2 = z1 + ((z2 - z1) * resta) / L; }
        resta -= L;
        marcas.seg(x1, z1, x2, z2, 0.012, SOL, 0.95, 0.006);
      }
      if (frac >= 1 && plano.alvo) {
        const [ax, az] = S(plano.alvo[0], plano.alvo[1]);
        marcas.anel(ax, az, 0.05, SOL, 0.95, 0.007);
      }
    }
    if (ballEst.at > -1e8) {
      const [bx, bz] = S(ballEst.x, ballEst.y), [gx, gz] = S(GOL.linhaX, GOL.centroY);
      marcas.tracejado(bx, bz, gx, gz, 0.004, TINTA, 0.55);
    }
    placar.texto = plano.falhou ? "sem caminho: tire o livro da frente"
      : plano.caminho ? `${plano.expandidos} casas testadas · caminho de ${Math.round((plano.comprimento ?? 0) * 100)} cm` : "";
  }

  // Mover: polígono de apoio (pés no chão) e centro de massa real.
  const pesGeoms = [];
  for (let gi = 0; gi < model.ngeom; gi++) {
    const b = model.geom_bodyid[gi];
    if ((b === ankleIds.left || b === ankleIds.right) && model.geom_group[gi] === 3) pesGeoms.push(gi);
  }
  function cascoConvexo(pts) {
    if (pts.length < 3) return pts;
    pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = [], hi = [];
    for (const p of pts) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
    for (let i = pts.length - 1; i >= 0; i--) { const p = pts[i]; while (hi.length >= 2 && cr(hi[hi.length - 2], hi[hi.length - 1], p) <= 0) hi.pop(); hi.push(p); }
    hi.pop(); lo.pop();
    return lo.concat(hi);
  }
  const dentro = (poly, x, y) => {
    let d = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i], [xj, yj] = poly[j];
      if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) d = !d;
    }
    return d;
  };
  function desenharMover() {
    // Cantos das caixas (AABB local) dos geoms de colisão dos pés que estão
    // encostando no chão.
    const pts = [];
    const gx = data.geom_xpos, gm = data.geom_xmat, aabb = model.geom_aabb;
    for (const gi of pesGeoms) {
      const c = [aabb[gi * 6], aabb[gi * 6 + 1], aabb[gi * 6 + 2]], h = [aabb[gi * 6 + 3], aabb[gi * 6 + 4], aabb[gi * 6 + 5]];
      const cantos = [];
      for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
        const lx = c[0] + sx * h[0], ly = c[1] + sy * h[1], lz = c[2] + sz * h[2], m = gi * 9;
        cantos.push([
          gx[gi * 3] + gm[m] * lx + gm[m + 1] * ly + gm[m + 2] * lz,
          gx[gi * 3 + 1] + gm[m + 3] * lx + gm[m + 4] * ly + gm[m + 5] * lz,
          gx[gi * 3 + 2] + gm[m + 6] * lx + gm[m + 7] * ly + gm[m + 8] * lz,
        ]);
      }
      const zmin = Math.min(...cantos.map((p) => p[2]));
      if (zmin > 0.012) continue; // pé no ar
      for (const p of cantos) if (p[2] < zmin + 0.012) pts.push([p[0], p[1]]);
    }
    const poly = cascoConvexo(pts);
    const ar = poly.map(([x, y]) => S(x, y));
    for (let i = 0; i < ar.length; i++) {
      const [x1, z1] = ar[i], [x2, z2] = ar[(i + 1) % ar.length];
      raioX.seg(x1, z1, x2, z2, 0.003, TINTA, 0.9, 0.003);
    }
    if (poly.length >= 3) {
      const xs = poly.map((p) => p[0]), ys = poly.map((p) => p[1]);
      for (let x = Math.min(...xs); x <= Math.max(...xs); x += 0.008) {
        for (let y = Math.min(...ys); y <= Math.max(...ys); y += 0.008) {
          if (dentro(poly, x, y)) { const [sx, sz] = S(x, y); raioX.ponto(sx, sz, 0.003, TINTA, 0.45, 0.003); }
        }
      }
    }
    // Centro de massa do pato inteiro (subárvore do tronco).
    const com = data.subtree_com, a = trunkId * 3;
    const cx = com[a], cy = com[a + 1], cz = com[a + 2];
    const [sx, sz] = S(cx, cy);
    // Fio de prumo vertical, virado pra câmera.
    _v1.set(camera.position.x - sx, 0, camera.position.z - sz).normalize(); // normal (y local)
    _v2.set(0, 1, 0);                                                       // z local -> vertical
    _v3.crossVectors(_v1, _v2);                                             // x local
    _mb.makeBasis(_v3, _v1, _v2);
    _qa.setFromRotationMatrix(_mb);
    raioX.pose(_v3.set(sx, cz / 2, sz), _qa, 0.002, cz, TINTA, 0.9);
    raioX.pose(_v3.set(sx, cz, sz), _qa, 0.026, 0.026, SOL, 1, 1);
    raioX.pose(_v3.set(sx, cz, sz), _qa, 0.008, 0.008, SOL, 1, 0);
    const sobre = poly.length >= 3 && dentro(poly, cx, cy);
    if (sobre) raioX.ponto(sx, sz, 0.014, SOL, 1, 0.004);
    else raioX.anel(sx, sz, 0.02, TINTA, 0.9, 0.004);
    if (mover.alvo) {
      const q = data.qpos, [tx, tz] = S(q[0], q[1]), [px, pz] = S(mover.alvo[0], mover.alvo[1]);
      marcas.seta(tx, tz, px, pz, 0.004, SOL, 0.95, 0.008);
    }
    placar.texto = sobre ? "centro de massa sobre os pés" : "centro de massa fora dos pés: caiu";
  }

  // Aprender: o traço de cada queda (do começo até onde caiu).
  function desenharAprender() {
    for (const f of aprender.queda) {
      if (!f) continue;
      const [x1, z1] = S(f.ini[0], f.ini[1]), [x2, z2] = S(f.q[0], f.q[1]);
      marcas.seg(x1, z1, x2, z2, 0.004, TINTA, 0.6, 0.003);
      marcas.ponto(x2, z2, 0.016, TINTA, 0.85, 0.004);
    }
    if (ballActiveAprender()) {
      const q = data.qpos, [bx, bz] = S(q[ballQ], q[ballQ + 1]);
      marcas.anel(bx, bz, 0.12, SOL, 0.9, 0.005);
    }
    const k = aprender.quedas[aprender.geracao] ?? 0;
    placar.texto = `geração ${aprender.geracao + 1} · ${k} ${k === 1 ? "queda" : "quedas"}`;
  }
  const ballActiveAprender = () => mode === "aprender";

  // Juntas passivas (as rodas do patins): só visuais, direto do qpos.
  const juntasExtras = kin.bodies
    .filter((b) => b.joint && b.joint.type === "hinge" && !JOINT_NAMES.includes(b.joint.name))
    .map((b) => ({ nome: b.joint.name, adr: model.jnt(b.joint.name).qposadr }));
  function syncScene() {
    for (const j of juntasExtras) setJoint(rig, j.nome, data.qpos[j.adr]);
    const q = data.qpos;
    trunk.position.set(q[0], q[1], q[2]);
    trunk.quaternion.set(q[4], q[5], q[6], q[3]);
    for (let j = 0; j < NUM_JOINTS; j++) setJoint(rig, JOINT_NAMES[j], q[qposAdr[j]]);
    ballMesh.position.set(q[ballQ], q[ballQ + 1], q[ballQ + 2]);
    ballMesh.quaternion.set(q[ballQ + 4], q[ballQ + 5], q[ballQ + 6], q[ballQ + 3]);
    syncLocalization();
  }

  // ── Câmera da cabeça (câmera "head_camera" do MJCF) ──────────────────
  const headCam = new THREE.PerspectiveCamera(72, VISION_W / VISION_H, 0.005, 6);
  const _m4 = new THREE.Matrix4();
  function syncHeadCam() {
    const p = data.cam_xpos, X = data.cam_xmat;
    const a = headCamId * 3, m = headCamId * 9;
    // O quadro "head_camera" exportado pelo onshape-to-robot não segue a
    // convenção do MuJoCo (olhar pra -Z, +Y pra cima): o eixo óptico é o +Z
    // do quadro e o "pra cima" é o -X (verificado no navegador). Montamos a
    // base da câmera do three (olha pra -Z, +Y pra cima) a partir disso e
    // trocamos MJCF -> three: (x, y, z) -> (x, z, -y).
    //   X3 = -colY, Y3 = -colX, Z3 = -colZ (colunas de cam_xmat)
    _m4.set(
      -X[m + 1], -X[m], -X[m + 2], 0,
      -X[m + 7], -X[m + 6], -X[m + 8], 0,
      X[m + 4], X[m + 3], X[m + 5], 0,
      0, 0, 0, 1,
    );
    headCam.quaternion.setFromRotationMatrix(_m4);
    headCam.position.set(p[a], p[a + 2], -p[a + 1]);
    headCam.updateMatrixWorld(true);
  }

  let vision = null;
  function ensureVision() {
    if (vision || !visionCanvas) return vision;
    const vr = new THREE.WebGLRenderer({ canvas: visionCanvas, antialias: false });
    vr.setPixelRatio(1);
    vr.setSize(VISION_W, VISION_H, false);
    const rt = new THREE.WebGLRenderTarget(VISION_W, VISION_H, {
      depthBuffer: true, type: THREE.HalfFloatType,
      minFilter: THREE.NearestFilter, magFilter: THREE.NearestFilter,
    });
    const quadScene = new THREE.Scene();
    const quadCam = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    const mat = new THREE.ShaderMaterial({
      uniforms: {
        tImg: { value: rt.texture },
        texel: { value: new THREE.Vector2(1 / VISION_W, 1 / VISION_H) },
        layer: { value: 0 },
        box: { value: new THREE.Vector4() },
        hasBox: { value: 0 },
      },
      vertexShader: "varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0.0, 1.0); }",
      fragmentShader: VISION_FRAG,
    });
    quadScene.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), mat));
    vision = { vr, rt, quadScene, quadCam, mat, pixels: new Uint16Array(VISION_W * VISION_H * 4), frame: 0 };
    return vision;
  }

  const half = THREE.DataUtils.fromHalfFloat;
  // Máscara de cor -> maior mancha conectada (a sala tem outros pixels
  // amarelados soltos; a bola é a maior mancha contínua).
  const maskBuf = new Uint8Array(VISION_W * VISION_H);
  const stack = new Int32Array(VISION_W * VISION_H);
  function detectBall(px) {
    for (let k = 0; k < VISION_W * VISION_H; k++) {
      const i = k * 4;
      maskBuf[k] = isBallPixel(aces(half(px[i])), aces(half(px[i + 1])), aces(half(px[i + 2]))) ? 1 : 0;
    }
    let n = 0, sx = 0, sy = 0, x0 = 0, y0 = 0, x1 = -1, y1 = -1;
    for (let k = 0; k < VISION_W * VISION_H; k++) {
      if (maskBuf[k] !== 1) continue;
      // Preenche a mancha a partir de k.
      let cn = 0, csx = 0, csy = 0, cx0 = 1e9, cy0 = 1e9, cx1 = -1, cy1 = -1, top = 0;
      stack[top++] = k; maskBuf[k] = 2;
      while (top > 0) {
        const c = stack[--top];
        const x = c % VISION_W, y = (c / VISION_W) | 0;
        cn++; csx += x; csy += y;
        if (x < cx0) cx0 = x; if (x > cx1) cx1 = x;
        if (y < cy0) cy0 = y; if (y > cy1) cy1 = y;
        if (x > 0 && maskBuf[c - 1] === 1) { maskBuf[c - 1] = 2; stack[top++] = c - 1; }
        if (x < VISION_W - 1 && maskBuf[c + 1] === 1) { maskBuf[c + 1] = 2; stack[top++] = c + 1; }
        if (y > 0 && maskBuf[c - VISION_W] === 1) { maskBuf[c - VISION_W] = 2; stack[top++] = c - VISION_W; }
        if (y < VISION_H - 1 && maskBuf[c + VISION_W] === 1) { maskBuf[c + VISION_W] = 2; stack[top++] = c + VISION_W; }
      }
      // Bola é redonda: caixa quase quadrada e bem preenchida (a não ser
      // cortada pela borda de baixo, perto dos pés).
      const bw = cx1 - cx0 + 1, bh = cy1 - cy0 + 1;
      const cortada = cy0 === 0;
      const redonda = cortada || (bw / bh > 0.5 && bw / bh < 2 && cn / (bw * bh) > 0.4);
      if (redonda && cn > n) { n = cn; sx = csx; sy = csy; x0 = cx0; y0 = cy0; x1 = cx1; y1 = cy1; }
    }
    const now = simTime;
    if (n < 4) { det.seen = false; det.n = n; return; }
    // readPixels vem de baixo pra cima: y=0 é a base da imagem.
    det.seen = true; det.n = n; det.at = now;
    det.cx = (sx / n + 0.5) / VISION_W;
    det.cy = 1 - (sy / n + 0.5) / VISION_H;
    det.x0 = x0 / VISION_W; det.x1 = (x1 + 1) / VISION_W;
    det.y0 = 1 - (y1 + 1) / VISION_H; det.y1 = 1 - y0 / VISION_H; // em coordenadas de imagem (0 = topo)
    lastSeen.cx = det.cx; lastSeen.top = det.y0; lastSeen.at = now;
    // Anel: centro e raio (em altura de imagem) da mancha, com folga.
    const rh = (Math.max(x1 - x0 + 1, y1 - y0 + 1) / 2) / VISION_H;
    vision.mat.uniforms.box.value.set((sx / n + 0.5) / VISION_W, (sy / n + 0.5) / VISION_H, rh + 0.03, 0);
  }

  function renderVision() {
    const v = ensureVision();
    if (!v) return;
    syncHeadCam();
    v.vr.setRenderTarget(v.rt);
    // O pato não aparece na própria câmera.
    rig.placer.visible = false;
    v.vr.render(scene, headCam);
    rig.placer.visible = true;
    if (v.frame++ % 2 === 0) {
      v.vr.readRenderTargetPixels(v.rt, 0, 0, VISION_W, VISION_H, v.pixels);
      detectBall(v.pixels);
    }
    v.mat.uniforms.hasBox.value = det.seen ? 1 : 0;
    v.vr.setRenderTarget(null);
    v.vr.render(v.quadScene, v.quadCam);
  }

  // ── Ponteiro: arrastar a bola (só no modo visão) ─────────────────────
  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -(BALL_RADIUS + 0.02));
  const hit = new THREE.Vector3();
  const setNdc = (e) => {
    const r = canvas.getBoundingClientRect();
    ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
  };
  const toTarget = () => {
    if (!ray.ray.intersectPlane(plane, hit)) return null;
    return [clamp(hit.x, PISO.xmin, PISO.xmax), clamp(-hit.z, PISO.ymin, PISO.ymax), BALL_RADIUS + 0.02];
  };
  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const toGround = () => {
    if (!ray.ray.intersectPlane(ground, hit)) return null;
    const b = pf.bounds;
    return [clamp(hit.x, b.xmin, b.xmax), clamp(-hit.z, b.ymin, b.ymax)];
  };
  const canGrabBall = () => mode === "visao" || mode === "gancho" || mode === "jogar";
  let arrasto = null; // { i, dx, dy } obstáculo sendo arrastado
  const obstSob = () => {
    if (mode !== "decidir") return -1;
    let melhor = -1, dist = Infinity;
    obst.forEach((o, i) => {
      const h = ray.intersectObject(o.mesh, true)[0];
      if (h && h.distance < dist) { dist = h.distance; melhor = i; }
    });
    return melhor;
  };
  const onDown = (e) => {
    setNdc(e);
    if (mode === "mover" && ray.intersectObject(rig.placer, true).length) {
      const t = toGround();
      if (!t) return;
      mover.alvo = t;
      controls.enabled = false;
      canvas.setPointerCapture(e.pointerId);
      canvas.style.cursor = "grabbing";
      return;
    }
    const oi = obstSob();
    if (oi >= 0) {
      const t = toGround();
      if (!t) return;
      arrasto = { i: oi, dx: obst[oi].x - t[0], dy: obst[oi].y - t[1] };
      controls.enabled = false;
      canvas.setPointerCapture(e.pointerId);
      canvas.style.cursor = "grabbing";
      return;
    }
    if (usesLaser() && mode !== "decidir" && ray.intersectObject(rig.placer, true).length) {
      const t = toGround();
      if (!t) return;
      carry = { target: t };
      controls.enabled = false;
      canvas.setPointerCapture(e.pointerId);
      canvas.style.cursor = "grabbing";
      return;
    }
    if (!canGrabBall()) return;
    if (!ray.intersectObject(ballMesh).length) return;
    const t = toTarget();
    if (!t) return;
    grab = { target: t };
    controls.enabled = false;
    canvas.setPointerCapture(e.pointerId);
    canvas.style.cursor = "grabbing";
  };
  const onMove = (e) => {
    setNdc(e);
    if (mover.alvo) {
      const t = toGround();
      if (t) mover.alvo = t;
    } else if (arrasto) {
      const t = toGround();
      if (t) { const o = obst[arrasto.i]; setObstaculo(arrasto.i, t[0] + arrasto.dx, t[1] + arrasto.dy, o.yaw); }
    } else if (carry) {
      const t = toGround();
      if (t) carry.target = t;
    } else if (grab) {
      const t = toTarget();
      if (t) grab.target = t;
    } else {
      const over = (mode === "mover" && ray.intersectObject(rig.placer, true).length) || obstSob() >= 0 || (usesLaser() && mode !== "decidir" && ray.intersectObject(rig.placer, true).length) ||
        (canGrabBall() && ray.intersectObject(ballMesh).length);
      canvas.style.cursor = over ? "grab" : "";
    }
  };
  const onUp = (e) => {
    if (!grab && !carry && !arrasto && !mover.alvo) return;
    arrasto = null;
    mover.alvo = null;
    if (carry) {
      // Larga o pato em pé no chão, onde o visitante soltou.
      const q = data.qpos;
      q[0] = carry.target[0]; q[1] = carry.target[1]; q[2] = 0.13;
      carry = null;
    }
    endGrab();
    controls.enabled = true;
    canvas.style.cursor = "";
    if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
  };
  canvas.addEventListener("pointerdown", onDown, true);
  canvas.addEventListener("pointermove", onMove);
  canvas.addEventListener("pointerup", onUp);
  canvas.addEventListener("pointercancel", onUp);
  cleanups.push(() => {
    canvas.removeEventListener("pointerdown", onDown, true);
    canvas.removeEventListener("pointermove", onMove);
    canvas.removeEventListener("pointerup", onUp);
    canvas.removeEventListener("pointercancel", onUp);
  });

  // ── Teclado ──────────────────────────────────────────────────────────
  const KEYMAP = {
    ArrowUp: "fwd", KeyW: "fwd", ArrowDown: "back", KeyS: "back",
    ArrowLeft: "left", KeyA: "left", ArrowRight: "right", KeyD: "right",
  };
  const onKeyDown = (e) => {
    const k = KEYMAP[e.code];
    if (k) { e.preventDefault(); keys.add(k); return; }
    if (e.code === "Space" && !e.repeat) {
      e.preventDefault();
      // Chuta na hora, perto ou longe, parado ou andando (e às vezes cai).
      if (mode === "controle") triggerKick(ballLocal()[1] >= 0 ? "left" : "right");
    }
    if (e.code === "KeyR" && !e.repeat) resetScene();
  };
  const onKeyUp = (e) => { const k = KEYMAP[e.code]; if (k) keys.delete(k); };
  const onBlur = () => keys.clear();
  window.addEventListener("keydown", onKeyDown);
  window.addEventListener("keyup", onKeyUp);
  window.addEventListener("blur", onBlur);
  cleanups.push(() => {
    window.removeEventListener("keydown", onKeyDown);
    window.removeEventListener("keyup", onKeyUp);
    window.removeEventListener("blur", onBlur);
  });

  // ── Render ───────────────────────────────────────────────────────────
  function resize() {
    const w = canvas.clientWidth, h = canvas.clientHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / Math.max(1, h);
    camera.updateProjectionMatrix();
  }
  resize();
  const ro = new ResizeObserver(resize);
  ro.observe(canvas);
  cleanups.push(() => ro.disconnect());

  // Render principal através da Percepcao, seguindo o tronco do pato (o +x
  // local do tronco é a frente).
  function drawMain() {
    percepcao.seguir(trunk);
    percepcao.render(renderer, scene, camera);
  }

  // Enquadramento por modo (regra da thread de design): alto pro mapa e pra
  // grade, perto dos pés pro equilíbrio.
  function enquadramento(m) {
    const q = data.qpos, [dx, dz] = [q[0], -q[1]];
    switch (m) {
      case "odometria": case "laser": case "jogar":
        return { p: [0.05, 2.6, 2.3], a: [0.1, 0, -0.35], segue: false };
      case "decidir":
        return { p: [0.1, 2.0, 1.55], a: [0.15, 0, -0.06], segue: false };
      case "mover":
        return { p: [dx - 0.45, 0.32, dz + 0.5], a: [dx, 0.08, dz], segue: true };
      case "aprender":
        return { p: [0.05, 1.05, 1.35], a: [0.0, 0.08, -0.06], segue: false };
      default:
        return { p: cena.camera.posicao.toArray(), a: cena.camera.alvo.toArray(), segue: true };
    }
  }
  let voo = null;
  function voarPara(m) {
    const e = enquadramento(m);
    voo = {
      t0: performance.now(),
      p0: camera.position.clone(), a0: controls.target.clone(),
      p1: new THREE.Vector3(...e.p), a1: new THREE.Vector3(...e.a),
    };
  }
  controls.addEventListener("start", () => { voo = null; });

  const _follow = new THREE.Vector3();
  const _goalTarget = new THREE.Vector3();
  let raf = 0;
  function frame() {
    raf = requestAnimationFrame(frame);
    syncScene();
    // Voo de câmera entre enquadramentos (1,4 s, smoothstep); o visitante
    // girando a câmera cancela.
    if (voo) {
      const k = Math.min(1, (performance.now() - voo.t0) / 1400), e = k * k * (3 - 2 * k);
      camera.position.lerpVectors(voo.p0, voo.p1, e);
      controls.target.lerpVectors(voo.a0, voo.a1, e);
      if (k >= 1) voo = null;
    }
    // Câmera segue devagar o meio do caminho entre o pato e a bola nos
    // enquadramentos que seguem (nos altos, mapa e grade, fica parada).
    if (!voo && !grab && !carry && enquadramento(mode).segue) {
      const q = data.qpos;
      // Sem bola em jogo (mover, aprender), segue só o pato.
      const semBola = mode === "mover" || mode === "aprender";
      const bx = semBola ? q[0] : q[ballQ], by = semBola ? q[1] : q[ballQ + 1];
      _goalTarget.set((q[0] * 2 + bx) / 3, controls.target.y, -(q[1] * 2 + by) / 3);
      _follow.copy(_goalTarget).sub(controls.target).multiplyScalar(0.02);
      controls.target.add(_follow);
      camera.position.add(_follow);
    }
    controls.update();
    drawMain();
    if (usesVision()) renderVision();
  }
  frame();
  progress(null);

  // Gancho de depuração no console, só em desenvolvimento (window.__pato).
  const debugHandle = {
    get qpos() { return Array.from(data.qpos.slice(0, 3)).concat(Array.from(data.qpos.slice(ballQ, ballQ + 3))); },
    get det() { return { ...det }; },
    get mode() { return mode; },
    get cmd() { return Array.from(cmd); },
    get yaw() { return yawOf(data.qpos); },
    get simTime() { return simTime; },
    get goals() { return goals; },
    get twist() { return Array.from(twist); },
    get vision() { return vision; },
    set headDown(v) { HEAD_DOWN = v; },
    set hold(v) { hold = v; },
    set twist(v) { twistOverride = v; },
    kick(foot) { return triggerKick(foot); },
    requestKick,
    get windup() { return windup; },
    footFwd(foot) { const x = data.xpos, a = ankleIds[foot] * 3, q = data.qpos, y = yawOf(q); return Math.cos(y) * (x[a] - q[0]) + Math.sin(y) * (x[a + 1] - q[1]); },
    get kickStep() { return kickRun?.steps ?? -1; },
    footDist(foot) { const x = data.xpos, a = ankleIds[foot] * 3, q = data.qpos; return Math.hypot(x[a] - q[ballQ], x[a + 1] - q[ballQ + 1], x[a + 2] - q[ballQ + 2]); },
    reset: resetSim,
    placeInFront(d, lat = 0) {
      const q = data.qpos, y = yawOf(q);
      placeBall(q[0] + Math.cos(y) * d - Math.sin(y) * lat, q[1] + Math.sin(y) * d + Math.cos(y) * lat);
    },
    // Pra testar com a aba em segundo plano (sem requestAnimationFrame).
    tick() { syncScene(); drawMain(); if (usesVision()) renderVision(); },
    get odo() { return { ...odo }; },
    get est() { return est && { ...est }; },
    get pfStats() { return pfStats; },
    get ballEst() { return { ...ballEst }; },
    get nav() { return { ...nav }; },
    get mapN() { return mapN; },
    carryTo(x, y) { carry = { target: [x, y] }; },
    setPose(x, y, yaw) { const q = data.qpos; q[0] = x; q[1] = y; q[3] = Math.cos(yaw / 2); q[4] = 0; q[5] = 0; q[6] = Math.sin(yaw / 2); mujoco.mj_forward(model, data); },
    maskStats() {
      const px = vision.pixels, out = {};
      for (let k = 0; k < VISION_W * VISION_H; k++) {
        const i = k * 4, r = aces(half(px[i])), g = aces(half(px[i + 1])), b = aces(half(px[i + 2]));
        if (!isBallPixel(r, g, b)) continue;
        const mx = Math.max(r, g, b), mn = Math.min(r, g, b), d = mx - mn;
        let h = mx === r ? ((g - b) / d) % 6 : mx === g ? (b - r) / d + 2 : (r - g) / d + 4; h *= 60; if (h < 0) h += 360;
        const key = `${Math.round(h / 4) * 4}|${(d / mx).toFixed(1)}`;
        out[key] = (out[key] || 0) + 1;
      }
      return out;
    },
    drop() { const q = data.qpos; if (carry) { q[0] = carry.target[0]; q[1] = carry.target[1]; q[2] = 0.13; } carry = null; },
    rayTest() { return lidarScan(); },
    checkTruth() {
      const { beams, sensor } = lidarScan();
      const q = data.qpos, t = yawOf(q), c = Math.cos(t), sn = Math.sin(t);
      const sx = q[0] + c * sensor.ox - sn * sensor.oy, sy = q[1] + sn * sensor.ox + c * sensor.oy;
      return beams.map((b) => {
        const a = t + sensor.oth + b.a;
        return [+b.a.toFixed(2), +b.r.toFixed(2), +rayBoxes(MAP_BOXES, sx, sy, Math.cos(a), Math.sin(a), LIDAR_MAX).toFixed(2)];
      });
    },
    get mapBoxes() { return MAP_BOXES; },
    empurrar(dx, dy) { const q = data.qpos; mover.alvo = dx === null ? null : [q[0] + dx, q[1] + dy]; },
    setEquilibrio,
    moverStatus() { return { equilibrio: mover.equilibrio, quedas: { ...mover.quedas }, pronto: gravador.tamanho >= 120, ciclo: mover.ciclo?.length ?? 0 }; },
    get marcasN() { return [marcas.n, raioX.n, marcas.mesh.count, marcas.mesh.visible]; },
    get plano() { return { ...plano, caminho: plano.caminho && plano.caminho.length }; },
    get gradeOcupadas() { let n = 0; for (const v of grade.ocup) if (v >= 0.5) n++; return n; },
    moverObstaculo(i, x, y, yaw) { setObstaculo(i, x, y, yaw ?? obst[i].yaw); },
    get obst() { return obst.map((o, i) => ({ nome: o.nome, x: o.x, y: o.y, gx: data.geom_xpos[obstIds[i] * 3], gy: data.geom_xpos[obstIds[i] * 3 + 1], gz: data.geom_xpos[obstIds[i] * 3 + 2] })); },
    rayGeomTest(x, y, z, dx, dy, dz) {
      const gid = new Int32Array([-7]); const arr = [-7];
      const d1 = mujoco.mj_ray(model, data, [x, y, z], [dx, dy, dz], new Uint8Array([1, 0, 1, 0, 0, 0]), true, -1, gid, new Float64Array(3));
      let d2 = null, err = null;
      try { d2 = mujoco.mj_ray(model, data, [x, y, z], [dx, dy, dz], [1, 0, 1, 0, 0, 0], true, -1, arr, null); } catch (e) { err = String(e); }
      return { d1, gid: gid[0], d2, arr, err };
    },
    testeChao() { let n = 0, b = 0, f = 0; laserChao((hx, hy, bateu) => { n++; if (bateu) b++; else f++; }); return { n, b, f, floorId, ids: obstIds }; },
    get policy() { return policy; },
    get recovery() { return recovery; },
    camera, controls, placeBall, headCam,
    get camRaw() { return { p: Array.from(data.cam_xpos.slice(headCamId * 3, headCamId * 3 + 3)), m: Array.from(data.cam_xmat.slice(headCamId * 9, headCamId * 9 + 9)) }; },
    kickBall(vx, vy) { data.qvel[ballV] = vx; data.qvel[ballV + 1] = vy; },
  };
  if (process.env.NODE_ENV !== "production") window.__pato = debugHandle;

  const api = {
    setMode(m) {
      const prev = mode;
      mode = m;
      keys.clear();
      twist.fill(0);
      if (!canGrabBall()) endGrab();
      if (!usesLaser()) carry = null;
      if (m === "solto" || m === "visao") {
        wander.until = 0; wander.nextKick = simTime + 1.5;
        if (ballRespawnAt === null) ballRespawnAt = simTime + 0.4;
      }
      if (m === "gancho") resetScene();
      // No jogar o pato continua de onde estava (sem teletransporte, que o
      // filtro trataria como sequestro): só entram o livro e a bola.
      if (m === "jogar") layoutForMode();
      // Odometria começa do ponto de partida, de frente pra sala.
      if (m === "odometria") resetScene();
      if (m === "laser" && prev !== "jogar") {
        // Sem palpite: partículas espalhadas pela sala inteira, mapa vazio.
        pf.scatter();
        est = null;
        mapN = 0; mapCells.clear();
        layoutForMode();
      }
      if (m === "jogar") { nav.stage = "ir"; ballEst.at = -1e9; }
      if (m === "mover") {
        resetScene();
        mover.equilibrio = true; mover.ciclo = null; mover.quedas = { com: 0, sem: 0 }; mover.alvo = null;
        gravador.limpar();
      }
      if (m === "aprender") {
        resetScene();
        iniciarAprender();
        carregarGeracoes().then(() => { if (mode === "aprender" && aprender.sessoes) { resetScene(); iniciarAprender(); } });
      }
      if (m === "decidir") {
        resetScene();
        pf.scatter(); est = null;
        grade.ocup.fill(0); grade.versao++;
        plano.caminho = null; plano.em = -1e9; plano.olhando = null;
        nav.stage = "ir"; ballEst.at = -1e9;
      }
      voarPara(m);
      // Com o laser ligado, o pato "percebe" a sala inteira: a área nítida
      // da Percepcao cresce (decisão nossa, não do arquivo de design).
      const r = percepcao.matPerc.uniforms.uRaio.value;
      if (usesLaser()) { r.x = 3.2; r.y = 5.5; } else { r.x = 1.0; r.y = 3.0; }
    },
    moverStatus() { return { equilibrio: mover.equilibrio, quedas: { ...mover.quedas }, pronto: gravador.tamanho >= 120 }; },
    // Celular: joystick (x = girar, y = andar, de -1 a 1) e botão de chute.
    setToque(x, y) { toque.x = x; toque.y = y; },
    chutar() { if (mode === "controle") triggerKick(ballLocal()[1] >= 0 ? "left" : "right"); },
    setEquilibrio,
    aprenderStatus() {
      return {
        geracao: aprender.geracao, geracoes: GERACOES, sigma: aprender.sigma, quedas: [...aprender.quedas],
        fim: aprender.fim, progresso: Math.min(1, (simTime - aprender.inicio) / GERACAO_S),
        alcance: [...aprender.alcance], distancia: Math.hypot(aprender.alvo[0] - aprender.ini[0], aprender.alvo[1] - aprender.ini[1]),
        real: !!aprender.sessoes, iteracao: aprender.iteracoes?.[aprender.geracao] ?? null, origem: aprender.origem ?? "",
      };
    },
    // Placar do modo atual ("posição encontrada", "N casas testadas"...).
    placar() { return placar.texto; },
    // Textos da interface: embaixo deles a sala vira papel (Percepcao).
    reservar(elementos) { percepcao.reservar(elementos); },
    setVisionLayer(t) {
      const v = ensureVision();
      if (v) v.mat.uniforms.layer.value = t;
    },
    reset: resetSim,
    dispose() {
      disposed = true;
      running = false;
      if (window.__pato === debugHandle) delete window.__pato;
      cancelAnimationFrame(raf);
      for (const c of cleanups) c();
      controls.dispose();
      percepcao.dispose();
      renderer.dispose();
      if (vision) { vision.vr.dispose(); vision.rt.dispose(); }
      try { data.delete(); model.delete(); vfs.delete(); } catch {}
    },
  };
  debugHandle.setMode = (m) => api.setMode(m);
  return api;
}
