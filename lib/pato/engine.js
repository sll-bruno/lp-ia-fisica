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
import { ParticleFilter } from "./localize.js";

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
const BALL_RADIUS = 0.05;

const POLICIES = {
  walk: "/pato/policies/BEST_alpha_walking.onnx",
  kickL: "/pato/policies/ball_kick_left.onnx",
  kickR: "/pato/policies/ball_kick_right.onnx",
  stand: "/pato/policies/BEST_alpha_stand.onnx",
};

// ── Arena e gol (nossos) ───────────────────────────────────────────────
// Coordenadas MJCF: x pra frente (rumo ao gol), y pra esquerda, z pra cima.
export const ARENA_HALF = 1.0;
// Paredes e gol mais altos que o sensor da cabeça (~0,22 m), pra que o laser
// do modo localizar enxergue a sala.
const WALL_H = 0.3, WALL_T = 0.03;
const GOAL_DEPTH = 0.22;
const GOAL_X = ARENA_HALF - GOAL_DEPTH; // linha do gol
const GOAL_HW = 0.28; // meia largura da boca
const GOAL_H = 0.28;
const POST = 0.012;
const SPAWN = [-0.45, 0];
// Bola um pouco além do alcance do pé: o primeiro chute pega no ar.
const BALL_START = [SPAWN[0] + 0.3, 0.02];

// Obstáculo do modo localizar ("um livro" no chão entre a bola e o gol):
// baixo demais pro laser, fora do mapa. Fica estacionado longe nos outros
// modos.
const BOOK = { pos: [0.42, 0.0], half: [0.08, 0.12, 0.018] };
const PARK = [50, 50];
// Caixa alta num canto: quebra a simetria da sala quadrada. Sem ela, o
// filtro às vezes se fixava na pose "espelhada" (girada 180°), igualmente
// compatível com as paredes; o gol sozinho era um sinal fraco demais.
const CRATE = { pos: [-0.82, 0.82], half: [0.09, 0.09, 0.17] };

// Laser (LiDAR 2D) saindo do sensor "tof" da cabeça do pato.
const LIDAR_RAYS = 90;
const LIDAR_MAX = 3;
const LIDAR_EVERY = 5; // passos de controle entre varreduras (10 Hz)

// Cor da bola: um magenta que não aparece no pato (laranja/amarelo/creme),
// pra máscara de cor separar bem.
const BALL_COLOR = "#d6336c";

// Visão da cabeça: resolução baixa de propósito (é o que a gente mostra).
export const VISION_W = 128, VISION_H = 96;

function yawOf(qpos) {
  return Math.atan2(
    2 * (qpos[3] * qpos[6] + qpos[4] * qpos[5]),
    1 - 2 * (qpos[5] * qpos[5] + qpos[6] * qpos[6]),
  );
}
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const wrapA = (a) => Math.atan2(Math.sin(a), Math.cos(a));

// ── Detecção de bola na CPU: mesmo critério da máscara do shader ───────
// Pixels lineares da render target -> sRGB -> HSV; bola = matiz magenta,
// saturação e brilho mínimos.
const SRGB_LUT = new Float32Array(256);
for (let i = 0; i < 256; i++) {
  const c = i / 255;
  SRGB_LUT[i] = c <= 0.0031308 ? c * 12.92 : 1.055 * Math.pow(c, 1 / 2.4) - 0.055;
}
function isBallPixel(r, g, b) {
  const max = Math.max(r, g, b), min = Math.min(r, g, b);
  const v = max, d = max - min;
  if (v < 0.18) return false;
  const s = max > 0 ? d / max : 0;
  if (s < 0.45) return false;
  let h;
  if (max === r) h = ((g - b) / d) % 6;
  else if (max === g) h = (b - r) / d + 2;
  else h = (r - g) / d + 4;
  h *= 60;
  if (h < 0) h += 360;
  return h > 300 || h < 2;
}

const VISION_FRAG = /* glsl */ `
  uniform sampler2D tImg;
  uniform vec2 texel;
  uniform float layer;
  uniform vec4 box; // x0, y0, x1, y1 em uv
  uniform float hasBox;
  varying vec2 vUv;
  vec3 toSrgb(vec3 c) {
    return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c));
  }
  vec3 px(vec2 o) { return toSrgb(texture2D(tImg, vUv + o * texel).rgb); }
  float lum(vec3 c) { return dot(c, vec3(0.299, 0.587, 0.114)); }
  float isBall(vec3 c) {
    float mx = max(c.r, max(c.g, c.b));
    float mn = min(c.r, min(c.g, c.b));
    float d = mx - mn;
    if (mx < 0.18) return 0.0;
    if (d / max(mx, 1e-5) < 0.45) return 0.0;
    float h;
    if (mx == c.r) h = mod((c.g - c.b) / d, 6.0);
    else if (mx == c.g) h = (c.b - c.r) / d + 2.0;
    else h = (c.r - c.g) / d + 4.0;
    h *= 60.0;
    if (h < 0.0) h += 360.0;
    return (h > 300.0 || h < 2.0) ? 1.0 : 0.0;
  }
  void main() {
    vec3 c = px(vec2(0.0));
    // Sobel 3x3 sobre a luminância
    float tl = lum(px(vec2(-1.0, 1.0))), t = lum(px(vec2(0.0, 1.0))), tr = lum(px(vec2(1.0, 1.0)));
    float l = lum(px(vec2(-1.0, 0.0))), r = lum(px(vec2(1.0, 0.0)));
    float bl = lum(px(vec2(-1.0, -1.0))), b = lum(px(vec2(0.0, -1.0))), br = lum(px(vec2(1.0, -1.0)));
    float gx = (tr + 2.0 * r + br) - (tl + 2.0 * l + bl);
    float gy = (tl + 2.0 * t + tr) - (bl + 2.0 * b + br);
    float e = clamp(length(vec2(gx, gy)) * 1.6, 0.0, 1.0);
    vec3 edges = vec3(e);
    float m = isBall(c);
    vec3 mask = mix(vec3(e * 0.18), vec3(1.0), m);
    vec3 col = layer < 1.0 ? mix(c, edges, layer) : mix(edges, mask, layer - 1.0);
    if (hasBox > 0.5 && layer > 1.0) {
      vec2 p = vUv;
      vec2 th = texel * 1.0;
      bool inside = p.x > box.x - th.x && p.x < box.z + th.x && p.y > box.y - th.y && p.y < box.w + th.y;
      bool core = p.x > box.x + th.x && p.x < box.z - th.x && p.y > box.y + th.y && p.y < box.w - th.y;
      if (inside && !core) col = mix(col, vec3(1.0, 0.82, 0.2), clamp(layer - 1.0, 0.0, 1.0));
    }
    gl_FragColor = vec4(col, 1.0);
  }
`;

export async function createPato({ canvas, visionCanvas, onProgress, onGoal }) {
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
    const src = await (await fetch(`${MODEL_DIR}/robot_allcollisions.xml`)).text();
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
    const box = (name, pos, size) => world.appendChild(el("geom", {
      name, type: "box", pos: pos.join(" "), size: size.join(" "),
    }));
    root.appendChild(el("option", { timestep: String(TIMESTEP) }));
    world.appendChild(el("geom", { name: "floor", type: "plane", size: "0 0 0.05", pos: "0 0 0" }));
    const off = ARENA_HALF + WALL_T / 2, span = ARENA_HALF + WALL_T;
    box("wall_px", [off, 0, WALL_H / 2], [WALL_T / 2, span, WALL_H / 2]);
    box("wall_nx", [-off, 0, WALL_H / 2], [WALL_T / 2, span, WALL_H / 2]);
    box("wall_py", [0, off, WALL_H / 2], [span, WALL_T / 2, WALL_H / 2]);
    box("wall_ny", [0, -off, WALL_H / 2], [span, WALL_T / 2, WALL_H / 2]);
    // Gol encostado na parede +x: traves, travessão, laterais e teto.
    const gy = GOAL_HW + POST;
    const cx = GOAL_X + GOAL_DEPTH / 2;
    box("post_l", [GOAL_X, gy, GOAL_H / 2], [POST, POST, GOAL_H / 2]);
    box("post_r", [GOAL_X, -gy, GOAL_H / 2], [POST, POST, GOAL_H / 2]);
    box("crossbar", [GOAL_X, 0, GOAL_H], [POST, gy + POST, POST]);
    box("goal_side_l", [cx, gy, GOAL_H / 2], [GOAL_DEPTH / 2, 0.004, GOAL_H / 2]);
    box("goal_side_r", [cx, -gy, GOAL_H / 2], [GOAL_DEPTH / 2, 0.004, GOAL_H / 2]);
    box("goal_roof", [cx, 0, GOAL_H + 0.004], [GOAL_DEPTH / 2, gy, 0.004]);
    box("crate", [CRATE.pos[0], CRATE.pos[1], CRATE.half[2]], CRATE.half);
    // Livro: geom estático, estacionado longe até o modo localizar.
    world.appendChild(el("geom", {
      name: "book", type: "box", pos: `${PARK[0]} ${PARK[1]} ${BOOK.half[2]}`,
      size: BOOK.half.join(" "), group: "2",
    }));
    // Bola leve (sensação de bola de praia), igual ao simulador oficial.
    const ball = el("body", { name: "ball", pos: `${BALL_START[0]} ${BALL_START[1]} ${BALL_RADIUS}` });
    ball.appendChild(el("freejoint", { name: "ball_freejoint" }));
    ball.appendChild(el("geom", {
      name: "ball_geom", type: "sphere", size: String(BALL_RADIUS),
      mass: "0.03", friction: "0.4 0.01 0.003", solref: "0.03 0.4", condim: "6",
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
      qpos: `${SPAWN[0]} ${SPAWN[1]} 0.12 1 0 0 0 ${qj} ${BALL_START[0]} ${BALL_START[1]} ${BALL_RADIUS} 1 0 0 0`,
      ctrl: Array.from(DEFAULT_POSE).join(" "),
    }));
    root.appendChild(kf);
    const meshFiles = [...doc.querySelectorAll("asset > mesh")].map((m) => m.getAttribute("file"));
    return { xml: new XMLSerializer().serializeToString(doc), meshFiles };
  }

  const [mujoco, { xml, meshFiles }, kin] = await Promise.all([
    loadMujoco({ locateFile: (p) => (p.endsWith(".wasm") ? "/pato/wasm/mujoco.wasm" : p) }),
    buildPhysicsXml(),
    loadKinematics(),
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
    ort.InferenceSession.create(POLICIES.walk, opts),
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

  // Mapa conhecido pro filtro de partículas: as mesmas caixas da física que
  // ficam na altura do laser (paredes, traves e laterais do gol).
  const MAP_BOXES = (() => {
    const off = ARENA_HALF + WALL_T / 2, span = ARENA_HALF + WALL_T;
    const gy = GOAL_HW + POST, cx = GOAL_X + GOAL_DEPTH / 2;
    return [
      { cx: off, cy: 0, hx: WALL_T / 2, hy: span },
      { cx: -off, cy: 0, hx: WALL_T / 2, hy: span },
      { cx: 0, cy: off, hx: span, hy: WALL_T / 2 },
      { cx: 0, cy: -off, hx: span, hy: WALL_T / 2 },
      { cx: GOAL_X, cy: gy, hx: POST, hy: POST },
      { cx: GOAL_X, cy: -gy, hx: POST, hy: POST },
      { cx, cy: gy, hx: GOAL_DEPTH / 2, hy: 0.004 },
      { cx, cy: -gy, hx: GOAL_DEPTH / 2, hy: 0.004 },
      { cx: CRATE.pos[0], cy: CRATE.pos[1], hx: CRATE.half[0], hy: CRATE.half[1] },
    ];
  })();
  const pf = new ParticleFilter({
    boxes: MAP_BOXES,
    bounds: { xmin: -ARENA_HALF + 0.08, xmax: GOAL_X - 0.05, ymin: -ARENA_HALF + 0.08, ymax: ARENA_HALF - 0.08 },
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
    if (mode === "jogar") return [-0.3 + Math.random() * 0.3, -0.2 + Math.random() * 0.4];
    // Longe do pato e fora do gol.
    const qpos = data.qpos;
    for (let i = 0; i < 20; i++) {
      const x = -0.75 + Math.random() * 1.3;
      const y = -0.7 + Math.random() * 1.4;
      if (Math.hypot(x - qpos[0], y - qpos[1]) > 0.35) return [x, y];
    }
    return [0, 0];
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

  const activeSession = () => (recovery?.state === "recovering" ? sessions.stand : sessions[policy]);

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
  function keyboardTwist() {
    let vx = 0, wz = 0;
    if (keys.has("fwd")) vx += VEL_FWD;
    if (keys.has("back")) vx += VEL_BACK;
    const turn = vx === 0 ? TURN_INPLACE : TURN_WALKING;
    if (keys.has("left")) wz += turn;
    if (keys.has("right")) wz -= turn;
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
    if (Math.max(Math.abs(q[0]), Math.abs(q[1])) > ARENA_HALF - 0.2) wander.wz = Math.sign(wander.wz || 1) * VEL_ANG;
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
      // quanto mais baixo o topo da bola, mais perto ela está (calibrado:
      // topo em 0,8 ~ 13 cm do tronco, perto do alcance do pé).
      if (det.y0 > 0.8 && Math.abs(ex) < 0.14) {
        // Para, se firma e chuta (o pé sai do lado da bola na imagem).
        if (requestKick(ex < 0 ? "left" : "right")) seek.kickedAt = t;
        return;
      }
      if (Math.abs(ex) > 0.15) {
        twist[0] = SEEK_VX; twist[2] = VEL_ANG * seek.dir;
      } else {
        twist[0] = det.y0 > 0.45 ? 0.2 : VEL_FWD;
        twist[2] = clamp(-2.6 * ex, -VEL_ANG, VEL_ANG);
      }
    } else if (t - lastSeen.at < 0.6 && lastSeen.top > 0.72 && Math.abs(lastSeen.cx - 0.5) < 0.2) {
      // Sumiu por baixo da imagem: está nos pés. Chuta.
      if (requestKick(lastSeen.cx < 0.5 ? "left" : "right")) seek.kickedAt = t;
      return;
    } else {
      // Procura: arco curto pro lado em que a bola foi vista por último.
      // Encostado numa parede, o arco vira marcha à ré.
      const q = data.qpos;
      const nearWall = Math.max(Math.abs(q[0]), Math.abs(q[1])) > ARENA_HALF - 0.16;
      twist[0] = nearWall ? VEL_BACK : SEEK_VX;
      twist[2] = VEL_ANG * seek.dir;
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
    if (t < stall.boostUntil) twist[0] = VEL_FWD;
  }

  // ── Localizar ────────────────────────────────────────────────────────
  const usesVision = () => mode === "visao" || mode === "gancho" || mode === "jogar";
  const usesLaser = () => mode === "laser" || mode === "jogar";
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
  // (só geoms do grupo 0: paredes, gol, chão; o pato é grupo 3, a bola 1, o
  // livro 2). Os pontos de impacto viram o mapa desenhado no chão.
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
  let mapN = 0, mapDirty = false;
  const rayGroups = new Uint8Array([1, 0, 0, 0, 0, 0]);
  const rayGeom = new Int32Array(1);
  const rayNormal = new Float64Array(3);
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
      if (hit && mapN < MAP_MAX) {
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
    }
    scan.ox = ox; scan.oy = oy; scan.oz = oz; scan.n++;
    mapDirty = true;
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
  function updateBallEstimate(t) {
    if (!est || !det.seen || t - det.at > 0.25) return;
    // Distância pela largura aparente da bola (10 cm), corrigida pela altura
    // da câmera; rumo pela posição na imagem. Bola cortada embaixo não serve.
    const w = (det.x1 - det.x0) * HFOV;
    if (det.y1 >= 0.98 || w <= 0) return;
    const d3 = BALL_RADIUS / Math.tan(w / 2);
    const dh = Math.sqrt(Math.max(0, d3 * d3 - 0.17 * 0.17)) + 0.07;
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
    const gx = GOAL_X + 0.05;
    let ux = gx - ballEst.x, uy = -ballEst.y;
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
    if (fresh && det.y0 > 0.8 && Math.abs(det.cx - 0.5) < 0.14) {
      if (requestKick(det.cx < 0.5 ? "left" : "right")) { seek.kickedAt = t; nav.stage = "ir"; }
      return;
    }
    const e = wrapA(goalDir - est.t);
    twist[0] = 0.2;
    twist[2] = fresh ? clamp(-2.6 * (det.cx - 0.5), -VEL_ANG, VEL_ANG) : clamp(2 * e, -VEL_ANG, VEL_ANG);
    if (t - nav.since > 6) nav.stage = "ir";
    stallCheck(t);
  }

  // Disposição da cena por modo: livro e pose inicial.
  function layoutForMode() {
    const book = mode === "jogar";
    model.geom_pos[bookId * 3] = book ? BOOK.pos[0] : PARK[0];
    model.geom_pos[bookId * 3 + 1] = book ? BOOK.pos[1] : PARK[1];
    const q = data.qpos;
    if (mode === "gancho") {
      // De costas pro gol: com só a visão, ele chuta pra parede.
      q[3] = 0; q[4] = 0; q[5] = 0; q[6] = 1;
      placeBall(SPAWN[0] - 0.3, 0.06);
    } else if (mode === "jogar") {
      // A bola atrás do livro: o caminho até o gol passa por cima dele.
      const far = Math.hypot(q[0] + 0.05, q[1] - 0.1) > 0.3;
      placeBall(far ? -0.05 : 0.15, far ? 0.1 : -0.15);
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
  let twistOverride = null; // depuração
  async function controlStep() {
    simTime += CTRL_DT;
    if (twistOverride) { twist[0] = twistOverride[0]; twist[2] = twistOverride[1]; }
    else if (mode === "controle") keyboardTwist();
    else if (mode === "solto") wanderTwist(simTime);
    else if (mode === "odometria" || mode === "laser") keyboardTwist();
    else if (mode === "jogar") locTwist(simTime);
    else visionTwist(simTime);
    // Na visão, a cabeça olha um pouco pra baixo pra enxergar o chão à frente.
    // Durante o preparo e o chute a cabeça volta ao neutro: com ela
    // inclinada o chute direito derrubava o pato (2 de 2 nos testes).
    const kicking = windup || policy !== "walk" || postKickLock > 0;
    headTarget[1] = usesVision() && !kicking ? HEAD_DOWN : 0;

    if (recovery?.state !== "fallen") {
      const session = activeSession();
      const out = await session.run({
        [session.inputNames[0]]: new ort.Tensor("float32", buildObs(), [1, OBS_SIZE]),
      });
      if (disposed) return;
      const act = out[session.outputNames[0]].data;
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
      if (policy === "walk" && postKickLock === 0) {
        if (++fallDebounce >= 10) { fallDebounce = 0; recovery = { state: "fallen", steps: 0 }; }
      } else {
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
    else if (bx > GOAL_X + BALL_RADIUS * 0.6 && Math.abs(by) < GOAL_HW && !grab) {
      // Uns 4 s depois do gol a cena reseta: pato e bola voltam ao início.
      goalCooldown = GOAL_RESET_S + 1;
      sceneResetAt = simTime + GOAL_RESET_S;
      ballRespawnAt = null;
      goals++;
      onGoal?.(mode);
    }
    if (Math.abs(bx) > ARENA_HALF + 0.2 || Math.abs(by) > ARENA_HALF + 0.2 || !Number.isFinite(bx)) {
      ballRespawnAt = simTime;
    }
    // Bola parada colada na parede por 6 s (fora do modo controle): o pato
    // sozinho não sabe tirá-la do canto, então ela volta pro campo.
    const atWall = Math.max(Math.abs(bx), Math.abs(by)) > ARENA_HALF - 0.12;
    const slow = Math.hypot(data.qvel[ballV], data.qvel[ballV + 1]) < 0.05;
    if (mode !== "controle" && atWall && slow && !grab) {
      ballStuckFor += CTRL_DT;
      if (ballStuckFor > 6 && ballRespawnAt === null) ballRespawnAt = simTime;
    } else ballStuckFor = 0;
    if (sceneResetAt !== null && simTime >= sceneResetAt) resetScene();
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
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  const BG = new THREE.Color("#e8e6e1");
  const scene = new THREE.Scene();
  scene.background = BG;
  scene.fog = new THREE.Fog(BG, 4, 9);

  const camera = new THREE.PerspectiveCamera(38, 1, 0.02, 30);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.enablePan = false;
  controls.minDistance = 0.6;
  controls.maxDistance = 4;
  controls.maxPolarAngle = Math.PI * 0.46;
  camera.position.set(SPAWN[0] - 1.25, 1.05, 0.95);
  controls.target.set(SPAWN[0] + 0.5, 0.05, 0);

  scene.add(new THREE.HemisphereLight(0xffffff, 0xb9b4aa, 1.4));
  const sun = new THREE.DirectionalLight(0xffffff, 2.2);
  sun.position.set(-1.2, 3, 1.4);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -1.4, right: 1.4, top: 1.4, bottom: -1.4, near: 0.5, far: 7 });
  sun.shadow.bias = -0.0005;
  sun.shadow.normalBias = 0.01;
  scene.add(sun);

  // Tudo que vive em coordenadas MJCF fica sob este grupo (z pra cima).
  const mj = new THREE.Group();
  mj.rotation.x = -Math.PI / 2;
  scene.add(mj);

  const floorMat = new THREE.MeshStandardMaterial({ color: "#d9d6cf", roughness: 0.95 });
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(2 * ARENA_HALF, 2 * ARENA_HALF), floorMat);
  floor.receiveShadow = true;
  mj.add(floor);
  // Linhas finas no chão: meio-campo e área do gol.
  const lineMat = new THREE.LineBasicMaterial({ color: "#a9a59c" });
  const line = (pts) => {
    const g = new THREE.BufferGeometry().setFromPoints(pts.map(([x, y]) => new THREE.Vector3(x, y, 0.001)));
    mj.add(new THREE.Line(g, lineMat));
  };
  line([[0, -ARENA_HALF], [0, ARENA_HALF]]);
  line([[GOAL_X, -GOAL_HW], [GOAL_X, GOAL_HW]]);
  line([[GOAL_X, -GOAL_HW - 0.2], [GOAL_X - 0.25, -GOAL_HW - 0.2], [GOAL_X - 0.25, GOAL_HW + 0.2], [GOAL_X, GOAL_HW + 0.2]]);

  const wallMat = new THREE.MeshStandardMaterial({ color: "#f3f1ec", roughness: 0.8 });
  const addBox = (pos, size, mat, parent = mj) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(size[0] * 2, size[1] * 2, size[2] * 2), mat);
    m.position.set(...pos);
    m.castShadow = true;
    m.receiveShadow = true;
    parent.add(m);
    return m;
  };
  {
    const off = ARENA_HALF + WALL_T / 2, span = ARENA_HALF + WALL_T;
    addBox([off, 0, WALL_H / 2], [WALL_T / 2, span, WALL_H / 2], wallMat);
    addBox([-off, 0, WALL_H / 2], [WALL_T / 2, span, WALL_H / 2], wallMat);
    addBox([0, off, WALL_H / 2], [span, WALL_T / 2, WALL_H / 2], wallMat);
    addBox([0, -off, WALL_H / 2], [span, WALL_T / 2, WALL_H / 2], wallMat);
  }
  {
    const postMat = new THREE.MeshStandardMaterial({ color: "#2b2a28", roughness: 0.6 });
    const netMat = new THREE.MeshStandardMaterial({
      color: "#2b2a28", roughness: 0.9, transparent: true, opacity: 0.12, side: THREE.DoubleSide, depthWrite: false,
    });
    const gy = GOAL_HW + POST, cx = GOAL_X + GOAL_DEPTH / 2;
    addBox([GOAL_X, gy, GOAL_H / 2], [POST, POST, GOAL_H / 2], postMat);
    addBox([GOAL_X, -gy, GOAL_H / 2], [POST, POST, GOAL_H / 2], postMat);
    addBox([GOAL_X, 0, GOAL_H], [POST, gy + POST, POST], postMat);
    for (const m of [
      addBox([cx, gy, GOAL_H / 2], [GOAL_DEPTH / 2, 0.002, GOAL_H / 2], netMat),
      addBox([cx, -gy, GOAL_H / 2], [GOAL_DEPTH / 2, 0.002, GOAL_H / 2], netMat),
      addBox([cx, 0, GOAL_H + 0.004], [GOAL_DEPTH / 2, gy, 0.002], netMat),
      addBox([ARENA_HALF - 0.003, 0, GOAL_H / 2], [0.002, gy, GOAL_H / 2], netMat),
    ]) m.castShadow = false;
  }

  const ballMesh = new THREE.Mesh(
    new THREE.SphereGeometry(BALL_RADIUS, 40, 24),
    new THREE.MeshStandardMaterial({ color: BALL_COLOR, roughness: 0.55 }),
  );
  ballMesh.castShadow = true;
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

  // Mapa desenhado no chão: cada ponto é um impacto de raio do laser.
  const mapGeom = new THREE.BufferGeometry();
  const mapPos = new Float32Array(MAP_MAX * 3);
  mapGeom.setAttribute("position", new THREE.BufferAttribute(mapPos, 3));
  mapGeom.setDrawRange(0, 0);
  const mapCloud = new THREE.Points(mapGeom, new THREE.PointsMaterial({ color: "#3a3631", size: 0.02, sizeAttenuation: true }));
  mapCloud.frustumCulled = false;
  mj.add(mapCloud);
  // Partículas do filtro.
  const partGeom = new THREE.BufferGeometry();
  const partPos = new Float32Array(pf.n * 3);
  partGeom.setAttribute("position", new THREE.BufferAttribute(partPos, 3));
  const partCloud = new THREE.Points(partGeom, new THREE.PointsMaterial({ color: "#c2410c", size: 0.016, sizeAttenuation: true }));
  partCloud.frustumCulled = false;
  mj.add(partCloud);
  // Raios da varredura atual.
  const rayGeomVis = new THREE.BufferGeometry();
  const rayPos = new Float32Array(LIDAR_RAYS * 6);
  rayGeomVis.setAttribute("position", new THREE.BufferAttribute(rayPos, 3));
  const rayLines = new THREE.LineSegments(rayGeomVis, new THREE.LineBasicMaterial({ color: "#d6336c", transparent: true, opacity: 0.18 }));
  rayLines.frustumCulled = false;
  mj.add(rayLines);
  addBox([CRATE.pos[0], CRATE.pos[1], CRATE.half[2]], CRATE.half,
    new THREE.MeshStandardMaterial({ color: "#c9c3b8", roughness: 0.85 }));
  // Livro (o obstáculo).
  const bookMesh = addBox([PARK[0], PARK[1], BOOK.half[2]], BOOK.half,
    new THREE.MeshStandardMaterial({ color: "#6b4f3a", roughness: 0.8 }));

  // Tudo isso é desenho pro visitante, não existe pro pato: vai pra camada
  // 1, que a câmera da cabeça não renderiza (os raios magenta viravam
  // "bola" na máscara de cor).
  for (const o of [ghost.placer, mapCloud, partCloud, rayLines]) o.traverse((c) => c.layers.set(1));
  camera.layers.enable(1);

  const _qa = new THREE.Quaternion(), _qb = new THREE.Quaternion(), _zAxis = new THREE.Vector3(0, 0, 1);
  let lastScanDrawn = -1;
  function syncLocalization() {
    const laser = usesLaser();
    const q = data.qpos;
    bookMesh.visible = mode === "jogar";
    bookMesh.position.set(model.geom_pos[bookId * 3], model.geom_pos[bookId * 3 + 1], BOOK.half[2]);
    mapCloud.visible = laser;
    partCloud.visible = laser;
    rayLines.visible = laser;
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
    if (!laser) return;
    if (mapDirty) {
      for (let i = 0; i < mapN; i++) {
        mapPos[i * 3] = mapPts[i * 2]; mapPos[i * 3 + 1] = mapPts[i * 2 + 1]; mapPos[i * 3 + 2] = 0.003;
      }
      mapGeom.attributes.position.needsUpdate = true;
      mapGeom.setDrawRange(0, mapN);
      mapDirty = false;
    }
    for (let i = 0; i < pf.n; i++) {
      partPos[i * 3] = pf.x[i]; partPos[i * 3 + 1] = pf.y[i]; partPos[i * 3 + 2] = 0.005;
    }
    partGeom.attributes.position.needsUpdate = true;
    if (scan.n !== lastScanDrawn) {
      lastScanDrawn = scan.n;
      for (let k = 0; k < LIDAR_RAYS; k++) {
        const o = k * 6;
        rayPos[o] = scan.ox; rayPos[o + 1] = scan.oy; rayPos[o + 2] = scan.oz;
        rayPos[o + 3] = scan.hx[k]; rayPos[o + 4] = scan.hy[k]; rayPos[o + 5] = scan.oz;
      }
      rayGeomVis.attributes.position.needsUpdate = true;
    }
  }

  function syncScene() {
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
    const rt = new THREE.WebGLRenderTarget(VISION_W, VISION_H, { depthBuffer: true });
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
    vision = { vr, rt, quadScene, quadCam, mat, pixels: new Uint8Array(VISION_W * VISION_H * 4), frame: 0 };
    return vision;
  }

  function detectBall(px) {
    let n = 0, sx = 0, sy = 0, x0 = 1e9, y0 = 1e9, x1 = -1, y1 = -1;
    for (let y = 0; y < VISION_H; y++) {
      for (let x = 0; x < VISION_W; x++) {
        const i = (y * VISION_W + x) * 4;
        if (!isBallPixel(SRGB_LUT[px[i]], SRGB_LUT[px[i + 1]], SRGB_LUT[px[i + 2]])) continue;
        n++; sx += x; sy += y;
        if (x < x0) x0 = x; if (x > x1) x1 = x;
        if (y < y0) y0 = y; if (y > y1) y1 = y;
      }
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
    vision.mat.uniforms.box.value.set(x0 / VISION_W, y0 / VISION_H, (x1 + 1) / VISION_W, (y1 + 1) / VISION_H);
  }

  function renderVision() {
    const v = ensureVision();
    if (!v) return;
    syncHeadCam();
    v.vr.setRenderTarget(v.rt);
    v.vr.render(scene, headCam);
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
    const lim = ARENA_HALF - BALL_RADIUS - 0.02;
    return [clamp(hit.x, -lim, lim), clamp(-hit.z, -lim, lim), BALL_RADIUS + 0.02];
  };
  const ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const toGround = () => {
    if (!ray.ray.intersectPlane(ground, hit)) return null;
    const b = pf.bounds;
    return [clamp(hit.x, b.xmin, b.xmax), clamp(-hit.z, b.ymin, b.ymax)];
  };
  const canGrabBall = () => mode === "visao" || mode === "gancho" || mode === "jogar";
  const onDown = (e) => {
    setNdc(e);
    if (usesLaser() && ray.intersectObject(rig.placer, true).length) {
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
    if (carry) {
      const t = toGround();
      if (t) carry.target = t;
    } else if (grab) {
      const t = toTarget();
      if (t) grab.target = t;
    } else {
      const over = (usesLaser() && ray.intersectObject(rig.placer, true).length) ||
        (canGrabBall() && ray.intersectObject(ballMesh).length);
      canvas.style.cursor = over ? "grab" : "";
    }
  };
  const onUp = (e) => {
    if (!grab && !carry) return;
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

  const _follow = new THREE.Vector3();
  const _goalTarget = new THREE.Vector3();
  let raf = 0;
  function frame() {
    raf = requestAnimationFrame(frame);
    syncScene();
    // Câmera segue devagar o meio do caminho entre o pato e a bola (no
    // localizar, fica na vista geral da sala).
    if (!grab && !carry && !showsGhost()) {
      const q = data.qpos;
      _goalTarget.set((q[0] * 2 + q[ballQ]) / 3, 0.05, -(q[1] * 2 + q[ballQ + 1]) / 3);
      _follow.copy(_goalTarget).sub(controls.target).multiplyScalar(0.02);
      controls.target.add(_follow);
      camera.position.add(_follow);
    }
    controls.update();
    renderer.render(scene, camera);
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
    tick() { syncScene(); renderer.render(scene, camera); if (usesVision()) renderVision(); },
    get odo() { return { ...odo }; },
    get est() { return est && { ...est }; },
    get pfStats() { return pfStats; },
    get ballEst() { return { ...ballEst }; },
    get nav() { return { ...nav }; },
    get mapN() { return mapN; },
    carryTo(x, y) { carry = { target: [x, y] }; },
    drop() { const q = data.qpos; if (carry) { q[0] = carry.target[0]; q[1] = carry.target[1]; q[2] = 0.13; } carry = null; },
    rayTest() { return lidarScan(); },
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
        mapN = 0; mapCells.clear(); mapDirty = true;
        layoutForMode();
      }
      if (m === "jogar") { nav.stage = "ir"; ballEst.at = -1e9; }
      if (showsGhost()) {
        // Vista geral da sala pra ver o fantasma, o mapa e as partículas.
        controls.target.set(-0.05, 0, 0);
        camera.position.set(-1.55, 2.25, 1.35);
      }
    },
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
      renderer.dispose();
      if (vision) { vision.vr.dispose(); vision.rt.dispose(); }
      try { data.delete(); model.delete(); vfs.delete(); } catch {}
    },
  };
  debugHandle.setMode = (m) => api.setMode(m);
  return api;
}
