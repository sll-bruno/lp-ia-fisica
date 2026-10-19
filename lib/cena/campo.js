import * as THREE from 'three';
import { mat, texturaCanvas } from './util.js';

// Pitch layout, in meters, in the pitch group's own frame (goal at +x).
export const CAMPO = {
  posicao: new THREE.Vector2(0.24, -0.06),   // where the pitch sits on the room floor (x, z)
  meioComprimento: 0.8, meiaLargura: 0.5,
  area: { fundo: 0.26, meiaLargura: 0.34 }, pequenaArea: { fundo: 0.09, meiaLargura: 0.19 },
  marcaPenalti: 0.18, raioCirculo: 0.15, larguraLinha: 0.012,
  // start positions used by the approved still frame, in the pitch frame
  pato: { x: 0.14, z: 0.05, yaw: -0.24 },
  bola: { x: 0.3, z: 0.09, raio: 0.0335 },
  // goal mouth between the two flip-flops, on the +x goal line (pitch frame); a goal = ball centre past x with |z| < meiaLargura
  gol: { linhaX: 0.8, meiaLargura: 0.15 },   // flip-flops sit at z ±0.2; their inner edges are ~±0.15
};

// White painted markings + the flip-flop goal. Ball and duck are left to the physics.
export function criarCampo() {
  const campo = new THREE.Group(); campo.name = 'campo';
  campo.position.set(CAMPO.posicao.x, 0, CAMPO.posicao.y);
  const LW = CAMPO.larguraLinha;
  const tinta = mat('#f2eee4', { roughness: 0.9, transparent: true, opacity: 0.88, polygonOffset: true, polygonOffsetFactor: -2 });
  const Y = 0.0025; // lifted off the floor so the lines don't z-fight into dashes

  const linha = (x1, z1, x2, z2) => {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(Math.hypot(x2 - x1, z2 - z1) + LW, LW), tinta);
    m.rotation.order = 'YXZ'; m.rotation.y = -Math.atan2(z2 - z1, x2 - x1); m.rotation.x = -Math.PI / 2;
    m.position.set((x1 + x2) / 2, Y, (z1 + z2) / 2); m.receiveShadow = true; campo.add(m);
  };
  const arco = (cx, cz, r, ini = 0, comp = Math.PI * 2) => {
    const m = new THREE.Mesh(new THREE.RingGeometry(r - LW / 2, r + LW / 2, 64, 1, ini, comp), tinta);
    m.rotation.x = -Math.PI / 2; m.position.set(cx, Y, cz); m.receiveShadow = true; campo.add(m);
  };
  const marca = (cx, cz) => {
    const m = new THREE.Mesh(new THREE.CircleGeometry(0.011, 20), tinta);
    m.rotation.x = -Math.PI / 2; m.position.set(cx, Y, cz); campo.add(m);
  };

  const FL = CAMPO.meioComprimento, FW = CAMPO.meiaLargura, PA = CAMPO.area.fundo, PW = CAMPO.area.meiaLargura;
  const GA = CAMPO.pequenaArea.fundo, GW = CAMPO.pequenaArea.meiaLargura, PS = CAMPO.marcaPenalti, CR = CAMPO.raioCirculo;
  linha(-FL, -FW, FL, -FW); linha(-FL, FW, FL, FW); linha(-FL, -FW, -FL, FW); linha(FL, -FW, FL, FW);
  linha(0, -FW, 0, FW); arco(0, 0, CR); marca(0, 0);
  for (const sx of [1, -1]) {
    const gx = sx * FL, px = gx - sx * PA, ax = gx - sx * GA;
    linha(gx, -PW, px, -PW); linha(px, -PW, px, PW); linha(px, PW, gx, PW);
    linha(gx, -GW, ax, -GW); linha(ax, -GW, ax, GW); linha(ax, GW, gx, GW);
    marca(gx - sx * PS, 0);
    const meia = Math.acos((PA - PS) / CR);            // the "D": the part of the circle outside the box
    arco(gx - sx * PS, 0, CR, (sx > 0 ? Math.PI : 0) - meia, meia * 2);
  }
  for (const [cx, cz, ini] of [[FL, FW, Math.PI / 2], [FL, -FW, Math.PI], [-FL, -FW, Math.PI * 1.5], [-FL, FW, 0]])
    arco(cx, cz, 0.03, ini, Math.PI / 2);              // corner arcs

  const c1 = chinelo('#2f5fa8'); c1.position.set(FL + 0.02, 0, -0.2); c1.rotation.y = 0.35;
  const c2 = chinelo('#2f5fa8'); c2.position.set(FL + 0.05, 0, 0.2); c2.rotation.y = -0.2 + Math.PI;
  c1.name = 'trave-esquerda'; c2.name = 'trave-direita';
  campo.add(c1, c2);
  return campo;
}

// A flip-flop, 26 cm long, pointing +x. Two of them are the goal posts.
export function chinelo(color) {
  const s = new THREE.Shape();
  s.moveTo(-0.13, 0);
  s.bezierCurveTo(-0.13, 0.042, -0.08, 0.048, -0.03, 0.038);
  s.bezierCurveTo(0.03, 0.03, 0.08, 0.058, 0.12, 0.04);
  s.bezierCurveTo(0.14, 0.025, 0.14, -0.025, 0.12, -0.04);
  s.bezierCurveTo(0.08, -0.058, 0.03, -0.03, -0.03, -0.038);
  s.bezierCurveTo(-0.08, -0.048, -0.13, -0.042, -0.13, 0);
  const g = new THREE.Group();
  const branco = new THREE.Mesh(new THREE.ExtrudeGeometry(s, { depth: 0.012, bevelEnabled: true, bevelThickness: .002, bevelSize: .002, bevelSegments: 2 }), mat('#f3efe6', { roughness: .8 }));
  const topo = new THREE.Mesh(new THREE.ExtrudeGeometry(s, { depth: 0.004, bevelEnabled: false }), mat(color, { roughness: .7 }));
  branco.rotation.x = topo.rotation.x = -Math.PI / 2;
  topo.position.y = 0.013;
  branco.castShadow = topo.castShadow = branco.receiveShadow = topo.receiveShadow = true;
  g.add(branco, topo);
  const tira = mat(color, { roughness: .6 });
  for (const lado of [1, -1]) {
    const curva = new THREE.CatmullRomCurve3([
      new THREE.Vector3(0.075, 0.017, 0), new THREE.Vector3(0.04, 0.035, lado * 0.022),
      new THREE.Vector3(-0.01, 0.03, lado * 0.036), new THREE.Vector3(-0.03, 0.017, lado * 0.04)]);
    const m = new THREE.Mesh(new THREE.TubeGeometry(curva, 20, 0.0045, 8), tira);
    m.castShadow = true; g.add(m);
  }
  return g;
}

// Tennis-ball texture (felt + seam). Put it on the physics ball's mesh.
export function texturaBola(rand) {
  return texturaCanvas(512, 256, g => {
    g.fillStyle = '#cfe04a'; g.fillRect(0, 0, 512, 256);
    for (let i = 0; i < 9000; i++) { g.fillStyle = rand() > .5 ? 'rgba(255,255,220,.18)' : 'rgba(90,110,0,.12)'; g.fillRect(rand() * 512, rand() * 256, 1.5, 1.5); }
    g.strokeStyle = '#f4f2e6'; g.lineWidth = 7; g.beginPath();
    for (let x = 0; x <= 512; x += 2) { const y = 128 + Math.sin(x / 512 * Math.PI * 4) * 62; x ? g.lineTo(x, y) : g.moveTo(x, y); }
    g.stroke();
  });
}

// Reference ball for when there is no physics (the standalone demo).
export function criarBola(textura) {
  const b = new THREE.Mesh(new THREE.SphereGeometry(CAMPO.bola.raio, 48, 32), new THREE.MeshStandardMaterial({ map: textura, roughness: 1 }));
  b.position.set(CAMPO.bola.x, CAMPO.bola.raio, CAMPO.bola.z); b.rotation.set(0.4, 1.1, 0.2);
  b.castShadow = b.receiveShadow = true; b.name = 'bola';
  return b;
}
