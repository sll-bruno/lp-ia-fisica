// Marcas no chão: o que o pato "sabe" e o que acontece agora, desenhado na
// própria sala (regra da thread de design: tinta pro que ele sabe, laranja
// só pro que está acontecendo ou é o objetivo).
//
// Um InstancedMesh de quads deitados no chão, com cor por instância e três
// formas no shader: ponto (disco de borda suave), anel e quad chapado. Dois
// conjuntos: chão (com teste de profundidade, some atrás dos móveis) e raio
// X (por cima de tudo, pro centro de massa e o polígono dos pés).
//
// Coordenadas da sala (three): x, z no chão, y pra cima.

import * as THREE from "three";

export const TINTA = [0.137, 0.133, 0.122];
export const SOL = [0.85, 0.41, 0.17];
const PONTO = 0, ANEL = 1, QUAD = 2;

const VERT = /* glsl */ `
  attribute vec4 aCor;
  attribute float aForma;
  varying vec2 vUv;
  varying vec4 vCor;
  varying float vForma;
  void main() {
    vUv = uv; vCor = aCor; vForma = aForma;
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * instanceMatrix * vec4(position, 1.0);
  }
`;
const FRAG = /* glsl */ `
  varying vec2 vUv;
  varying vec4 vCor;
  varying float vForma;
  void main() {
    vec2 p = vUv * 2.0 - 1.0;
    float d = length(p);
    float fw = fwidth(d) * 1.5;
    float a = 1.0;
    if (vForma < 0.5) a = 1.0 - smoothstep(1.0 - fw, 1.0, d);
    else if (vForma < 1.5) a = smoothstep(0.72 - fw, 0.72, d) * (1.0 - smoothstep(1.0 - fw, 1.0, d));
    a *= vCor.a;
    if (a < 0.01) discard;
    gl_FragColor = vec4(vCor.rgb, a);
  }
`;

export class Marcas {
  constructor({ max = 8000, raioX = false } = {}) {
    const geo = new THREE.PlaneGeometry(1, 1);
    geo.rotateX(-Math.PI / 2); // deitado no chão (normal pra cima)
    this.cor = new Float32Array(max * 4);
    this.forma = new Float32Array(max);
    geo.setAttribute("aCor", new THREE.InstancedBufferAttribute(this.cor, 4));
    geo.setAttribute("aForma", new THREE.InstancedBufferAttribute(this.forma, 1));
    const mat = new THREE.ShaderMaterial({
      vertexShader: VERT, fragmentShader: FRAG,
      transparent: true, depthWrite: false, depthTest: !raioX,
    });
    this.mesh = new THREE.InstancedMesh(geo, mat, max);
    this.mesh.frustumCulled = false;
    this.mesh.count = 0;
    if (raioX) this.mesh.renderOrder = 1;
    this.mesh.layers.set(1); // a câmera da cabeça não vê
    this.max = max;
    this.n = 0;
    this._m = new THREE.Matrix4();
    this._q = new THREE.Quaternion();
    this._p = new THREE.Vector3();
    this._s = new THREE.Vector3();
    this._eixo = new THREE.Vector3(0, 1, 0);
  }

  inicio() { this.n = 0; }

  _add(m, cor, alfa, forma) {
    if (this.n >= this.max) return;
    const i = this.n++;
    this.mesh.setMatrixAt(i, m);
    this.cor[i * 4] = cor[0]; this.cor[i * 4 + 1] = cor[1]; this.cor[i * 4 + 2] = cor[2]; this.cor[i * 4 + 3] = alfa;
    this.forma[i] = forma;
  }

  // Quad no chão: centro (x, z), comprimento ao longo do yaw, largura.
  chao(x, z, comp, larg, yaw, cor, alfa, y = 0.004, forma = QUAD) {
    this._q.setFromAxisAngle(this._eixo, yaw);
    this._m.compose(this._p.set(x, y, z), this._q, this._s.set(comp, 1, larg));
    this._add(this._m, cor, alfa, forma);
  }
  ponto(x, z, diam, cor, alfa, y = 0.004) { this.chao(x, z, diam, diam, 0, cor, alfa, y, PONTO); }
  anel(x, z, diam, cor, alfa, y = 0.004) { this.chao(x, z, diam, diam, 0, cor, alfa, y, ANEL); }

  // Quad com pose qualquer (ex.: o fio de prumo, de pé e virado pra câmera).
  pose(pos, quat, sx, sz, cor, alfa, forma = QUAD) {
    this._m.compose(pos, quat, this._s.set(sx, 1, sz));
    this._add(this._m, cor, alfa, forma);
  }

  seg(x1, z1, x2, z2, larg, cor, alfa, y) {
    const dx = x2 - x1, dz = z2 - z1, L = Math.hypot(dx, dz);
    if (L < 1e-5) return;
    this.chao((x1 + x2) / 2, (z1 + z2) / 2, L, larg, -Math.atan2(dz, dx), cor, alfa, y);
  }
  tracejado(x1, z1, x2, z2, larg, cor, alfa, passo = 0.03, y) {
    const L = Math.hypot(x2 - x1, z2 - z1), n = Math.floor(L / passo);
    for (let i = 0; i < n; i += 2) {
      const a = i / n, b = Math.min(1, (i + 1) / n);
      this.seg(x1 + (x2 - x1) * a, z1 + (z2 - z1) * a, x1 + (x2 - x1) * b, z1 + (z2 - z1) * b, larg, cor, alfa, y);
    }
  }
  seta(x1, z1, x2, z2, larg, cor, alfa, y) {
    this.seg(x1, z1, x2, z2, larg, cor, alfa, y);
    const ang = Math.atan2(z1 - z2, x1 - x2), t = 5 * larg;
    for (const s of [0.5, -0.5]) {
      this.seg(x2, z2, x2 + Math.cos(ang + s) * t, z2 + Math.sin(ang + s) * t, larg, cor, alfa, y);
    }
  }

  fim() {
    this.mesh.count = this.n;
    this.mesh.instanceMatrix.needsUpdate = true;
    this.mesh.geometry.attributes.aCor.needsUpdate = true;
    this.mesh.geometry.attributes.aForma.needsUpdate = true;
  }
}
