import * as THREE from 'three';
import { cor } from './util.js';

// "What the duck perceives": everything within reach of the duck (a circle around it plus the cone its
// head camera looks into) renders sharp; beyond that it breaks into halftone stipple and then white paper.
//
// It is a two-pass effect that replaces the normal render:
//   1. the scene, in linear HDR, into a render target;
//   2. the scene again with one override material that writes, per pixel, how far outside perception it is;
//   3. a full-screen pass that tone-maps pass 1 and mixes sharp / stipple / paper using pass 2.
// Call `render(renderer, scene, camera)` every frame instead of `renderer.render`.

const VERT_PERC = `varying vec3 vW;
void main(){ vec4 w = modelMatrix * vec4(position, 1.); vW = w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }`;

// 0 = inside perception (sharp), 1 = beyond (paper).
const FRAG_PERC = `uniform vec2 uPato; uniform vec2 uFrente; uniform vec4 uRaio; uniform vec2 uAltura; varying vec3 vW;
void main(){
  vec2 d = vW.xz - uPato;
  float perto = smoothstep(uRaio.x, uRaio.y, length(d));
  float frente = dot(d, uFrente);
  float lado = abs(dot(d, vec2(-uFrente.y, uFrente.x)));
  float noCone = step(0., frente) * smoothstep(frente * .75 + .05, frente * .45, lado);   // ~70° field of view
  float cone = mix(1., smoothstep(uRaio.z, uRaio.w, frente), noCone);
  float f = min(perto, cone);
  f = max(f, smoothstep(uAltura.x, uAltura.y, vW.y));                                   // high things fade too
  gl_FragColor = vec4(f, 0., 0., 1.);
}`;

const FRAG_POST = `uniform sampler2D tCena, tPerc; uniform vec2 uRes; uniform float uCelula; uniform float uExposicao; uniform vec3 uPapel;
varying vec2 vUv;
vec3 aces(vec3 x){ return clamp((x * (2.51 * x + .03)) / (x * (2.43 * x + .59) + .14), 0., 1.); }
vec3 cena(vec2 uv){ return aces(texture2D(tCena, uv).rgb * uExposicao); }
float bayer4(vec2 p){
  p = mod(floor(p), 4.);
  float m[16];
  m[0]=0.;m[1]=8.;m[2]=2.;m[3]=10.;m[4]=12.;m[5]=4.;m[6]=14.;m[7]=6.;m[8]=3.;m[9]=11.;m[10]=1.;m[11]=9.;m[12]=15.;m[13]=7.;m[14]=13.;m[15]=5.;
  int i = int(p.y * 4. + p.x);
  for (int k = 0; k < 16; k++) if (k == i) return (m[k] + .5) / 16.;
  return .5;
}
void main(){
  vec2 px = vUv * uRes;
  vec3 col = cena(vUv);
  float f = texture2D(tPerc, vUv).r;

  // stipple: one dot per cell, sized by how dark the scene is there, shrinking as it leaves perception
  vec2 cel = floor(px / uCelula);
  vec2 cuv = (cel + .5) * uCelula / uRes;
  vec3 cc = cena(cuv);
  float fc = texture2D(tPerc, cuv).r;
  float escuro = 1. - dot(cc, vec3(.299, .587, .114));
  float some = 1. - smoothstep(.3, .7, fc);
  float raio = uCelula * .42 * sqrt(clamp(escuro * .9 + .12, 0., 1.)) * some;
  float dist = length(px - (cel + .5) * uCelula);
  float ponto = 1. - smoothstep(raio - .7, raio + .7, dist);
  vec3 tinta = mix(cc, uPapel, .15);
  vec3 pontilhado = mix(uPapel, tinta, ponto * mix(.35, .8, some));

  // soft crossfade from sharp to stipple, with a little ordered-dither grain at the edge
  float fica = 1. - smoothstep(.12, .3, f);
  float k = clamp(fica + (bayer4(px) - .5) * .35, 0., 1.);
  gl_FragColor = vec4(mix(pontilhado, col, smoothstep(0., 1., k)), 1.);
}`;

export class Percepcao {
  constructor({
    raioPerto = [1.0, 3.0],       // sharp up to 1 m around the duck, gone by 3 m
    alcanceCone = [2.0, 3.8],     // along the head camera's view: sharp to 2 m, gone by 3.8 m
    altura = [0.95, 1.9],         // above ~1 m things start to dissolve
    celula = 4.5,                 // stipple cell in CSS pixels
    exposicao = 1.1,
    papel = '#f7f6f2',
    amostras = 4,
    qualidade = 'auto',
  } = {}) {
    if (qualidade === 'auto') qualidade = Percepcao.qualidadeSugerida(); this.qualidade = qualidade; this.leve = qualidade === 'leve'; if (this.leve) amostras = 0;
    this.celulaCss = celula;
    this.pato = new THREE.Vector2(0, 0);
    this.frente = new THREE.Vector2(1, 0);
    this.matPerc = new THREE.ShaderMaterial({
      uniforms: {
        uPato: { value: this.pato }, uFrente: { value: this.frente },
        uRaio: { value: new THREE.Vector4(raioPerto[0], raioPerto[1], alcanceCone[0], alcanceCone[1]) },
        uAltura: { value: new THREE.Vector2(altura[0], altura[1]) },
      },
      vertexShader: VERT_PERC, fragmentShader: FRAG_PERC,
    });
    this.rtCena = new THREE.WebGLRenderTarget(1, 1, { samples: amostras, type: THREE.HalfFloatType });  // linear HDR
    this.rtPerc = new THREE.WebGLRenderTarget(1, 1);
    this.matPost = new THREE.ShaderMaterial({
      uniforms: {
        tCena: { value: this.rtCena.texture }, tPerc: { value: this.rtPerc.texture },
        uRes: { value: new THREE.Vector2(1, 1) }, uCelula: { value: celula },
        uExposicao: { value: exposicao }, uPapel: { value: new THREE.Vector3(...cor(papel).toArray()) },
      },
      vertexShader: `varying vec2 vUv; void main(){ vUv = uv; gl_Position = vec4(position.xy, 0., 1.); }`,
      fragmentShader: FRAG_POST, depthTest: false, depthWrite: false,
    });
    this.cenaPost = new THREE.Scene();
    this.cenaPost.add(new THREE.Mesh(new THREE.PlaneGeometry(2, 2), this.matPost));
    this.camPost = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1);
    this.branco = new THREE.Color(1, 1, 1);
    this._tam = new THREE.Vector2();
    this._v = new THREE.Vector3();
  }

  // Duck position on the floor (world x, z) and heading: yaw 0 looks along +x, like Object3D.rotation.y.
  setPato(x, z, yaw = 0) {
    this.pato.set(x, z);
    this.frente.set(Math.cos(yaw), -Math.sin(yaw));
  }

  // Follow a live object (e.g. the duck's trunk body from MuJoCo). `eixoFrente` is the object's local forward axis.
  seguir(obj, eixoFrente = new THREE.Vector3(1, 0, 0)) {
    obj.updateWorldMatrix(true, false);
    obj.getWorldPosition(this._v);
    this.pato.set(this._v.x, this._v.z);
    this._v.copy(eixoFrente).transformDirection(obj.matrixWorld);
    const n = Math.hypot(this._v.x, this._v.z) || 1;
    this.frente.set(this._v.x / n, this._v.z / n);
  }

  _ajustar(renderer) {
    renderer.getDrawingBufferSize(this._tam);
    const { x: w, y: h } = this._tam;
    if (this.rtCena.width !== w || this.rtCena.height !== h) {
      this.rtCena.setSize(w, h); this.rtPerc.setSize(this.leve ? Math.ceil(w / 2) : w, this.leve ? Math.ceil(h / 2) : h);
      this.matPost.uniforms.uRes.value.set(w, h);
      this.matPost.uniforms.uCelula.value = Math.round(this.celulaCss * renderer.getPixelRatio());
    }
  }

  render(renderer, scene, camera) {
    this._ajustar(renderer);
    const alvoAntes = renderer.getRenderTarget();
    renderer.setRenderTarget(this.rtCena); renderer.render(scene, camera);

    const fundo = scene.background, override = scene.overrideMaterial, autoSombra = renderer.shadowMap.autoUpdate;
    scene.background = this.branco; scene.overrideMaterial = this.matPerc; renderer.shadowMap.autoUpdate = false;
    renderer.setRenderTarget(this.rtPerc); renderer.render(scene, camera);
    scene.background = fundo; scene.overrideMaterial = override; renderer.shadowMap.autoUpdate = autoSombra;

    renderer.setRenderTarget(alvoAntes); renderer.render(this.cenaPost, this.camPost);
  }

  static qualidadeSugerida() { if (typeof window === 'undefined') return 'completa'; const toque = window.matchMedia?.('(pointer: coarse)').matches; const fraco = (navigator.deviceMemory ?? 8) <= 4 || (navigator.hardwareConcurrency ?? 8) <= 4; return toque || fraco ? 'leve' : 'completa'; }

  dispose() {
    this.rtCena.dispose(); this.rtPerc.dispose(); this.matPerc.dispose(); this.matPost.dispose();
  }
}
