import * as THREE from 'three';
import { criarRand, cor } from './util.js';
import { criarSala, texturaPiso, SALA } from './sala.js';
import { criarCampo, texturaBola, criarBola, CAMPO } from './campo.js';
import { criarLuz } from './luz.js';

import { Percepcao } from './percepcao.js';
export { Percepcao };
export { caixasColisao, mjcfColisao, GRUPO_COLISAO } from './colisao.js';
export { CAMPO, SALA };

// Builds the approved room. The pieces come back separately so the prototype can mount them under its own scene:
//   sala  – floor, walls, furniture, objects (static, no physics)
//   campo – painted pitch + flip-flop goal, already placed on the floor
//   luz   – window sun + sky fill
//   texturaBola – put on the physics ball's mesh
//   criarBolaReferencia() – a static ball, only for when there is no physics
//   pato  – where the duck starts in world coordinates (x, z, yaw)
export function criarCena({ seed = 7, qualidade = 'auto' } = {}) {
  if (qualidade === 'auto') qualidade = Percepcao.qualidadeSugerida();
  const rand = criarRand(seed);
  // creation order matters: it keeps the random sequence (book colours, plant leaves…) identical to the study
  const piso = texturaPiso(rand);
  const bolaTex = texturaBola(rand);
  const sala = criarSala(rand, piso);
  const campo = criarCampo();
  const luz = criarLuz({ leve: qualidade === 'leve' });
  return {
    sala, campo, luz, qualidade,
    texturaBola: bolaTex,
    criarBolaReferencia: () => criarBola(bolaTex),          // in pitch coordinates: add it to `campo`
    fundo: cor('#f7f6f2'),
    pato: { x: CAMPO.posicao.x + CAMPO.pato.x, z: CAMPO.posicao.y + CAMPO.pato.z, yaw: CAMPO.pato.yaw },
    camera: { fov: 36, posicao: new THREE.Vector3(-1.05, 0.5, 1.45), alvo: new THREE.Vector3(0.7, 0.3, -0.05) },
  };
}

// Renderer settings the look depends on.
export function configurarRenderer(renderer) {
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFShadowMap;   // PCF (not PCFSoft) so the sun's shadow.radius softens edges
}
