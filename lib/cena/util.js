import * as THREE from 'three';

// Deterministic random so the room looks the same on every load (same seed as the approved study).
export function criarRand(seed = 7) {
  let s = seed;
  return () => (s = (s * 16807) % 2147483647) / 2147483647;
}

// The approved study ran on three r147 with color management off, so hex colors went to the GPU as-is.
// Creating colors in linear space keeps those exact values on current three without touching the
// global ColorManagement setting (which the duck and the rest of the prototype rely on).
export const cor = (hex) =>
  new THREE.Color().setHex(typeof hex === 'number' ? hex : parseInt(String(hex).replace('#', ''), 16), THREE.LinearSRGBColorSpace);

export const mat = (hex, props = {}) => new THREE.MeshStandardMaterial({ color: cor(hex), ...props });

export function texturaCanvas(w, h, desenhar) {
  const c = document.createElement('canvas'); c.width = w; c.height = h;
  desenhar(c.getContext('2d'), w, h);
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

export function sombras(obj, cast = true, receive = true) {
  obj.traverse(o => { if (o.isMesh) { o.castShadow = cast; o.receiveShadow = receive; } });
  return obj;
}
