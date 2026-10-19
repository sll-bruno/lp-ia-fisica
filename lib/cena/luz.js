import * as THREE from 'three';
import { cor, texturaCanvas } from './util.js';

// Low late-afternoon sun through a four-pane window (a cookie on a spot light) plus a soft sky fill.
// Intensities are for current three (physical light units). The approved study ran on r147's legacy
// lights; these values reproduce it: legacy × π, and the spot's old linear falloff folded into a
// constant intensity (decay 0) measured at the pitch.
export function criarLuz({ leve = false } = {}) {
  const luz = new THREE.Group(); luz.name = 'luz';
  const janela = texturaCanvas(256, 256, g => {
    g.fillStyle = '#000'; g.fillRect(0, 0, 256, 256); g.fillStyle = '#fff';
    for (const [x, y, w, h] of [[78, 70, 46, 52], [132, 70, 46, 52], [78, 130, 46, 52], [132, 130, 46, 52]]) g.fillRect(x, y, w, h);
  });
  janela.colorSpace = THREE.NoColorSpace;

  const sol = new THREE.SpotLight(cor('#ffbf78'), 3.6 * Math.PI * 0.67, 0, 0.42, 0.35, 0);
  sol.name = 'sol';
  sol.position.set(-3.4, 1.35, 2.4);
  sol.target.position.set(0.25, 0, -0.1);
  sol.map = janela;
  sol.castShadow = true;
  sol.shadow.mapSize.setScalar(leve ? 512 : 1024);
  sol.shadow.radius = leve ? 3 : 4;      // soft edges; needs renderer.shadowMap.type = THREE.PCFShadowMap
  sol.shadow.bias = -0.0002;
  sol.shadow.camera.near = 1; sol.shadow.camera.far = 9;

  const ceu = new THREE.HemisphereLight(cor('#e6ebf3'), cor('#9a7656'), 0.85 * Math.PI);
  ceu.name = 'ceu';
  luz.add(sol, sol.target, ceu);
  return luz;
}
