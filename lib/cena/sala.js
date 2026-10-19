import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/examples/jsm/geometries/RoundedBoxGeometry.js';
import { mat, texturaCanvas, sombras } from './util.js';

// Room layout in meters. The floor is y = 0; the back wall (behind the goal) faces -x, the side wall faces +z.
export const SALA = { paredeFundoX: 1.75, paredeLateralZ: -1.6, altura: 2.7 };

// Basket-weave "taco" parquet, 21 cm cells.
export function texturaPiso(rand) {
  const t = texturaCanvas(1024, 1024, g => {
    const cell = 128, n = 3, w = cell / n;
    const tons = ['#c7a57c', '#bf9c72', '#cfae86', '#b8946a', '#c4a176', '#cba982'];
    for (let i = 0; i < 1024 / cell; i++) for (let j = 0; j < 1024 / cell; j++) {
      const horiz = (i + j) % 2 === 0;
      for (let k = 0; k < n; k++) {
        const x = i * cell + (horiz ? 0 : k * w), y = j * cell + (horiz ? k * w : 0);
        const pw = horiz ? cell : w, ph = horiz ? w : cell;
        g.fillStyle = tons[Math.floor(rand() * tons.length)];
        g.fillRect(x, y, pw, ph);
        g.globalAlpha = 0.08;
        for (let l = 0; l < 14; l++) {
          g.strokeStyle = rand() > .5 ? '#3a2310' : '#d9a878';
          g.lineWidth = 0.6 + rand();
          g.beginPath();
          if (horiz) { const yy = y + rand() * ph; g.moveTo(x, yy); g.bezierCurveTo(x + pw * .3, yy + rand() * 3 - 1.5, x + pw * .7, yy + rand() * 3 - 1.5, x + pw, yy); }
          else { const xx = x + rand() * pw; g.moveTo(xx, y); g.bezierCurveTo(xx + rand() * 3 - 1.5, y + ph * .3, xx + rand() * 3 - 1.5, y + ph * .7, xx, y + ph); }
          g.stroke();
        }
        g.globalAlpha = 1;
        g.strokeStyle = 'rgba(60,36,16,.35)'; g.lineWidth = 1; g.strokeRect(x + .5, y + .5, pw - 1, ph - 1);
      }
    }
  });
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(4.5, 4.5);   // one tile = 8 cells ≈ 1.68 m, repeated over the 7.5 m floor
  t.anisotropy = 8;
  return t;
}

// Everything in the room except the pitch: floor, walls, TV rack with books, sofa, prints, rug, small objects, plant.
// `piso` is the floor texture, made first so the random sequence matches the approved study.
export function criarSala(rand, piso) {
  const sala = new THREE.Group(); sala.name = 'sala';
  const WX = SALA.paredeFundoX, WZ = SALA.paredeLateralZ;

  const chao = new THREE.Mesh(new THREE.PlaneGeometry(7.5, 7.5), new THREE.MeshStandardMaterial({ map: piso, roughness: 0.62, metalness: 0 }));
  chao.rotation.x = -Math.PI / 2; chao.receiveShadow = true; chao.name = 'chao'; sala.add(chao);

  // walls + baseboards
  const reboco = mat('#ddd5c8', { roughness: 0.95 });
  const fundo = new THREE.Mesh(new THREE.PlaneGeometry(4.5, SALA.altura), reboco);
  fundo.rotation.y = -Math.PI / 2; fundo.position.set(WX, SALA.altura / 2, 0.6); fundo.receiveShadow = true;
  const lateral = new THREE.Mesh(new THREE.PlaneGeometry(5, SALA.altura), reboco);
  lateral.position.set(-0.75, SALA.altura / 2, WZ); lateral.receiveShadow = true;
  const rodapeMat = mat('#f4f0e8', { roughness: 0.5 });
  const r1 = new THREE.Mesh(new THREE.BoxGeometry(0.016, 0.08, 4.5), rodapeMat); r1.position.set(WX - 0.008, 0.04, 0.6);
  const r2 = new THREE.Mesh(new THREE.BoxGeometry(5, 0.08, 0.016), rodapeMat); r2.position.set(-0.75, 0.04, WZ + 0.008);
  r1.receiveShadow = r2.receiveShadow = true;
  sala.add(fundo, lateral, r1, r2);

  const caixa = (w, h, d, m, x, y, z, pai) => {
    const b = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), m);
    b.position.set(x, y, z); b.castShadow = b.receiveShadow = true; pai.add(b); return b;
  };

  // low rack against the back wall, long side along z, with the TV on it
  const rack = new THREE.Group(); rack.name = 'rack'; rack.position.set(WX - 0.22, 0, -0.05); sala.add(rack);
  const carvalho = mat('#b08a62', { roughness: 0.6 });
  const escuro = mat('#2a2622', { roughness: 0.5 });
  caixa(0.4, 0.03, 1.5, carvalho, 0, 0.44, 0, rack);
  caixa(0.4, 0.03, 1.5, carvalho, 0, 0.12, 0, rack);
  caixa(0.4, 0.32, 0.03, carvalho, 0, 0.28, -0.735, rack);
  caixa(0.4, 0.32, 0.03, carvalho, 0, 0.28, 0.735, rack);
  caixa(0.4, 0.32, 0.025, carvalho, 0, 0.28, 0.2, rack);
  caixa(0.02, 0.32, 1.5, carvalho, 0.19, 0.28, 0, rack);
  for (const z of [-0.68, 0.68]) for (const x of [-0.16, 0.16]) {
    const p = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.009, 0.11, 8), escuro); p.position.set(x, 0.055, z); rack.add(p);
  }
  caixa(0.06, 0.05, 0.3, escuro, 0.02, 0.48, 0, rack);
  caixa(0.035, 0.62, 1.08, escuro, 0, 0.83, 0, rack);
  const tela = new THREE.Mesh(new THREE.PlaneGeometry(1.04, 0.58), mat('#0e0f11', { roughness: 0.32, metalness: 0 }));
  tela.rotation.y = -Math.PI / 2; tela.position.set(-0.019, 0.83, 0); tela.name = 'tela-tv'; rack.add(tela);

  // books: a shelf of spines, a few lying down, a stack on the floor
  const lombadas = ['#7a3b2e', '#d8c7a3', '#2f4a5c', '#a8633a', '#4d5a3b', '#e2ddd2', '#8c2f39', '#35322e', '#c49a4a'];
  const livro = () => mat(lombadas[Math.floor(rand() * lombadas.length)], { roughness: 0.8 });
  let z = -0.7;
  while (z < 0.17) {
    const t = 0.022 + rand() * 0.03, h = 0.2 + rand() * 0.08;
    caixa(0.24 + rand() * 0.04, h, t, livro(), 0.02, 0.135 + h / 2, z + t / 2, rack);
    z += t + 0.002;
    if (rand() > 0.85) z += 0.04;
  }
  let y = 0.135;
  for (let i = 0; i < 4; i++) {
    const h = 0.025 + rand() * 0.02;
    const b = caixa(0.22 + rand() * 0.06, h, 0.3 + rand() * 0.06, livro(), 0.02, y + h / 2, 0.45, rack);
    b.rotation.y = (rand() - 0.5) * 0.2; y += h;
  }
  y = 0;
  for (let i = 0; i < 3; i++) {
    const h = 0.03 + rand() * 0.02;
    const b = caixa(0.2 + rand() * 0.05, h, 0.27, livro(), WX - 0.2, y + h / 2, -1.0 + rand() * 0.02, sala);
    b.rotation.y = 0.3 + (rand() - 0.5) * 0.3; y += h;
  }

  // sofa against the side wall
  const RB = (w, h, d, r, m) => { const b = new THREE.Mesh(new RoundedBoxGeometry(w, h, d, 4, r), m); b.castShadow = b.receiveShadow = true; return b; };
  const tecido = mat('#8b8a72', { roughness: 1 });
  const sofa = new THREE.Group(); sofa.name = 'sofa'; sofa.position.set(-0.35, 0, WZ + 0.47); sala.add(sofa);
  { const b = RB(1.9, 0.2, 0.86, 0.03, tecido); b.position.set(0, 0.23, 0); sofa.add(b);
    for (const x of [-0.46, 0.46]) { const c = RB(0.9, 0.13, 0.66, 0.05, tecido); c.position.set(x, 0.395, 0.08); sofa.add(c); }
    const enc = RB(1.9, 0.48, 0.2, 0.05, tecido); enc.position.set(0, 0.5, -0.33); sofa.add(enc);
    for (const x of [-0.46, 0.46]) { const c = RB(0.88, 0.36, 0.16, 0.07, tecido); c.position.set(x, 0.58, -0.2); c.rotation.x = -0.12; sofa.add(c); }
    for (const x of [-1.0, 1.0]) { const a = RB(0.16, 0.42, 0.86, 0.05, tecido); a.position.set(x, 0.41, 0); sofa.add(a); }
    const almofada = RB(0.36, 0.32, 0.11, 0.05, mat('#d39a3e', { roughness: 1 }));
    almofada.position.set(0.68, 0.6, -0.1); almofada.rotation.set(-0.25, -0.35, 0.08); sofa.add(almofada);
    const pe = mat('#3d2a1c', { roughness: .5 });
    for (const [x, zz] of [[-0.98, 0.36], [0.98, 0.36], [-0.98, -0.36], [0.98, -0.36]]) {
      const p = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.011, 0.13, 10), pe); p.position.set(x, 0.065, zz); p.castShadow = true; sofa.add(p);
    } }

  // two framed prints above the sofa
  const quadro = (desenhar, w, h) => {
    const t = texturaCanvas(256, Math.round(256 * h / w), (g, cw, ch) => { g.fillStyle = '#efe9dd'; g.fillRect(0, 0, cw, ch); desenhar(g, cw, ch); });
    const f = new THREE.Group();
    const moldura = new THREE.Mesh(new THREE.BoxGeometry(w + 0.04, h + 0.04, 0.025), mat('#2b2622', { roughness: .6 }));
    const arte = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshStandardMaterial({ map: t, roughness: .9 })); arte.position.z = 0.0135;
    moldura.castShadow = true; arte.receiveShadow = moldura.receiveShadow = true; f.add(moldura, arte); return f;
  };
  const q1 = quadro((g, w, h) => { g.fillStyle = '#c2643a'; g.beginPath(); g.arc(w * .45, h * .42, w * .26, 0, Math.PI * 2); g.fill(); g.fillStyle = '#2f4a5c'; g.fillRect(w * .18, h * .66, w * .64, h * .1); }, 0.5, 0.66);
  q1.position.set(-0.7, 1.32, WZ + 0.013);
  const q2 = quadro((g, w, h) => { g.strokeStyle = '#4d5a3b'; g.lineWidth = 5; for (let i = 0; i < 7; i++) { g.beginPath(); g.moveTo(w * .15, h * (.2 + i * .1)); g.bezierCurveTo(w * .4, h * (.1 + i * .1), w * .6, h * (.3 + i * .1), w * .85, h * (.2 + i * .1)); g.stroke(); } }, 0.34, 0.44);
  q2.position.set(-0.1, 1.24, WZ + 0.013);
  sala.add(q1, q2);

  // rug, diagonal, under the front of the sofa and clear of the pitch
  const tapeteTex = texturaCanvas(512, 256, g => {
    g.fillStyle = '#e4dccb'; g.fillRect(0, 0, 512, 256);
    for (let i = 0; i < 6000; i++) { g.fillStyle = rand() > .5 ? 'rgba(120,100,70,.10)' : 'rgba(255,255,255,.18)'; g.fillRect(rand() * 512, rand() * 256, 2, 1); }
    g.strokeStyle = '#a8633a'; g.lineWidth = 6; g.strokeRect(18, 18, 476, 220); g.lineWidth = 2; g.strokeRect(30, 30, 452, 196);
  });
  const tapete = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.008, 0.95), new THREE.MeshStandardMaterial({ map: tapeteTex, roughness: 1 }));
  tapete.position.set(-0.75, 0.004, WZ + 0.62); tapete.rotation.y = 0.32; tapete.receiveShadow = true; tapete.name = 'tapete';
  sala.add(tapete);

  // small things on the floor: pouf, mug, remote, magazines, a sneaker
  const puff = RB(0.42, 0.3, 0.42, 0.12, mat('#c9b79a', { roughness: 1 })); puff.position.set(-1.35, 0.15, -0.35);
  const ceramica = mat('#ece6da', { roughness: .35 });
  const caneca = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.038, 0.095, 24), ceramica); caneca.position.set(0.38, 0.0475, -0.95);
  const alca = new THREE.Mesh(new THREE.TorusGeometry(0.025, 0.007, 8, 16, Math.PI), ceramica); alca.position.set(0.42, 0.05, -0.95); alca.rotation.z = -Math.PI / 2;
  caneca.castShadow = alca.castShadow = true;
  const controle = RB(0.05, 0.02, 0.19, 0.008, mat('#1f1f22', { roughness: .5 })); controle.position.set(0.15, 0.01, -0.82); controle.rotation.y = 0.7;
  sala.add(puff, caneca, alca, controle);
  let yy = 0;
  for (let i = 0; i < 3; i++) {
    const m = RB(0.22, 0.008, 0.29, 0.003, mat(['#d8d2c4', '#8c2f39', '#2f4a5c'][i], { roughness: .7 }));
    m.position.set(-1.15, yy + 0.004, 0.55); m.rotation.y = 0.4 + i * 0.15; yy += 0.008; sala.add(m);
  }
  const sola = RB(0.27, 0.025, 0.1, 0.012, mat('#e0d6c2', { roughness: .9 }));
  const cabedal = RB(0.22, 0.07, 0.09, 0.035, mat('#f2f0ea', { roughness: .8 })); cabedal.position.set(-0.02, 0.045, 0);
  const tenis = new THREE.Group(); tenis.add(sola, cabedal); tenis.position.set(-1.2, 0.013, 0.95); tenis.rotation.y = 2.2;
  sala.add(tenis);

  sala.add(criarPlanta(rand));
  return sala;
}

// Plant in the corner: pot, soil, and leaves on stems in a golden-angle rosette.
function criarPlanta(rand) {
  const planta = new THREE.Group(); planta.name = 'planta'; planta.position.set(1.5, 0, 1.0);
  const barro = mat('#b5603b', { roughness: .9, side: THREE.DoubleSide });
  const vaso = new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.08, 0.22, 32, 1, true), barro); vaso.position.y = 0.11;
  const borda = new THREE.Mesh(new THREE.TorusGeometry(0.11, 0.008, 8, 40), barro); borda.rotation.x = Math.PI / 2; borda.position.y = 0.22;
  const base = new THREE.Mesh(new THREE.CircleGeometry(0.08, 32), barro); base.rotation.x = -Math.PI / 2; base.position.y = 0.002;
  const terra = new THREE.Mesh(new THREE.CircleGeometry(0.105, 32), mat('#3b2a1e', { roughness: 1 })); terra.rotation.x = -Math.PI / 2; terra.position.y = 0.2;
  planta.add(vaso, borda, base, terra);

  const forma = new THREE.Shape();
  forma.moveTo(0, 0);
  forma.bezierCurveTo(0.03, 0.05, 0.11, 0.055, 0.17, 0);
  forma.bezierCurveTo(0.11, -0.055, 0.03, -0.05, 0, 0);
  const folhaGeo = new THREE.ShapeGeometry(forma, 16);
  folhaGeo.rotateX(-Math.PI / 2);
  const pos = folhaGeo.attributes.position;               // fold along the midrib and droop toward the tip
  for (let i = 0; i < pos.count; i++) { const x = pos.getX(i), z = pos.getZ(i); pos.setY(i, Math.abs(z) * 0.45 - x * x * 1.6); }
  folhaGeo.computeVertexNormals();
  const folhaMat = mat('#3d6b3a', { roughness: .65, side: THREE.DoubleSide });
  const caule = mat('#5d7f43', { roughness: .8 });
  for (let i = 0; i < 12; i++) {
    const a = i * 2.39996 + rand() * 0.3;
    const alcance = 0.05 + rand() * 0.1, h = 0.3 + rand() * 0.32;
    const dir = new THREE.Vector3(Math.cos(a), 0, Math.sin(a));
    const ponta = new THREE.Vector3(dir.x * alcance, h, dir.z * alcance);
    const curva = new THREE.CatmullRomCurve3([
      new THREE.Vector3(dir.x * 0.01, 0.2, dir.z * 0.01),
      new THREE.Vector3(dir.x * alcance * 0.35, 0.2 + (h - 0.2) * 0.55, dir.z * alcance * 0.35),
      ponta]);
    planta.add(new THREE.Mesh(new THREE.TubeGeometry(curva, 12, 0.0035, 6), caule));
    const folha = new THREE.Mesh(folhaGeo, folhaMat);
    folha.position.copy(ponta);
    folha.rotation.order = 'YZX';
    folha.rotation.y = -a; folha.rotation.z = 0.35 + rand() * 0.5;
    const s = 0.8 + rand() * 0.45; folha.scale.set(s, s, s);
    planta.add(folha);
  }
  return sombras(planta);
}
