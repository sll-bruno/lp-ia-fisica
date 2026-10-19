// A sala de casa (lib/cena, da thread de design) vista pela física e pelo
// laser. Tudo aqui em coordenadas MJCF: x igual ao da sala, y = -z da sala,
// z pra cima. Um giro rotation.y = θ da sala vira yaw θ no MJCF.
//
// As formas são aproximadas (caixas e cilindros) e ficam onde estão os
// móveis visuais de lib/cena/sala.js e lib/cena/campo.js. Se a sala mudar
// lá, estas medidas precisam acompanhar.

import { SALA, CAMPO, caixasColisao, mjcfColisao } from "../cena/cena.js";

const PX = CAMPO.posicao.x, PZ = CAMPO.posicao.y; // campo na sala (x, z)
const toMj = (x, z) => [x, -z];

// Gol: entre os dois chinelos, na linha x = linhaX do campo.
export const GOL = {
  linhaX: PX + CAMPO.gol.linhaX,
  centroY: -PZ,
  meiaLargura: CAMPO.gol.meiaLargura,
};

// Posições iniciais da foto aprovada.
export const INICIO = {
  pato: { x: PX + CAMPO.pato.x, y: -(PZ + CAMPO.pato.z), yaw: CAMPO.pato.yaw },
  bola: { x: PX + CAMPO.bola.x, y: -(PZ + CAMPO.bola.z) },
};
export const BOLA_RAIO = CAMPO.bola.raio;

// Retângulo do campo pintado, em MJCF.
export const CAMPO_MJ = {
  xmin: PX - CAMPO.meioComprimento, xmax: PX + CAMPO.meioComprimento,
  ymin: -PZ - CAMPO.meiaLargura, ymax: -PZ + CAMPO.meiaLargura,
};

// Onde o pato pode estar (piso livre da sala, longe das paredes abertas).
export const PISO = { xmin: -1.85, xmax: 1.28, ymin: -1.5, ymax: 0.66 };

const WX = SALA.paredeFundoX;

// Caixas de colisão da thread de design (lib/cena/colisao.js), com o sofá,
// o puff e o vaso ("todos"), porque eles ajudam o laser a se localizar.
export const CAIXAS = caixasColisao({ incluir: "todos" });
// O laser fica a ~0,22 m do chão: só enxerga caixas mais altas que isso.
const ALTURA_LASER = 0.2;
const vistaPeloLaser = (c) => c.centro[1] + c.tamanho[1] / 2 > ALTURA_LASER;

// XML das caixas de colisão: as altas no grupo do design (4, que o laser
// enxerga); os chinelos, baixos demais pro laser horizontal, no grupo 2.
export function colisaoXml() {
  const altas = CAIXAS.filter(vistaPeloLaser), baixas = CAIXAS.filter((c) => !vistaPeloLaser(c));
  const xml = mjcfColisao(altas) + "\n" + mjcfColisao(baixas, { grupo: 2 });
  const doc = new DOMParser().parseFromString(`<r>${xml}</r>`, "text/xml");
  return [...doc.querySelectorAll("geom")];
}

// Geoms nossos, além das caixas do design: objetos baixos do chão (a bola
// para neles) e dois limites invisíveis nos lados abertos da sala.
// grupo 2 = baixos demais pro laser; grupo 5 = limites (não existem na cena,
// o laser não vê).
export function geomsDaSala() {
  const g = [];
  const box = (name, [x, y], z, [hx, hy, hz], group, yaw = 0) =>
    g.push({ name, type: "box", pos: [x, y, z], size: [hx, hy, hz], group, yaw });
  const cyl = (name, [x, y], r, h, group) =>
    g.push({ name, type: "cylinder", pos: [x, y, h / 2], size: [r, h / 2], group, yaw: 0 });
  box("livros_chao", toMj(WX - 0.2, -1.0), 0.06, [0.12, 0.14, 0.06], 2, 0.3);
  cyl("caneca", toMj(0.38, -0.95), 0.042, 0.095, 2);
  box("tenis", toMj(-1.2, 0.95), 0.04, [0.135, 0.05, 0.04], 2, 2.2);
  box("limite_x", [PISO.xmin - 0.25, -0.5], 0.3, [0.05, 2.5, 0.3], 5);
  box("limite_y", [-0.3, PISO.ymin - 0.25], 0.3, [2.5, 0.05, 0.3], 5);
  return g;
}

// Mapa que o pato conhece pro filtro de partículas: as caixas do design que
// o laser enxerga (paredes, rack, sofá, puff, vaso), alinhadas aos eixos.
export function mapaDoLaser() {
  return CAIXAS.filter(vistaPeloLaser).map((c) => ({
    cx: c.centro[0], cy: -c.centro[2], hx: c.tamanho[0] / 2, hy: c.tamanho[2] / 2,
  }));
}
