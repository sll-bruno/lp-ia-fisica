import { CAMPO } from './campo.js';
import { SALA } from './sala.js';

// Collision shapes that match the visuals, so the physics ball stops at the flip-flops, the TV rack and the walls.
// Everything is a box: center and full size in three's world (y up, meters) plus a yaw about the vertical.
//   padrao: flip-flops + rack + walls   (the recommended default)
//   todos:  also the sofa, the pouf and the plant pot
export function caixasColisao({ incluir = 'padrao' } = {}) {
  const WX = SALA.paredeFundoX, WZ = SALA.paredeLateralZ;
  const P = CAMPO.posicao, FL = CAMPO.meioComprimento;
  const caixas = [
    // flip-flops: the 26 × 10 cm footprint (same poses as in campo.js), but 8 cm tall instead of 2 cm.
    // A real 2 cm sole lets the 6.7 cm ball ride over it; the taller invisible box makes it act as a post.
    { nome: 'chinelo-esquerdo', centro: [P.x + FL + 0.02, 0.04, P.y - 0.2], tamanho: [0.26, 0.08, 0.1], yaw: 0.35 },
    { nome: 'chinelo-direito', centro: [P.x + FL + 0.05, 0.04, P.y + 0.2], tamanho: [0.26, 0.08, 0.1], yaw: -0.2 + Math.PI },
    // TV rack down to the floor, so the ball can't roll under the bottom shelf
    { nome: 'rack', centro: [WX - 0.22, 0.23, -0.05], tamanho: [0.4, 0.46, 1.5], yaw: 0 },
    // walls, 10 cm thick, just behind the visible plaster
    { nome: 'parede-fundo', centro: [WX + 0.05, 0.5, 0.6], tamanho: [0.1, 1.0, 4.5], yaw: 0 },
    { nome: 'parede-lateral', centro: [-0.75, 0.5, WZ - 0.05], tamanho: [5, 1.0, 0.1], yaw: 0 },
  ];
  if (incluir === 'todos') caixas.push(
    { nome: 'sofa', centro: [-0.35, 0.3, WZ + 0.47], tamanho: [2.16, 0.6, 0.86], yaw: 0 },
    { nome: 'puff', centro: [-1.35, 0.15, -0.35], tamanho: [0.42, 0.3, 0.42], yaw: 0 },
    { nome: 'vaso', centro: [1.5, 0.11, 1.0], tamanho: [0.22, 0.22, 0.22], yaw: 0 },
  );
  return caixas;
}

// The same boxes as MJCF <geom>s, for the MuJoCo <worldbody>. Assumes the usual mapping between the two worlds:
// three (x, y, z) = MuJoCo (x, z, -y), i.e. MuJoCo is z-up and three's -z is MuJoCo's +y.
// A three yaw (rotation.y) is the same angle about MuJoCo's z.
//
// Invisible but solid: alpha stays 1, because mj_ray skips geoms with alpha 0 and the LiDAR would see through walls.
// What hides them is the group: 4 is off in MuJoCo's viewer by default and is not used by the duck's own geoms (0–3).
// The three.js side must not draw this group (the room is drawn by sala.js). Ray the LiDAR with group 4 enabled.
export const GRUPO_COLISAO = 4;
export function mjcfColisao(caixas = caixasColisao(), { grupo = GRUPO_COLISAO, contype = 1, conaffinity = 1, friccao = '0.8 0.005 0.0001' } = {}) {
  const f = (n) => Number(n.toFixed(4));
  return caixas.map(({ nome, centro: [x, y, z], tamanho: [sx, sy, sz], yaw }) =>
    `<geom name="${nome}" type="box" pos="${f(x)} ${f(-z)} ${f(y)}" size="${f(sx / 2)} ${f(sz / 2)} ${f(sy / 2)}" ` +
    `euler="0 0 ${f(yaw)}" contype="${contype}" conaffinity="${conaffinity}" friction="${friccao}" ` +
    `rgba="0.9 0.3 0.2 1" group="${grupo}"/>`
  ).join('\n');
}
