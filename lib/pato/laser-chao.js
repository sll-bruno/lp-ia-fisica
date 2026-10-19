// Laser inclinado: anéis de raios saindo do mesmo sensor da cabeça, mas
// apontados pro chão em ângulos diferentes. Num chão vazio cada raio bate no
// piso a uma distância conhecida; se bate antes, tem alguma coisa ali
// (livro, chinelo, caixa). É assim que o pato enxerga o que é baixo demais
// pro laser horizontal. Mesmo princípio de LiDARs inclinados e câmeras de
// profundidade apontadas pro chão em robôs de verdade.

// Distâncias (no chão) onde cada anel toca o piso, de perto pra longe.
const ANEIS = [];
for (let r = 0.22; r < 1.6; r *= 1.1) ANEIS.push(r);
const AZIMUTES = 40;

export function criarLaserChao({ mujoco, model, data, siteId, floorId }) {
  const groups = new Uint8Array([1, 0, 1, 0, 1, 0]); // chão (0), coisas baixas (2), paredes e móveis (4)
  // Saídas do mj_ray (id do geom atingido) precisam ser buffers da própria
  // memória do WASM; um TypedArray comum não é escrito de volta.
  const geomBuf = new mujoco.IntBuffer(1);
  const normalBuf = new mujoco.DoubleBuffer(3);
  const pnt = new Float64Array(3), vec = new Float64Array(3);

  // Varre e chama `cada(hx, hy, bateuEmAlgo, ox, oy)` por raio.
  return function varrer(cada) {
    const p = data.site_xpos, m = data.site_xmat, a = siteId * 3, b = siteId * 9;
    const ox = p[a], oy = p[a + 1], oz = p[a + 2];
    const yaw = Math.atan2(m[b + 5], m[b + 2]);
    pnt[0] = ox; pnt[1] = oy; pnt[2] = oz;
    for (let k = 0; k < AZIMUTES; k++) {
      const az = yaw + (k / AZIMUTES) * 2 * Math.PI;
      const c = Math.cos(az), s = Math.sin(az);
      for (const r of ANEIS) {
        const L = Math.hypot(r, oz);
        vec[0] = (c * r) / L; vec[1] = (s * r) / L; vec[2] = -oz / L;
        const d = mujoco.mj_ray(model, data, pnt, vec, groups, true, -1, geomBuf, normalBuf);
        if (d < 0) continue;
        const hx = ox + vec[0] * d, hy = oy + vec[1] * d;
        const noChao = geomBuf.GetView()[0] === floorId;
        cada(hx, hy, !noChao, ox, oy);
        if (!noChao) break; // bateu em algo: os anéis de trás estão na sombra
      }
    }
  };
}
