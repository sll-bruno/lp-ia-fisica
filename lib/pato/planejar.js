// Decidir: grade de ocupação e A*.
//
// A grade cobre o piso livre da sala em células de CELULA metros. Cada
// célula guarda um "quanto acho que está ocupada" que o laser atualiza:
// raio que bate em algo marca a célula do impacto, raio que passa livre por
// uma célula a limpa aos poucos (assim um objeto arrastado pra outro lugar
// some do mapa quando o pato olha de novo pra onde ele estava).

export const CELULA = 0.04;

export class Grade {
  constructor(bounds) {
    this.b = bounds;
    this.nx = Math.ceil((bounds.xmax - bounds.xmin) / CELULA);
    this.ny = Math.ceil((bounds.ymax - bounds.ymin) / CELULA);
    this.ocup = new Float32Array(this.nx * this.ny); // 0 livre .. 1 ocupada
    this.versao = 0;
  }

  idx(x, y) {
    const i = Math.floor((x - this.b.xmin) / CELULA), j = Math.floor((y - this.b.ymin) / CELULA);
    if (i < 0 || j < 0 || i >= this.nx || j >= this.ny) return -1;
    return j * this.nx + i;
  }
  centro(k) {
    const i = k % this.nx, j = (k / this.nx) | 0;
    return [this.b.xmin + (i + 0.5) * CELULA, this.b.ymin + (j + 0.5) * CELULA];
  }

  // Um raio do laser: livre de (ox,oy) até o impacto; impacto ocupado.
  raio(ox, oy, hx, hy, bateu) {
    const d = Math.hypot(hx - ox, hy - oy);
    const passos = Math.floor(d / (CELULA * 0.7));
    for (let s = 1; s < passos - 1; s++) {
      const k = this.idx(ox + ((hx - ox) * s) / passos, oy + ((hy - oy) * s) / passos);
      if (k >= 0 && this.ocup[k] > 0) { this.ocup[k] = Math.max(0, this.ocup[k] - 0.2); this.versao++; }
    }
    if (bateu) {
      const k = this.idx(hx, hy);
      if (k >= 0 && this.ocup[k] < 1) { this.ocup[k] = Math.min(1, this.ocup[k] + 0.5); this.versao++; }
    }
  }

  // Grade "inflada" pelo raio do pato: células onde o centro dele não cabe.
  inflar(raio) {
    const r = Math.ceil(raio / CELULA);
    const out = new Uint8Array(this.nx * this.ny);
    for (let j = 0; j < this.ny; j++) for (let i = 0; i < this.nx; i++) {
      if (this.ocup[j * this.nx + i] < 0.5) continue;
      for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
        if (di * di + dj * dj > r * r) continue;
        const ii = i + di, jj = j + dj;
        if (ii >= 0 && jj >= 0 && ii < this.nx && jj < this.ny) out[jj * this.nx + ii] = 1;
      }
    }
    return out;
  }
}

// A* em 8 vizinhos sobre uma grade bloqueada (Uint8Array, 1 = bloqueado).
// `extra(k)` devolve custo adicional por célula (ex.: evitar a bola).
export function aEstrela(grade, bloq, de, para, extra = null) {
  const { nx, ny } = grade;
  const s = grade.idx(de[0], de[1]), t = grade.idx(para[0], para[1]);
  if (s < 0 || t < 0) return null;
  const n = nx * ny;
  const g = new Float32Array(n).fill(Infinity);
  const veio = new Int32Array(n).fill(-1);
  const fechado = new Uint8Array(n);
  // Heap binário simples por f = g + h.
  const heap = [];
  const ti = t % nx, tj = (t / nx) | 0;
  const h = (k) => { const di = Math.abs((k % nx) - ti), dj = Math.abs(((k / nx) | 0) - tj); return Math.max(di, dj) + 0.414 * Math.min(di, dj); };
  const push = (k, f) => {
    heap.push([f, k]);
    let c = heap.length - 1;
    while (c > 0) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; }
  };
  const pop = () => {
    const top = heap[0], last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let c = 0;
      for (;;) {
        const l = 2 * c + 1, r = l + 1;
        let m = c;
        if (l < heap.length && heap[l][0] < heap[m][0]) m = l;
        if (r < heap.length && heap[r][0] < heap[m][0]) m = r;
        if (m === c) break;
        [heap[m], heap[c]] = [heap[c], heap[m]]; c = m;
      }
    }
    return top;
  };
  g[s] = 0;
  push(s, h(s));
  const viz = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, 1.414], [1, -1, 1.414], [-1, 1, 1.414], [-1, -1, 1.414]];
  let expandidos = 0;
  while (heap.length) {
    const [, k] = pop();
    if (fechado[k]) continue;
    fechado[k] = 1;
    expandidos++;
    if (k === t) break;
    const i = k % nx, j = (k / nx) | 0;
    for (const [di, dj, c] of viz) {
      const ii = i + di, jj = j + dj;
      if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
      const kk = jj * nx + ii;
      // A célula de partida pode estar "bloqueada" (o pato encostado num
      // móvel); deixa sair dela.
      if (bloq[kk] && kk !== t) continue;
      const ng = g[k] + c + (extra ? extra(kk) : 0);
      if (ng < g[kk]) { g[kk] = ng; veio[kk] = k; push(kk, ng + h(kk)); }
    }
  }
  if (veio[t] < 0 && s !== t) return null;
  const caminho = [];
  for (let k = t; k >= 0; k = veio[k]) { caminho.push(grade.centro(k)); if (k === s) break; }
  caminho.reverse();
  return { caminho: suavizar(caminho, grade, bloq), expandidos };
}

// Corta cantos: liga pontos que se enxergam sem atravessar células bloqueadas.
function suavizar(pts, grade, bloq) {
  if (pts.length < 3) return pts;
  const livre = (a, b) => {
    const d = Math.hypot(b[0] - a[0], b[1] - a[1]);
    const n = Math.ceil(d / (CELULA * 0.5));
    for (let s = 1; s < n; s++) {
      const k = grade.idx(a[0] + ((b[0] - a[0]) * s) / n, a[1] + ((b[1] - a[1]) * s) / n);
      if (k >= 0 && bloq[k]) return false;
    }
    return true;
  };
  const out = [pts[0]];
  let i = 0;
  while (i < pts.length - 1) {
    let j = pts.length - 1;
    while (j > i + 1 && !livre(pts[i], pts[j])) j--;
    out.push(pts[j]);
    i = j;
  }
  return out;
}
