// Localização do pato: mapa conhecido da arena em 2D, raycast contra esse
// mapa e um filtro de partículas (localização de Monte Carlo, com a
// variante "aumentada" de Thrun, Burgard e Fox pra se recuperar de um
// sequestro). Tudo em coordenadas MJCF no plano do chão: x, y, rumo.

// Mapa: caixas alinhadas aos eixos { cx, cy, hx, hy }. Só o que está na
// altura do laser entra (paredes, traves e laterais do gol); o que é mais
// baixo (bola, livro) o laser não vê e o mapa não tem.
export function rayBoxes(boxes, ox, oy, dx, dy, maxR) {
  let best = maxR;
  for (let i = 0; i < boxes.length; i++) {
    const b = boxes[i];
    // Slab: interseção do raio com [cx-hx, cx+hx] x [cy-hy, cy+hy].
    let t0 = 0, t1 = best;
    if (Math.abs(dx) < 1e-9) {
      if (ox < b.cx - b.hx || ox > b.cx + b.hx) continue;
    } else {
      let a = (b.cx - b.hx - ox) / dx, c = (b.cx + b.hx - ox) / dx;
      if (a > c) { const s = a; a = c; c = s; }
      if (a > t0) t0 = a;
      if (c < t1) t1 = c;
      if (t0 > t1) continue;
    }
    if (Math.abs(dy) < 1e-9) {
      if (oy < b.cy - b.hy || oy > b.cy + b.hy) continue;
    } else {
      let a = (b.cy - b.hy - oy) / dy, c = (b.cy + b.hy - oy) / dy;
      if (a > c) { const s = a; a = c; c = s; }
      if (a > t0) t0 = a;
      if (c < t1) t1 = c;
      if (t0 > t1) continue;
    }
    if (t0 > 0 && t0 < best) best = t0;
  }
  return best;
}

function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

export class ParticleFilter {
  constructor({ boxes, bounds, n = 1500, maxRange = 3, sigma = 0.08 }) {
    this.boxes = boxes;
    this.bounds = bounds; // { xmin, xmax, ymin, ymax } onde pode haver pato
    this.n = n;
    this.maxRange = maxRange;
    this.sigma = sigma;
    this.x = new Float32Array(n);
    this.y = new Float32Array(n);
    this.t = new Float32Array(n);
    this.w = new Float64Array(n);
    this.tmp = { x: new Float32Array(n), y: new Float32Array(n), t: new Float32Array(n) };
    // MCL aumentado: médias lentas e rápidas da verossimilhança.
    this.wSlow = 0;
    this.wFast = 0;
    this.injected = 0; // partículas aleatórias na última atualização
    this.scatter();
  }

  randomPose(i) {
    const b = this.bounds;
    this.x[i] = b.xmin + Math.random() * (b.xmax - b.xmin);
    this.y[i] = b.ymin + Math.random() * (b.ymax - b.ymin);
    this.t[i] = (Math.random() * 2 - 1) * Math.PI;
  }

  // Espalha tudo pela arena (localização global, sem palpite inicial).
  scatter() {
    for (let i = 0; i < this.n; i++) { this.randomPose(i); this.w[i] = 1 / this.n; }
    this.wSlow = 0;
    this.wFast = 0;
    this.updates = 0;
  }

  // Movimento: deslocamento no referencial do tronco (avanço dx, giro dth)
  // com ruído. O avanço vem da velocidade comandada, que a política cumpre
  // só em parte, por isso o fator de escala largo.
  predict(dx, dth) {
    for (let i = 0; i < this.n; i++) {
      const s = 0.25 + Math.random() * 0.95;
      const d = dx * s + gauss() * 0.006;
      const th = this.t[i] + dth + gauss() * 0.03;
      this.x[i] += Math.cos(th) * d + gauss() * 0.004;
      this.y[i] += Math.sin(th) * d + gauss() * 0.004;
      this.t[i] = wrap(th);
      const b = this.bounds;
      if (this.x[i] < b.xmin || this.x[i] > b.xmax || this.y[i] < b.ymin || this.y[i] > b.ymax) {
        this.randomPose(i);
      }
    }
  }

  // Medição: feixes { a (ângulo relativo ao sensor), r (alcance medido) }.
  // sensor = deslocamento do laser no referencial do tronco { ox, oy, oth }.
  update(beams, sensor) {
    const { n, boxes, maxRange, sigma } = this;
    const inv2s2 = 1 / (2 * sigma * sigma);
    this.temper = Math.min(0.25, 0.03 + 0.012 * this.updates++);
    const zHit = 0.85, zRand = 0.15 / maxRange;
    const norm = 1 / (Math.sqrt(2 * Math.PI) * sigma);
    let maxLog = -Infinity;
    const logsArr = new Float64Array(n);
    let likeSum = 0;
    for (let i = 0; i < n; i++) {
      const c = Math.cos(this.t[i]), s = Math.sin(this.t[i]);
      const sx = this.x[i] + c * sensor.ox - s * sensor.oy;
      const sy = this.y[i] + s * sensor.ox + c * sensor.oy;
      const st = this.t[i] + sensor.oth;
      let lsum = 0;
      for (let k = 0; k < beams.length; k++) {
        const a = st + beams[k].a;
        const exp = rayBoxes(boxes, sx, sy, Math.cos(a), Math.sin(a), maxRange);
        const e = beams[k].r - exp;
        lsum += Math.log(zHit * norm * Math.exp(-e * e * inv2s2) + zRand);
      }
      // Verossimilhança média por feixe (pra MCL aumentado) e peso temperado.
      likeSum += Math.exp(lsum / beams.length);
      // Recozimento: logo depois de espalhar, a verossimilhança é bem
      // suavizada, pra nuvem não colapsar cedo demais num máximo local (a
      // sala quadrada tem poses quase simétricas). Endurece em ~2 s.
      logsArr[i] = lsum * this.temper;
      if (logsArr[i] > maxLog) maxLog = logsArr[i];
    }
    let sum = 0;
    for (let i = 0; i < n; i++) { this.w[i] *= Math.exp(logsArr[i] - maxLog); sum += this.w[i]; }
    for (let i = 0; i < n; i++) this.w[i] /= sum || 1;

    const wAvg = likeSum / n;
    this.wSlow = this.wSlow === 0 ? wAvg : this.wSlow + 0.02 * (wAvg - this.wSlow);
    this.wFast = this.wFast === 0 ? wAvg : this.wFast + 0.3 * (wAvg - this.wFast);
    const pRandom = Math.max(0, 1 - this.wFast / this.wSlow);
    // Sequestro detectado: volta a suavizar enquanto a nuvem se refaz.
    if (pRandom > 0.3) this.updates = Math.min(this.updates, 4);

    let neff = 0;
    for (let i = 0; i < n; i++) neff += this.w[i] * this.w[i];
    neff = 1 / neff;
    if (neff < n * 0.5 || pRandom > 0) this.resample(pRandom);
    return { neff, pRandom, wAvg };
  }

  // Reamostragem de baixa variância, trocando uma fração por poses
  // aleatórias quando a verossimilhança despenca (sequestro).
  resample(pRandom) {
    const { n, tmp } = this;
    const step = 1 / n;
    let r = Math.random() * step, c = this.w[0], j = 0;
    let injected = 0;
    for (let i = 0; i < n; i++) {
      const u = r + i * step;
      while (u > c && j < n - 1) { j++; c += this.w[j]; }
      if (Math.random() < pRandom) {
        const b = this.bounds;
        tmp.x[i] = b.xmin + Math.random() * (b.xmax - b.xmin);
        tmp.y[i] = b.ymin + Math.random() * (b.ymax - b.ymin);
        tmp.t[i] = (Math.random() * 2 - 1) * Math.PI;
        injected++;
      } else {
        tmp.x[i] = this.x[j]; tmp.y[i] = this.y[j]; tmp.t[i] = this.t[j];
      }
    }
    this.x.set(tmp.x); this.y.set(tmp.y); this.t.set(tmp.t);
    this.w.fill(1 / n);
    this.injected = injected;
  }

  // Estimativa: média ponderada (rumo pela média circular) e espalhamento.
  estimate() {
    let x = 0, y = 0, c = 0, s = 0;
    for (let i = 0; i < this.n; i++) {
      const w = this.w[i];
      x += w * this.x[i]; y += w * this.y[i];
      c += w * Math.cos(this.t[i]); s += w * Math.sin(this.t[i]);
    }
    let spread = 0;
    for (let i = 0; i < this.n; i++) spread += this.w[i] * Math.hypot(this.x[i] - x, this.y[i] - y);
    return { x, y, t: Math.atan2(s, c), spread };
  }
}
