// Mover: "sem equilíbrio" é a marcha da própria política gravada e tocada
// de volta em malha aberta, sem olhar sensor nenhum.
//
// Gravamos as ações (alvos das 14 juntas) que a política produz andando
// devagar, achamos o período do passo pela autocorrelação de uma junta do
// quadril e guardamos um ciclo inteiro. Tocar o ciclo em loop reproduz o
// mesmo movimento de pernas, mas sem correção: se algo empurra o pato, nada
// no ciclo muda pra compensar.

export function criarGravador(numJuntas, passosMax = 250) {
  const buf = [];
  return {
    gravar(acao) {
      buf.push(Float32Array.from(acao));
      if (buf.length > passosMax) buf.shift();
    },
    get tamanho() { return buf.length; },
    // Um ciclo do passo, a partir do fim da gravação.
    ciclo(junta = 2) {
      if (buf.length < 80) return null;
      const n = buf.length;
      const x = buf.map((a) => a[junta]);
      const media = x.reduce((s, v) => s + v, 0) / n;
      // Autocorrelação; o período é o primeiro pico depois que ela fica
      // negativa (meio passo pro outro lado).
      const ac = [];
      for (let lag = 0; lag < Math.min(90, n / 2); lag++) {
        let s = 0;
        for (let i = lag; i < n; i++) s += (x[i] - media) * (x[i - lag] - media);
        ac.push(s / (n - lag));
      }
      let periodo = 0, negativo = false;
      for (let lag = 1; lag < ac.length - 1; lag++) {
        if (ac[lag] < 0) negativo = true;
        if (negativo && ac[lag] > 0 && ac[lag] >= ac[lag - 1] && ac[lag] >= ac[lag + 1]) { periodo = lag; break; }
      }
      if (!periodo) return null;
      return buf.slice(n - periodo).map((a) => Float32Array.from(a));
    },
    limpar() { buf.length = 0; },
  };
}
