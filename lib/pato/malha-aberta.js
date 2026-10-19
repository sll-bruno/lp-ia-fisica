// Mover: "sem equilíbrio" é a marcha da própria política gravada e tocada
// de volta em malha aberta, sem olhar sensor nenhum.
//
// Gravamos as ações (alvos das 14 juntas) que a política produz andando
// devagar, achamos o período do passo (o atraso em que as ações se repetem)
// e guardamos um ciclo inteiro. Tocar o ciclo em loop reproduz o
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
    // Período: o atraso (0,3 a 1,6 s) em que o vetor inteiro das 14 ações
    // mais se repete, pra o fim do ciclo emendar no começo sem tranco.
    ciclo() {
      if (buf.length < 120) return null;
      const n = buf.length, ult = buf[n - 1];
      let melhor = Infinity, periodo = 0;
      for (let lag = 15; lag <= Math.min(80, n - 2); lag++) {
        const a = buf[n - 1 - lag];
        let d = 0;
        for (let j = 0; j < numJuntas; j++) d += (a[j] - ult[j]) ** 2;
        if (d < melhor) { melhor = d; periodo = lag; }
      }
      if (!periodo) return null;
      return buf.slice(n - periodo).map((a) => Float32Array.from(a));
    },
    limpar() { buf.length = 0; },
  };
}
