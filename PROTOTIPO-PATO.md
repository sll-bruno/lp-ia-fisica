# Protótipo: "Faça o pato marcar um gol"

Abertura da landing page sobre IA física e o primeiro modo, **perceber**, em 3D.
O v0 anterior continua em `/v0`.

## Rodar

```bash
npm install
npm run dev
```

Abra http://localhost:3000. `npm run dev` e `npm run build` copiam antes os
binários WASM (`scripts/copy-wasm.mjs` → `public/pato/wasm/`, fora do git).

Atalhos de revisão: `?modo=solto` ou `?modo=visao` pulam direto pro modo.
Tecla `R` reposiciona o pato. Em desenvolvimento, `window.__pato` expõe o
estado no console.

## Fluxo

1. **Controle**: setas/WASD andam (política de andar), espaço chuta com o pé do
   lado da bola (chute cego de 0,5 s). A bola começa um pouco fora do alcance.
2. **Gol**: mensagem "Quem marcou foi você…" e botão "Solte o controle".
3. **Solto**: comandos aleatórios e chutes no vazio; aparece "Dar visão ao pato".
4. **Perceber**: janela com a câmera da cabeça (câmera `head_camera` do MJCF,
   128×96), controle deslizante pixels → bordas (Sobel em shader) → bola
   (máscara de cor + caixa). O pato acha a bola só pela imagem: centróide e
   caixa da máscara magenta → gira, anda, para e chuta. Dá pra arrastar a bola.

## Origem e licença

Física, políticas e modelo vêm do simulador oficial
[pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
(Space no Hugging Face, **Apache-2.0**, cópia da licença em
`public/pato/LICENSE-microduck-simulator.txt`), que por sua vez usa o MJCF e as
políticas de pollen-robotics/microduck e microduck_rl (Apache-2.0).

- `public/pato/robot/`: MJCF `robot_allcollisions.xml`, `kinematics.json`,
  `microduck.glb`, STLs.
- `public/pato/policies/`: andar, chute esquerdo/direito, levantar.
- `lib/pato/duck.js`: rig portado de `app/src/game/duck.js`.
- `lib/pato/engine.js`: laço de controle portado de `app/src/game/game.js`
  (observação de 61 dimensões, 50 Hz, recuperação de queda); arena, gol, modos,
  visão e controlador são novos.
