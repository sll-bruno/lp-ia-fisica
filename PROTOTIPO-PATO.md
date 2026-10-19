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

1. **Controle**: WASD andam (setas também), Espaço chuta na hora com o pé do
   lado da bola (chute cego de 0,5 s), perto ou longe, parado ou andando.
   Chutando andando ele costuma cair e se levanta sozinho, de propósito: antes
   de aprender, o pato cai. A bola começa um pouco fora do alcance.
2. **Gol**: mensagem "Quem marcou foi você…" e botão "Solte o controle", que
   ficam até o clique. Em qualquer modo, ~4 s depois de um gol a cena reseta
   (pato e bola voltam às posições iniciais).
3. **Solto**: comandos aleatórios e chutes no vazio na hora (também cai às
   vezes); aparece "Dar visão ao pato".
4. **Perceber**: janela com a câmera da cabeça (câmera `head_camera` do MJCF,
   128×96), controle deslizante pixels → bordas (Sobel em shader) → bola
   (máscara de cor + caixa). O pato acha a bola só pela imagem: centróide e
   caixa da máscara magenta → gira, anda, para e chuta. Dá pra arrastar a bola.

5. **Se localizar** (`?modo=localizar`):
   - **Gancho**: depois da visão, o pato recomeça de costas pro gol e chuta
     pra parede. "Ele vê a bola. Mas não sabe onde está, nem onde fica o gol."
   - **Contar os passos (odometria)**: um pato fantasma integra só o comando
     que a política recebeu (avanço e giro), nunca a pose real. O desvio é o
     real entre comando e execução: a política anda ~0,1 m/s quando o comando
     pede 0,25, e gira ~1,5 rad/s quando pede 2,5. Medido: 0,59 m de erro
     depois de 4 s andando reto; 1,3 m depois de 15 s de passeio.
   - **Ligar o laser**: 90 raios horizontais por varredura, a 10 Hz, saindo do
     sensor "tof" da cabeça via `mj_ray` (só geoms do grupo 0: paredes, gol e
     uma caixa no canto; o pato, a bola e o livro ficam fora). Os impactos
     viram o mapa no chão (grade de 1,5 cm). Filtro de partículas 2D (1500
     partículas, 45 feixes por atualização, MCL aumentado contra sequestro) no
     mapa conhecido. Movimento do filtro: avanço comandado com ruído largo +
     giro do giroscópio da IMU.
   - **Sequestro**: arrastar o pato com o mouse o carrega pelo ar (pose do
     corpo livre escrita direto) e o larga em outro lugar.
   - **Deixar ele jogar**: com a pose estimada, ele vai a um ponto atrás da
     bola na linha bola-gol, se alinha com o gol e ataca reto. Um livro entre
     a bola e o gol (geom estático baixo, fora do mapa do laser) fica no
     caminho e ele não desvia.

6. **Decidir** (`?modo=decidir`): grade de ocupação de 4 cm montada pelo que
   os lasers tocam: o horizontal (paredes, móveis) e um laser inclinado, com
   anéis de raios saindo do mesmo sensor da cabeça rumo ao chão; se o raio
   bate antes do chão, tem algo baixo ali (livro, chinelo, caixa), e se chega
   no chão a célula é limpa. A* em 8 vizinhos sobre a grade inflada pelo raio
   do pato (8 cm), da pose estimada até um ponto atrás da bola na linha
   bola-gol (tenta ângulos e distâncias vizinhos se esse estiver bloqueado);
   a rota é suavizada e seguida mirando 18 cm à frente. O visitante arrasta o
   livro, o chinelo e a caixa; a rota se refaz a cada 0,4 s com o que o pato
   já viu. Rota em tinta escura, ponto perseguido em laranja.

## A sala de casa

A cena visual é a sala aprovada pela thread de design, em `lib/cena/`
(copiada sem alterações: `cena.js`, `sala.js`, `campo.js`, `luz.js`,
`percepcao.js`, `util.js`, `r3f.jsx`; o `r3f.jsx` não é usado, o protótipo
não usa react-three-fiber). O motor monta `criarCena()` na própria cena e
desenha cada quadro com `Percepcao.render`, seguindo o tronco do pato.

A física da sala está em `lib/pato/sala-fisica.js`: caixas e cilindros
aproximando paredes, rack, sofá, puff, vaso, pilha de livros, caneca, tênis
e os dois chinelos do gol, mais dois limites invisíveis nos lados abertos.
O rack é fechado até o chão (o vão de 11 cm sob ele engoliria a bola). O
laser enxerga só paredes, rack, sofá, puff e vaso (o resto é baixo demais).

A bola é a de tênis da cena (raio 3,35 cm, 57 g). A visão procura o
amarelo-esverdeado dela (matiz 50–78) e fica com a maior mancha conectada; a
câmera da cabeça usa o mesmo tom da Percepcao (ACES, exposição 1,1).

## Ajustes que não existem no simulador oficial

- **Paredes de 30 cm e gol de 28 cm** (antes 12 e 18): o sensor da cabeça
  fica a ~22 cm do chão e o laser precisa enxergar a sala.
- **Caixa alta num canto**: a sala quadrada tem poses quase simétricas, e o
  filtro às vezes se fixava na pose girada 180°. A caixa quebra a simetria.

- **Giro mais forte**: o teclado manda 2,5 rad/s parado e 1,8 andando (o oficial
  usa 1,0). A política gira parada a ~0,1 rad/s com 1,0; com 2,5 gira ~1,5 rad/s
  sem cair. Fora da faixa de treino, estável nos testes.
- **Zona morta**: parada, a política ignora comandos fracos. O modo solto e o
  controlador da visão usam comandos fortes e dão um empurrão quando o pato empaca.
- **Chute parado, só no modo visão**: o chute passa por uma preparação (comando
  zero até o pato ficar firme), pra percepção ficar legível. A cabeça volta ao
  neutro antes do chute: inclinada, o chute direito derrubava. Em todos os
  modos há 1 s de pausa depois do chute (o oficial usa 0,4 s).
- **Impulso no chute**: parado, o pé só alcança ~11 cm. Se o pé toca a bola, ela
  sai com pelo menos 1,4 m/s; se a bola estava ao alcance e o pé passou raspando,
  ela recebe o mesmo impulso no passo em que o pé está mais à frente.
- **Bola presa**: fora do modo controle, bola parada colada na parede por 6 s
  volta pro campo.

## Origem e licença

Física, políticas e modelo vêm do simulador oficial
[pollen-robotics/microduck-simulator](https://huggingface.co/spaces/pollen-robotics/microduck-simulator)
(Space no Hugging Face, **Apache-2.0**, cópia da licença em
`public/pato/LICENSE-microduck-simulator.txt`), que por sua vez usa o MJCF e as
políticas de pollen-robotics/microduck e microduck_rl (Apache-2.0).

- `public/pato/robot/`: MJCF `robot_allcollisions.xml`, `kinematics.json`,
  `microduck.glb`, STLs.

**Atenção, malhas 3D:** os STLs do Microduck (e o `microduck.glb`, que é
derivado deles) são **CC BY-NC-SA 4.0** (Creative Commons
Atribuição-NãoComercial-CompartilhaIgual), não Apache-2.0. Ver
`public/pato/LICENSE-MODELS-microduck_rl.md`. Pode usar em projeto não
comercial, creditando a Pollen Robotics com link para
https://github.com/pollen-robotics/microduck_rl e para a licença. Uso
comercial exigiria falar com a Pollen. O MJCF, as políticas e o código seguem
Apache-2.0.
- `public/pato/policies/`: andar, chute esquerdo/direito, levantar.
- `lib/pato/duck.js`: rig portado de `app/src/game/duck.js`.
- `lib/pato/engine.js`: laço de controle portado de `app/src/game/game.js`
  (observação de 61 dimensões, 50 Hz, recuperação de queda); arena, gol, modos,
  visão e controlador são novos.

## Ideia pra depois: modo aprender (não implementado)

Mostrar o treino por reforço: no começo o pato cai muito (como no controle e
no modo solto, chutando andando) e, ao longo do treino, cai cada vez menos.
Isso também explica por que no modo visão ele já não cai: ali ele para, se
firma e chuta, que é o comportamento "depois do treino".
