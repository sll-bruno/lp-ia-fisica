export type HistoryBeat = {
  id: string;
  period: string;
  title: string;
  description: string;
};

export const historyBeats: HistoryBeat[] = [
  {
    id: "automacao",
    period: "1960 — 1980",
    title: "Automação rígida",
    description:
      "Braços programados ponto a ponto, repetindo a mesma sequência para sempre. Sem sensores, sem reação: o robô não sabe o que encontra pela frente.",
  },
  {
    id: "controle",
    period: "1980 — 2000",
    title: "Controle por sensores",
    description:
      "Malhas de realimentação e controle contínuo. O robô passa a medir força, posição e velocidade — e corrigir o próprio movimento em tempo real.",
  },
  {
    id: "slam",
    period: "2000 — 2010",
    title: "Mapeamento e localização",
    description:
      "SLAM: o robô constrói um mapa do ambiente enquanto se localiza nele. Veículos autônomos saem do laboratório e cruzam o deserto no DARPA Grand Challenge.",
  },
  {
    id: "percepcao",
    period: "2010 — 2018",
    title: "Percepção com redes neurais",
    description:
      "Visão computacional profunda reconhece objetos, pessoas e obstáculos em cenas nunca vistas antes. A percepção deixa de depender de regras escritas à mão.",
  },
  {
    id: "imitacao",
    period: "2018 — 2023",
    title: "Aprendizado por imitação",
    description:
      "Em vez de programar cada tarefa, alguém demonstra. Um braço líder, guiado por humano, ensina um braço seguidor — e a demonstração vira dado de treino.",
  },
  {
    id: "fundacao",
    period: "2023 — hoje",
    title: "Modelos de fundação",
    description:
      "Políticas generalistas como a família π0 recebem uma instrução em linguagem natural e generalizam para tarefas novas, com pouco ajuste fino. Autonomia deixa de ser por tarefa — passa a ser por modelo.",
  },
];

export type TrailStage = {
  id: string;
  index: string;
  label: string;
  title: string;
  summary: string;
  mission: string;
};

export const trailStages: TrailStage[] = [
  {
    id: "percepcao",
    index: "01",
    label: "ESTÁGIO 01",
    title: "Percepção",
    summary:
      "Câmeras, LIDAR e sensores de força transformam o mundo físico em dados. Antes de decidir qualquer coisa, o robô precisa enxergar.",
    mission:
      "Missão: identifique três sensores diferentes e o tipo de dado que cada um entrega.",
  },
  {
    id: "localizacao",
    index: "02",
    label: "ESTÁGIO 02",
    title: "Localização & Mapeamento",
    summary:
      "Onde eu estou? O que existe ao meu redor? SLAM resolve as duas perguntas ao mesmo tempo, mesmo em ambientes nunca visitados.",
    mission:
      "Missão: compare localização por GPS e localização por SLAM — quando cada uma falha?",
  },
  {
    id: "planejamento",
    index: "03",
    label: "ESTÁGIO 03",
    title: "Planejamento",
    summary:
      "Com um mapa e uma meta, o robô calcula um caminho — e recalcula a cada obstáculo novo. Planejar é decidir sob incerteza, em tempo real.",
    mission:
      "Missão: desenhe uma rota simples que precise replanejar ao encontrar um obstáculo.",
  },
  {
    id: "controle",
    index: "04",
    label: "ESTÁGIO 04",
    title: "Controle",
    summary:
      "Da decisão ao movimento: controladores traduzem um plano em torque, velocidade e posição de cada junta, dezenas de vezes por segundo.",
    mission:
      "Missão: entenda por que um controlador PID reage diferente a um robô leve e a um robô pesado.",
  },
  {
    id: "manipulacao",
    index: "05",
    label: "ESTÁGIO 05",
    title: "Manipulação",
    summary:
      "O estágio onde tudo se junta: perceber, localizar, planejar e controlar — para pegar, mover e soltar algo no mundo real.",
    mission:
      "Missão prática a seguir: veja um braço robótico aprender uma tarefa observando uma demonstração.",
  },
];

export type DemoStep = {
  id: string;
  label: string;
  title: string;
  description: string;
  detail: string;
};

export const demoSteps: DemoStep[] = [
  {
    id: "demonstracao",
    label: "01 · DEMONSTRAÇÃO",
    title: "Alguém guia o braço líder",
    description:
      "Com compensação de gravidade ativada, uma pessoa move o braço líder livremente — o braço seguidor replica cada posição, com feedback de força.",
    detail: "modo: teleoperação · feedback de força ativo",
  },
  {
    id: "dados",
    label: "02 · DADOS",
    title: "A demonstração vira dataset",
    description:
      "Posições, velocidades e as imagens das câmeras são gravadas quadro a quadro — cada repetição da tarefa é uma trajetória de treino.",
    detail: "trajetórias gravadas: 47 · câmeras: 2",
  },
  {
    id: "politica",
    label: "03 · POLÍTICA",
    title: "Um modelo aprende o padrão",
    description:
      "Uma política de aprendizado por imitação é treinada sobre as demonstrações — e aprende a generalizar alguma coisa a mais, a partir do que viu.",
    detail: "policy: π0 fine-tuning · instrução: \"pegue o cubo laranja\"",
  },
  {
    id: "autonomia",
    label: "04 · AUTONOMIA",
    title: "O braço repete sozinho",
    description:
      "Sem ninguém segurando o líder, o braço seguidor executa a tarefa a partir da instrução em linguagem — percebendo, decidindo e agindo em ciclo.",
    detail: "execução: autônoma · supervisão: humana",
  },
];

export type Achievement = {
  id: string;
  title: string;
  unlocked: boolean;
};

export const achievements: Achievement[] = [
  { id: "primeiro-sinal", title: "Primeiro sinal", unlocked: true },
  { id: "mapa-proprio", title: "Mapa próprio", unlocked: true },
  { id: "rota-recalculada", title: "Rota recalculada", unlocked: true },
  { id: "mao-firme", title: "Mão firme", unlocked: false },
  { id: "primeira-politica", title: "Primeira política", unlocked: false },
  { id: "autonomia-total", title: "Autonomia total", unlocked: false },
];
