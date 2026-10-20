"use client";

import { useEffect, useRef, useState } from "react";
import { createPato, VISION_W, VISION_H } from "@/lib/pato/engine";

type Phase =
  | "carregando" | "controle" | "gol" | "solto" | "visao"
  | "gancho" | "odometria" | "laser" | "jogar" | "decidir" | "mover" | "aprender" | "patins";
type Mode = "solto" | "visao" | "gancho" | "odometria" | "laser" | "jogar" | "decidir" | "mover" | "aprender" | "patins";
type MoverStatus = { equilibrio: boolean; quedas: { com: number; sem: number }; pronto: boolean };
type AprenderStatus = { geracao: number; geracoes: number; sigma: number; quedas: number[]; fim: boolean; progresso: number };
// Atalhos pra revisão: ?modo=solto, ?modo=visao, ?modo=localizar.
const SHORTCUTS: Record<string, Mode> = {
  solto: "solto", visao: "visao", localizar: "gancho", decidir: "decidir",
  mover: "mover", aprender: "aprender", patins: "patins",
};
type Api = NonNullable<Awaited<ReturnType<typeof createPato>>>;

const LAYERS = ["pixels", "bordas", "bola"];

export default function PatoExperience() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const visionRef = useRef<HTMLCanvasElement>(null);
  const apiRef = useRef<Api | null>(null);
  const phaseRef = useRef<Phase>("carregando");
  const [phase, setPhaseState] = useState<Phase>("carregando");
  // O final troca de corpo: um motor novo com o pato de patins.
  const [variante, setVariante] = useState<"pernas" | "patins">(() =>
    typeof window !== "undefined" && new URLSearchParams(window.location.search).get("modo") === "patins"
      ? "patins" : "pernas");
  const [progress, setProgress] = useState<string | null>("preparando");
  const [failed, setFailed] = useState<string | null>(null);
  // Botão de avançar de cada fase aparece depois de um tempo nela.
  const [canAdvance, setCanAdvance] = useState(false);
  const [layer, setLayer] = useState(0);
  const [toast, setToast] = useState<string | null>(null);
  const [pressed, setPressed] = useState<Set<string>>(() => new Set());
  const [moverSt, setMoverSt] = useState<MoverStatus | null>(null);
  // Tela de toque: joystick e botão de chute no lugar do teclado.
  const [toqueAtivo, setToqueAtivo] = useState(false);
  useEffect(() => {
    const mq = window.matchMedia("(pointer: coarse)");
    const f = () => setToqueAtivo(mq.matches);
    f();
    mq.addEventListener("change", f);
    return () => mq.removeEventListener("change", f);
  }, []);
  const [aprenderSt, setAprenderSt] = useState<AprenderStatus | null>(null);

  const setPhase = (p: Phase) => {
    phaseRef.current = p;
    setCanAdvance(false);
    setPhaseState(p);
  };

  useEffect(() => {
    let api: Api | null = null;
    let cancelled = false;
    // O StrictMode monta, desmonta e monta de novo em sequência: o atraso
    // evita dois motores disputando o mesmo canvas.
    const start = window.setTimeout(async () => {
      try {
        const created = await createPato({
          canvas: canvasRef.current!,
          visionCanvas: visionRef.current!,
          variante,
          onProgress: (msg: string | null) => setProgress(msg),
          onGoal: () => {
            const p = phaseRef.current;
            if (p === "controle") setPhase("gol");
            else if (p !== "gol") {
              setToast(p === "odometria" || p === "laser" ? "Gol." : "Gol do pato.");
              window.setTimeout(() => setToast(null), 2200);
            }
          },
        });
        if (cancelled) created?.dispose();
        else if (created) {
          api = created;
          apiRef.current = created;
          const modo = variante === "patins"
            ? "patins"
            : SHORTCUTS[new URLSearchParams(window.location.search).get("modo") ?? ""];
          if (modo) {
            created.setMode(modo);
            setPhase(modo);
          } else setPhase("controle");
        }
      } catch (err) {
        console.error(err);
        setFailed(err instanceof Error ? err.message : String(err));
      }
    }, 0);
    return () => {
      cancelled = true;
      window.clearTimeout(start);
      api?.dispose();
      apiRef.current = null;
    };
  }, [variante]);

  // Teclas acesas na dica enquanto estão apertadas.
  useEffect(() => {
    const CODES: Record<string, string> = {
      KeyW: "W", ArrowUp: "W", KeyA: "A", ArrowLeft: "A",
      KeyS: "S", ArrowDown: "S", KeyD: "D", ArrowRight: "D", Space: "Espaço",
    };
    const update = (code: string, on: boolean) => {
      const k = CODES[code];
      if (!k) return;
      setPressed((prev) => {
        if (prev.has(k) === on) return prev;
        const next = new Set(prev);
        if (on) next.add(k);
        else next.delete(k);
        return next;
      });
    };
    const down = (e: KeyboardEvent) => update(e.code, true);
    const up = (e: KeyboardEvent) => update(e.code, false);
    const blur = () => setPressed(new Set());
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    window.addEventListener("blur", blur);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
      window.removeEventListener("blur", blur);
    };
  }, []);

  useEffect(() => {
    const wait: Partial<Record<Phase, number>> = {
      solto: 3500, visao: 9000, gancho: 7000, odometria: 9000, laser: 9000, jogar: 12000,
      decidir: 20000, mover: 15000,
    };
    const ms = wait[phase];
    if (ms === undefined) return;
    const t = window.setTimeout(() => setCanAdvance(true), ms);
    return () => window.clearTimeout(t);
  }, [phase]);

  useEffect(() => {
    apiRef.current?.setVisionLayer(layer);
  }, [layer, phase]);

  // Mover e aprender: placar lido do motor.
  useEffect(() => {
    if (phase !== "mover" && phase !== "aprender") return;
    const id = window.setInterval(() => {
      const api = apiRef.current;
      if (!api) return;
      if (phase === "mover") setMoverSt(api.moverStatus());
      else setAprenderSt(api.aprenderStatus());
    }, 300);
    return () => window.clearInterval(id);
  }, [phase]);

  // Embaixo de cada texto a sala vira papel (Percepcao.reservar, até 4).
  useEffect(() => {
    const els = [...document.querySelectorAll<HTMLElement>("[data-reserva]")];
    apiRef.current?.reservar(els);
  }, [phase, canAdvance, toast, aprenderSt?.fim]);

  const go = (m: Mode) => {
    apiRef.current?.setMode(m);
    setPhase(m);
  };
  const NEXT: Partial<Record<Phase, [Mode, string]>> = {
    solto: ["visao", "Dar visão ao pato"],
    visao: ["gancho", "Ensinar o pato a se localizar"],
    gancho: ["odometria", "Contar os passos"],
    odometria: ["laser", "Ligar o laser"],
    laser: ["jogar", "Deixar ele jogar"],
    jogar: ["decidir", "Ensinar o pato a escolher o caminho"],
    decidir: ["mover", "E se alguém empurrar?"],
    mover: ["aprender", "Quem ensinou?"],
  };
  const next = NEXT[phase];
  const showsVision = phase === "visao" || phase === "gancho" || phase === "jogar";
  const localizing = phase === "gancho" || phase === "odometria" || phase === "laser" || phase === "jogar";
  const decidindo = phase === "decidir";
  const keysHint = toqueAtivo ? null : (
    <div className="pato-keys" aria-label="WASD andar, Espaço chutar">
      {["W", "A", "S", "D"].map((k) => (
        <span key={k} className={`pato-key${pressed.has(k) ? " is-down" : ""}`}>
          {k}
        </span>
      ))}
      <span className="pato-key-label">andar</span>
      {phase === "controle" && (
        <>
          <span className="pato-key-sep">·</span>
          <span className={`pato-key pato-key-wide${pressed.has("Espaço") ? " is-down" : ""}`}>
            Espaço
          </span>
          <span className="pato-key-label">chutar</span>
        </>
      )}
    </div>
  );

  const layerIdx = Math.round(layer);
  const usaTeclado = phase === "controle" || phase === "odometria" || phase === "laser" || phase === "patins";

  return (
    <div className="pato-root">
      <canvas ref={canvasRef} className="pato-canvas" />

      {phase === "carregando" && (
        <div className="pato-center">
          <p className="pato-muted">{failed ? `não deu pra carregar: ${failed}` : `${progress ?? "quase lá"}…`}</p>
        </div>
      )}

      {phase === "controle" && (
        <div data-reserva className="pato-top">
          <h1 className="pato-title">Faça o pato marcar um gol</h1>
          {keysHint}
        </div>
      )}

      {phase === "gol" && (
        <div data-reserva className="pato-top pato-top-interactive">
          <div className="pato-card">
            <p className="pato-lead">
              Quem marcou foi você. Seus olhos acharam a bola, você calculou o ângulo e decidiu a
              hora. O pato só chutou.
            </p>
            <button className="pato-button" onClick={() => go("solto")}>
              Solte o controle
            </button>
          </div>
        </div>
      )}

      {next && canAdvance && (
        <div className="pato-bottom">
          <button className="pato-button" onClick={() => go(next[0])}>
            {next[1]}
          </button>
        </div>
      )}

      {phase === "visao" && (
        <>
          <p data-reserva className="pato-brain">
            Em 1959, Hubel e Wiesel acharam neurônios no córtex visual que só disparam pra linhas
            numa certa inclinação. As bordas que você vê aqui são isso.
          </p>
          <aside data-reserva className="pato-ficha">
            <p>
              <span className="pato-ficha-label">pra estudar</span> convolução, OpenCV
            </p>
            <p>
              <span className="pato-ficha-label">projeto opcional</span> dar uma câmera ao seu
              próprio simulador
            </p>
          </aside>
        </>
      )}

      {phase === "gancho" && (
        <div data-reserva className="pato-top">
          <h1 className="pato-title pato-title-small">
            Ele vê a bola. Mas não sabe onde está, nem onde fica o gol.
          </h1>
        </div>
      )}

      {phase === "odometria" && (
        <>
          <div data-reserva className="pato-left">
            <p>
              Ande de olhos fechados até a porta e você erra do mesmo jeito. Seu cérebro também
              conta passos, com as células de grade (Prêmio Nobel de 2014).
            </p>
            {keysHint}
          </div>
          <p data-reserva className="pato-legend">
            <span className="pato-swatch pato-swatch-ghost" /> onde o pato acha que está, contando
            só os comandos que recebeu
          </p>
        </>
      )}

      {phase === "laser" && (
        <>
          <div data-reserva className="pato-left">
            <p>
              As células de lugar no hipocampo (O&apos;Keefe, 1971) só disparam quando você está
              num ponto específico do espaço.
            </p>
            {keysHint}
            <p className="pato-muted pato-small">arraste o pato pra sequestrá-lo</p>
          </div>
          <p data-reserva className="pato-legend">
            <span className="pato-swatch pato-swatch-ray" /> o laser agora ·{" "}
            <span className="pato-swatch pato-swatch-map" /> o que ele já tocou ·{" "}
            <span className="pato-swatch pato-swatch-part" /> palpites de onde ele está ·{" "}
            <span className="pato-swatch pato-swatch-ghost" /> o melhor palpite
            <br />
            Localização simplificada: filtro de partículas em 2D com o mapa da sala já conhecido,
            não um SLAM completo.
          </p>
        </>
      )}

      {phase === "jogar" && (
        <>
          <div data-reserva className="pato-top">
            <h1 className="pato-title pato-title-small">
              Agora ele sabe onde está. Mas ainda não sabe desviar.
            </h1>
          </div>
          <p data-reserva className="pato-legend">
            Localização simplificada: filtro de partículas em 2D com o mapa da sala já conhecido,
            não um SLAM completo.
          </p>
        </>
      )}

      {decidindo && (
        <>
          <div data-reserva className="pato-left">
            <p>
              Antes de escolher um caminho, ratos &quot;ensaiam&quot; rotas: as células de lugar
              disparam em sequência ao longo do trajeto que eles vão fazer (Pfeiffer e Foster,
              2013).
            </p>
            <p className="pato-muted pato-small">arraste o livro, o chinelo ou a caixa</p>
          </div>
          <p data-reserva className="pato-legend">
            <span className="pato-swatch pato-swatch-map" /> o que os lasers acham ocupado ·{" "}
            <span className="pato-swatch pato-swatch-route" /> a rota ·{" "}
            <span className="pato-swatch pato-swatch-ray" /> pra onde ele vai agora
            <br />
            Planejamento simplificado: A* numa grade 2D de 4 cm, com o pato tratado como um círculo
            e sem prever como ele vira.
          </p>
          <aside data-reserva className="pato-ficha">
            <p>
              <span className="pato-ficha-label">pra estudar</span> grafos e A* (a introdução ao A*
              do Red Blob Games), planejamento de movimento
            </p>
            <p>
              <span className="pato-ficha-label">projeto opcional</span> planejar rotas no seu
              simulador
            </p>
          </aside>
        </>
      )}

      {phase === "mover" && (
        <>
          <div data-reserva className="pato-top pato-top-interactive">
            <h1 className="pato-title pato-title-small">O plano está certo. Mas e se alguém empurrar?</h1>
            <div className="pato-toggle" role="group" aria-label="equilíbrio">
              <button
                className={moverSt?.equilibrio !== false ? "is-on" : ""}
                onClick={() => apiRef.current?.setEquilibrio(true)}
              >
                com equilíbrio
              </button>
              <button
                className={moverSt?.equilibrio === false ? "is-on" : ""}
                disabled={!moverSt?.pronto}
                onClick={() => apiRef.current?.setEquilibrio(false)}
              >
                sem equilíbrio
              </button>
            </div>
            <p className="pato-muted pato-small">
              clique e arraste o pato pra empurrar · quedas: com equilíbrio {moverSt?.quedas.com ?? 0},
              sem {moverSt?.quedas.sem ?? 0}
            </p>
          </div>
          <div data-reserva className="pato-left">
            <p>
              O ouvido interno sente a inclinação, e o cerebelo corrige o passo em milissegundos, sem
              você perceber. Os dois IMUs do pato são o ouvido interno dele.
            </p>
            <p className="pato-muted pato-small">
              Sem equilíbrio: a mesma marcha, gravada da política e repetida sem olhar os sensores.
              Com equilíbrio: a política lendo os IMUs a cada passo.
            </p>
          </div>
          <aside data-reserva className="pato-ficha">
            <p>
              <span className="pato-ficha-label">pra estudar</span> controle em malha fechada, PID,
              pêndulo invertido
            </p>
            <p>
              <span className="pato-ficha-label">projeto opcional</span> equilibrar um pêndulo
              invertido no seu simulador
            </p>
          </aside>
          {canAdvance && (
            <p data-reserva className="pato-legend">
              Mas quem ensinou o pato a se equilibrar? Ninguém programou isso.
            </p>
          )}
        </>
      )}

      {phase === "aprender" && aprenderSt && (
        <>
          <div data-reserva className="pato-top">
            <h1 className="pato-title pato-title-small">
              {aprenderSt.fim
                ? "Ninguém programou o passo. Ele aprendeu tentando."
                : `Geração ${aprenderSt.geracao + 1} de ${aprenderSt.geracoes}`}
            </h1>
            <div className="pato-geracoes" aria-label="quedas por geração">
              {aprenderSt.quedas.map((q, i) => (
                <div key={i} className={`pato-geracao${i === aprenderSt.geracao && !aprenderSt.fim ? " is-now" : ""}`}>
                  <span className="pato-geracao-barra" style={{ height: `${Math.min(60, q * 12 + 2)}px` }} />
                  <span className="pato-geracao-n">{i <= aprenderSt.geracao || aprenderSt.fim ? q : ""}</span>
                </div>
              ))}
            </div>
            <p className="pato-muted pato-small">quedas por geração</p>
          </div>
          {aprenderSt.fim && (
            <div className="pato-bottom">
              <button
                className="pato-button"
                onClick={() => {
                  setPhase("carregando");
                  setProgress("trocando de corpo");
                  setVariante("patins");
                }}
              >
                Mesmo método, corpo novo
              </button>
            </div>
          )}
          <div data-reserva className="pato-left">
            <p>
              Quando algo dá melhor do que o esperado, neurônios de dopamina disparam: é um sinal de
              erro de previsão de recompensa (Schultz, 1997). O aprendizado por reforço usa a mesma
              ideia.
            </p>
            <p className="pato-muted pato-small">
              Aproximação: aqui o treino é simulado com ruído na política final. O treino real leva
              horas em GPU.
            </p>
          </div>
          <aside data-reserva className="pato-ficha">
            <p>
              <span className="pato-ficha-label">pra estudar</span> aprendizado por reforço (o curso
              Deep RL do Hugging Face) e o repositório microduck_rl
            </p>
            <p>
              <span className="pato-ficha-label">projeto opcional</span> treinar o seu pato no seu
              simulador
            </p>
          </aside>
        </>
      )}

      {phase === "patins" && (
        <div data-reserva className="pato-top">
          <h1 className="pato-title pato-title-small">
            Ninguém programou o pato pra patinar. Ele aprendeu do mesmo jeito, em simulação.
          </h1>
          {keysHint}
        </div>
      )}

      {localizing && phase !== "gancho" && (
        <aside data-reserva className="pato-ficha">
          <p>
            <span className="pato-ficha-label">pra estudar</span> o livro Probabilistic Robotics
            (Thrun, Burgard e Fox), localização de Monte Carlo, SLAM
          </p>
          <p>
            <span className="pato-ficha-label">projeto opcional</span> dar um LiDAR ao seu
            simulador e desenhar o mapa
          </p>
        </aside>
      )}

      {toast && <p className="pato-toast">{toast}</p>}

      {toqueAtivo && usaTeclado && (
        <Joystick
          onMove={(x, y) => apiRef.current?.setToque(x, y)}
          onKick={phase === "controle" ? () => apiRef.current?.chutar() : undefined}
        />
      )}

      <footer className="pato-credito">
        Microduck ©{" "}
        <a href="https://www.pollen-robotics.com" target="_blank" rel="noreferrer">Pollen Robotics</a>. Modelos 3D sob{" "}
        <a href="https://creativecommons.org/licenses/by-nc-sa/4.0/" target="_blank" rel="noreferrer">CC BY-NC-SA 4.0</a>; código e
        políticas sob{" "}
        <a href="https://www.apache.org/licenses/LICENSE-2.0" target="_blank" rel="noreferrer">Apache-2.0</a> (
        <a href="https://github.com/pollen-robotics/microduck_rl" target="_blank" rel="noreferrer">microduck_rl</a>,{" "}
        <a href="https://huggingface.co/spaces/pollen-robotics/microduck-simulator" target="_blank" rel="noreferrer">simulador</a>).
      </footer>

      <div className="pato-vision" hidden={!showsVision}>
        <div className="pato-vision-head">
          <span>câmera da cabeça</span>
          <span className="pato-muted">arraste a bola</span>
        </div>
        <canvas
          ref={visionRef}
          width={VISION_W}
          height={VISION_H}
          className="pato-vision-canvas"
        />
        <input
          type="range"
          min={0}
          max={2}
          step={0.01}
          value={layer}
          onChange={(e) => setLayer(Number(e.target.value))}
          aria-label="camada da visão"
          className="pato-range"
        />
        <div className="pato-layers">
          {LAYERS.map((l, i) => (
            <button
              key={l}
              className={i === layerIdx ? "is-on" : ""}
              onClick={() => setLayer(i)}
            >
              {l}
            </button>
          ))}
        </div>
        <p className="pato-note">
          Visão simplificada: bordas e cor, sem rede neural. O pato procura o amarelo da bola de tênis, centraliza e
          anda até ele.
        </p>
      </div>
    </div>
  );
}

// Joystick virtual: arrastar o polegar no círculo anda (pra cima/baixo) e
// gira (pros lados). Solta, o pato para.
function Joystick({ onMove, onKick }: { onMove: (x: number, y: number) => void; onKick?: () => void }) {
  const base = useRef<HTMLDivElement>(null);
  const [pos, setPos] = useState({ x: 0, y: 0 });
  const R = 48;
  const atualizar = (e: React.PointerEvent) => {
    const r = base.current!.getBoundingClientRect();
    let dx = e.clientX - (r.left + r.width / 2), dy = e.clientY - (r.top + r.height / 2);
    const n = Math.hypot(dx, dy);
    if (n > R) { dx *= R / n; dy *= R / n; }
    setPos({ x: dx, y: dy });
    onMove(dx / R, -dy / R);
  };
  const soltar = () => { setPos({ x: 0, y: 0 }); onMove(0, 0); };
  return (
    <div className="pato-toque">
      <div
        ref={base}
        className="pato-joystick"
        onPointerDown={(e) => { e.currentTarget.setPointerCapture(e.pointerId); atualizar(e); }}
        onPointerMove={(e) => { if (e.currentTarget.hasPointerCapture(e.pointerId)) atualizar(e); }}
        onPointerUp={soltar}
        onPointerCancel={soltar}
        aria-label="andar e girar"
      >
        <span className="pato-joystick-pino" style={{ transform: `translate(${pos.x}px, ${pos.y}px)` }} />
      </div>
      {onKick && (
        <button className="pato-chute" onPointerDown={onKick} aria-label="chutar">
          chutar
        </button>
      )}
    </div>
  );
}
