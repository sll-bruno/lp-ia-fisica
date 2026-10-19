"use client";

import { useEffect, useRef, useState } from "react";
import { createPato, VISION_W, VISION_H } from "@/lib/pato/engine";

type Phase =
  | "carregando" | "controle" | "gol" | "solto" | "visao"
  | "gancho" | "odometria" | "laser" | "jogar";
type Mode = "solto" | "visao" | "gancho" | "odometria" | "laser" | "jogar";
// Atalhos pra revisão: ?modo=solto, ?modo=visao, ?modo=localizar.
const SHORTCUTS: Record<string, Mode> = { solto: "solto", visao: "visao", localizar: "gancho" };
type Api = NonNullable<Awaited<ReturnType<typeof createPato>>>;

const LAYERS = ["pixels", "bordas", "bola"];

export default function PatoExperience() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const visionRef = useRef<HTMLCanvasElement>(null);
  const apiRef = useRef<Api | null>(null);
  const phaseRef = useRef<Phase>("carregando");
  const [phase, setPhaseState] = useState<Phase>("carregando");
  const [progress, setProgress] = useState<string | null>("preparando");
  const [failed, setFailed] = useState<string | null>(null);
  // Botão de avançar de cada fase aparece depois de um tempo nela.
  const [canAdvance, setCanAdvance] = useState(false);
  const [layer, setLayer] = useState(0);
  const [toast, setToast] = useState<string | null>(null);
  const [pressed, setPressed] = useState<Set<string>>(() => new Set());

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
          const modo = SHORTCUTS[new URLSearchParams(window.location.search).get("modo") ?? ""];
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
  }, []);

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
      solto: 3500, visao: 9000, gancho: 7000, odometria: 9000, laser: 9000,
    };
    const ms = wait[phase];
    if (ms === undefined) return;
    const t = window.setTimeout(() => setCanAdvance(true), ms);
    return () => window.clearTimeout(t);
  }, [phase]);

  useEffect(() => {
    apiRef.current?.setVisionLayer(layer);
  }, [layer, phase]);

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
  };
  const next = NEXT[phase];
  const showsVision = phase === "visao" || phase === "gancho" || phase === "jogar";
  const localizing = phase === "gancho" || phase === "odometria" || phase === "laser" || phase === "jogar";
  const keysHint = (
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

  return (
    <div className="pato-root">
      <canvas ref={canvasRef} className="pato-canvas" />

      {phase === "carregando" && (
        <div className="pato-center">
          <p className="pato-muted">{failed ? `não deu pra carregar: ${failed}` : `${progress ?? "quase lá"}…`}</p>
        </div>
      )}

      {phase === "controle" && (
        <div className="pato-top">
          <h1 className="pato-title">Faça o pato marcar um gol</h1>
          {keysHint}
        </div>
      )}

      {phase === "gol" && (
        <div className="pato-top pato-top-interactive">
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
          <p className="pato-brain">
            Em 1959, Hubel e Wiesel acharam neurônios no córtex visual que só disparam pra linhas
            numa certa inclinação. As bordas que você vê aqui são isso.
          </p>
          <aside className="pato-ficha">
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
        <div className="pato-top">
          <h1 className="pato-title pato-title-small">
            Ele vê a bola. Mas não sabe onde está, nem onde fica o gol.
          </h1>
        </div>
      )}

      {phase === "odometria" && (
        <>
          <div className="pato-left">
            <p>
              Ande de olhos fechados até a porta e você erra do mesmo jeito. Seu cérebro também
              conta passos, com as células de grade (Prêmio Nobel de 2014).
            </p>
            {keysHint}
          </div>
          <p className="pato-legend">
            <span className="pato-swatch pato-swatch-ghost" /> onde o pato acha que está, contando
            só os comandos que recebeu
          </p>
        </>
      )}

      {phase === "laser" && (
        <>
          <div className="pato-left">
            <p>
              As células de lugar no hipocampo (O&apos;Keefe, 1971) só disparam quando você está
              num ponto específico do espaço.
            </p>
            {keysHint}
            <p className="pato-muted pato-small">arraste o pato pra sequestrá-lo</p>
          </div>
          <p className="pato-legend">
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
          <div className="pato-top">
            <h1 className="pato-title pato-title-small">
              Agora ele sabe onde está. Mas ainda não sabe desviar.
            </h1>
          </div>
          <p className="pato-legend">
            Localização simplificada: filtro de partículas em 2D com o mapa da sala já conhecido,
            não um SLAM completo.
          </p>
        </>
      )}

      {localizing && phase !== "gancho" && (
        <aside className="pato-ficha">
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
