"use client";

import { useEffect, useRef, useState } from "react";
import { createPato, VISION_W, VISION_H } from "@/lib/pato/engine";

type Phase = "carregando" | "controle" | "gol" | "solto" | "visao";
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
  const [canGiveSight, setCanGiveSight] = useState(false);
  const [layer, setLayer] = useState(0);
  const [toast, setToast] = useState<string | null>(null);

  const setPhase = (p: Phase) => {
    phaseRef.current = p;
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
            else if (p === "visao") {
              setToast("Gol do pato.");
              window.setTimeout(() => setToast(null), 2200);
            }
          },
        });
        if (cancelled) created?.dispose();
        else if (created) {
          api = created;
          apiRef.current = created;
          // Atalho pra revisão: ?modo=solto ou ?modo=visao pula direto.
          const modo = new URLSearchParams(window.location.search).get("modo");
          if (modo === "solto" || modo === "visao") {
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

  useEffect(() => {
    if (phase !== "solto") return;
    const t = window.setTimeout(() => setCanGiveSight(true), 3500);
    return () => window.clearTimeout(t);
  }, [phase]);

  useEffect(() => {
    apiRef.current?.setVisionLayer(layer);
  }, [layer, phase]);

  const releaseControl = () => {
    apiRef.current?.setMode("solto");
    setPhase("solto");
  };
  const giveSight = () => {
    apiRef.current?.setMode("visao");
    setPhase("visao");
  };

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
          <div className="pato-keys" aria-label="setas ou WASD para andar, espaço para chutar">
            <span className="pato-key">←</span>
            <span className="pato-key">↑</span>
            <span className="pato-key">↓</span>
            <span className="pato-key">→</span>
            <span className="pato-key pato-key-wide">espaço</span>
          </div>
        </div>
      )}

      {phase === "gol" && (
        <div className="pato-top pato-top-interactive">
          <div className="pato-card">
            <p className="pato-lead">
              Quem marcou foi você. Seus olhos acharam a bola, você calculou o ângulo e decidiu a
              hora. O pato só chutou.
            </p>
            <button className="pato-button" onClick={releaseControl}>
              Solte o controle
            </button>
          </div>
        </div>
      )}

      {phase === "solto" && canGiveSight && (
        <div className="pato-bottom">
          <button className="pato-button" onClick={giveSight}>
            Dar visão ao pato
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
          {toast && <p className="pato-toast">{toast}</p>}
        </>
      )}

      <div className="pato-vision" hidden={phase !== "visao"}>
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
          Visão simplificada: bordas e cor, sem rede neural. O pato procura o magenta, centraliza e
          anda até ele.
        </p>
      </div>
    </div>
  );
}
