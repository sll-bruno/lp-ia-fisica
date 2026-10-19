"use client";

import { useState } from "react";
import { demoSteps } from "@/lib/content";

export function ManipulationDemo() {
  const [activeIndex, setActiveIndex] = useState(0);
  const active = demoSteps[activeIndex];
  const isLast = activeIndex === demoSteps.length - 1;

  return (
    <section id="manipulacao" className="relative py-28">
      <div className="mx-auto max-w-6xl px-6">
        <div className="mb-12 max-w-2xl">
          <p className="mb-4 text-xs font-medium tracking-[0.2em] text-accent">
            ESTÁGIO 05 · MANIPULAÇÃO · DEMO
          </p>
          <h2 className="font-serif text-3xl leading-tight text-foreground sm:text-4xl">
            Veja um braço robótico aprender uma tarefa, passo a passo.
          </h2>
          <p className="mt-4 text-muted">
            Baseado no fluxo real de teleoperação com feedback de força e
            aprendizado por imitação usado em braços como o WidowX AI, com
            políticas da família π0.
          </p>
        </div>

        <div className="grid gap-8 rounded-3xl border border-line bg-background-raised/60 p-6 md:grid-cols-[minmax(0,1fr)_minmax(0,1.2fr)] md:p-10">
          <div className="flex flex-col justify-between gap-8">
            <div className="flex gap-2">
              {demoSteps.map((step, i) => (
                <button
                  key={step.id}
                  onClick={() => setActiveIndex(i)}
                  className={`h-1.5 flex-1 rounded-full transition-colors ${
                    i <= activeIndex ? "bg-signal" : "bg-line"
                  }`}
                  aria-label={step.label}
                />
              ))}
            </div>

            <div>
              <span className="text-xs tracking-wide text-signal">
                {active.label}
              </span>
              <h3 className="mt-3 font-serif text-2xl text-foreground">
                {active.title}
              </h3>
              <p className="mt-4 text-sm leading-relaxed text-muted">
                {active.description}
              </p>
              <p className="mt-6 font-mono text-[11px] tracking-wide text-muted-dim">
                {active.detail}
              </p>
            </div>

            <div className="flex items-center gap-3">
              <button
                onClick={() =>
                  setActiveIndex((i) => Math.max(0, i - 1))
                }
                disabled={activeIndex === 0}
                className="rounded-full border border-line px-4 py-2 text-xs text-muted transition-colors hover:border-signal/40 hover:text-foreground disabled:opacity-30"
              >
                ← Anterior
              </button>
              <button
                onClick={() =>
                  setActiveIndex((i) => Math.min(demoSteps.length - 1, i + 1))
                }
                disabled={isLast}
                className="rounded-full bg-signal px-4 py-2 text-xs font-medium text-background transition-opacity hover:opacity-90 disabled:opacity-30"
              >
                {isLast ? "Fim da demonstração" : "Próximo passo →"}
              </button>
            </div>
          </div>

          <DemoVisual stepIndex={activeIndex} />
        </div>

        <p className="mt-4 text-xs text-muted-dim">
          Demonstração ilustrativa, baseada em fluxos reais de teleoperação e
          aprendizado por imitação — não é uma transmissão ao vivo de um
          robô físico.
        </p>
      </div>
    </section>
  );
}

function DemoVisual({ stepIndex }: { stepIndex: number }) {
  const leaderActive = stepIndex === 0;
  const collecting = stepIndex === 1;
  const training = stepIndex === 2;
  const autonomous = stepIndex === 3;

  return (
    <div className="relative flex min-h-[320px] flex-col justify-center rounded-2xl border border-line bg-background p-8">
      <div className="flex items-center justify-between gap-6">
        <ArmNode
          label="Líder"
          sublabel={leaderActive ? "em uso" : "em repouso"}
          glowing={leaderActive}
          dim={autonomous || training}
        />
        <div className="flex flex-1 flex-col items-center gap-2">
          <div className="h-px w-full bg-gradient-to-r from-accent/60 via-signal/40 to-signal/60" />
          <span className="whitespace-nowrap font-mono text-[10px] tracking-wide text-muted-dim">
            {leaderActive && "posição + força →"}
            {collecting && "gravando trajetória"}
            {training && "treinando política"}
            {autonomous && "instrução →"}
          </span>
        </div>
        <ArmNode
          label="Seguidor"
          sublabel={autonomous ? "autônomo" : "replica posição"}
          glowing={autonomous}
          dim={false}
        />
      </div>

      <div className="mt-10 grid grid-cols-4 gap-2">
        {Array.from({ length: 12 }).map((_, i) => {
          const filled = collecting
            ? i < 9
            : training
              ? true
              : autonomous
                ? true
                : i < 2;
          return (
            <span
              key={i}
              className={`h-1.5 rounded-full transition-colors ${
                filled ? "bg-signal/70" : "bg-line"
              }`}
            />
          );
        })}
      </div>
      <span className="mt-3 font-mono text-[10px] tracking-wide text-muted-dim">
        {training
          ? "dataset → política π0"
          : "trajetórias de demonstração gravadas"}
      </span>
    </div>
  );
}

function ArmNode({
  label,
  sublabel,
  glowing,
  dim,
}: {
  label: string;
  sublabel: string;
  glowing: boolean;
  dim: boolean;
}) {
  return (
    <div
      className={`flex flex-col items-center gap-2 transition-opacity ${
        dim ? "opacity-40" : "opacity-100"
      }`}
    >
      <div
        className={`flex h-16 w-16 items-center justify-center rounded-full border-2 transition-all ${
          glowing
            ? "border-signal shadow-[0_0_20px_4px_rgba(94,234,212,0.35)]"
            : "border-line"
        }`}
      >
        <span className="h-3 w-3 rounded-full bg-foreground/80" />
      </div>
      <span className="text-xs font-medium text-foreground">{label}</span>
      <span className="font-mono text-[10px] text-muted-dim">{sublabel}</span>
    </div>
  );
}
