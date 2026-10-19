import { trailStages } from "@/lib/content";

export function LearningTrail() {
  return (
    <section id="trilha" className="relative py-28">
      <div className="mx-auto max-w-6xl px-6">
        <div className="mb-16 max-w-2xl">
          <p className="mb-4 text-xs font-medium tracking-[0.2em] text-signal">
            A PARTIR DAQUI · SUA TRILHA
          </p>
          <h2 className="font-serif text-3xl leading-tight text-foreground sm:text-4xl">
            Cinco estágios para entender como um robô decide sozinho.
          </h2>
          <p className="mt-4 text-muted">
            Cada estágio constrói sobre o anterior. No final, os quatro se
            encontram em uma tarefa física real: manipulação.
          </p>
        </div>

        <div className="grid gap-5 md:grid-cols-5">
          {trailStages.map((stage, i) => (
            <div
              key={stage.id}
              className="group relative flex flex-col rounded-2xl border border-line bg-background-raised/60 p-5 transition-colors hover:border-signal/40"
            >
              <div className="mb-4 flex items-center justify-between">
                <span className="font-serif text-2xl text-muted-dim transition-colors group-hover:text-signal">
                  {stage.index}
                </span>
                {i < trailStages.length - 1 && (
                  <span className="hidden h-px flex-1 translate-x-2 bg-line md:block" />
                )}
              </div>
              <span className="mb-2 text-[11px] tracking-wide text-muted-dim">
                {stage.label}
              </span>
              <h3 className="mb-3 font-serif text-lg text-foreground">
                {stage.title}
              </h3>
              <p className="mb-4 text-sm leading-relaxed text-muted">
                {stage.summary}
              </p>
              <p className="mt-auto border-t border-line pt-3 text-xs leading-relaxed text-accent/90">
                {stage.mission}
              </p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}
