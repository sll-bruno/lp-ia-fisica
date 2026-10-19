import { historyBeats } from "@/lib/content";

export function EvolutionTimeline() {
  return (
    <section id="origem" className="relative py-28">
      <div className="mx-auto max-w-6xl px-6">
        <div className="mb-16 max-w-2xl">
          <p className="mb-4 text-xs font-medium tracking-[0.2em] text-accent">
            ORIGEM · ANTES DA SUA TRILHA
          </p>
          <h2 className="font-serif text-3xl leading-tight text-foreground sm:text-4xl">
            Rápido, mas sem pular nada: como a robótica autônoma chegou até
            aqui.
          </h2>
          <p className="mt-4 text-muted">
            Seis marcos, da automação cega aos modelos que aprendem uma
            tarefa nova só de ouvir uma instrução.
          </p>
        </div>

        <div className="relative">
          <div className="absolute left-[15px] top-2 bottom-2 w-px bg-gradient-to-b from-accent/60 via-accent/20 to-signal/60 md:left-1/2" />

          <ol className="space-y-10">
            {historyBeats.map((beat, i) => {
              const alignRight = i % 2 === 1;
              const isLast = i === historyBeats.length - 1;
              return (
                <li
                  key={beat.id}
                  className={`relative flex flex-col gap-3 pl-10 md:w-1/2 md:pl-0 ${
                    alignRight
                      ? "md:ml-auto md:items-start md:pl-12"
                      : "md:mr-auto md:items-end md:pr-12 md:text-right"
                  }`}
                >
                  <span
                    className={`absolute left-0 top-1 h-[11px] w-[11px] rounded-full border ${
                      isLast
                        ? "border-signal bg-signal shadow-[0_0_12px_2px_rgba(94,234,212,0.6)]"
                        : "border-accent bg-background"
                    } md:left-1/2 md:-translate-x-1/2`}
                  />
                  <span className="text-xs tracking-wide text-muted-dim">
                    {beat.period}
                  </span>
                  <h3 className="font-serif text-xl text-foreground">
                    {beat.title}
                  </h3>
                  <p className="max-w-sm text-sm leading-relaxed text-muted">
                    {beat.description}
                  </p>
                  {isLast && (
                    <span className="mt-1 inline-flex items-center gap-2 rounded-full border border-signal/40 px-3 py-1 text-[11px] font-medium tracking-wide text-signal">
                      VOCÊ ESTÁ AQUI
                    </span>
                  )}
                </li>
              );
            })}
          </ol>
        </div>
      </div>
    </section>
  );
}
