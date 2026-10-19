import { achievements } from "@/lib/content";

export function ProgressSection() {
  const unlockedCount = achievements.filter((a) => a.unlocked).length;

  return (
    <section id="progresso" className="relative py-28">
      <div className="mx-auto max-w-6xl px-6">
        <div className="mb-16 max-w-2xl">
          <p className="mb-4 text-xs font-medium tracking-[0.2em] text-accent">
            MERECE SER VISTO
          </p>
          <h2 className="font-serif text-3xl leading-tight text-foreground sm:text-4xl">
            Sua trilha, acompanhada do começo ao fim.
          </h2>
          <p className="mt-4 text-muted">
            Progresso ilustrativo — é assim que fica o seu passaporte ao
            concluir os cinco estágios.
          </p>
        </div>

        <div className="grid gap-6 rounded-3xl border border-line bg-background-raised/60 p-8 md:grid-cols-[1fr_1.4fr] md:p-10">
          <div className="flex flex-col justify-between gap-8">
            <div>
              <span className="text-xs tracking-wide text-muted-dim">
                EXPLORADOR · NÍVEL 03
              </span>
              <h3 className="mt-2 font-serif text-2xl text-foreground">
                Rota em andamento
              </h3>
              <div className="mt-6">
                <div className="flex items-center justify-between text-xs text-muted-dim">
                  <span>3 / 5 estágios</span>
                  <span>60%</span>
                </div>
                <div className="mt-2 h-2 rounded-full bg-line">
                  <div className="h-2 w-[60%] rounded-full bg-gradient-to-r from-accent to-signal" />
                </div>
              </div>
            </div>

            <div className="grid grid-cols-3 gap-4 text-center">
              <Stat label="XP" value="640" />
              <Stat label="Sequência" value="4 dias" />
              <Stat label="Conquistas" value={`${unlockedCount}/${achievements.length}`} />
            </div>
          </div>

          <div>
            <span className="text-xs tracking-wide text-muted-dim">
              CONQUISTAS
            </span>
            <div className="mt-4 grid grid-cols-2 gap-3 sm:grid-cols-3">
              {achievements.map((a) => (
                <div
                  key={a.id}
                  className={`rounded-xl border p-4 text-center transition-colors ${
                    a.unlocked
                      ? "border-signal/30 bg-signal/5"
                      : "border-line opacity-40"
                  }`}
                >
                  <span
                    className={`mx-auto mb-2 flex h-8 w-8 items-center justify-center rounded-full text-xs ${
                      a.unlocked
                        ? "bg-signal text-background"
                        : "bg-line text-muted-dim"
                    }`}
                  >
                    {a.unlocked ? "✓" : "·"}
                  </span>
                  <span className="text-xs text-foreground">{a.title}</span>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div>
      <div className="font-serif text-2xl text-foreground">{value}</div>
      <div className="mt-1 text-[11px] tracking-wide text-muted-dim">
        {label}
      </div>
    </div>
  );
}
