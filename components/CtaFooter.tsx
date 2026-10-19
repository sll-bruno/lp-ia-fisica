export function CtaFooter() {
  return (
    <footer className="relative border-t border-line py-20">
      <div className="mx-auto max-w-6xl px-6">
        <div className="flex flex-col items-start justify-between gap-8 md:flex-row md:items-end">
          <div className="max-w-xl">
            <p className="mb-4 text-xs font-medium tracking-[0.2em] text-signal">
              SUA ROTA COMEÇA AGORA
            </p>
            <h2 className="font-serif text-3xl leading-tight text-foreground sm:text-4xl">
              O robô autônomo de amanhã aprende com quem entende a rota de
              hoje.
            </h2>
          </div>
          <a
            href="#trilha"
            className="shrink-0 rounded-full bg-accent px-6 py-3 text-sm font-medium text-background transition-opacity hover:opacity-90"
          >
            Começar minha trilha →
          </a>
        </div>

        <div className="mt-16 flex flex-col gap-4 border-t border-line pt-8 text-xs text-muted-dim sm:flex-row sm:items-center sm:justify-between">
          <span>Rota Autônoma · projeto feito para a 202lab</span>
          <span>Conteúdo técnico referenciado na documentação do Trossen Arm</span>
        </div>
      </div>
    </footer>
  );
}
