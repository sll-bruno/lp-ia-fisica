const links = [
  { href: "#origem", label: "Origem" },
  { href: "#trilha", label: "A trilha" },
  { href: "#manipulacao", label: "Demo" },
  { href: "#progresso", label: "Progresso" },
];

export function Nav() {
  return (
    <header className="fixed inset-x-0 top-0 z-50 border-b border-line bg-background/80 backdrop-blur">
      <div className="mx-auto flex max-w-6xl items-center justify-between px-6 py-4">
        <a href="#top" className="flex items-center gap-2">
          <span className="flex h-7 w-7 items-center justify-center rounded-full border border-signal/40 text-[10px] font-semibold tracking-wider text-signal">
            RA
          </span>
          <span className="font-serif text-sm tracking-wide text-foreground">
            Rota Autônoma
          </span>
        </a>
        <nav className="hidden items-center gap-8 text-sm text-muted md:flex">
          {links.map((link) => (
            <a
              key={link.href}
              href={link.href}
              className="transition-colors hover:text-foreground"
            >
              {link.label}
            </a>
          ))}
        </nav>
        <a
          href="#trilha"
          className="rounded-full bg-accent px-4 py-2 text-sm font-medium text-background transition-opacity hover:opacity-90"
        >
          Começar a trilha
        </a>
      </div>
    </header>
  );
}
