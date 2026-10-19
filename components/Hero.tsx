export function Hero() {
  return (
    <section
      id="top"
      className="relative flex min-h-screen flex-col justify-center overflow-hidden pt-24"
    >
      <div className="pointer-events-none absolute inset-0">
        <RouteIllustration />
        <div className="absolute inset-0 bg-gradient-to-t from-background via-transparent to-background/40" />
      </div>

      <div className="relative mx-auto w-full max-w-6xl px-6">
        <p className="mb-6 text-xs font-medium tracking-[0.2em] text-signal">
          UMA TRILHA SOBRE ROBÓTICA AUTÔNOMA
        </p>
        <h1 className="max-w-3xl font-serif text-4xl leading-[1.1] text-foreground sm:text-5xl md:text-6xl">
          De braços programados à mão até robôs que aprendem sozinhos.
        </h1>
        <p className="mt-6 max-w-xl text-base leading-relaxed text-muted sm:text-lg">
          Uma rota curta pela história da robótica autônoma — e o começo da
          sua própria, com percepção, planejamento e um braço robótico real
          aprendendo uma tarefa diante de você.
        </p>
        <div className="mt-10 flex flex-wrap items-center gap-4">
          <a
            href="#origem"
            className="rounded-full bg-accent px-6 py-3 text-sm font-medium text-background transition-opacity hover:opacity-90"
          >
            Seguir a rota →
          </a>
          <span className="text-xs tracking-wide text-muted-dim">
            6 marcos históricos · 5 estágios · 1 demo ao vivo
          </span>
        </div>
      </div>

      <div className="relative mx-auto mt-20 flex w-full max-w-6xl items-center gap-3 px-6 text-xs tracking-wide text-muted-dim">
        <span className="h-px w-10 bg-muted-dim" />
        ROLE PARA EXPLORAR
      </div>
    </section>
  );
}

function RouteIllustration() {
  return (
    <svg
      viewBox="0 0 1200 800"
      className="h-full w-full opacity-90"
      preserveAspectRatio="xMidYMid slice"
    >
      <defs>
        <linearGradient id="routeLine" x1="0" y1="0" x2="1" y2="0">
          <stop offset="0%" stopColor="#ff7a45" stopOpacity="0.9" />
          <stop offset="55%" stopColor="#ff7a45" stopOpacity="0.5" />
          <stop offset="100%" stopColor="#5eead4" stopOpacity="0.8" />
        </linearGradient>
        <radialGradient id="glow" cx="50%" cy="50%" r="50%">
          <stop offset="0%" stopColor="#5eead4" stopOpacity="0.5" />
          <stop offset="100%" stopColor="#5eead4" stopOpacity="0" />
        </radialGradient>
        <pattern
          id="grid"
          width="40"
          height="40"
          patternUnits="userSpaceOnUse"
        >
          <path
            d="M 40 0 L 0 0 0 40"
            fill="none"
            stroke="rgba(236,232,223,0.05)"
            strokeWidth="1"
          />
        </pattern>
      </defs>

      <rect width="1200" height="800" fill="url(#grid)" />

      <path
        d="M -50 650 C 150 600, 250 500, 380 480 S 560 520, 650 440 S 820 280, 950 260 S 1150 140, 1260 120"
        fill="none"
        stroke="url(#routeLine)"
        strokeWidth="2.5"
        strokeDasharray="1 10"
        strokeLinecap="round"
      />
      <path
        d="M -50 650 C 150 600, 250 500, 380 480 S 560 520, 650 440 S 820 280, 950 260 S 1150 140, 1260 120"
        fill="none"
        stroke="url(#routeLine)"
        strokeWidth="1"
        strokeOpacity="0.5"
      />

      {[
        { cx: 380, cy: 480 },
        { cx: 650, cy: 440 },
        { cx: 950, cy: 260 },
      ].map((p, i) => (
        <g key={i}>
          <circle cx={p.cx} cy={p.cy} r="26" fill="url(#glow)" />
          <circle
            cx={p.cx}
            cy={p.cy}
            r="4.5"
            fill="#0a0d12"
            stroke="#5eead4"
            strokeWidth="1.5"
          />
        </g>
      ))}

      <circle cx="1260" cy="120" r="50" fill="url(#glow)" opacity="0.8" />
      <circle cx="1260" cy="120" r="6" fill="#5eead4" />
    </svg>
  );
}
