import { Nav } from "@/components/Nav";
import { Hero } from "@/components/Hero";
import { EvolutionTimeline } from "@/components/EvolutionTimeline";
import { LearningTrail } from "@/components/LearningTrail";
import { ManipulationDemo } from "@/components/ManipulationDemo";
import { ProgressSection } from "@/components/ProgressSection";
import { CtaFooter } from "@/components/CtaFooter";

export default function Home() {
  return (
    <div className="flex min-h-screen flex-1 flex-col">
      <Nav />
      <main className="flex-1">
        <Hero />
        <EvolutionTimeline />
        <LearningTrail />
        <ManipulationDemo />
        <ProgressSection />
      </main>
      <CtaFooter />
    </div>
  );
}
