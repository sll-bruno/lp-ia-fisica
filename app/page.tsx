import type { Metadata } from "next";
import { PatoLoader } from "@/components/pato/PatoLoader";
import "./pato.css";

export const metadata: Metadata = {
  title: "Faça o pato marcar um gol",
  description: "Protótipo: um robô de verdade, simulado no navegador, e o que falta pra ele ver.",
};

export default function Home() {
  return <PatoLoader />;
}
