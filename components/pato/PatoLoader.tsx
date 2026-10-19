"use client";

import dynamic from "next/dynamic";

// MuJoCo WASM, onnxruntime-web e WebGL só existem no navegador.
const PatoExperience = dynamic(() => import("./PatoExperience"), { ssr: false });

export function PatoLoader() {
  return <PatoExperience />;
}
