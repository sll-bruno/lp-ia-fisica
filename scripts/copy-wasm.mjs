// Copia os binários WASM do MuJoCo e do onnxruntime-web para public/pato/wasm,
// de onde o protótipo os carrega em tempo de execução.
import { copyFileSync, mkdirSync } from "node:fs";
import { join } from "node:path";

const out = join(process.cwd(), "public/pato/wasm");
mkdirSync(out, { recursive: true });
const nm = join(process.cwd(), "node_modules");
const mujocoDir = join(nm, "@mujoco/mujoco");
const ortDir = join(nm, "onnxruntime-web/dist");
copyFileSync(join(mujocoDir, "mujoco.wasm"), join(out, "mujoco.wasm"));
copyFileSync(join(ortDir, "ort-wasm-simd-threaded.wasm"), join(out, "ort-wasm-simd-threaded.wasm"));
console.log("wasm copiado para public/pato/wasm");
