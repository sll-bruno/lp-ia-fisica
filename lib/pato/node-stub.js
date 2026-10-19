// Substitui módulos do Node ("module", "fs"...) que o loader Emscripten do
// MuJoCo só usa quando roda no Node; no navegador esses ramos nunca executam.
export const createRequire = () => () => ({});
export default {};
