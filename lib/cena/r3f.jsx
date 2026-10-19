import { useEffect, useMemo } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { criarCena, configurarRenderer, Percepcao } from './cena.js';

// react-three-fiber wrappers. Inside <Canvas>:
//
//   const patoRef = useRef()            // the duck's trunk object from the MuJoCo scene
//   <SalaGolNaSala bolaReferencia={false} />
//   <PercepcaoDoPato alvo={patoRef} />   // takes over rendering; leave it out to see the plain scene
//
// Give <Canvas> the `shadows` prop; <SalaGolNaSala> switches the shadow type to PCF itself (the sun's soft edge needs it).

export function useCenaGolNaSala(opcoes) {
  return useMemo(() => criarCena(opcoes), []);   // eslint-disable-line react-hooks/exhaustive-deps
}

export function ConfigurarRenderer() {
  const gl = useThree(s => s.gl);
  useEffect(() => { configurarRenderer(gl); }, [gl]);
  return null;
}

export function SalaGolNaSala({ cena: dada, bolaReferencia = false, comCampo = true, comLuz = true }) {
  const propria = useCenaGolNaSala();
  const cena = dada ?? propria;
  const scene = useThree(s => s.scene);
  useEffect(() => {
    const anterior = scene.background;
    scene.background = cena.fundo;
    return () => { scene.background = anterior; };
  }, [scene, cena]);
  const bola = useMemo(() => (bolaReferencia ? cena.criarBolaReferencia() : null), [cena, bolaReferencia]);
  useEffect(() => {
    if (!bola) return;
    cena.campo.add(bola);
    return () => { cena.campo.remove(bola); };
  }, [cena, bola]);
  return (
    <>
      <ConfigurarRenderer />
      <primitive object={cena.sala} />
      {comCampo && <primitive object={cena.campo} />}
      {comLuz && <primitive object={cena.luz} />}
    </>
  );
}

// The sharp-where-the-duck-perceives effect. `alvo` is a ref to the object to follow (its local +x is "forward");
// without it, `pato` gives a fixed position {x, z, yaw}.
export function PercepcaoDoPato({ alvo, pato, eixoFrente, ...opcoes }) {
  const percepcao = useMemo(() => new Percepcao(opcoes), []);   // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => () => percepcao.dispose(), [percepcao]);
  useFrame(({ gl, scene, camera }) => {
    if (alvo?.current) percepcao.seguir(alvo.current, eixoFrente);
    else if (pato) percepcao.setPato(pato.x, pato.z, pato.yaw);
    percepcao.render(gl, scene, camera);
  }, 1);   // priority > 0: r3f stops its own render and this one draws the frame
  return null;
}
