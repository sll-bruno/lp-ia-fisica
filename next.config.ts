import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  /* config options here */
  cacheComponents: true,
  partialPrefetching: true,
  turbopack: {
    // O loader Emscripten do MuJoCo importa "module" num ramo só de Node.
    resolveAlias: {
      module: { browser: "./lib/pato/node-stub.js" },
    },
    rules: {
      "*.css": {
        loaders: ["@tailwindcss/turbopack"],
        as: "*.css",
      },
    },
  },
};

export default nextConfig;
