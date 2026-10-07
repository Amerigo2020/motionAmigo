#!/usr/bin/env bash
# Builds the WebAssembly module for the browser demo into web/pkg.
# Needs wasm-pack and, optionally, binaryen's wasm-opt (version 116 or newer) for a smaller module.
set -euo pipefail
cd "$(dirname "$0")/.."
wasm-pack build crates/motionamigo-wasm --release --target web --out-dir ../../web/pkg
rm -f web/pkg/.gitignore web/pkg/package.json web/pkg/README.md
# Old wasm-opt releases (before version 116) corrupt the externref table of wasm-bindgen output.
if command -v wasm-opt >/dev/null && [ "$(wasm-opt --version | grep -o '[0-9]\+' | head -1)" -ge 116 ]; then
  wasm-opt -O3 \
    --enable-simd --enable-bulk-memory --enable-nontrapping-float-to-int --enable-sign-ext \
    --enable-mutable-globals --enable-multivalue --enable-reference-types \
    web/pkg/motionamigo_wasm_bg.wasm -o web/pkg/motionamigo_wasm_bg.wasm
fi
mkdir -p web/scenes
cp examples/scenes/*.json web/scenes/
ls -l web/pkg web/scenes
