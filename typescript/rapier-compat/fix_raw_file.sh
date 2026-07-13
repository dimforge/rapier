#!/bin/bash

for feature in \
2d 2d-deterministic 2d-simd \
3d 3d-deterministic 3d-simd
do

# The wasm-bindgen module is always named after the crate (rapier_wasm2d/rapier_wasm3d),
# whatever the feature variant (-deterministic/-simd) of the build.
dimension="${feature%%-*}"

echo "export * from \"./rapier_wasm${dimension}\";" > "builds/${feature}/pkg/dist/raw.d.ts"

done;
