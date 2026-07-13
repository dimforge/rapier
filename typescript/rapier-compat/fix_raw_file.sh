#!/bin/bash

for feature in \
2d 2d-deterministic 2d-simd \
3d 3d-deterministic 3d-simd
do

dimension="${feature%%-*}"

echo "export * from \"./rapier_wasm${dimension}\";" > "builds/${feature}/pkg/dist/raw.d.ts"

done;