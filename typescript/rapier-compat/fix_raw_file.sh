#!/bin/bash

for feature in \
2d 2d-deterministic 2d-simd \
3d 3d-deterministic 3d-simd
do

echo 'export * from "'"./rapier_wasm$feature"'"' > builds/${feature}/pkg/raw.d.ts
echo 'export * from "'"./rapier_wasm$feature"'"' > builds/${feature}/pkg/raw.d.ts

done;