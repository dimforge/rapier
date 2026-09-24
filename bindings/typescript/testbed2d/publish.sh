#!/bin/bash

npm run build
rsync -av dist/ crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/demos2d
# rsync -av dist/Box2D_v2.3.1_min.wasm.wasm crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/Box2D_v2.3.1_min.wasm.wasm
