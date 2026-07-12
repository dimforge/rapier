#!/bin/bash

npm run build
rsync -av --delete-after dist/ crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/demos3d
# rsync -av dist/ammo.wasm.wasm crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/ammo.wasm.wasm
# rsync -av dist/physx.release.wasm crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/physx.release.wasm
