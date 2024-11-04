#!/bin/bash

currdir=$(pwd)

### Publish rapier3d-meshloader.
# FIXME: This is commented while we have not created the corresponding crate on crates.io
# cd "crates/rapier3d-meshloader" && cargo publish $DRY_RUN || exit 1
# cd "$currdir" || exit 2

### Publish rapier3d-urdf.
cd "crates/rapier3d-urdf" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2