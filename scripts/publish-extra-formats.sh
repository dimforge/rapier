#!/bin/bash

currdir=$(pwd)

### Publish rapier3d-stl.
cd "crates/rapier3d-stl" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2

### Publish rapier3d-urdf.
cd "crates/rapier3d-urdf" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2