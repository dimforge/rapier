#! /bin/bash

currdir=$(pwd)

### Publish rapier3d-stl.
cd "crates/rapier3d-stl" && cargo publish $DRY_RUN
cd "$currdir" || exit

### Publish rapier3d-urdf.
cd "crates/rapier3d-urdf" && cargo publish $DRY_RUN
cd "$currdir" || exit