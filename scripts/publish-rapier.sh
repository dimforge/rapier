#!/bin/bash

tmp=$(mktemp -d)

echo "$tmp"

cp -r src "$tmp"/.
cp -r LICENSE README.md "$tmp"/.

### Publish the 2D version.
sed 's#\.\./\.\./src#src#g' crates/rapier2d/Cargo.toml > "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2


### Publish the 3D version.
sed 's#\.\./\.\./src#src#g' crates/rapier3d/Cargo.toml > "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2

### Publish the 2D f64 version.
sed 's#\.\./\.\./src#src#g' crates/rapier2d-f64/Cargo.toml > "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish $DRY_RUN || exit 1
cd "$currdir" || exit 2


### Publish the 3D f64 version.
sed 's#\.\./\.\./src#src#g' crates/rapier3d-f64/Cargo.toml > "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish $DRY_RUN || exit 1

rm -rf "$tmp"