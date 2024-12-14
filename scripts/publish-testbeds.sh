#!/bin/bash

gsed -v >> /dev/null
if [ $? == 0 ]; then
    gsed=gsed
else
    # Hopefully installed sed is the gnu one.
    gsed=sed
fi

tmp=$(mktemp -d)

echo "$tmp"

cp -r src "$tmp"/.
cp -r src_testbed "$tmp"/.
cp -r crates "$tmp"/.
cp -r LICENSE README.md "$tmp"/.

### Publish the 2D version.
$gsed 's#\.\./\.\./src#src#g' crates/rapier_testbed2d/Cargo.toml > "$tmp"/Cargo.toml
$gsed -i 's#\.\./rapier#./crates/rapier#g' "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish $DRY_RUN
cd "$currdir" || exit

### Publish the 3D version.
$gsed 's#\.\./\.\./src#src#g' crates/rapier_testbed3d/Cargo.toml > "$tmp"/Cargo.toml
$gsed -i 's#\.\./rapier#./crates/rapier#g' "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish $DRY_RUN

### Publish the 2D f64 version.
$gsed 's#\.\./\.\./src#src#g' crates/rapier_testbed2d-f64/Cargo.toml > "$tmp"/Cargo.toml
$gsed -i 's#\.\./rapier#./crates/rapier#g' "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish $DRY_RUN
cd "$currdir" || exit

### Publish the 3D f64 version.
$gsed 's#\.\./\.\./src#src#g' crates/rapier_testbed3d-f64/Cargo.toml > "$tmp"/Cargo.toml
$gsed -i 's#\.\./rapier#./crates/rapier#g' "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish $DRY_RUN

rm -rf "$tmp"
