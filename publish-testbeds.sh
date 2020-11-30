#! /bin/bash

tmp=$(mktemp -d)

echo "$tmp"

cp -r src "$tmp"/.
cp -r src_testbed "$tmp"/.
cp -r build "$tmp"/.
cp -r LICENSE README.md "$tmp"/.

### Publish the 2D version.
sed 's#\.\./\.\./src#src#g' build/rapier_testbed2d/Cargo.toml > "$tmp"/Cargo.toml
sed -i 's#\.\./rapier#./build/rapier#g' "$tmp"/Cargo.toml
currdir=$(pwd)
cd "$tmp" && cargo publish
cd "$currdir" || exit


### Publish the 3D version.
sed 's#\.\./\.\./src#src#g' build/rapier_testbed3d/Cargo.toml > "$tmp"/Cargo.toml
sed -i 's#\.\./rapier#./build/rapier#g' "$tmp"/Cargo.toml
cp -r LICENSE README.md "$tmp"/.
cd "$tmp" && cargo publish

rm -rf "$tmp"
