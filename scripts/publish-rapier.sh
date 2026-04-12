#!/bin/bash

# Publish each rapier crate. Because all four crates share a single `src/`
# directory (via `path = "../../src/lib.rs"`) which lives outside each crate
# root, `cargo publish` cannot package it directly. We work around this by
# temporarily copying the source into each crate directory, patching the lib
# path, publishing from the workspace (so workspace deps are resolved), and
# cleaning up afterwards.
#
# Always runs in dry-run mode unless DRY_RUN is explicitly set to empty
# (done by publish-all.sh when PUBLISH_MODE=1).

set -euo pipefail

DRY_RUN="${DRY_RUN---dry-run}"

publish_crate() {
    local crate_dir="$1"

    echo "Publishing $crate_dir..."

    # Copy source into crate directory
    cp -r src "$crate_dir"/src

    # Patch lib path from ../../src to src
    cp "$crate_dir"/Cargo.toml "$crate_dir"/Cargo.toml.bak
    sed 's#\.\./\.\./src#src#g' "$crate_dir"/Cargo.toml.bak > "$crate_dir"/Cargo.toml

    # Publish from the workspace root (workspace deps get resolved)
    local crate_name
    crate_name=$(basename "$crate_dir")
    cargo publish -p "$crate_name" --allow-dirty $DRY_RUN

    # Restore original Cargo.toml and remove copied source
    mv "$crate_dir"/Cargo.toml.bak "$crate_dir"/Cargo.toml
    rm -rf "$crate_dir"/src
}

publish_crate crates/rapier2d
publish_crate crates/rapier3d
publish_crate crates/rapier2d-f64
publish_crate crates/rapier3d-f64
