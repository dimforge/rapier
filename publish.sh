#! /bin/bash
#
# Publishes every rapier crate to crates.io with a single
# `cargo publish --workspace` invocation:
#
#   - rapier2d / rapier3d / rapier2d-f64 / rapier3d-f64
#   - rapier_testbed2d / rapier_testbed3d / rapier_testbed2d-f64 / rapier_testbed3d-f64
#   - rapier3d-meshloader / rapier3d-urdf / mjcf-rs / rapier3d-mjcf
#
# Cargo computes the dependency order itself and waits for each crate to become
# available on the registry before publishing the ones that depend on it. The
# example crates and the python-binding crates are marked `publish = false`, so
# `--workspace` skips them.
#
# Why this script exists
# ----------------------
# The four main crates share a single source tree at `<repo>/src`, referenced
# from each manifest as `path = "../../src/lib.rs"`. The four testbed crates
# likewise share `<repo>/src_testbed` via `path = "../../src_testbed/lib.rs"`.
# Those paths point outside the crate directory, which `cargo publish` refuses
# to package.
#
# To work around it *only during publishing*, this script temporarily, for each
# affected crate:
#   1. rewrites the `[lib] path` to a crate-local one (e.g. `src/lib.rs`), and
#   2. creates a symlink inside the crate pointing at the shared source tree.
#
# Cargo follows the symlink and bundles the real source into each `.crate`. A
# trap restores the manifests and removes the symlinks on exit (including on
# error or Ctrl-C), leaving the tree exactly as it was.
#
# Extra arguments are forwarded to `cargo publish`, e.g.:
#   ./publish.sh --dry-run
#   ./publish.sh --token "$CARGO_TOKEN"
#
# Requires cargo >= 1.90 (for `cargo publish --workspace`).

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

# Crates sharing `src/lib.rs`.
SRC_CRATES=(rapier2d rapier3d rapier2d-f64 rapier3d-f64)
# Crates sharing `src_testbed/lib.rs`.
TESTBED_CRATES=(rapier_testbed2d rapier_testbed3d rapier_testbed2d-f64 rapier_testbed3d-f64)

# Refuse to run on a dirty tree: the only diff during publishing must be our own
# temporary edits, so the restore at the end is guaranteed to be correct.
if [ -n "$(git status --porcelain)" ]; then
    echo "error: working tree is not clean. Commit or stash changes before publishing." >&2
    exit 1
fi

backup_dir="$(mktemp -d)"

cleanup() {
    for crate in "${SRC_CRATES[@]}" "${TESTBED_CRATES[@]}"; do
        # Remove the symlink we created (only if it is in fact a symlink).
        for link in "crates/$crate/src" "crates/$crate/src_testbed"; do
            [ -L "$link" ] && rm -f "$link"
        done
        # Restore the original manifest.
        if [ -f "$backup_dir/$crate.Cargo.toml" ]; then
            cp "$backup_dir/$crate.Cargo.toml" "crates/$crate/Cargo.toml"
        fi
    done
    rm -rf "$backup_dir"
}
trap cleanup EXIT INT TERM

# Apply the temporary symlink layout: `shared_dir` lives at the repo root and is
# linked into each crate, while the manifest's `[lib] path` is made crate-local.
link_shared() {
    local crate="$1" shared_dir="$2"
    local manifest="crates/$crate/Cargo.toml"

    cp "$manifest" "$backup_dir/$crate.Cargo.toml"

    local tmp
    tmp="$(mktemp)"
    sed "s#path = \"\.\./\.\./$shared_dir/lib.rs\"#path = \"$shared_dir/lib.rs\"#" "$manifest" > "$tmp"
    mv "$tmp" "$manifest"

    ln -s "../../$shared_dir" "crates/$crate/$shared_dir"
}

for crate in "${SRC_CRATES[@]}"; do
    link_shared "$crate" src
done
for crate in "${TESTBED_CRATES[@]}"; do
    link_shared "$crate" src_testbed
done

# Publish the whole workspace. `--allow-dirty` is required because our temporary
# edits make the tree dirty; the clean-tree check above keeps that safe.
cargo publish --workspace --allow-dirty "$@"
