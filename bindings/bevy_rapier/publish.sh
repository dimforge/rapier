#! /bin/bash
#
# Publishes both bevy_rapier crates to crates.io with a single
# `cargo publish --workspace` invocation:
#
#   - bevy_rapier2d / bevy_rapier3d
#
# Cargo computes the dependency order itself and waits for each crate to become
# available on the registry before publishing the ones that depend on it. The
# `bevy_rapier_benches3d` and `ci` crates are marked `publish = false`, so
# `--workspace` skips them.
#
# Why this script exists
# ----------------------
# Both crates share a single source tree at `<repo>/src`, referenced from each
# manifest as `path = "../src/lib.rs"`. That path points outside the crate
# directory, which `cargo publish` refuses to package.
#
# To work around it *only during publishing*, this script temporarily, for each
# affected crate:
#   1. rewrites the `[lib] path` to a crate-local one (i.e. `src/lib.rs`), and
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
SRC_CRATES=(bevy_rapier2d bevy_rapier3d)

# Refuse to run on a dirty tree: the only diff during publishing must be our own
# temporary edits, so the restore at the end is guaranteed to be correct.
if [ -n "$(git status --porcelain)" ]; then
    echo "error: working tree is not clean. Commit or stash changes before publishing." >&2
    exit 1
fi

backup_dir="$(mktemp -d)"

cleanup() {
    for crate in "${SRC_CRATES[@]}"; do
        # Remove the symlink we created (only if it is in fact a symlink).
        [ -L "$crate/src" ] && rm -f "$crate/src"
        # Restore the original manifest.
        if [ -f "$backup_dir/$crate.Cargo.toml" ]; then
            cp "$backup_dir/$crate.Cargo.toml" "$crate/Cargo.toml"
        fi
    done
    rm -rf "$backup_dir"
}
trap cleanup EXIT INT TERM

# Apply the temporary symlink layout: the shared `src` dir lives at the repo
# root and is linked into each crate, while the manifest's `[lib] path` is made
# crate-local.
link_shared() {
    local crate="$1"
    local manifest="$crate/Cargo.toml"

    cp "$manifest" "$backup_dir/$crate.Cargo.toml"

    local tmp
    tmp="$(mktemp)"
    sed 's#path = "\.\./src/lib.rs"#path = "src/lib.rs"#' "$manifest" > "$tmp"
    mv "$tmp" "$manifest"

    ln -s ../src "$crate/src"
}

for crate in "${SRC_CRATES[@]}"; do
    link_shared "$crate"
done

# Publish the whole workspace. `--allow-dirty` is required because our temporary
# edits make the tree dirty; the clean-tree check above keeps that safe.
cargo publish --workspace --allow-dirty "$@"
