#!/bin/bash

# Build the API references embedded by the website (docs/api) from this repository's sources.
# Usage: ./scripts/build-api-docs.sh [js|c|py]...
# Without argument, the JavaScript (typedoc), C (Doxygen), and Python (Sphinx) references are built.
#
# - JavaScript: prepares the non-deterministic 2D/3D packages of `bindings/typescript/`, builds them, runs
#   typedoc, and copies the result to static/javascript2d and static/javascript3d.
#   Requires cargo, wasm-pack and npm (see bindings/typescript/README.md).
# - C: runs the Doxygen reference of `bindings/c/doxygen` (all dimension/precision variants) and copies the
#   HTML to static/c. Requires cmake, a C compiler, python3 and Doxygen 1.9.4+.
# - Python: builds the `rapier3d` package and its Sphinx docs with `bindings/python/dev.sh docs` (in the virtual
#   environment it manages, see bindings/python/README.md), and copies the HTML to static/python.
#   Requires a Rust toolchain and python3.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBSITE_DIR="$(dirname "$SCRIPT_DIR")"
RAPIER_DIR="$(dirname "$WEBSITE_DIR")"
STATIC_DIR="$WEBSITE_DIR/static"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

build_js() {
    local dim
    for dim in 2 3; do
        echo -e "${BLUE}Building the JavaScript ${dim}D API reference${NC}"
        (
            cd "$RAPIER_DIR/bindings/typescript"
            cargo run --quiet -p prepare_builds -- -d "dim${dim}" -f non-deterministic
            cd "builds/rapier${dim}d"
            npm install --no-audit --no-fund
            npm run clean
            npm run build:wasm
            npm run build:ts
            rm -rf docs
            npm run build:doc
        )
        rm -rf "$STATIC_DIR/javascript${dim}d"
        cp -r "$RAPIER_DIR/bindings/typescript/builds/rapier${dim}d/docs" "$STATIC_DIR/javascript${dim}d"
        echo -e "${GREEN}JavaScript ${dim}D API reference copied to static/javascript${dim}d${NC}"
    done
}

build_c() {
    local build_dir="$RAPIER_DIR/target/website-c-docs"
    echo -e "${BLUE}Building the C API reference${NC}"
    rm -rf "$build_dir/html"
    cmake -S "$RAPIER_DIR/bindings/c/doxygen" -B "$build_dir"
    cmake --build "$build_dir" --target rapier_docs --parallel
    rm -rf "$STATIC_DIR/c"
    mkdir -p "$STATIC_DIR/c"
    # The XML output only serves the documentation checks.
    rsync -a --exclude 'xml/' "$build_dir/html/" "$STATIC_DIR/c/"
    echo -e "${GREEN}C API reference copied to static/c${NC}"
}

build_py() {
    echo -e "${BLUE}Building the Python API reference${NC}"
    rm -rf "$RAPIER_DIR/bindings/python/docs/_build/html"
    "$RAPIER_DIR/bindings/python/dev.sh" docs
    rm -rf "$STATIC_DIR/python"
    cp -r "$RAPIER_DIR/bindings/python/docs/_build/html" "$STATIC_DIR/python"
    echo -e "${GREEN}Python API reference copied to static/python${NC}"
}

targets=("$@")
if [ ${#targets[@]} -eq 0 ]; then
    targets=(js c py)
fi

for target in "${targets[@]}"; do
    case "$target" in
        js) build_js ;;
        c) build_c ;;
        py) build_py ;;
        *)
            echo "Unknown API reference: $target (expected js, c, or py)"
            exit 1
            ;;
    esac
done
