#!/bin/bash

# Build the rapier all_examples demos to WASM for the website.
# Usage: ./scripts/build-demos.sh [demo_name]
# If demo_name is provided (all_examples2 or all_examples3), only that demo is built.
#
# Examples that read assets from disk (URDF/MJCF robots, .obj meshes, scene
# dumps) are `#[cfg]`-ed out of wasm builds, see examples2d/all_examples2.rs
# and examples3d/all_examples3.rs.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEBSITE_DIR="$(dirname "$SCRIPT_DIR")"
RAPIER_DIR="$(dirname "$WEBSITE_DIR")"
DEMOS_DIR="$WEBSITE_DIR/static/demos"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Each demo is a binary with its own built-in example picker UI.
# demo name -> cargo package
DEMOS=(all_examples2 all_examples3)
package_of() {
    case "$1" in
        all_examples2) echo rapier-examples-2d ;;
        all_examples3) echo rapier-examples-3d ;;
        *) echo "" ;;
    esac
}

# Check for required tools
check_requirements() {
    local missing=()

    if ! command -v cargo &> /dev/null; then
        missing+=("cargo")
    fi

    if ! rustup target list --installed | grep -q wasm32-unknown-unknown; then
        missing+=("wasm32-unknown-unknown target (install with: rustup target add wasm32-unknown-unknown)")
    fi

    if [ ${#missing[@]} -gt 0 ]; then
        echo -e "${RED}Error: Missing required tools:${NC}"
        for tool in "${missing[@]}"; do
            echo "  - $tool"
        done
        exit 1
    fi
}

# wasm-bindgen requires the CLI version to exactly match the wasm-bindgen crate
# version in Cargo.lock. If the globally installed CLI doesn't match, install
# the right version locally under target/ (leaves the global install alone).
resolve_wasm_bindgen() {
    local required
    required=$(grep -A1 '^name = "wasm-bindgen"$' "$RAPIER_DIR/Cargo.lock" | grep '^version' | cut -d'"' -f2)
    if [ -z "$required" ]; then
        echo -e "${RED}Could not determine wasm-bindgen version from Cargo.lock${NC}" >&2
        exit 1
    fi

    if command -v wasm-bindgen &> /dev/null && [ "$(wasm-bindgen --version | awk '{print $2}')" = "$required" ]; then
        WASM_BINDGEN=wasm-bindgen
        return
    fi

    local local_root="$RAPIER_DIR/target/wasm-bindgen-cli/$required"
    WASM_BINDGEN="$local_root/bin/wasm-bindgen"
    if [ ! -x "$WASM_BINDGEN" ]; then
        echo -e "${BLUE}Installing wasm-bindgen-cli $required (to match Cargo.lock) into target/...${NC}"
        cargo install wasm-bindgen-cli --version "$required" --root "$local_root"
    fi
}

# Tunables (override via environment):
#   WASM_OPT_FLAGS=…  wasm-opt optimization flags      (default: -O3)
#   SKIP_WASM_OPT=1   skip wasm-opt entirely (fast iteration; larger .wasm)
WASM_OPT_FLAGS="${WASM_OPT_FLAGS:--O3}"

# Compile the demos in a single cargo invocation.
cargo_build() {
    local args=(build
        --manifest-path "$RAPIER_DIR/Cargo.toml"
        --target wasm32-unknown-unknown
        --release)
    local d
    for d in "$@"; do
        args+=(-p "$(package_of "$d")" --bin "$d")
    done
    cargo "${args[@]}"
}

# Post-process one already-compiled demo: wasm-bindgen + wasm-opt + index.html.
postprocess_demo() {
    local demo=$1
    local demo_dir="$DEMOS_DIR/$demo"
    local target_dir="$RAPIER_DIR/target/wasm32-unknown-unknown/release"

    mkdir -p "$demo_dir/pkg"

    if [ ! -f "$target_dir/$demo.wasm" ]; then
        echo -e "${RED}✗${NC} $demo (no .wasm — cargo build failed?)"
        return 1
    fi

    # Generate JS bindings with wasm-bindgen
    if ! "$WASM_BINDGEN" \
        "$target_dir/$demo.wasm" \
        --out-dir "$demo_dir/pkg" \
        --out-name example \
        --target web \
        --no-typescript 2>&1; then
        echo -e "${RED}✗${NC} $demo (wasm-bindgen failed)"
        return 1
    fi

    # Optimize with wasm-opt if available (skippable for fast iteration)
    if [ -z "$SKIP_WASM_OPT" ] && command -v wasm-opt &> /dev/null; then
        wasm-opt $WASM_OPT_FLAGS "$demo_dir/pkg/example_bg.wasm" -o "$demo_dir/pkg/example_bg.wasm" 2>/dev/null || true
    fi

    write_index_html "$demo_dir"

    echo -e "${GREEN}✓${NC} $demo"
    return 0
}

write_index_html() {
    local demo_dir=$1
    # Create index.html for the demo
    cat > "$demo_dir/index.html" << 'HTMLEOF'
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rapier Demo</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    html, body {
      width: 100%;
      height: 100%;
      overflow: hidden;
      background: #2b1a13;
    }
    canvas {
      width: 100% !important;
      height: 100% !important;
      display: block;
    }
    .loading {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      color: #d08b5f;
      font-family: system-ui, sans-serif;
      font-size: 14px;
      text-align: center;
      max-width: 80%;
    }
    .loading::after {
      content: '';
      display: block;
      width: 30px;
      height: 30px;
      margin: 10px auto;
      border: 3px solid #5a3a28;
      border-top-color: #d08b5f;
      border-radius: 50%;
      animation: spin 1s linear infinite;
    }
    .error {
      color: #ff6b6b;
      line-height: 1.5;
    }
    .error::after {
      display: none;
    }
    .error a {
      color: #ff6b6b;
    }
    @keyframes spin { to { transform: rotate(360deg); } }
  </style>
</head>
<body>
  <div class="loading" id="loading">Loading WebAssembly...</div>
  <script type="module">
    const loading = document.getElementById('loading');

    const showError = (html) => {
      loading.className = 'loading error';
      loading.innerHTML = html;
    };

    // The check runs before the demo module is imported, so an unsupported
    // browser never downloads the (large) WASM app.
    if (!navigator.gpu) {
      showError('WebGPU is required to render these demos, and it is not '
        + 'available in this browser. See '
        + '<a href="https://caniuse.com/webgpu" target="_blank" rel="noopener noreferrer">'
        + 'caniuse.com/webgpu</a> for browser support. On Firefox, enable '
        + 'dom.webgpu.enabled in about:config.');
    } else {
      import('./pkg/example.js')
        .then(({default: init}) => init())
        .then(() => {
          loading.style.display = 'none';
        })
        .catch(err => {
          console.error('WASM Error:', err);
          loading.className = 'loading error';
          loading.textContent = 'Error: ' + err.message;
        });
    }
  </script>
</body>
</html>
HTMLEOF
}

# Main logic
check_requirements
resolve_wasm_bindgen

cd "$RAPIER_DIR"

if [ -n "$1" ]; then
    # Build a single demo (compile + post-process).
    if [ -z "$(package_of "$1")" ]; then
        echo -e "${RED}Unknown demo '$1'.${NC} Available demos: ${DEMOS[*]}"
        exit 1
    fi
    echo -e "${BLUE}Building${NC} $1..."
    cargo_build "$1"
    postprocess_demo "$1"
else
    # Build all demos.
    echo -e "${BLUE}Compiling ${#DEMOS[@]} demos to WASM (single cargo build)...${NC}"
    cargo_build "${DEMOS[@]}"
    echo ""

    failed=0
    for demo in "${DEMOS[@]}"; do
        postprocess_demo "$demo" || failed=$((failed + 1))
    done

    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${GREEN}Success:${NC} $(( ${#DEMOS[@]} - failed ))"
    if [ $failed -gt 0 ]; then
        echo -e "${RED}Failed:${NC} $failed"
    fi
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    [ $failed -eq 0 ] || exit 1
fi
