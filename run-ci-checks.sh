#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_step() {
    echo -e "\n${YELLOW}==> $1${NC}"
}

print_success() {
    echo -e "${GREEN}==> $1 passed${NC}"
}

print_error() {
    echo -e "${RED}==> $1 failed${NC}"
    exit 1
}

# Track start time
START_TIME=$(date +%s)

# Check formatting
print_step "Checking formatting..."
cargo fmt -- --check || print_error "Format check"
print_success "Format check"

# Documentation
print_step "Building documentation..."
RUSTDOCFLAGS="-D warnings" cargo doc --features parallel,serde-serialize,debug-render \
    -p rapier3d -p rapier2d -p rapier3d-meshloader -p rapier3d-urdf || print_error "Documentation"
print_success "Documentation"

# Clippy - main workspace
print_step "Running clippy..."
RUSTFLAGS="-D warnings" cargo clippy || print_error "Clippy"
print_success "Clippy"

# Clippy - examples with features
print_step "Running clippy on rapier2d examples..."
RUSTFLAGS="-D warnings" cargo clippy -p rapier-examples-2d --features parallel || print_error "Clippy rapier2d examples"
print_success "Clippy rapier2d examples"

print_step "Running clippy on rapier3d examples..."
RUSTFLAGS="-D warnings" cargo clippy -p rapier-examples-3d --features parallel || print_error "Clippy rapier3d examples"
print_success "Clippy rapier3d examples"

# Build rapier2d and rapier3d
print_step "Building rapier2d..."
RUSTFLAGS="-D warnings" cargo build --verbose -p rapier2d || print_error "Build rapier2d"
print_success "Build rapier2d"

print_step "Building rapier3d..."
RUSTFLAGS="-D warnings" cargo build --verbose -p rapier3d || print_error "Build rapier3d"
print_success "Build rapier3d"

# Build with Parallel
print_step "Building rapier2d with Parallel..."
(cd crates/rapier2d && RUSTFLAGS="-D warnings" cargo build --verbose --features parallel) || print_error "Build rapier2d Parallel"
print_success "Build rapier2d Parallel"

print_step "Building rapier3d with Parallel..."
(cd crates/rapier3d && RUSTFLAGS="-D warnings" cargo build --verbose --features parallel) || print_error "Build rapier3d Parallel"
print_success "Build rapier3d Parallel"

# Build with 8-lanes SIMD
print_step "Building rapier3d with 8-lanes SIMD..."
(cd crates/rapier3d && RUSTFLAGS="-D warnings" cargo build --verbose --features simd8) || print_error "Build rapier3d 8-lanes SIMD"
print_success "Build rapier3d 8-lanes SIMD"

# Cross-platform SIMD determinism deny-list: the solver's SIMD kernels must stay
# IEEE-exact. `mul_add` is fused on NEON but not on baseline x86_64, and wide's
# `recip`/`recip_sqrt` are hardware approximations on SSE only. (Per-lane
# transcendentals like `simd_asin` are fine: they route through libm under
# `enhanced-determinism`.)
print_step "Checking the solver SIMD determinism deny-list..."
if grep -rnE "mul_add|\brecip\(|recip_sqrt" src/dynamics/solver/ src/utils/; then
    print_error "SIMD determinism deny-list (platform-divergent op in the solver)"
fi
print_success "SIMD determinism deny-list"

# Run tests
print_step "Running tests..."
cargo test || print_error "Tests"
print_success "Tests"

# Thread-count determinism: parallel results must be identical for any pool size.
# `serde-serialize` enables the test's broad/narrow-phase checksums (stored layout,
# not just float state).
print_step "Running parallel determinism tests..."
(cd crates/rapier3d && cargo test --features parallel,serde-serialize --test thread_count_determinism) || print_error "Parallel determinism tests"
print_success "Parallel determinism tests"

# `parallel` OFF must equal `parallel` ON: the same golden broad/narrow-phase checksum
# under both feature sets (a native server and a single-threaded wasm client are two
# different builds that must stay in lockstep).
print_step "Running parallel-path parity test (feature off)..."
cargo test -p rapier3d --release --features enhanced-determinism,serde-serialize --test parallel_path_parity || print_error "Parallel-path parity (feature off)"
print_success "Parallel-path parity (feature off)"

print_step "Running parallel-path parity test (feature on)..."
cargo test -p rapier3d --release --features enhanced-determinism,serde-serialize,parallel --test parallel_path_parity || print_error "Parallel-path parity (feature on)"
print_success "Parallel-path parity (feature on)"

# Snapshot round-trip: restoring a serialized world must continue the same simulation.
print_step "Running snapshot round-trip test (3D)..."
cargo test -p rapier3d --release --features serde-serialize --test snapshot_roundtrip || print_error "Snapshot round-trip (3D)"
print_success "Snapshot round-trip (3D)"

print_step "Running snapshot round-trip test (2D)..."
cargo test -p rapier2d --release --features serde-serialize --test snapshot_roundtrip || print_error "Snapshot round-trip (2D)"
print_success "Snapshot round-trip (2D)"

# Snapshot portability: the golden that the wasm CI job checks on 32-bit pointers. Failing
# here first means the golden needs re-minting; failing only on wasm means a
# target-dependent encoding is back in the stored state.
print_step "Running snapshot portability test (3D)..."
cargo test -p rapier3d --release --features enhanced-determinism,serde-serialize --test snapshot_portability || print_error "Snapshot portability (3D)"
print_success "Snapshot portability (3D)"

print_step "Running snapshot portability test (2D)..."
cargo test -p rapier2d --release --features enhanced-determinism,serde-serialize --test snapshot_portability || print_error "Snapshot portability (2D)"
print_success "Snapshot portability (2D)"

# The same two goldens under wasm32 — the check that makes them portability tests rather
# than regression tests. Skipped rather than failed when the target or Node is missing,
# since neither is needed for the rest of this script; CI's `wasm-determinism` job always
# runs it.
if rustup target list --installed | grep -q '^wasm32-wasip1$' && command -v node > /dev/null; then
    export CARGO_TARGET_WASM32_WASIP1_RUNNER="node $(pwd)/.github/scripts/run-wasi.mjs"
    print_step "Running snapshot portability test (3D, wasm32-wasip1)..."
    cargo test -p rapier3d --release --features enhanced-determinism,serde-serialize --target wasm32-wasip1 --test snapshot_portability || print_error "Snapshot portability (3D, wasm32)"
    print_success "Snapshot portability (3D, wasm32)"

    print_step "Running snapshot portability test (2D, wasm32-wasip1)..."
    cargo test -p rapier2d --release --features enhanced-determinism,serde-serialize --target wasm32-wasip1 --test snapshot_portability || print_error "Snapshot portability (2D, wasm32)"
    print_success "Snapshot portability (2D, wasm32)"
    unset CARGO_TARGET_WASM32_WASIP1_RUNNER
else
    print_step "Skipping wasm32 snapshot portability (needs 'rustup target add wasm32-wasip1' and node >= 22)"
fi

# A single-worker pool must not deadlock on the deferred BVH optimization.
print_step "Running single-worker deferred-BVH test..."
cargo test -p rapier3d --release --features parallel --test single_worker_deferred_bvh || print_error "Single-worker deferred BVH"
print_success "Single-worker deferred BVH"

# `unsync-callbacks` drops the `Sync` bound from the hooks/event traits and keeps the
# callbacks on the thread driving the step. The test's callbacks hold a `Cell`, so it only
# compiles while that holds; the parity run pins that moving them changes no results.
print_step "Running unsync-callbacks test..."
cargo test -p rapier3d --release --features parallel,unsync-callbacks --test unsync_callbacks || print_error "Unsync callbacks"
print_success "Unsync callbacks"

# The bound is keyed on `unsync-callbacks` alone, so the no-`parallel` build is its own
# configuration rather than one that trivially has no bound.
print_step "Running unsync-callbacks test (no parallel)..."
cargo test -p rapier3d --release --features unsync-callbacks --test unsync_callbacks || print_error "Unsync callbacks (no parallel)"
print_success "Unsync callbacks (no parallel)"

print_step "Running parallel-path parity test (unsync-callbacks)..."
cargo test -p rapier3d --release --features enhanced-determinism,serde-serialize,parallel,unsync-callbacks --test parallel_path_parity || print_error "Parallel-path parity (unsync-callbacks)"
print_success "Parallel-path parity (unsync-callbacks)"

# Check testbed crates
print_step "Checking rapier_testbed2d..."
RUSTFLAGS="-D warnings" cargo check --verbose -p rapier_testbed2d || print_error "Check rapier_testbed2d"
print_success "Check rapier_testbed2d"

print_step "Checking rapier_testbed3d..."
RUSTFLAGS="-D warnings" cargo check --verbose -p rapier_testbed3d || print_error "Check rapier_testbed3d"
print_success "Check rapier_testbed3d"

# Check testbed with parallel feature
print_step "Checking rapier_testbed2d with parallel..."
(cd crates/rapier_testbed2d && RUSTFLAGS="-D warnings" cargo check --verbose --features parallel) || print_error "Check rapier_testbed2d parallel"
print_success "Check rapier_testbed2d parallel"

print_step "Checking rapier_testbed3d with parallel..."
(cd crates/rapier_testbed3d && RUSTFLAGS="-D warnings" cargo check --verbose --features parallel) || print_error "Check rapier_testbed3d parallel"
print_success "Check rapier_testbed3d parallel"

# Glam backend golden hashes: must produce identical bits on every platform this
# runs on (the cross-platform determinism contract).
print_step "Running glam backend determinism test..."
(cd crates/rapier3d && cargo test --features enhanced-determinism --test glam_backend_determinism) || print_error "Glam backend determinism test"
print_success "Glam backend determinism test"

# Check enhanced-determinism feature
print_step "Checking rapier2d with enhanced-determinism..."
(cd crates/rapier2d && RUSTFLAGS="-D warnings" cargo check --verbose --features enhanced-determinism) || print_error "Check rapier2d enhanced-determinism"
print_success "Check rapier2d enhanced-determinism"

print_step "Checking rapier3d with enhanced-determinism..."
(cd crates/rapier3d && RUSTFLAGS="-D warnings" cargo check --verbose --features enhanced-determinism) || print_error "Check rapier3d enhanced-determinism"
print_success "Check rapier3d enhanced-determinism"

# Check examples
print_step "Checking rapier-examples-2d..."
RUSTFLAGS="-D warnings" cargo check -j 1 --verbose -p rapier-examples-2d || print_error "Check rapier-examples-2d"
print_success "Check rapier-examples-2d"

print_step "Checking rapier-examples-3d..."
RUSTFLAGS="-D warnings" cargo check -j 1 --verbose -p rapier-examples-3d || print_error "Check rapier-examples-3d"
print_success "Check rapier-examples-3d"

# Calculate elapsed time
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))
MINUTES=$((ELAPSED / 60))
SECONDS=$((ELAPSED % 60))

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}All CI checks passed!${NC}"
echo -e "${GREEN}Total time: ${MINUTES}m ${SECONDS}s${NC}"
echo -e "${GREEN}========================================${NC}"
