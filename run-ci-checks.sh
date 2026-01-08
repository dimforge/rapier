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
RUSTDOCFLAGS="-D warnings" cargo doc --features parallel,simd-stable,serde-serialize,debug-render \
    -p rapier3d -p rapier2d -p rapier3d-meshloader -p rapier3d-urdf || print_error "Documentation"
print_success "Documentation"

# Clippy - main workspace
print_step "Running clippy..."
RUSTFLAGS="-D warnings" cargo clippy || print_error "Clippy"
print_success "Clippy"

# Clippy - examples with features
print_step "Running clippy on rapier2d examples..."
RUSTFLAGS="-D warnings" cargo clippy -p rapier-examples-2d --features parallel,simd-stable || print_error "Clippy rapier2d examples"
print_success "Clippy rapier2d examples"

print_step "Running clippy on rapier3d examples..."
RUSTFLAGS="-D warnings" cargo clippy -p rapier-examples-3d --features parallel,simd-stable || print_error "Clippy rapier3d examples"
print_success "Clippy rapier3d examples"

# Build rapier2d and rapier3d
print_step "Building rapier2d..."
RUSTFLAGS="-D warnings" cargo build --verbose -p rapier2d || print_error "Build rapier2d"
print_success "Build rapier2d"

print_step "Building rapier3d..."
RUSTFLAGS="-D warnings" cargo build --verbose -p rapier3d || print_error "Build rapier3d"
print_success "Build rapier3d"

# Build with SIMD
print_step "Building rapier2d with SIMD..."
(cd crates/rapier2d && RUSTFLAGS="-D warnings" cargo build --verbose --features simd-stable) || print_error "Build rapier2d SIMD"
print_success "Build rapier2d SIMD"

print_step "Building rapier3d with SIMD..."
(cd crates/rapier3d && RUSTFLAGS="-D warnings" cargo build --verbose --features simd-stable) || print_error "Build rapier3d SIMD"
print_success "Build rapier3d SIMD"

# Build with SIMD + Parallel
print_step "Building rapier2d with SIMD + Parallel..."
(cd crates/rapier2d && RUSTFLAGS="-D warnings" cargo build --verbose --features simd-stable --features parallel) || print_error "Build rapier2d SIMD Parallel"
print_success "Build rapier2d SIMD Parallel"

print_step "Building rapier3d with SIMD + Parallel..."
(cd crates/rapier3d && RUSTFLAGS="-D warnings" cargo build --verbose --features simd-stable --features parallel) || print_error "Build rapier3d SIMD Parallel"
print_success "Build rapier3d SIMD Parallel"

# Run tests
print_step "Running tests..."
cargo test || print_error "Tests"
print_success "Tests"

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
