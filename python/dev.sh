#!/usr/bin/env bash
#
# dev.sh — build, test, document, and run the Rapier Python bindings from a
# checkout. One entry point for everything in python/README.md.
#
# Usage (run from anywhere; paths are resolved relative to the repo):
#   python/dev.sh                 # all: build + test + docs + testbed (headless smoke)
#   python/dev.sh build           # build all four engine packages (editable)
#   python/dev.sh test            # build, then run the full test suite
#   python/dev.sh docs            # build, then build the HTML docs
#   python/dev.sh testbed         # build + install testbed, open the picker
#   python/dev.sh testbed examples3.domino3   # build + install testbed, run one example
#   python/dev.sh tour            # build + install testbed, run every example in turn
#                                 #   (close a window to advance; accepts 2d|3d|--start NAME)
#   python/dev.sh clean           # remove build artifacts + the managed venv
#
# Environment:
#   PROFILE=debug        # faster compiles, slower runtime (default: release)
#   RAPIER_PY_VENV=PATH  # use/create this venv (default: <repo>/.venv);
#                        # ignored if a virtualenv is already activated.
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PROFILE="${PROFILE:-release}"
VENV="${RAPIER_PY_VENV:-$REPO_ROOT/.venv}"
CRATES=(rapier-py-2d rapier-py-2d-f64 rapier-py-3d rapier-py-3d-f64)

log()  { printf '\n\033[1;34m==>\033[0m \033[1m%s\033[0m\n' "$*"; }
note() { printf '    %s\n' "$*"; }

# --- environment -----------------------------------------------------------

ensure_venv() {
    if [[ -z "${VIRTUAL_ENV:-}" ]]; then
        if [[ ! -d "$VENV" ]]; then
            log "Creating virtualenv at $VENV"
            python3 -m venv "$VENV"
        fi
        # shellcheck disable=SC1091
        source "$VENV/bin/activate"
    fi
    # maturin refuses to run with both VIRTUAL_ENV and CONDA_PREFIX set.
    unset CONDA_PREFIX 2>/dev/null || true
    python -m pip install --quiet --upgrade pip
}

pip_install() { python -m pip install --quiet "$@"; }

maturin_develop() {  # $1 = manifest path
    if [[ "$PROFILE" == "release" ]]; then
        maturin develop --release -m "$1"
    else
        maturin develop -m "$1"
    fi
}

# --- steps (each assumes ensure_venv already ran) --------------------------

build_all() {
    pip_install maturin
    for c in "${CRATES[@]}"; do
        log "Building $c ($PROFILE)"
        maturin_develop "python/$c/Cargo.toml"
    done
    python -c "import rapier2d, rapier2d_f64, rapier3d, rapier3d_f64 as _; \
print('built all four flavors, version', rapier3d.__version__)"
}

run_tests() {
    pip_install pytest pytest-timeout hypothesis numpy matplotlib
    log "Running the test suite"
    python -m pytest python/tests -q --timeout=120
}

build_docs() {
    pip_install sphinx furo sphinx-autodoc-typehints
    log "Building the HTML docs"
    ( cd python/docs && sphinx-build -q -b html . _build/html )
    note "Docs: $REPO_ROOT/python/docs/_build/html/index.html"
}

install_testbed() {
    pip_install panda3d numpy
    # Editable (-e) so edits to examples / camera / the testbed itself are
    # picked up by `python -m rapier_testbed` without reinstalling. --no-deps
    # because its rapier2d/rapier3d deps are the local builds from build_all.
    # Note: --no-deps must precede -e, else pip reads it as the -e target.
    pip_install --no-deps -e "$REPO_ROOT/python/rapier-testbed"
}

# --- commands --------------------------------------------------------------

cmd_build()   { ensure_venv; build_all; }
cmd_test()    { ensure_venv; build_all; run_tests; }
cmd_docs()    { ensure_venv; build_all; build_docs; }

cmd_testbed() {
    ensure_venv; build_all; install_testbed
    local target="${1:-}"
    if [[ -n "$target" ]]; then
        log "Running example: $target"
        python -m "rapier_testbed.$target"
    else
        log "Opening the testbed picker (Esc to quit)"
        python -m rapier_testbed
    fi
}

cmd_tour() {
    # Launch every example one after another (close a window to advance).
    # Extra args (2d|3d|--start NAME) pass through to examples_tour.py.
    ensure_venv; build_all; install_testbed
    log "Touring all examples (close each window to advance; Ctrl-C to stop)"
    python "$REPO_ROOT/python/examples_tour.py" "$@"
}

cmd_all() {
    ensure_venv
    build_all
    run_tests
    build_docs
    install_testbed
    log "Testbed headless smoke"
    PANDA_NO_WINDOW=1 python -m rapier_testbed.examples3.domino3 >/dev/null \
        && note "testbed OK"
    cat <<EOF

$(printf '\033[1;32mAll done.\033[0m')
  • Docs:     open $REPO_ROOT/python/docs/_build/html/index.html
  • Testbed:  python -m rapier_testbed                      # interactive picker
              python -m rapier_testbed.examples3.domino3    # one example
  • Activate the venv in new shells:  source ${VENV}/bin/activate
EOF
}

cmd_clean() {
    log "Removing build artifacts and managed venv"
    rm -rf "$VENV" python/docs/_build target/wheels
    find python -name '__pycache__' -type d -prune -exec rm -rf {} + 2>/dev/null || true
    find python -name '*.abi3.so' -delete 2>/dev/null || true
    note "done (run a 'cargo clean' yourself if you also want to wipe target/)"
}

usage() { sed -n '3,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; }

case "${1:-all}" in
    build)   shift; cmd_build "$@" ;;
    test)    shift; cmd_test "$@" ;;
    docs)    shift; cmd_docs "$@" ;;
    testbed) shift; cmd_testbed "${1:-}" ;;
    tour)    shift; cmd_tour "$@" ;;
    clean)   shift; cmd_clean "$@" ;;
    all|"")  cmd_all ;;
    -h|--help|help) usage ;;
    *) echo "dev.sh: unknown command '$1'" >&2; usage >&2; exit 1 ;;
esac
