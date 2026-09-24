#!/bin/bash
# Runs the Python snippets of the user-guide against the in-tree bindings.
# Build them first with `bindings/python/dev.sh build` (or set PYTHON to an interpreter where `rapier3d` is installed).
# Usage: ./run_snippets.sh [snippet.py]...

cd "$(dirname "${BASH_SOURCE[0]}")"
PYTHON="${PYTHON:-../../../../.venv/bin/python}"

snippets=("$@")
if [ ${#snippets[@]} -eq 0 ]; then
    snippets=(*.py)
fi

failed=0
for snippet in "${snippets[@]}"; do
    if "$PYTHON" "$snippet" > /dev/null; then
        echo "ok      $snippet"
    else
        echo "FAILED  $snippet"
        failed=1
    fi
done
exit $failed
