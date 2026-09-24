#!/usr/bin/env python3
"""Launch every testbed example one after another.

Each example opens in its own window; close it (press ``Esc`` or use the
window's close button) and the next example launches automatically. Press
``Ctrl-C`` in this terminal to stop the tour.

Requires the testbed to be installed (``python/dev.sh testbed``, or
``pip install --no-deps -e ./python/rapier-testbed``).

Usage:
    python python/examples_tour.py            # all (3D) examples
    python python/examples_tour.py --list     # print the example modules and exit
    python python/examples_tour.py --start NAME   # begin at the first module
                                                  # whose name contains NAME
"""
from __future__ import annotations

import os
import pkgutil
import subprocess
import sys


def example_modules() -> list[str]:
    """Fully-qualified module names for every runnable testbed example."""
    mods: list[str] = []
    pkg = __import__("rapier_testbed.examples3", fromlist=["_"])
    for info in pkgutil.walk_packages(pkg.__path__, prefix="rapier_testbed.examples3."):
        leaf = info.name.rsplit(".", 1)[-1]
        if info.ispkg or leaf == "utils" or ".utils." in info.name:
            continue
        mods.append(info.name)
    return sorted(mods)


def main(argv: list[str]) -> int:
    list_only = False
    start_at: str | None = None
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("-l", "--list"):
            list_only = True
        elif a == "--start":
            i += 1
            if i >= len(argv):
                print("--start needs a value", file=sys.stderr)
                return 2
            start_at = argv[i]
        elif a in ("-h", "--help"):
            print(__doc__)
            return 0
        else:
            print(f"unknown argument: {a!r} (try --help)", file=sys.stderr)
            return 2
        i += 1

    try:
        mods = example_modules()
    except ModuleNotFoundError:
        print(
            "rapier_testbed is not installed.\n"
            "  Run:  python/dev.sh testbed\n"
            "  or:   pip install --no-deps -e ./python/rapier-testbed",
            file=sys.stderr,
        )
        return 1

    if start_at:
        idx = next((k for k, m in enumerate(mods) if start_at in m), None)
        if idx is None:
            print(f"no example matches {start_at!r}", file=sys.stderr)
            return 2
        mods = mods[idx:]

    if list_only:
        print("\n".join(mods))
        return 0

    if os.environ.get("PANDA_NO_WINDOW"):
        print(
            "warning: PANDA_NO_WINDOW is set — examples will run headless and "
            "exit on their own rather than waiting for you to close a window.",
            file=sys.stderr,
        )

    total = len(mods)
    print(
        f"Touring {total} example(s). "
        "Close each window (Esc) to advance; Ctrl-C here to stop.\n",
        flush=True,
    )
    for n, mod in enumerate(mods, start=1):
        short = mod.split("rapier_testbed.", 1)[-1]
        print(f"[{n}/{total}] {short}", flush=True)
        try:
            result = subprocess.run([sys.executable, "-m", mod])
        except KeyboardInterrupt:
            print("\nTour stopped.")
            return 0
        if result.returncode != 0:
            # Some examples are intentional "not yet ported" stubs that raise
            # NotImplementedError; keep going so one stub doesn't end the tour.
            print(
                f"    (exited with code {result.returncode}; continuing)",
                flush=True,
            )

    print("\nDone — every example has been shown.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
