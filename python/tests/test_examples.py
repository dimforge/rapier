"""Run each example script and assert its final printed line matches a
known snapshot.

Each example is invoked as a subprocess (OOM-safe) with a 30s wall-time
cap. Determinism was verified by hand at authoring time; if a result here
ever drifts, either fix the regression or update the snapshot below.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

import pytest


EXAMPLES_DIR = Path(__file__).resolve().parent.parent / "examples"

# Map example script -> expected final stdout line. A plain string is matched
# exactly; a compiled regex is matched with `.search()` for outputs that are
# not bit-reproducible across architectures (the ray-cast vehicle sim drifts
# between x86_64 and aarch64), where we assert the shape/direction instead.
EXPECTED: dict[str, str | re.Pattern[str]] = {
    "hello_world.py": "final: y=0.60 (rest height ~0.6)",
    "joints/pendulum.py": "tip: x=+0.00 y=+3.50",
    "joints/six_dof_motor.py": "motor: lin.x=3.52 ang.z=0.49",
    "character/stairs.py": "climbed: x=13.50 y=0.28",
    # The vehicle controller's exact speed depends on per-architecture floating
    # point (rapier is not bit-reproducible across arches without
    # enhanced-determinism). Assert it drove backward at a real speed instead.
    "vehicle/drive.py": re.compile(r"^vehicle: speed=-\d+\.\d+ km/h vx=[-+]\d+\.\d+$"),
    "urdf/load_simple.py": "urdf: name=two_link links=2 joints=1",
    "render/matplotlib_animation.py": "matplotlib: segments=27000 frames=120",
    "serde/snapshot_restore.py": "snapshot: snap.y=0.58 later.y=0.60 bytes=1767",
    # perf/many_bodies prints a timing-dependent ms/frame value; we only
    # assert that it ran with the expected shape.
    "perf/many_bodies.py": "perf: bodies=100 frames=240 ms_per_frame_present=True",
    "parity/balls3.py": (
        "parity: (1, 1, 1)=(-1.0,+1.5,-1.0) (2, 2, 2)=(-0.0,+2.5,-0.0) "
        "(2, 3, 2)=(+0.0,+3.5,+0.0)"
    ),
}


@pytest.mark.parametrize("relpath", sorted(EXPECTED))
def test_example_runs(relpath: str, tmp_path: Path) -> None:
    script = EXAMPLES_DIR / relpath
    assert script.exists(), f"example missing: {script}"
    # Use tmp_path as cwd so example artifacts (e.g. matplotlib image output)
    # don't pollute the repo.
    env = os.environ.copy()
    res = subprocess.run(
        [sys.executable, str(script)],
        capture_output=True,
        text=True,
        timeout=30,
        cwd=str(tmp_path),
        env=env,
    )
    assert res.returncode == 0, (
        f"example {relpath} exited {res.returncode}\n"
        f"stdout:\n{res.stdout}\nstderr:\n{res.stderr}"
    )
    # The final non-empty line of stdout must match the expected snapshot.
    lines = [ln for ln in res.stdout.splitlines() if ln.strip()]
    assert lines, f"example {relpath} produced no stdout output"
    expected = EXPECTED[relpath]
    if isinstance(expected, re.Pattern):
        matched = expected.search(lines[-1]) is not None
    else:
        matched = lines[-1] == expected
    assert matched, (
        f"example {relpath} drifted:\n"
        f"  expected: {expected!r}\n"
        f"  actual:   {lines[-1]!r}\n"
        f"  full stdout:\n{res.stdout}"
    )
