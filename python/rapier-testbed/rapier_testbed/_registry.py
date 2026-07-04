"""Global registry of ported testbed examples.

Each example module imports :func:`register` and adds an :class:`Example`
entry at import time. The :func:`run` launcher in :mod:`rapier_testbed._picker`
walks :data:`EXAMPLES` to display the picker.

Example::

    from rapier_testbed import register

    def init_world(testbed):
        # build sets, hand them to the testbed...
        ...

    register("Collisions", "Primitives", init_world)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - import-cycle guard
    from ._testbed import Testbed


InitWorldFn = Callable[["Testbed"], None]


@dataclass(frozen=True)
class Example:
    """One entry in the picker.

    :param category: Free-form grouping name (e.g. ``"Collisions"``,
        ``"Joints"``). Used by the picker to group entries.
    :param name: Human-readable example name (e.g. ``"Primitives"``).
    :param init_fn: Callable that receives a fresh :class:`Testbed` and
        populates it via :meth:`Testbed.set_world` and friends.
    :param dim: spatial dimension of the scene (always ``3``).
    """

    category: str
    name: str
    init_fn: InitWorldFn
    dim: int = 3


# Global, populated by `register(...)` calls in each example module's import.
EXAMPLES: List[Example] = []


def register(
    category: str,
    name: str,
    init_fn: InitWorldFn,
    *,
    dim: int = 3,
) -> Example:
    """Register an example with the global testbed registry.

    Re-registering the same ``(category, name, dim)`` triple replaces the
    existing entry — useful when iterating in a Jupyter notebook.
    """
    entry = Example(category=category, name=name, init_fn=init_fn, dim=dim)
    # Drop any previous entry with the same identity, then append.
    for i, existing in enumerate(EXAMPLES):
        if (
            existing.category == category
            and existing.name == name
            and existing.dim == dim
        ):
            EXAMPLES[i] = entry
            return entry
    EXAMPLES.append(entry)
    return entry


def find(category: str, name: str, *, dim: int = 3) -> Optional[Example]:
    """Look up a registered example by ``(category, name, dim)``."""
    for ex in EXAMPLES:
        if ex.category == category and ex.name == name and ex.dim == dim:
            return ex
    return None


def by_label(label: str) -> Optional[Example]:
    """Look up an example by its display label ``"Category / Name"``."""
    for ex in EXAMPLES:
        if f"{ex.category} / {ex.name}" == label:
            return ex
    return None


__all__ = ["Example", "EXAMPLES", "InitWorldFn", "register", "find", "by_label"]
