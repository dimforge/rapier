"""Python `Protocol` shim for the debug-render backend API.

This is a duck-typing protocol (`typing.Protocol`); user classes don't need to
inherit from it — they just need to implement the right method names.

The Rust ``DebugRenderPipeline.render(..., backend=obj)`` accepts any of:

- the built-in ``DebugLineCollector`` (fast path; pure-Rust buffer),
- a Python object exposing ``draw_line(object, a, b, color) -> None``.

Calling into Python per line is significantly slower than the
``DebugLineCollector`` fast path; prefer the collector for anything beyond
prototyping or pure-Python visualization integrations.
"""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class DebugRenderBackend(Protocol):
    """Receiver of debug-render line draw calls.

    Implement this protocol on a class and pass an instance to
    :meth:`DebugRenderPipeline.render`. Each call to ``draw_line`` receives:

    * ``object`` — a :class:`DebugRenderObject` describing what is being
      rendered (collider shape, AABB, joint anchor, contact, ...).
    * ``a``, ``b`` — line endpoints as ``Vec2``/``Vec3`` (or whatever the
      cdylib's vector type is for the current ``(dim, scalar)``).
    * ``color`` — a :class:`DebugColor` (HSLA tuple; use ``.rgba`` for RGBA
      conversion if your downstream renderer expects RGB).

    Exceptions raised inside ``draw_line`` are stashed and re-raised after
    ``render()`` returns — subsequent ``draw_line`` calls within the same
    ``render`` invocation become no-ops once the slot is filled.
    """

    def draw_line(
        self,
        object: Any,
        a: Any,
        b: Any,
        color: Any,
    ) -> None:
        """Draw a single line segment.

        ``object`` is a :class:`DebugRenderObject`. ``a`` / ``b`` are vectors
        (``Vec2`` in 2D, ``Vec3`` in 3D). ``color`` is a :class:`DebugColor`
        (HSLA storage; call ``.rgba`` to convert to RGB).
        """
        ...


__all__ = ["DebugRenderBackend"]
