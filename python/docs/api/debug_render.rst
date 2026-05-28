Debug-render
============

Adapters for emitting line-segment debug renderings of a physics
world. The cdylib provides the pipeline and stylistic enums; a small
pure-Python :class:`DebugRenderBackend` ABC sits on top so you can
plug in your favorite renderer (matplotlib, pyglet, custom GL, …).

.. currentmodule:: rapier3d

.. autoclass:: DebugRenderPipeline
.. autoclass:: DebugRenderStyle
.. autoclass:: DebugRenderMode
.. autoclass:: DebugRenderObject
.. autoclass:: DebugColor
.. autoclass:: DebugLineCollector

Backend ABC
-----------

.. autoclass:: rapier3d.DebugRenderBackend
   :members:
