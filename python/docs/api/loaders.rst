Loaders
=======

Loaders for external scene formats. All three are **3D-only / f32**
— see :doc:`../limitations` for the rationale.

URDF
----

.. automodule:: rapier3d.loaders.urdf
   :members:

Mesh
----

.. automodule:: rapier3d.loaders.mesh
   :members:

MJCF (stub)
-----------

.. automodule:: rapier3d.loaders.mjcf
   :members:

The MJCF loader is a stub in this branch — only the
:class:`~rapier3d.MjcfError` exception type is exposed. Full MJCF
support arrives when the ``mjcf`` Rust branch is merged. See
:doc:`../limitations`.
