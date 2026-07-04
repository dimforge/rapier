Loaders
=======

Loaders for external scene formats, backed by the upstream
``rapier3d-urdf``, ``rapier3d-meshloader``, and ``rapier3d-mjcf`` crates.

URDF
----

.. automodule:: rapier3d.loaders.urdf
   :members:

Mesh
----

.. automodule:: rapier3d.loaders.mesh
   :members:

MJCF
----

.. automodule:: rapier3d.loaders.mjcf
   :members:

Parses MuJoCo MJCF (XML) models into rapier bodies, colliders, joints,
and ``<equality>`` loop closures. Actuator, sensor, and keyframe
elements are not surfaced to Python.
