Loaders
=======

Loaders for external scene formats. All three are **3D-only / f32**:
the upstream ``rapier3d-urdf``, ``rapier3d-meshloader``, and
``rapier3d-mjcf`` crates are 3D-only and have no ``-f64`` variant, so
the loaders are registered solely by the f32-3D ``rapier3d`` cdylib.

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
