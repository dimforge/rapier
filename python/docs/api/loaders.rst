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
and ``<equality>`` loop closures. The
:class:`~rapier3d.loaders.mjcf.MjcfRobotHandles` returned by the insertion
drive the ``<actuator>`` elements
(:meth:`~rapier3d.loaders.mjcf.MjcfRobotHandles.apply_controls`), apply the
``<keyframe>`` elements
(:meth:`~rapier3d.loaders.mjcf.MjcfRobotHandles.apply_keyframe`) and apply
the ``<contact>`` rules
(:meth:`~rapier3d.loaders.mjcf.MjcfRobotHandles.contact_hooks`). Sensors
are not surfaced to Python.
