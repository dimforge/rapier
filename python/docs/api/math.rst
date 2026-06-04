Math
====

Vectors, points, rotations, isometries, and a handful of free
helpers. The 3D / f32 types live in ``rapier3d``; 2D and f64
variants share the same API surface in their respective sub-modules.

.. currentmodule:: rapier3d

Vectors and points
------------------

.. autoclass:: Vec3
.. autoclass:: Point3

.. data:: AngVector3

   Alias of :class:`Vec3`, used for 3D angular quantities (angular
   velocity, torque). In 2D the corresponding quantity is a plain
   ``float`` and there is no ``AngVector2`` type.

Rotations and orientations
--------------------------

.. autoclass:: Rotation3
.. autoclass:: Quaternion
.. autoclass:: Isometry3

Free helpers
------------

.. autofunction:: rotation_from_angle

.. currentmodule:: rapier3d.math

Pure-Python helpers re-exported under ``rapier3d.math``:

.. automodule:: rapier3d.math
   :no-index:

2D math types
-------------

The same surface in 2D lives under :mod:`rapier2d`:

.. currentmodule:: rapier2d

.. autoclass:: Vec2
.. autoclass:: Point2
.. autoclass:: Rotation2
.. autoclass:: Isometry2

In 2D the angular velocity / acceleration is a plain ``float``, so
there is no separate ``AngVector2`` type.
