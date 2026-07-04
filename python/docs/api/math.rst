Math
====

Vectors, points, rotations, isometries, and a handful of free
helpers, in the 3D / f32 ``rapier3d`` package.

.. currentmodule:: rapier3d

Vectors and points
------------------

.. autoclass:: Vec3
.. autoclass:: Point3

.. data:: AngVector3

   Alias of :class:`Vec3`, used for 3D angular quantities (angular
   velocity, torque).

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
