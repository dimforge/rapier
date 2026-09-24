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

.. module:: rapier3d.math

Deterministic functions
-----------------------

Computed by Rapier's math backend, in the engine's precision (``f32``). With
the ``determinism`` feature (see :attr:`rapier3d.BuildFeatures.enhanced_determinism`),
they give bit-identical results on every platform, unlike Python's :mod:`math`
module or NumPy.

.. autofunction:: sin
.. autofunction:: cos
.. autofunction:: tan
.. autofunction:: asin
.. autofunction:: acos
.. autofunction:: atan
.. autofunction:: atan2
.. autofunction:: exp
.. autofunction:: log
.. autofunction:: pow
.. autofunction:: sqrt

Helpers
-------

.. autofunction:: lerp
.. autofunction:: linear_interp
.. autofunction:: wrap_to_pi
