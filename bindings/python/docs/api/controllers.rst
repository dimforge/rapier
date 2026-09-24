Controllers
===========

Higher-level helpers built on top of the core physics pipeline:
kinematic character controller, PD / PID position controllers, and
the ray-cast vehicle controller.

.. currentmodule:: rapier3d

Shared types
------------

.. autoclass:: AxesMask
.. autoclass:: CharacterLength

Kinematic character controller
------------------------------

.. autoclass:: KinematicCharacterController
.. autoclass:: CharacterAutostep
.. autoclass:: CharacterCollision
.. autoclass:: EffectiveCharacterMovement

PD / PID controllers
--------------------

.. autoclass:: PdController
.. autoclass:: PidController
.. autoclass:: PdErrors
.. autoclass:: PidCorrection

Ray-cast vehicle controller (3D-only)
-------------------------------------

.. autoclass:: DynamicRayCastVehicleController
.. autoclass:: WheelTuning
.. autoclass:: Wheel
.. autoclass:: RayCastInfo
