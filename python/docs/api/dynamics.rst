Dynamics
========

Rigid bodies, the body set, mass properties, integration parameters,
and the supporting velocity/position/CCD sub-state objects.

.. currentmodule:: rapier3d

Rigid bodies
------------

.. autoclass:: RigidBody
.. autoclass:: RigidBodyBuilder
.. autoclass:: RigidBodySet
.. autoclass:: RigidBodyHandle
.. autoclass:: RigidBodyType
.. autoclass:: LockedAxes

Sub-state components
--------------------

These objects expose the slices of a :class:`RigidBody`'s state that
the solver groups together. They're useful when you want to copy /
restore a single facet of a body without round-tripping the whole
thing.

.. autoclass:: RigidBodyActivation
.. autoclass:: RigidBodyDamping
.. autoclass:: RigidBodyDominance
.. autoclass:: RigidBodyCcd
.. autoclass:: RigidBodyVelocity
.. autoclass:: RigidBodyForces
.. autoclass:: RigidBodyMassProps
.. autoclass:: RigidBodyPosition
.. autoclass:: RigidBodyAdditionalMassProps

Mass properties
---------------

.. autoclass:: MassProperties

Integration parameters
----------------------

.. autoclass:: IntegrationParameters
.. autoclass:: SpringCoefficients
.. autoclass:: CoefficientCombineRule
.. autoclass:: FrictionModel

Solver-side helpers
-------------------

.. autoclass:: IslandManager
.. autoclass:: CCDSolver
.. autoclass:: ImpulseJointSet
.. autoclass:: MultibodyJointSet
