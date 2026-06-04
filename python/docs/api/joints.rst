Joints
======

Impulse and multibody joints, builders, motor / limit configuration,
and the supporting handle / index types.

.. currentmodule:: rapier3d

Joint flags and configuration
-----------------------------

.. autoclass:: JointEnabled
.. autoclass:: MotorModel
.. autoclass:: JointLimits
.. autoclass:: JointMotor
.. autoclass:: JointAxesMask
.. autoclass:: JointAxis

Handles and indices
-------------------

.. autoclass:: ImpulseJointHandle
.. autoclass:: MultibodyJointHandle
.. autoclass:: MultibodyIndex
.. autoclass:: MultibodyLinkId
.. autoclass:: InverseKinematicsOption

Fixed joint
-----------

.. autoclass:: FixedJoint
.. autoclass:: FixedJointBuilder

Revolute joint
--------------

.. autoclass:: RevoluteJoint
.. autoclass:: RevoluteJointBuilder

Prismatic joint
---------------

.. autoclass:: PrismaticJoint
.. autoclass:: PrismaticJointBuilder

Spherical joint (3D-only)
-------------------------

.. autoclass:: SphericalJoint
.. autoclass:: SphericalJointBuilder

Rope joint
----------

.. autoclass:: RopeJoint
.. autoclass:: RopeJointBuilder

Spring joint
------------

.. autoclass:: SpringJoint
.. autoclass:: SpringJointBuilder

Generic joint
-------------

.. autoclass:: GenericJoint
.. autoclass:: GenericJointBuilder

Runtime joint records
---------------------

.. autoclass:: ImpulseJoint
.. autoclass:: MultibodyLink
.. autoclass:: Multibody
