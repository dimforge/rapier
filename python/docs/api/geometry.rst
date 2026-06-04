Geometry & shapes
=================

Colliders, the shape family (``SharedShape`` and its concrete
variants), bounding volumes, mesh utilities, interaction groups,
contact pairs, and the broad/narrow-phase types.

.. currentmodule:: rapier3d

Colliders
---------

.. autoclass:: Collider
.. autoclass:: ColliderBuilder
.. autoclass:: ColliderSet
.. autoclass:: ColliderHandle
.. autoclass:: ColliderType
.. autoclass:: ColliderEnabled
.. autoclass:: ColliderMaterial
.. autoclass:: ColliderFlags
.. autoclass:: ColliderParent
.. autoclass:: ColliderPosition
.. autoclass:: ColliderMassProps

Shapes
------

.. autoclass:: ShapeType
.. autoclass:: SharedShape
.. autoclass:: Ball
.. autoclass:: Cuboid
.. autoclass:: Capsule
.. autoclass:: Triangle
.. autoclass:: TriMesh
.. autoclass:: HeightField
.. autoclass:: Compound
.. autoclass:: Cylinder
.. autoclass:: Cone
.. autoclass:: ConvexPolyhedron

``Cylinder``, ``Cone``, and ``ConvexPolyhedron`` are 3D-only. The 2D
packages instead expose the flat shapes below.

2D-only shapes
~~~~~~~~~~~~~~

The same surface in 2D lives under :mod:`rapier2d`:

.. currentmodule:: rapier2d

.. autoclass:: Segment
.. autoclass:: Polyline
.. autoclass:: ConvexPolygon

.. currentmodule:: rapier3d

Bounding volumes
----------------

.. autoclass:: Aabb
.. autoclass:: BoundingSphere

Mesh utilities
--------------

.. autoclass:: MeshConverter
.. autoclass:: TriMeshFlags

Interaction & filtering
-----------------------

.. autoclass:: ActiveEvents
.. autoclass:: ActiveHooks
.. autoclass:: ActiveCollisionTypes
.. autoclass:: Group
.. autoclass:: InteractionTestMode
.. autoclass:: InteractionGroups
.. autoclass:: CollisionEventFlags

Contact events & manifolds
--------------------------

.. autoclass:: CollisionEvent
.. autoclass:: ContactForceEvent
.. autoclass:: ContactData
.. autoclass:: ContactManifold
.. autoclass:: ContactManifoldData
.. autoclass:: ContactPair
.. autoclass:: IntersectionPair
.. autoclass:: ColliderPair
.. autoclass:: BroadPhasePairEvent
.. autoclass:: SolverContact

Broad / narrow phase
--------------------

.. autoclass:: BroadPhaseBvh
.. autoclass:: BvhOptimizationStrategy
.. autoclass:: NarrowPhase
