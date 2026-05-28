Pipeline & queries
==================

The lower-level pipelines that :class:`PhysicsWorld` drives for you, plus
the spatial / shape query machinery.

.. note::

   Most programs should use :doc:`world` rather than wiring these
   pipelines up by hand. :class:`PhysicsWorld` owns a
   :class:`PhysicsPipeline` and :class:`QueryPipeline` and steps them for
   you; the types below are for advanced, hand-rolled simulation loops.

.. currentmodule:: rapier3d

Pipelines
---------

.. autoclass:: PhysicsPipeline
.. autoclass:: CollisionPipeline

Query pipeline & filters
------------------------

.. autoclass:: QueryPipeline
.. autoclass:: QueryFilter
.. autoclass:: QueryFilterFlags

Ray & shape casts
-----------------

.. autoclass:: Ray
.. autoclass:: RayIntersection
.. autoclass:: PointProjection
.. autoclass:: ShapeCastHit
.. autoclass:: ShapeCastOptions
.. autoclass:: ShapeCastStatus
.. autoclass:: FeatureId
.. autoclass:: NonlinearRigidMotion

Performance counters
--------------------

.. autoclass:: Counters
.. autoclass:: StagesCounters
.. autoclass:: CollisionDetectionCounters
.. autoclass:: SolverCounters
.. autoclass:: CCDCounters
