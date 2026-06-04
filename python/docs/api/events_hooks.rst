Events & hooks
==============

Channel-backed event collection plus the Python ABCs you implement to
participate in the solver's filtering and contact-modification hooks.

.. currentmodule:: rapier3d

Solver-side context
-------------------

.. autoclass:: SolverFlags
.. autoclass:: PairFilterContext
.. autoclass:: ContactModificationContext

Event collection
----------------

.. autoclass:: ChannelEventCollector

Python ABCs
-----------

Implement these as subclasses to plug Python callbacks into the
solver. They live in pure-Python helper modules (not the cdylib) and
are re-exported at the package root for convenience.

.. autoclass:: rapier3d.EventHandler
   :members:
.. autoclass:: rapier3d.PhysicsHooks
   :members:
