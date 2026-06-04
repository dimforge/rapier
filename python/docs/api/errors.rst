Errors
======

.. currentmodule:: rapier3d

Every exception the bindings raise derives from a single base,
:class:`RapierError` (itself a subclass of the built-in
:class:`Exception`). Catching ``RapierError`` catches every
rapier-specific failure; catch the leaf types when you need to tell them
apart.

Each ``(dim, scalar)`` package has its **own** error tree — there is no
cross-flavor base shared between, say, ``rapier3d`` and ``rapier2d``, so
catch the error type from the package you imported.

Base
----

.. autoexception:: RapierError
   :show-inheritance:

Core errors
-----------

.. autoexception:: InvalidHandle
   :show-inheritance:
.. autoexception:: QueryFailure
   :show-inheritance:
.. autoexception:: SerializationError
   :show-inheritance:

Loader errors
-------------

Raised by the loaders (see :doc:`loaders`). The error types are exported
by every package for a uniform error tree, but only the f32-3D
``rapier3d`` package ships loaders that actually raise them — the 2D and
f64 packages expose no loader machinery.

.. autoexception:: MeshConversionError
   :show-inheritance:
.. autoexception:: MeshLoaderError
   :show-inheritance:
.. autoexception:: UrdfError
   :show-inheritance:
.. autoexception:: MjcfError
   :show-inheritance:
