Errors
======

.. currentmodule:: rapier3d

Every exception the bindings raise derives from a single base,
:class:`RapierError` (itself a subclass of the built-in
:class:`Exception`). Catching ``RapierError`` catches every
rapier-specific failure; catch the leaf types when you need to tell them
apart.

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

Raised by the loaders (see :doc:`loaders`).

.. autoexception:: MeshConversionError
   :show-inheritance:
.. autoexception:: MeshLoaderError
   :show-inheritance:
.. autoexception:: UrdfError
   :show-inheritance:
.. autoexception:: MjcfError
   :show-inheritance:
