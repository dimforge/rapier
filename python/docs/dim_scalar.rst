Dimensions and scalar precision
===============================

Rapier's Python bindings ship as **four** independent PyPI packages, one
per ``(dim, scalar)`` flavor. They share an identical API differing only in
dimension and scalar type:

.. list-table::
   :header-rows: 1
   :widths: 20 18 10 12

   * - PyPI package
     - import name
     - dim
     - scalar
   * - ``rapier3d``
     - ``rapier3d``
     - 3
     - f32
   * - ``rapier3d-f64``
     - ``rapier3d_f64``
     - 3
     - f64
   * - ``rapier2d``
     - ``rapier2d``
     - 2
     - f32
   * - ``rapier2d-f64``
     - ``rapier2d_f64``
     - 2
     - f64

Install only the flavor(s) you need (``pip install rapier3d``). ``rapier3d``
is the canonical reference these docs are written against, matching
``use rapier3d::prelude::*`` in Rust.

Recommended import patterns
---------------------------

.. code-block:: python

    # The common case: 3D, single precision.
    import rapier3d as rp

    # 2D, single precision.
    import rapier2d as rp

    # 3D, double precision.
    import rapier3d_f64 as rp

    # 2D, double precision.
    import rapier2d_f64 as rp

The Python API surface is identical across ``(dim, scalar)`` modulo
dim-specific types:

* ``Vec2`` / ``Point2`` / ``Rotation2`` / ``Isometry2`` versus
  ``Vec3`` / ``Point3`` / ``Rotation3`` / ``Quaternion`` / ``Isometry3``.
* ``SphericalJoint`` and the ray-cast vehicle controller are 3D-only.
* ``Cylinder``, ``Cone``, and ``ConvexPolyhedron`` are 3D-only shapes;
  ``Segment``, ``Polyline``, and ``ConvexPolygon`` are 2D-only.
* Angular quantities (angular velocity, torque) are a plain ``float`` in
  2D — there is no ``AngVector2`` type. In 3D, ``AngVector3`` is an alias
  for :class:`~rapier3d.Vec3`.

Choosing a precision
--------------------

Single precision (f32) is the default and is what the rest of the
Rapier ecosystem ships. It's the right answer for almost all use
cases — games, robotics simulation, and physics-based animation.

Double precision (f64) is available for cases that require it: large
worlds where coordinates exceed ~1e5 metres, long-running deterministic
simulations where round-off drift matters, or comparisons against
reference solutions computed in f64.

The two flavors have separate cdylibs and separate types — instances
do not interoperate. ``Vec3(1, 2, 3)`` from ``rapier3d`` is a
**different class** from ``Vec3(1, 2, 3)`` from ``rapier3d_f64``.

Snapshot blobs are *not* tagged with their flavor: always restore a
snapshot through the same ``(dim, scalar)`` flavor that produced it,
otherwise the floats will be silently misinterpreted.
