rapier — Python bindings for the Rapier physics engine
======================================================

.. warning::

   **AI-generated code disclaimer.** These bindings were
   written almost entirely by AI agents (Claude). A human reviewed the
   high-level plan and the final outputs, but did not hand-write the
   bulk of the code. Bug reports and PRs welcome.

`Rapier <https://rapier.rs>`_ is a fast, deterministic 2D/3D physics
engine written in Rust. It is exposed to Python via
`PyO3 <https://pyo3.rs/>`_ and `maturin <https://www.maturin.rs/>`_,
targeting Python ≥ 3.9 via the stable ``abi3`` ABI, as **four packages** —
``rapier2d``, ``rapier3d``, ``rapier2d-f64``, ``rapier3d-f64`` — one per
``(dim, scalar)`` flavor. These docs are written against ``rapier3d``
(3D / f32); see :doc:`dim_scalar` for the others.

The Python surface mirrors the Rust crates 1:1 (same type names, same
method names, builder kwargs added on top), so anything you can read in
the `Rust docs <https://docs.rs/rapier3d>`_ has a direct Python
translation.

**Start with** :class:`~rapier3d.PhysicsWorld` — it bundles every
sub-state into one object and runs the simulation with a single
``step()`` call. See :doc:`api/world` for the full entry-point guide; the
remaining API pages cover the individual pieces it wraps.

Quick start
-----------

A ball dropped onto a plane::

    import rapier3d as rp

    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Big static ground cuboid centered at the origin.
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())

    # A dynamic ball 5 m above the ground.
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[rp.Collider.ball(0.5)],
    )

    for _ in range(240):
        world.step()

    pos = world.rigid_bodies[ball].translation
    print(f"ball came to rest at y = {pos.y:.3f}")

``rapier3d`` provides the **3D / f32** flavor. The other ``(dim, scalar)``
combinations live in the ``rapier2d``, ``rapier3d-f64``, and ``rapier2d-f64``
packages — see :doc:`dim_scalar` for the full table.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Guide

   getting_started
   dim_scalar
   changelog

.. toctree::
   :maxdepth: 2
   :caption: API reference

   api/world
   api/math
   api/dynamics
   api/geometry
   api/joints
   api/pipeline
   api/events_hooks
   api/controllers
   api/loaders
   api/debug_render
   api/serde
   api/errors

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
