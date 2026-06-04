Getting started
===============

Install
-------

Each ``(dim, scalar)`` flavor is its own PyPI package. Install the one you
need::

    pip install rapier3d        # 3D, f32 (the common case)
    pip install rapier2d        # 2D, f32
    pip install rapier3d-f64    # 3D, f64
    pip install rapier2d-f64    # 2D, f64

Wheels are ``abi3`` (Python ≥ 3.9) and are published for Linux
(manylinux + musllinux, x86_64 + aarch64), macOS (arm64), and
Windows x64.

To build from a `Rapier checkout <https://github.com/dimforge/rapier>`_
instead — e.g. to develop against local engine changes — use ``maturin``
on the relevant crate::

    maturin develop --release -m python/rapier-py-3d/Cargo.toml   # editable install
    maturin build   --release -m python/rapier-py-3d/Cargo.toml   # produces a .whl

For deterministic builds (libm-based transcendentals for cross-platform
bit-reproducibility) add ``-F determinism``::

    maturin build --release -F determinism -m python/rapier-py-3d/Cargo.toml

You can also export ``RAPIER_PY_DETERMINISM=1`` before importing the
package; note this is informational unless the ``determinism`` feature was
also compiled in.

First simulation
----------------

The ``rapier3d.PhysicsWorld`` umbrella aggregates every sub-state the
engine needs (body / collider / joint sets, broad / narrow phase,
island manager, CCD solver, integration parameters). For most users
it's the right entry point:

.. code-block:: python

    import rapier3d as rp

    world = rp.PhysicsWorld(gravity=(0, -9.81, 0))

    # Ground cuboid.
    world.colliders.insert(rp.Collider.cuboid(50, 0.1, 50).build())

    # Falling ball.
    ball = world.add_body(
        rp.RigidBody.dynamic(translation=(0, 5, 0)),
        colliders=[rp.Collider.ball(0.5)],
    )

    for _ in range(240):
        world.step()

    pos = world.rigid_bodies[ball].translation
    print(f"ball came to rest at y = {pos.y:.3f}")

``world.add_body(...)`` is the Pythonic shortcut for inserting a builder
and attaching colliders to the resulting rigid body. The lower-level
``world.rigid_bodies.insert(...)`` + ``world.colliders.insert_with_parent(...)``
flow is also available and matches the Rust API.

What to read next
-----------------

* :doc:`dim_scalar` — how to pick a ``(dim, scalar)`` flavor.
* :doc:`api/dynamics` — rigid bodies, mass, integration.
* :doc:`api/pipeline` — stepping a world manually & query pipelines.
* :doc:`api/joints` — impulse + multibody joints, motors, limits.
* :doc:`api/controllers` — kinematic character, PID/PD, ray-cast
  vehicle.
