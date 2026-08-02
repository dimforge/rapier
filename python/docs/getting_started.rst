Getting started
===============

Install
-------

Install the ``rapier3d`` package (3D, f32) from PyPI::

    pip install rapier3d

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

Choosing the number of threads
------------------------------

The bindings always ship the multi-threaded engine, and ``step()``
releases the GIL while it runs. By default a world spreads its parallel
stages over as many workers as there are logical CPUs; give it a
different worker count with ``set_num_threads``::

    world.set_num_threads(4)   # four workers, private to this world
    print(world.num_threads)   # -> 4

    world.set_num_threads(1)   # everything inline on the calling thread
    world.set_num_threads(None)  # back to one worker per logical CPU

Each world gets its own pool, so several worlds stepped from different
Python threads don't compete for one another's workers. On CPUs that mix
performance and efficiency cores, passing the performance-core count is
usually faster than the default: the solver's stages advance at the speed
of their slowest worker.

The worker count never changes the result of a simulation — stepping the
same scene with 1 and with 8 workers gives bit-identical states.

What to read next
-----------------

* :doc:`api/dynamics` — rigid bodies, mass, integration.
* :doc:`api/pipeline` — stepping a world manually & query pipelines.
* :doc:`api/joints` — impulse + multibody joints, motors, limits.
* :doc:`api/controllers` — kinematic character, PID/PD, ray-cast
  vehicle.
