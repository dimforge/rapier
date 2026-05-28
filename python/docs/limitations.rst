Known limitations
=================

State of the Python bindings as of the current alpha. The Rust →
Python coverage is broad (635+ pytest tests pass), but there are a
handful of intentional gaps and a few rough edges to be aware of.

MJCF unsupported in this branch
-------------------------------

``crates/rapier3d-mjcf/`` lives on a separate worktree (``mjcf``
branch) and is not pulled into this workspace's ``Cargo.toml``. Only
``MjcfError`` is exposed (stub re-export). The Cassie smoke test
(``tests/test_loaders.py::test_cassie_mjcf_smoke``) is
``pytest.mark.skipif``-gated on the fixture's presence and auto-enables
once MJCF lands.

Snapshot blobs are not flavor-tagged
------------------------------------

:meth:`rapier3d.PhysicsWorld.snapshot` emits a bincode blob with a
``b"RPYS"`` magic prefix and a u32 version, but **not** a
``(dim, scalar)`` tag. Restoring an f32 snapshot through
``rapier3d_f64.PhysicsWorld.restore(...)`` will silently mis-decode
the floats. Always restore through the same flavor that snapshotted.

``rapier-py-3d-f64`` lacks URDF / mesh loaders
----------------------------------------------

The upstream ``rapier3d-urdf`` and ``rapier3d-meshloader`` crates have
no ``-f64`` variant, so the f64-3D cdylib does not expose any loader
machinery. Symbols are absent entirely; consumers must drop to
``rapier3d`` (the f32 variant) to parse robot files.

``cargo fmt`` quirk inside macros
---------------------------------

The PyO3 ``signature(...)`` multi-line attribute inside the
``define_*_types!`` macros could not be formatted with normal
multi-line indentation — rustfmt iterates the indent further on each
invocation. The two affected attributes in
``rapier-py-core/src/{controllers,debug_render}.rs`` were collapsed
onto a single line; this is functionally equivalent and stable under
``cargo fmt --check``.

``rapier3d-urdf`` clippy lint (upstream, pre-existing)
------------------------------------------------------

Under rustc ≥ 1.95, ``rapier3d-urdf/src/lib.rs:324`` triggers
``clippy::useless_conversion``. The clippy check for the Python
bindings runs with ``--no-deps`` to bypass this — the bindings
themselves are clean.

Determinism mode
----------------

``RAPIER_PY_DETERMINISM=1`` and the ``determinism`` Cargo feature
route to single-threaded execution paths. Compile with
``maturin build --release -F determinism`` for a wheel that bakes the
toggle in; the runtime env var is the lighter knob.
