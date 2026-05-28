Changelog
=========

The Python bindings track the underlying Rust crates and share their
release cadence. The authoritative changelog is the Cargo
``CHANGELOG.md`` at the root of the
`Rapier repository
<https://github.com/dimforge/rapier/blob/master/CHANGELOG.md>`_.

A summary of what's in the current alpha is in
:doc:`limitations`.

Unreleased
----------

**Breaking — repackaged into four PyPI packages.** The single ``rapier``
umbrella package (with ``rapier.dim2`` / ``rapier.dim3.f64`` submodules) has
been replaced by four independent packages, one per ``(dim, scalar)`` flavor:

.. list-table::
   :header-rows: 1

   * - Before
     - After
   * - ``import rapier`` (3D f32 default)
     - ``import rapier3d``
   * - ``from rapier import dim2``
     - ``import rapier2d``
   * - ``from rapier.dim3 import f64``
     - ``import rapier3d_f64``
   * - ``from rapier.dim2 import f64``
     - ``import rapier2d_f64``

There is no longer a cross-flavor ``rapier.RapierError`` base; each package
exposes its own error tree. Each package is a standard ``abi3`` maturin
wheel, so installs and platform support (manylinux/musllinux, macOS, Windows)
now go through the normal wheel pipeline. The Panda3D testbed moved to a
separate ``rapier-testbed`` package.
