Changelog
=========

The Python bindings track the underlying Rust crates and share their
release cadence. The authoritative changelog is the Cargo
``CHANGELOG.md`` at the root of the
`Rapier repository
<https://github.com/dimforge/rapier/blob/master/CHANGELOG.md>`_.

Unreleased
----------

**Breaking — repackaged as ``rapier3d``.** The single ``rapier`` umbrella
package (with ``rapier.dim3`` submodules) has been replaced by the
``rapier3d`` package (3D / f32):

.. list-table::
   :header-rows: 1

   * - Before
     - After
   * - ``import rapier`` (3D f32 default)
     - ``import rapier3d``

The package is a standard ``abi3`` maturin wheel, so installs and platform
support (manylinux/musllinux, macOS, Windows) now go through the normal
wheel pipeline. The Panda3D testbed moved to a separate ``rapier-testbed``
package.
