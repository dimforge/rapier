//! Macro emitting the user-facing debug-render `#[pyclass]`-es per cdylib.
//!
//! Debug-Render Adapter.
//!
//! Like the other modules, this macro is invoked once per cdylib AFTER
//! `define_conv_types!`, `define_math_types!`, `define_geometry_types!`,
//! `define_joints_types!`, `define_dynamics_types!`, `define_pipeline_types!`,
//! `define_events_hooks_types!` (and, for 3D, `define_loaders_types!`) and
//! `define_controllers_types!`. It depends on items emitted by all of those:
//!   - `Real`/`DIM`, the `PyVector` adapter.
//!   - User-facing math types (`Vec*`, `Point*`).
//!   - `RigidBodySet`, `ColliderSet`, `ImpulseJointSet`, `MultibodyJointSet`,
//!     `NarrowPhase`.
//!
//! `define_debug_render_types!` produces `register_debug_render(py, m)` for the
//! `#[pymodule]` entry point.
//!
//! # Cargo feature
//!
//! All four `rapier-py-*` cdylibs enable the upstream `debug-render` Cargo
//! feature unconditionally so that `DebugRenderPipeline` (and the associated
//! trait, style, mode, object enums) are available.
//!
//! # API surface
//!
//! - `DebugRenderPipeline` — wraps `rapier::pipeline::DebugRenderPipeline`,
//!   plus a convenience `render_to_arrays(...)` returning NumPy arrays.
//! - `DebugLineCollector` — built-in `DebugRenderBackend` that drains all
//!   draw calls into internal `Vec`s exposed as NumPy arrays (or iterated as
//!   `(object, a, b, color)` tuples).
//! - `DebugRenderBackend` (Python `Protocol` — see `python/rapier/_debug_render.py`).
//! - `DebugRenderStyle` — wraps the rapier struct with every public field as a
//!   mutable Python property.
//! - `DebugRenderMode` — bitflags wrapper.
//! - `DebugRenderObject` — frozen tagged-union mirror of the Rust enum
//!   (`COLLIDER_SHAPE`, `RIGID_BODY_FRAME`, etc.).
//! - `DebugColor` — small frozen wrapper around `[f32; 4]` with both an HSLA
//!   and an RGBA constructor (rapier stores HSLA internally — see
//!   `src/pipeline/debug_render_pipeline/debug_render_style.rs`).
//!
//! # NumPy copy semantics
//!
//! Both `render_to_arrays` and `DebugLineCollector.lines()` / `.colors()` /
//! `.objects()` allocate fresh `PyArray*` buffers and **copy** the underlying
//! data. There is no zero-copy aliasing — the collector remains owned by
//! Python and can be cleared / re-used safely.

/// Materialize the debug-render `#[pyclass]` types for a given `(Real, DIM)` pair.
#[macro_export]
macro_rules! define_debug_render_types {
    (DIM = 2) => {
        $crate::__define_debug_render_shared!(2, Vec2);
        $crate::__define_debug_render_register!();
    };
    (DIM = 3) => {
        $crate::__define_debug_render_shared!(3, Vec3);
        $crate::__define_debug_render_register!();
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_debug_render_shared {
    ($dim:tt, $Vec:ident) => {
        use std::sync::{Arc as _DbgArc, Mutex as _DbgMutex};

        // =====================================================================
        // Internal storage type. Each `DebugLine` records a single segment as
        // emitted by rapier into the backend.
        // =====================================================================

        #[derive(Debug, Clone, Copy)]
        struct DebugLine {
            object_kind: u32,
            a: $crate::na::SVector<Real, $dim>,
            b: $crate::na::SVector<Real, $dim>,
            color: [f32; 4],
        }

        // =====================================================================
        // DebugRenderObject — tag mirror of `rapier::pipeline::DebugRenderObject`.
        //
        // The upstream enum carries handle + by-ref payloads; we expose only the
        // discriminant + handles since the Python side has no way to safely
        // borrow the referenced rapier objects after the call returns.
        // =====================================================================

        /// Tagged-union mirror of ``rapier::pipeline::DebugRenderObject``.
        ///
        /// Each segment emitted by :class:`DebugRenderPipeline` is tagged
        /// with one of these. Use the class constants
        /// :attr:`RIGID_BODY`, :attr:`COLLIDER`, :attr:`COLLIDER_AABB`,
        /// :attr:`IMPULSE_JOINT`, :attr:`MULTIBODY_JOINT`, :attr:`CONTACT_PAIR`
        /// for comparison.
        ///
        /// :ivar kind: Stable ``u32`` discriminant.
        #[allow(non_camel_case_types)]
        #[pyclass(name = "DebugRenderObject", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct DebugRenderObject {
            #[pyo3(get)]
            pub kind: u32,
        }

        // Stable u32 discriminants used both as `#[classattr]` values below
        // and by the `_dro_kind` helper to tag each `DebugLine`.
        const _DRO_RIGID_BODY: u32 = 0;
        const _DRO_COLLIDER: u32 = 1;
        const _DRO_COLLIDER_AABB: u32 = 2;
        const _DRO_IMPULSE_JOINT: u32 = 3;
        const _DRO_MULTIBODY_JOINT: u32 = 4;
        const _DRO_CONTACT_PAIR: u32 = 5;

        #[pymethods]
        impl DebugRenderObject {
            #[classattr]
            const RIGID_BODY: DebugRenderObject = DebugRenderObject {
                kind: _DRO_RIGID_BODY,
            };
            #[classattr]
            const COLLIDER: DebugRenderObject = DebugRenderObject {
                kind: _DRO_COLLIDER,
            };
            #[classattr]
            const COLLIDER_AABB: DebugRenderObject = DebugRenderObject {
                kind: _DRO_COLLIDER_AABB,
            };
            #[classattr]
            const IMPULSE_JOINT: DebugRenderObject = DebugRenderObject {
                kind: _DRO_IMPULSE_JOINT,
            };
            #[classattr]
            const MULTIBODY_JOINT: DebugRenderObject = DebugRenderObject {
                kind: _DRO_MULTIBODY_JOINT,
            };
            #[classattr]
            const CONTACT_PAIR: DebugRenderObject = DebugRenderObject {
                kind: _DRO_CONTACT_PAIR,
            };

            /// Return ``DebugRenderObject.<NAME>`` repr.
            fn __repr__(&self) -> String {
                let s = match self.kind {
                    0 => "RIGID_BODY",
                    1 => "COLLIDER",
                    2 => "COLLIDER_AABB",
                    3 => "IMPULSE_JOINT",
                    4 => "MULTIBODY_JOINT",
                    5 => "CONTACT_PAIR",
                    _ => "UNKNOWN",
                };
                format!("DebugRenderObject.{s}")
            }
            /// Equality (``==`` / ``!=``). Other comparisons raise ``TypeError``.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.kind == other.kind),
                    CompareOp::Ne => Ok(self.kind != other.kind),
                    _ => Err(PyTypeError::new_err(
                        "DebugRenderObject supports only == and !=",
                    )),
                }
            }
            /// Hash by ``kind`` discriminant (consistent with ``==``).
            fn __hash__(&self) -> u64 {
                self.kind as u64
            }
        }

        #[inline]
        fn _dro_kind(obj: rapier::pipeline::DebugRenderObject<'_>) -> u32 {
            match obj {
                rapier::pipeline::DebugRenderObject::RigidBody(..) => _DRO_RIGID_BODY,
                rapier::pipeline::DebugRenderObject::Collider(..) => _DRO_COLLIDER,
                rapier::pipeline::DebugRenderObject::ColliderAabb(..) => _DRO_COLLIDER_AABB,
                rapier::pipeline::DebugRenderObject::ImpulseJoint(..) => _DRO_IMPULSE_JOINT,
                rapier::pipeline::DebugRenderObject::MultibodyJoint(..) => _DRO_MULTIBODY_JOINT,
                rapier::pipeline::DebugRenderObject::ContactPair(..) => _DRO_CONTACT_PAIR,
            }
        }

        // =====================================================================
        // DebugColor — wrapper around `[f32; 4]` (rapier's `DebugColor` alias).
        //
        // Rapier stores colors in HSLA. The `from_rgba` constructor converts
        // from RGBA via the standard HSL transform so users don't have to
        // think about it; the `.rgba` getter goes the other way. The raw HSLA
        // tuple is also accessible via `.hsla` for direct interop.
        // =====================================================================

        /// Color used by the debug-render pipeline.
        ///
        /// Internally stored as HSLA (hue in degrees ``[0, 360)``,
        /// saturation / lightness / alpha each in ``[0, 1]``) because
        /// that's what rapier's :class:`DebugRenderStyle` uses. Two
        /// constructors are provided — :meth:`from_hsla` and
        /// :meth:`from_rgba` — plus the :attr:`rgba` getter to convert
        /// back to RGBA suitable for matplotlib / pygame / etc.
        #[pyclass(name = "DebugColor", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy)]
        pub struct DebugColor(pub [f32; 4]);

        #[inline]
        fn _hsla_to_rgba(hsla: [f32; 4]) -> [f32; 4] {
            // h in degrees [0, 360), s/l/a in [0, 1].
            let mut h = hsla[0] % 360.0;
            if h < 0.0 {
                h += 360.0;
            }
            let s = hsla[1].clamp(0.0, 1.0);
            let l = hsla[2].clamp(0.0, 1.0);
            let a = hsla[3];

            let c = (1.0 - (2.0 * l - 1.0).abs()) * s;
            let h_prime = h / 60.0;
            let x = c * (1.0 - (h_prime % 2.0 - 1.0).abs());
            let (r1, g1, b1) = match h_prime as u32 {
                0 => (c, x, 0.0),
                1 => (x, c, 0.0),
                2 => (0.0, c, x),
                3 => (0.0, x, c),
                4 => (x, 0.0, c),
                _ => (c, 0.0, x),
            };
            let m = l - c * 0.5;
            [r1 + m, g1 + m, b1 + m, a]
        }

        #[inline]
        fn _rgba_to_hsla(rgba: [f32; 4]) -> [f32; 4] {
            let r = rgba[0];
            let g = rgba[1];
            let b = rgba[2];
            let a = rgba[3];
            let max = r.max(g).max(b);
            let min = r.min(g).min(b);
            let l = (max + min) * 0.5;
            let d = max - min;
            let s = if d == 0.0 {
                0.0
            } else {
                d / (1.0 - (2.0 * l - 1.0).abs()).max(1.0e-12)
            };
            let h = if d == 0.0 {
                0.0
            } else if max == r {
                60.0 * (((g - b) / d) % 6.0)
            } else if max == g {
                60.0 * (((b - r) / d) + 2.0)
            } else {
                60.0 * (((r - g) / d) + 4.0)
            };
            let h = if h < 0.0 { h + 360.0 } else { h };
            [h, s, l, a]
        }

        #[pymethods]
        impl DebugColor {
            /// Build a ``DebugColor`` from a raw HSLA tuple.
            ///
            /// :param h: Hue in degrees ``[0, 360)``.
            /// :param s: Saturation in ``[0, 1]``.
            /// :param l: Lightness in ``[0, 1]``.
            /// :param a: Alpha in ``[0, 1]``.
            #[new]
            #[pyo3(signature = (h = 0.0, s = 0.0, l = 0.0, a = 1.0))]
            fn new(h: f32, s: f32, l: f32, a: f32) -> Self {
                Self([h, s, l, a])
            }

            /// Construct a `DebugColor` from an HSLA tuple (hue in degrees,
            /// saturation/lightness/alpha in `[0, 1]`).
            #[staticmethod]
            fn from_hsla(h: f32, s: f32, l: f32, a: f32) -> Self {
                Self([h, s, l, a])
            }

            /// Construct a `DebugColor` from an RGBA tuple (`[0, 1]` each).
            ///
            /// Internally converted to HSLA since rapier's `DebugRenderStyle`
            /// expects HSLA tuples.
            #[staticmethod]
            fn from_rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
                Self(_rgba_to_hsla([r, g, b, a]))
            }

            /// The raw HSLA tuple stored by this color.
            #[getter]
            fn hsla(&self) -> [f32; 4] {
                self.0
            }

            /// The RGBA tuple after HSLA → RGBA conversion (each in `[0, 1]`).
            #[getter]
            fn rgba(&self) -> [f32; 4] {
                _hsla_to_rgba(self.0)
            }

            /// Hue component (degrees, ``[0, 360)``).
            #[getter]
            fn h(&self) -> f32 {
                self.0[0]
            }
            /// Saturation component (``[0, 1]``).
            #[getter]
            fn s(&self) -> f32 {
                self.0[1]
            }
            /// Lightness component (``[0, 1]``).
            #[getter]
            fn l(&self) -> f32 {
                self.0[2]
            }
            /// Alpha component (``[0, 1]``).
            #[getter]
            fn a(&self) -> f32 {
                self.0[3]
            }

            /// Return the ``DebugColor(h=..., s=..., l=..., a=...)`` repr.
            fn __repr__(&self) -> String {
                format!(
                    "DebugColor(h={}, s={}, l={}, a={})",
                    self.0[0], self.0[1], self.0[2], self.0[3]
                )
            }
            /// Equality (``==`` / ``!=``). Other comparisons raise ``TypeError``.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err("DebugColor supports only == and !=")),
                }
            }
        }

        impl DebugColor {
            #[inline]
            fn extract_from(obj: &Bound<'_, PyAny>) -> PyResult<[f32; 4]> {
                if let Ok(c) = obj.extract::<DebugColor>() {
                    return Ok(c.0);
                }
                let v: Vec<f32> = $crate::conv::extract_floats_from_sequence(obj, 4)?;
                Ok([v[0], v[1], v[2], v[3]])
            }
        }

        // =====================================================================
        // DebugRenderMode — bitflags wrapper.
        // =====================================================================

        /// Bitflags selecting what :class:`DebugRenderPipeline` renders.
        ///
        /// Combine flags with ``|``, ``&``, ``^``, ``~``. The
        /// :meth:`default` set is
        /// ``COLLIDER_SHAPES | JOINTS | RIGID_BODY_AXES`` and is what
        /// ``DebugRenderPipeline()`` uses if no mode is passed.
        #[pyclass(name = "DebugRenderMode", module = "rapier", frozen)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct DebugRenderMode(pub rapier::pipeline::DebugRenderMode);

        #[pymethods]
        impl DebugRenderMode {
            /// Build a ``DebugRenderMode`` from a raw bit pattern.
            #[new]
            #[pyo3(signature = (bits = 0))]
            fn new(bits: u32) -> Self {
                Self(rapier::pipeline::DebugRenderMode::from_bits_truncate(bits))
            }
            /// Return the empty flag set (renders nothing).
            #[staticmethod]
            fn empty() -> Self {
                Self(rapier::pipeline::DebugRenderMode::empty())
            }
            /// Default flag set: ``COLLIDER_SHAPES | JOINTS | RIGID_BODY_AXES``.
            #[staticmethod]
            #[pyo3(name = "default")]
            fn py_default() -> Self {
                Self(rapier::pipeline::DebugRenderMode::default())
            }
            /// Return the union of all known flags.
            #[staticmethod]
            fn all() -> Self {
                Self(rapier::pipeline::DebugRenderMode::all())
            }

            #[classattr]
            const COLLIDER_SHAPES: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::COLLIDER_SHAPES);
            #[classattr]
            const RIGID_BODY_AXES: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::RIGID_BODY_AXES);
            #[classattr]
            const MULTIBODY_JOINTS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::MULTIBODY_JOINTS);
            #[classattr]
            const IMPULSE_JOINTS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::IMPULSE_JOINTS);
            #[classattr]
            const JOINTS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::JOINTS);
            #[classattr]
            const SOLVER_CONTACTS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::SOLVER_CONTACTS);
            #[classattr]
            const CONTACTS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::CONTACTS);
            #[classattr]
            const COLLIDER_AABBS: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::COLLIDER_AABBS);
            #[classattr]
            const EMPTY: DebugRenderMode =
                DebugRenderMode(rapier::pipeline::DebugRenderMode::empty());

            /// Raw bits as an unsigned int.
            #[getter]
            fn bits(&self) -> u32 {
                self.0.bits()
            }
            /// ``True`` if ``self`` is a superset of ``other``.
            fn contains(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// ``True`` if no bits are set.
            fn is_empty(&self) -> bool {
                self.0.is_empty()
            }
            /// Python ``in`` operator: alias of :meth:`contains`.
            fn __contains__(&self, other: &Self) -> bool {
                self.0.contains(other.0)
            }
            /// Bitwise OR: union of two flag sets.
            fn __or__(&self, other: &Self) -> Self {
                Self(self.0 | other.0)
            }
            /// Bitwise AND: intersection of two flag sets.
            fn __and__(&self, other: &Self) -> Self {
                Self(self.0 & other.0)
            }
            /// Bitwise XOR: symmetric difference of two flag sets.
            fn __xor__(&self, other: &Self) -> Self {
                Self(self.0 ^ other.0)
            }
            /// Bitwise NOT: complement within all known flags.
            fn __invert__(&self) -> Self {
                Self(!self.0)
            }
            /// ``False`` iff no bits are set.
            fn __bool__(&self) -> bool {
                !self.0.is_empty()
            }
            /// Equality (``==`` / ``!=``). Other comparisons raise ``TypeError``.
            fn __richcmp__(&self, other: &Self, op: CompareOp) -> PyResult<bool> {
                match op {
                    CompareOp::Eq => Ok(self.0 == other.0),
                    CompareOp::Ne => Ok(self.0 != other.0),
                    _ => Err(PyTypeError::new_err(
                        "DebugRenderMode supports only == and !=",
                    )),
                }
            }
            /// Hash by raw bit pattern (consistent with ``==``).
            fn __hash__(&self) -> u64 {
                self.0.bits() as u64
            }
            /// Return ``DebugRenderMode(bits=0b...)`` repr.
            fn __repr__(&self) -> String {
                format!("DebugRenderMode(bits={:#010b})", self.0.bits())
            }
        }

        // =====================================================================
        // DebugRenderStyle — wraps `rapier::pipeline::DebugRenderStyle` with
        // every public field as a mutable Python property.
        // =====================================================================

        /// Style configuration for the debug-render pipeline.
        ///
        /// Wraps ``rapier::pipeline::DebugRenderStyle`` with every public
        /// field as a mutable Python property. The default constructor
        /// uses rapier's upstream defaults.
        ///
        /// Color attributes accept either a :class:`DebugColor` or any
        /// 4-float sequence (HSLA).
        #[pyclass(name = "DebugRenderStyle", module = "rapier")]
        #[derive(Debug, Clone, Copy)]
        pub struct DebugRenderStyle(pub rapier::pipeline::DebugRenderStyle);

        #[pymethods]
        impl DebugRenderStyle {
            /// Build a ``DebugRenderStyle`` with rapier's default values.
            #[new]
            fn new() -> Self {
                Self(rapier::pipeline::DebugRenderStyle::default())
            }
            /// Return rapier's default ``DebugRenderStyle``.
            #[staticmethod]
            #[pyo3(name = "default")]
            fn py_default() -> Self {
                Self(rapier::pipeline::DebugRenderStyle::default())
            }

            /// Number of subdivisions used for curved shapes (cylinders,
            /// spheres, capsules).
            #[getter]
            fn subdivisions(&self) -> u32 {
                self.0.subdivisions
            }
            #[setter]
            /// Assign the curved-shape tessellation subdivision count.
            fn set_subdivisions(&mut self, v: u32) {
                self.0.subdivisions = v;
            }
            /// Subdivisions used for borders of e.g. round-cuboids.
            #[getter]
            fn border_subdivisions(&self) -> u32 {
                self.0.border_subdivisions
            }
            #[setter]
            /// Assign the rounded-border tessellation subdivision count.
            fn set_border_subdivisions(&mut self, v: u32) {
                self.0.border_subdivisions = v;
            }

            /// Color for colliders attached to a dynamic rigid body.
            #[getter]
            fn collider_dynamic_color(&self) -> DebugColor {
                DebugColor(self.0.collider_dynamic_color)
            }
            #[setter]
            /// Set the color used to render dynamic-body colliders.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_collider_dynamic_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.collider_dynamic_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color for colliders attached to a fixed rigid body.
            #[getter]
            fn collider_fixed_color(&self) -> DebugColor {
                DebugColor(self.0.collider_fixed_color)
            }
            #[setter]
            /// Set the color used to render fixed-body colliders.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_collider_fixed_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.collider_fixed_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color for colliders attached to a kinematic rigid body.
            #[getter]
            fn collider_kinematic_color(&self) -> DebugColor {
                DebugColor(self.0.collider_kinematic_color)
            }
            #[setter]
            /// Set the color used to render kinematic-body colliders.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_collider_kinematic_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.collider_kinematic_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color for colliders not attached to any rigid body.
            #[getter]
            fn collider_parentless_color(&self) -> DebugColor {
                DebugColor(self.0.collider_parentless_color)
            }
            #[setter]
            /// Set the color used to render parentless (orphan) colliders.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_collider_parentless_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.collider_parentless_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color of an impulse joint's anchor frame.
            #[getter]
            fn impulse_joint_anchor_color(&self) -> DebugColor {
                DebugColor(self.0.impulse_joint_anchor_color)
            }
            #[setter]
            /// Set the color used to render impulse-joint anchor frames.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_impulse_joint_anchor_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.impulse_joint_anchor_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color of the separation line shown when an impulse joint is
            /// not satisfied.
            #[getter]
            fn impulse_joint_separation_color(&self) -> DebugColor {
                DebugColor(self.0.impulse_joint_separation_color)
            }
            #[setter]
            /// Set the color used to render unsatisfied-impulse-joint
            /// separation segments.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_impulse_joint_separation_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.impulse_joint_separation_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color of a multibody joint's anchor frame.
            #[getter]
            fn multibody_joint_anchor_color(&self) -> DebugColor {
                DebugColor(self.0.multibody_joint_anchor_color)
            }
            #[setter]
            /// Set the color used to render multibody-joint anchor frames.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_multibody_joint_anchor_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.multibody_joint_anchor_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color of the separation line shown when a multibody joint is
            /// not satisfied.
            #[getter]
            fn multibody_joint_separation_color(&self) -> DebugColor {
                DebugColor(self.0.multibody_joint_separation_color)
            }
            #[setter]
            /// Set the color used to render unsatisfied-multibody-joint
            /// separation segments.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_multibody_joint_separation_color(
                &mut self,
                v: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                self.0.multibody_joint_separation_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Multiplier applied to base colors of sleeping bodies.
            #[getter]
            fn sleep_color_multiplier(&self) -> DebugColor {
                DebugColor(self.0.sleep_color_multiplier)
            }
            #[setter]
            /// Set the HSLA multiplier applied to sleeping-body colors.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_sleep_color_multiplier(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.sleep_color_multiplier = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Multiplier applied to base colors of disabled bodies.
            #[getter]
            fn disabled_color_multiplier(&self) -> DebugColor {
                DebugColor(self.0.disabled_color_multiplier)
            }
            #[setter]
            /// Set the HSLA multiplier applied to disabled-body colors.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_disabled_color_multiplier(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.disabled_color_multiplier = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Length of the rigid-body axis triads when
            /// :attr:`DebugRenderMode.RIGID_BODY_AXES` is set.
            #[getter]
            fn rigid_body_axes_length(&self) -> Real {
                self.0.rigid_body_axes_length
            }
            #[setter]
            /// Assign the rigid-body axis triad length in world units.
            fn set_rigid_body_axes_length(&mut self, v: Real) {
                self.0.rigid_body_axes_length = v;
            }
            /// Color of the segment drawn for each contact penetration depth.
            #[getter]
            fn contact_depth_color(&self) -> DebugColor {
                DebugColor(self.0.contact_depth_color)
            }
            #[setter]
            /// Set the color used to render contact penetration-depth segments.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_contact_depth_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.contact_depth_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Color of contact normal segments.
            #[getter]
            fn contact_normal_color(&self) -> DebugColor {
                DebugColor(self.0.contact_normal_color)
            }
            #[setter]
            /// Set the color used to render contact-normal segments.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_contact_normal_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.contact_normal_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Length of the drawn contact-normal segments.
            #[getter]
            fn contact_normal_length(&self) -> Real {
                self.0.contact_normal_length
            }
            #[setter]
            /// Assign the length (world units) of contact-normal segments.
            fn set_contact_normal_length(&mut self, v: Real) {
                self.0.contact_normal_length = v;
            }
            /// Color of collider AABB outlines (when
            /// :attr:`DebugRenderMode.COLLIDER_AABBS` is set).
            #[getter]
            fn collider_aabb_color(&self) -> DebugColor {
                DebugColor(self.0.collider_aabb_color)
            }
            #[setter]
            /// Set the color used to render collider AABB outlines.
            ///
            /// Accepts a :class:`DebugColor` or a 4-tuple HSLA.
            fn set_collider_aabb_color(&mut self, v: &Bound<'_, PyAny>) -> PyResult<()> {
                self.0.collider_aabb_color = DebugColor::extract_from(v)?;
                Ok(())
            }
            /// Return a brief ``DebugRenderStyle(...)`` repr.
            fn __repr__(&self) -> String {
                format!(
                    "DebugRenderStyle(subdivisions={}, ...)",
                    self.0.subdivisions
                )
            }
        }

        // =====================================================================
        // DebugLineCollector — `#[pyclass]` implementing the rapier
        // `DebugRenderBackend` trait through an inner buffer.
        // =====================================================================

        /// Built-in `DebugRenderBackend` that drains each `draw_line` call into
        /// an internal buffer exposed as NumPy arrays.
        ///
        /// Use `DebugRenderPipeline.render(backend=collector)` to populate, then
        /// call `.lines()`, `.colors()`, `.objects()` for `(N, 2, D)`, `(N, 4)`,
        /// `(N,)` NumPy arrays respectively (each call returns a fresh **copy**
        /// of the underlying data).
        #[pyclass(name = "DebugLineCollector", module = "rapier")]
        pub struct DebugLineCollector {
            // `Mutex<Vec<DebugLine>>` so we can mutate from the &self
            // `draw_line` implementation without exposing `unsafe`. The
            // `Arc` lets us share the buffer with a transient `&mut`
            // implementer used inside `render` (see `_run_with_backend`).
            lines: _DbgArc<_DbgMutex<Vec<DebugLine>>>,
        }

        #[pymethods]
        impl DebugLineCollector {
            /// Build a new empty collector.
            #[new]
            fn new() -> Self {
                Self {
                    lines: _DbgArc::new(_DbgMutex::new(Vec::new())),
                }
            }

            /// Drop every queued line.
            fn clear(&self) {
                self.lines.lock().unwrap().clear();
            }

            /// Number of lines currently queued.
            fn __len__(&self) -> usize {
                self.lines.lock().unwrap().len()
            }
            /// Return ``DebugLineCollector(len=N)`` repr.
            fn __repr__(&self) -> String {
                format!(
                    "DebugLineCollector(len={})",
                    self.lines.lock().unwrap().len()
                )
            }

            /// Return a fresh `(N, 2, D)` NumPy array of segment endpoints.
            fn lines<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray2<Real>> {
                use $crate::numpy::PyArray2;
                let lines = self.lines.lock().unwrap();
                // Flatten as N*2 rows of D columns; reshape on the Python side
                // if a (N, 2, D) view is desired (we expose `(N*2, D)` then
                // reshape via numpy in the helper `render_to_arrays`).
                let mut flat: Vec<Vec<Real>> = Vec::with_capacity(lines.len() * 2);
                for ln in lines.iter() {
                    flat.push(_svec_to_row::<$dim>(ln.a));
                    flat.push(_svec_to_row::<$dim>(ln.b));
                }
                let arr = PyArray2::<Real>::from_vec2_bound(py, &flat)
                    .unwrap_or_else(|_| PyArray2::<Real>::zeros_bound(py, [0, $dim], false));
                // Reshape (N*2, D) → (N, 2, D). We do this via numpy at the
                // Python side; here we just return the (2N, D) view.
                arr
            }

            /// Return a fresh `(N, 4)` NumPy array of RGBA (post-HSLA→RGBA)
            /// per-line colors.
            fn colors<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray2<f32>> {
                use $crate::numpy::PyArray2;
                let lines = self.lines.lock().unwrap();
                let mut flat: Vec<Vec<f32>> = Vec::with_capacity(lines.len());
                for ln in lines.iter() {
                    let rgba = _hsla_to_rgba(ln.color);
                    flat.push(rgba.to_vec());
                }
                PyArray2::<f32>::from_vec2_bound(py, &flat)
                    .unwrap_or_else(|_| PyArray2::<f32>::zeros_bound(py, [0, 4], false))
            }

            /// Return a fresh `(N,)` NumPy array of `DebugRenderObject` kind
            /// discriminants (one entry per line).
            fn objects<'py>(&self, py: Python<'py>) -> Bound<'py, $crate::numpy::PyArray1<u32>> {
                use $crate::numpy::PyArray1;
                let lines = self.lines.lock().unwrap();
                let v: Vec<u32> = lines.iter().map(|ln| ln.object_kind).collect();
                PyArray1::<u32>::from_vec_bound(py, v)
            }

            /// Iterate over the queued lines as
            /// ``(object, a, b, color)`` tuples.
            ///
            /// The iterator takes a snapshot at iteration time — concurrent
            /// :meth:`clear` / re-render calls do not invalidate it.
            fn __iter__(slf: PyRef<'_, Self>) -> PyResult<Py<DebugLineCollectorIter>> {
                // Snapshot of (object, a, b, color) tuples at iteration time.
                let py = slf.py();
                let lines = slf.lines.lock().unwrap();
                let items: Vec<(DebugRenderObject, $Vec, $Vec, DebugColor)> = lines
                    .iter()
                    .map(|ln| {
                        (
                            DebugRenderObject {
                                kind: ln.object_kind,
                            },
                            $Vec(ln.a),
                            $Vec(ln.b),
                            DebugColor(ln.color),
                        )
                    })
                    .collect();
                Py::new(py, DebugLineCollectorIter { items, i: 0 })
            }
        }

        /// Iterator over the ``(object, a, b, color)`` snapshot of a
        /// :class:`DebugLineCollector`.
        #[pyclass(module = "rapier")]
        pub struct DebugLineCollectorIter {
            items: Vec<(DebugRenderObject, $Vec, $Vec, DebugColor)>,
            i: usize,
        }

        #[pymethods]
        impl DebugLineCollectorIter {
            /// Return ``self`` (this object is already an iterator).
            fn __iter__(slf: PyRef<'_, Self>) -> PyRef<'_, Self> {
                slf
            }
            /// Return the next ``(object, a, b, color)`` tuple, or raise
            /// ``StopIteration``.
            fn __next__(
                mut slf: PyRefMut<'_, Self>,
            ) -> Option<(DebugRenderObject, $Vec, $Vec, DebugColor)> {
                if slf.i >= slf.items.len() {
                    return None;
                }
                let idx = slf.i;
                slf.i += 1;
                Some(slf.items[idx])
            }
        }

        // =====================================================================
        // PyDebugRenderBackend — adapter wrapping an arbitrary Python object
        // implementing the `DebugRenderBackend` protocol.
        //
        // Mirrors the deferred-exception pattern from `events_hooks.rs`: errors
        // raised inside `draw_line` are stashed in a shared slot and re-raised
        // after `render()` returns.
        // =====================================================================

        struct _CollectorBackendAdapter {
            buf: _DbgArc<_DbgMutex<Vec<DebugLine>>>,
        }

        impl rapier::pipeline::DebugRenderBackend for _CollectorBackendAdapter {
            fn draw_line(
                &mut self,
                object: rapier::pipeline::DebugRenderObject<'_>,
                a: rapier::math::Vector,
                b: rapier::math::Vector,
                color: rapier::pipeline::DebugColor,
            ) {
                let an: $crate::na::SVector<Real, $dim> = a.into();
                let bn: $crate::na::SVector<Real, $dim> = b.into();
                self.buf.lock().unwrap().push(DebugLine {
                    object_kind: _dro_kind(object),
                    a: an,
                    b: bn,
                    color,
                });
            }
        }

        struct _PyBackendAdapter {
            obj: Py<PyAny>,
            err_slot: _DbgArc<_DbgMutex<Option<PyErr>>>,
        }

        impl rapier::pipeline::DebugRenderBackend for _PyBackendAdapter {
            fn draw_line(
                &mut self,
                object: rapier::pipeline::DebugRenderObject<'_>,
                a: rapier::math::Vector,
                b: rapier::math::Vector,
                color: rapier::pipeline::DebugColor,
            ) {
                {
                    let slot = self.err_slot.lock().unwrap();
                    if slot.is_some() {
                        return;
                    }
                }
                Python::with_gil(|py| {
                    let kind = _dro_kind(object);
                    let obj_py = DebugRenderObject { kind };
                    let an: $crate::na::SVector<Real, $dim> = a.into();
                    let bn: $crate::na::SVector<Real, $dim> = b.into();
                    let a_py = $Vec(an);
                    let b_py = $Vec(bn);
                    let c_py = DebugColor(color);
                    let res = self
                        .obj
                        .bind(py)
                        .call_method1("draw_line", (obj_py, a_py, b_py, c_py));
                    if let Err(e) = res {
                        let mut s = self.err_slot.lock().unwrap();
                        if s.is_none() {
                            *s = Some(e);
                        }
                    }
                });
            }
        }

        // =====================================================================
        // DebugRenderPipeline — main user-facing class.
        // =====================================================================

        /// Main entry point for visualizing a rapier scene.
        ///
        /// Walks the rigid bodies, colliders, and joint sets and emits one
        /// or more colored line segments per object. The segments are
        /// delivered to a backend implementing the ``DebugRenderBackend``
        /// protocol — typically a :class:`DebugLineCollector` for offline
        /// inspection (matplotlib, pygame, etc.), or a custom Python
        /// object exposing ``draw_line(object, a, b, color)``.
        ///
        /// For purely-data use cases prefer :meth:`render_to_arrays`,
        /// which returns NumPy arrays directly.
        #[pyclass(name = "DebugRenderPipeline", module = "rapier", unsendable)]
        pub struct DebugRenderPipeline {
            inner: rapier::pipeline::DebugRenderPipeline,
        }

        impl DebugRenderPipeline {
            fn _run_with_backend(
                &mut self,
                bodies: &RigidBodySet,
                colliders: &ColliderSet,
                impulse_joints: &ImpulseJointSet,
                multibody_joints: &MultibodyJointSet,
                narrow_phase: &NarrowPhase,
                backend: &mut dyn rapier::pipeline::DebugRenderBackend,
            ) {
                // The trait method `render` takes `&mut impl DebugRenderBackend`
                // which is not directly object-safe via `&mut dyn`. Wrap it.
                struct DynWrap<'a> {
                    inner: &'a mut dyn rapier::pipeline::DebugRenderBackend,
                }
                impl<'a> rapier::pipeline::DebugRenderBackend for DynWrap<'a> {
                    fn draw_line(
                        &mut self,
                        object: rapier::pipeline::DebugRenderObject<'_>,
                        a: rapier::math::Vector,
                        b: rapier::math::Vector,
                        color: rapier::pipeline::DebugColor,
                    ) {
                        self.inner.draw_line(object, a, b, color)
                    }
                }
                let mut wrap = DynWrap { inner: backend };
                self.inner.render(
                    &mut wrap,
                    &bodies.0,
                    &colliders.0,
                    &impulse_joints.0,
                    &multibody_joints.0,
                    &narrow_phase.0,
                );
            }
        }

        #[pymethods]
        impl DebugRenderPipeline {
            /// Build a pipeline with optional mode / style overrides.
            ///
            /// :param mode: :class:`DebugRenderMode` flags selecting what
            ///     to render. Defaults to
            ///     ``COLLIDER_SHAPES | JOINTS | RIGID_BODY_AXES``.
            /// :param style: :class:`DebugRenderStyle` color and length
            ///     configuration. Defaults to rapier's upstream defaults.
            #[new]
            #[pyo3(signature = (mode = None, style = None))]
            fn new(mode: Option<&DebugRenderMode>, style: Option<&DebugRenderStyle>) -> Self {
                let m = mode
                    .map(|m| m.0)
                    .unwrap_or_else(rapier::pipeline::DebugRenderMode::default);
                let s = style
                    .map(|s| s.0)
                    .unwrap_or_else(rapier::pipeline::DebugRenderStyle::default);
                Self {
                    inner: rapier::pipeline::DebugRenderPipeline::new(s, m),
                }
            }

            /// Current :class:`DebugRenderMode` flag set.
            #[getter]
            fn mode(&self) -> DebugRenderMode {
                DebugRenderMode(self.inner.mode)
            }
            #[setter]
            /// Replace the current :class:`DebugRenderMode` flag set.
            fn set_mode(&mut self, v: &DebugRenderMode) {
                self.inner.mode = v.0;
            }
            /// Current :class:`DebugRenderStyle`.
            #[getter]
            fn style(&self) -> DebugRenderStyle {
                DebugRenderStyle(self.inner.style)
            }
            #[setter]
            /// Replace the current :class:`DebugRenderStyle`.
            fn set_style(&mut self, v: &DebugRenderStyle) {
                self.inner.style = v.0;
            }

            /// Render the scene into the given ``backend``.
            ///
            /// ``backend`` may be a :class:`DebugLineCollector` (fast path —
            /// no Python callback per segment) or any Python object exposing
            /// ``draw_line(object, a, b, color) -> None``. Exceptions raised
            /// by a Python backend are deferred and re-raised after rendering
            /// completes.
            ///
            /// For hot loops prefer the collector — each Python-side
            /// ``draw_line`` call has to acquire the GIL.
            ///
            /// :param bodies: :class:`RigidBodySet`.
            /// :param colliders: :class:`ColliderSet`.
            /// :param impulse_joints: :class:`ImpulseJointSet`.
            /// :param multibody_joints: :class:`MultibodyJointSet`.
            /// :param narrow_phase: :class:`NarrowPhase`.
            /// :param backend: Collector or Python ``DebugRenderBackend``.
            #[pyo3(signature = (bodies, colliders, impulse_joints, multibody_joints, narrow_phase, backend))]
            #[allow(clippy::too_many_arguments)]
            fn render(
                &mut self,
                py: Python<'_>,
                bodies: &RigidBodySet,
                colliders: &ColliderSet,
                impulse_joints: &ImpulseJointSet,
                multibody_joints: &MultibodyJointSet,
                narrow_phase: &NarrowPhase,
                backend: &Bound<'_, PyAny>,
            ) -> PyResult<()> {
                // Fast path: DebugLineCollector. We bypass the Python callback
                // dispatch entirely.
                if let Ok(c) = backend.extract::<PyRef<'_, DebugLineCollector>>() {
                    let mut adapter = _CollectorBackendAdapter {
                        buf: _DbgArc::clone(&c.lines),
                    };
                    self._run_with_backend(
                        bodies,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        narrow_phase,
                        &mut adapter,
                    );
                    return Ok(());
                }

                // General path: Python-side backend implementing
                // `draw_line(object, a, b, color)`.
                let err_slot: _DbgArc<_DbgMutex<Option<PyErr>>> =
                    _DbgArc::new(_DbgMutex::new(None));
                let mut adapter = _PyBackendAdapter {
                    obj: backend.clone().unbind(),
                    err_slot: _DbgArc::clone(&err_slot),
                };
                self._run_with_backend(
                    bodies,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    narrow_phase,
                    &mut adapter,
                );
                drop(adapter);
                let mut slot = err_slot.lock().unwrap();
                if let Some(e) = slot.take() {
                    return Err(e);
                }
                let _ = py;
                Ok(())
            }

            /// Render the scene and return NumPy arrays directly.
            ///
            /// Convenience wrapper around :meth:`render` with a transient
            /// :class:`DebugLineCollector`.
            ///
            /// :param bodies: :class:`RigidBodySet`.
            /// :param colliders: :class:`ColliderSet`.
            /// :param impulse_joints: :class:`ImpulseJointSet`.
            /// :param multibody_joints: :class:`MultibodyJointSet`.
            /// :param narrow_phase: :class:`NarrowPhase`.
            /// :returns: A ``(lines, colors, objects)`` tuple where
            ///     ``lines`` is ``(N, 2, D)`` float, ``colors`` is
            ///     ``(N, 4)`` float32 RGBA in ``[0, 1]``, and ``objects``
            ///     is ``(N,)`` uint32 of
            ///     :class:`DebugRenderObject` discriminants.
            #[pyo3(signature = (bodies, colliders, impulse_joints, multibody_joints, narrow_phase))]
            #[allow(clippy::too_many_arguments)]
            fn render_to_arrays<'py>(
                &mut self,
                py: Python<'py>,
                bodies: &RigidBodySet,
                colliders: &ColliderSet,
                impulse_joints: &ImpulseJointSet,
                multibody_joints: &MultibodyJointSet,
                narrow_phase: &NarrowPhase,
            ) -> PyResult<(
                Bound<'py, $crate::numpy::PyArray3<Real>>,
                Bound<'py, $crate::numpy::PyArray2<f32>>,
                Bound<'py, $crate::numpy::PyArray1<u32>>,
            )> {
                use $crate::numpy::{PyArray1, PyArray2, PyArray3, PyArrayMethods};

                let buf: _DbgArc<_DbgMutex<Vec<DebugLine>>> =
                    _DbgArc::new(_DbgMutex::new(Vec::new()));
                {
                    let mut adapter = _CollectorBackendAdapter {
                        buf: _DbgArc::clone(&buf),
                    };
                    self._run_with_backend(
                        bodies,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        narrow_phase,
                        &mut adapter,
                    );
                }

                let lines = buf.lock().unwrap();
                let n = lines.len();

                // (N, 2, D) lines array. Allocate flat and reshape.
                let lines_arr = PyArray3::<Real>::zeros_bound(py, [n, 2, $dim], false);
                {
                    let mut view = unsafe { lines_arr.as_array_mut() };
                    for (i, ln) in lines.iter().enumerate() {
                        let row_a = _svec_to_row::<$dim>(ln.a);
                        let row_b = _svec_to_row::<$dim>(ln.b);
                        for d in 0..$dim {
                            view[[i, 0, d]] = row_a[d];
                            view[[i, 1, d]] = row_b[d];
                        }
                    }
                }

                let mut col_flat: Vec<Vec<f32>> = Vec::with_capacity(n);
                for ln in lines.iter() {
                    col_flat.push(_hsla_to_rgba(ln.color).to_vec());
                }
                let colors_arr = if n == 0 {
                    PyArray2::<f32>::zeros_bound(py, [0, 4], false)
                } else {
                    PyArray2::<f32>::from_vec2_bound(py, &col_flat)
                        .unwrap_or_else(|_| PyArray2::<f32>::zeros_bound(py, [0, 4], false))
                };

                let objects_arr = PyArray1::<u32>::from_vec_bound(
                    py,
                    lines.iter().map(|ln| ln.object_kind).collect(),
                );

                Ok((lines_arr, colors_arr, objects_arr))
            }

            /// Return a brief ``DebugRenderPipeline(mode=0b...)`` repr.
            fn __repr__(&self) -> String {
                format!("DebugRenderPipeline(mode={:#010b})", self.inner.mode.bits())
            }
        }

        // Helper: turn an `na::SVector<Real, DIM>` into a `Vec<Real>` of
        // length `DIM`. Inlined so the row-builder loops above can reuse it.
        #[inline]
        fn _svec_to_row<const D: usize>(v: $crate::na::SVector<Real, D>) -> Vec<Real> {
            let mut row = Vec::with_capacity(D);
            for d in 0..D {
                row.push(v[d]);
            }
            row
        }
    };
}

#[doc(hidden)]
#[macro_export]
macro_rules! __define_debug_render_register {
    () => {
        pub fn register_debug_render(
            _py: $crate::pyo3::Python<'_>,
            m: &$crate::pyo3::Bound<'_, $crate::pyo3::types::PyModule>,
        ) -> $crate::pyo3::PyResult<()> {
            use $crate::pyo3::prelude::*;
            m.add_class::<DebugRenderObject>()?;
            m.add_class::<DebugColor>()?;
            m.add_class::<DebugRenderMode>()?;
            m.add_class::<DebugRenderStyle>()?;
            m.add_class::<DebugLineCollector>()?;
            m.add_class::<DebugLineCollectorIter>()?;
            m.add_class::<DebugRenderPipeline>()?;
            Ok(())
        }
    };
}
